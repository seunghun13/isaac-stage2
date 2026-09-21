"""Camera layout validation and session-layer application; no rendering or IO.

Only apply_layout imports pxr. Call it on the Kit main thread with timeline
stopped. It changes only its owned /MRO_Cameras subtree in the session layer,
never saves a layer, creates no render products, and starts no capture.
"""
import copy
import hashlib
import itertools
import json
import math
import re

ROOT_PATH = "/MRO_Cameras"
OWNER = "mro_camera_layout_runtime_v1"
MAX_CAMERAS = 12
# RTX in this deployed Kit clamps the canonical ZED focal value to an
# effective 0.5. Homogeneous scaling preserves the pinhole ratios with DOF off.
# This does not change the model's physical focal/sensor dimensions.
PINHOLE_REPRESENTATION_SCALE = 100.0


def _number(value, name, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    if positive and value <= 0:
        raise ValueError(name + " must be positive")
    return float(value)


def _vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + f" must contain {size} numbers")
    return [_number(x, name) for x in value]


def _unit(v, name):
    length = math.sqrt(sum(x*x for x in v))
    if length < 1e-10:
        raise ValueError(name + " is degenerate")
    return [x / length for x in v]


def _cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def _canonical_quaternion(q):
    q = _unit(q, "orientation_wxyz")
    for component in q:
        if abs(component) > 1e-12:
            return [-x for x in q] if component < 0 else q
    raise ValueError("zero quaternion")


def _matrix_to_quaternion(m):
    # m uses column-vector rotation convention; output is local-to-world wxyz.
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0)*2
        q = [s/4, (m[2][1]-m[1][2])/s, (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s]
    else:
        i = max(range(3), key=lambda j: m[j][j])
        j, k = (i+1) % 3, (i+2) % 3
        s = math.sqrt(max(0.0, 1+m[i][i]-m[j][j]-m[k][k]))*2
        v = [0.0]*3
        v[i], v[j], v[k] = s/4, (m[i][j]+m[j][i])/s, (m[i][k]+m[k][i])/s
        q = [(m[k][j]-m[j][k])/s] + v
    return _canonical_quaternion(q)


def rotate_vector(q, v):
    """Stdlib local-to-world rotation, useful to independently verify look-at."""
    w, x, y, z = q
    uv = _cross([x, y, z], v)
    uuv = _cross([x, y, z], uv)
    return [v[i] + 2*(w*uv[i] + uuv[i]) for i in range(3)]


def orientation_for_pose(position, pose):
    if not isinstance(pose, dict):
        raise ValueError("pose must be an object")
    mode = pose.get("type")
    if mode == "look_at":
        if set(pose) != {"type", "target_m", "up_world"}:
            raise ValueError("look_at pose requires only type, target_m, up_world")
        target = _vector(pose["target_m"], 3, "target_m")
        up = _unit(_vector(pose["up_world"], 3, "up_world"), "up_world")
        forward = _unit([target[i]-position[i] for i in range(3)], "look_at direction")
        right = _unit(_cross(forward, up), "look_at direction/up cross product")
        camera_up = _cross(right, forward)
        backward = [-v for v in forward]
        return _matrix_to_quaternion([[right[i], camera_up[i], backward[i]] for i in range(3)])
    if mode == "quaternion_wxyz":
        if set(pose) != {"type", "orientation_wxyz"}:
            raise ValueError("quaternion pose requires only type, orientation_wxyz")
        q = _vector(pose["orientation_wxyz"], 4, "orientation_wxyz")
        if abs(math.sqrt(sum(x*x for x in q))-1) > 1e-5:
            raise ValueError("orientation_wxyz must be a unit quaternion")
        return _canonical_quaternion(q)
    raise ValueError("pose.type must be look_at or quaternion_wxyz")


def millimeters_to_usd_camera_units(mm, meters_per_unit):
    """USD lens/filmback values use tenths of a scene unit, NOT millimeters.

    https://openusd.org/release/api/class_usd_geom_camera.html
    1 meter/unit => 1 camera unit = 0.1m = 100mm; hence mm/100.
    """
    return _number(mm, "millimeters", True)/(100*_number(meters_per_unit, "meters_per_unit", True))


def lens_representation(spec, meters_per_unit):
    """Separate physical mm, canonical USD units, and renderer representation."""
    physical = {"focal_length_mm": spec["focal_length_mm"],
                "sensor_width_mm": spec["sensor_width_mm"],
                "sensor_height_mm": spec["sensor_height_mm"]}
    canonical = {key: millimeters_to_usd_camera_units(physical[source], meters_per_unit)
                 for key, source in (("focalLength", "focal_length_mm"),
                                     ("horizontalAperture", "sensor_width_mm"),
                                     ("verticalAperture", "sensor_height_mm"))}
    return {"version": 2, "kind": "homogeneous_pinhole_representation",
            "homogeneous_scale": PINHOLE_REPRESENTATION_SCALE,
            "physical_reference_mm": physical,
            "physical_values_are_not_scaled": True,
            "canonical_usd_camera_units": canonical,
            "authored_usd_camera_units": {key: value*PINHOLE_REPRESENTATION_SCALE for key, value in canonical.items()},
            "depth_of_field_enabled": False,
            "scope": "Preserves centered pinhole K/FOV only; no physical lens-size or DOF claim",
            "evidence": "outputs/projection_20260909T160757_91dfb509/diagnostic.json"}


def camera_intrinsics(model, resolution_mode):
    pin = model["pinhole"]
    native = model["native_resolution_px"]
    resolution = model[resolution_mode + "_resolution_px"]
    for value in list(native) + list(resolution):
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError("camera resolution must use positive integer pixels")
    if len(native) != 2 or len(resolution) != 2:
        raise ValueError("camera resolution must be [width,height]")
    sx, sy = resolution[0]/native[0], resolution[1]/native[1]
    if abs(sx-sy) > 1e-10:
        raise ValueError("preview must preserve native aspect ratio; crop is not implemented")
    sensor_w = _number(pin["sensor_width_mm"], "sensor_width_mm", True)
    sensor_h = _number(pin["sensor_height_mm"], "sensor_height_mm", True)
    focal = _number(pin["focal_length_mm"], "focal_length_mm", True)
    fx, fy = focal/sensor_w*native[0], focal/sensor_h*native[1]
    for key, actual in (("fx_px", fx), ("fy_px", fy)):
        if not math.isclose(actual, _number(pin[key], key, True), rel_tol=1e-6, abs_tol=1e-5):
            raise ValueError("model physical pinhole and pixel intrinsics disagree: " + key)
    # Current two models are centered. Explicit rejection avoids a wrong offset sign.
    if pin["cx_px"] != native[0]/2 or pin["cy_px"] != native[1]/2:
        raise ValueError("non-centered principal point is not supported by this layout module")
    if pin.get("distortion_coefficients") is not None or pin.get("depth_of_field_enabled"):
        raise ValueError("only ideal centered pinhole with depth of field disabled is implemented")
    return {"resolution_px": list(resolution), "native_resolution_px": list(native),
            "intrinsics": {"fx_px": fx*sx, "fy_px": fy*sy,
                           "cx_px": pin["cx_px"]*sx, "cy_px": pin["cy_px"]*sy},
            "focal_length_mm": focal, "sensor_width_mm": sensor_w, "sensor_height_mm": sensor_h,
            "horizontal_fov_deg": math.degrees(2*math.atan(sensor_w/(2*focal))),
            "vertical_fov_deg": math.degrees(2*math.atan(sensor_h/(2*focal)))}


def validate_layout(config, models):
    """Return a JSON-serializable normalized plan. Does not import pxr or write."""
    if config.get("schema_version") != 1:
        raise ValueError("layout schema_version must be 1")
    height = _number(config.get("common_height_m"), "common_height_m", True)
    stations = config.get("stations")
    if not isinstance(stations, list) or not 1 <= len(stations) <= MAX_CAMERAS:
        raise ValueError("layout must contain 1 through 12 stations")
    catalog = models.get("models", models)
    bounds = config["placement_bounds_m"]
    xlim, ylim = _vector(bounds["x"], 2, "bounds.x"), _vector(bounds["y"], 2, "bounds.y")
    if not xlim[0] < xlim[1] or not ylim[0] < ylim[1]:
        raise ValueError("placement bounds are reversed")
    cameras = {}
    for station in stations:
        if not isinstance(station, dict) or set(station) - {"id", "model", "position_xy_m", "pose", "resolution_mode", "scheduled_fps"}:
            raise ValueError("unknown station field or station is not an object")
        sid = station.get("id")
        if not isinstance(sid, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]{0,63}", sid) or sid in cameras:
            raise ValueError("camera id must be unique and a valid USD identifier")
        model_id = station.get("model")
        if model_id not in catalog:
            raise ValueError("unknown camera model: " + str(model_id))
        xy = _vector(station.get("position_xy_m"), 2, "position_xy_m")
        if not (xlim[0] <= xy[0] <= xlim[1] and ylim[0] <= xy[1] <= ylim[1]):
            raise ValueError(sid + " is outside placement bounds")
        for box in config.get("conservative_keepout_boxes_xy_m", []):
            bx, by = _vector(box["x"], 2, "keepout.x"), _vector(box["y"], 2, "keepout.y")
            if bx[0] <= xy[0] <= bx[1] and by[0] <= xy[1] <= by[1]:
                raise ValueError(sid + " is inside conservative aircraft XY keepout")
        pos = xy + [height]
        mode = station.get("resolution_mode", "preview")
        if mode not in ("preview", "native"):
            raise ValueError("resolution_mode must be preview or native")
        fps = _number(station.get("scheduled_fps", catalog[model_id]["simulation"]["scheduled_fps"]), "scheduled_fps", True)
        spec = {"id": sid, "camera_id": sid, "model": model_id, "model_id": model_id,
                "prim_path": ROOT_PATH + "/" + sid, "position_m": pos,
                "orientation_wxyz": orientation_for_pose(pos, station["pose"]),
                "pose": copy.deepcopy(station["pose"]), "resolution_mode": mode,
                "scheduled_fps": fps, "render_product_created": False,
                "pose_convention": "USD camera local +X right, +Y up, -Z forward; quaternion local-to-world wxyz",
                **camera_intrinsics(catalog[model_id], mode)}
        cameras[sid] = spec
    requested = config.get("render_camera_ids", [])
    if not isinstance(requested, list) or any(not isinstance(x, str) for x in requested) or len(set(requested)) != len(requested) or any(x not in cameras for x in requested):
        raise ValueError("render_camera_ids must be unique camera ids from stations")
    min_sep = min((math.dist(a["position_m"], b["position_m"]) for a, b in itertools.combinations(cameras.values(), 2)), default=None)
    if min_sep is not None and min_sep < 0.1:
        raise ValueError("camera stations must be at least 0.1 m apart")
    fingerprint = hashlib.sha256(json.dumps({"layout": config, "models": models}, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return {"schema_version": 1, "owner": OWNER, "camera_root": ROOT_PATH,
            "cameras": cameras, "camera_count": len(cameras), "common_height_m": height,
            "minimum_pair_distance_m": min_sep, "config_models_sha256": fingerprint,
            "render_camera_ids": list(requested), "render_products_created": 0,
            "status": "validated_not_applied", "visibility_occlusion_verified": False,
            "lens_representation_version": 2,
            "pinhole_representation_scale": PINHOLE_REPRESENTATION_SCALE,
            "warnings": list(config.get("warnings", []))}


def apply_layout(stage, config, models):
    """Apply own camera prims only in the existing stage's session layer.

    Reapplication replaces our session-only subtree, preserving assets and all
    other session content. Main-thread use is required; no concurrent layer edits.
    """
    plan = validate_layout(config, models)
    from pxr import Gf, Sdf, Usd, UsdGeom
    if str(UsdGeom.GetStageUpAxis(stage)) != "Z":
        raise ValueError("layout coordinates require a Z-up stage")
    mpu = _number(UsdGeom.GetStageMetersPerUnit(stage), "stage metersPerUnit", True)
    session = stage.GetSessionLayer()
    if session is None or not session.permissionToEdit:
        raise ValueError("editable session layer required")
    existing = stage.GetPrimAtPath(ROOT_PATH)
    if existing:
        if existing.GetCustomDataByKey("mro:owner") != OWNER:
            raise ValueError("camera root already exists and is not owned by this module")
        for prim in Usd.PrimRange(existing):
            if any(spec.layer != session for spec in prim.GetPrimStack()):
                raise ValueError("refusing to replace camera subtree authored outside the session layer")
    previous = session.ExportToString()
    try:
        with Usd.EditContext(stage, session):
            if existing:
                stage.RemovePrim(ROOT_PATH)
            root = UsdGeom.Xform.Define(stage, ROOT_PATH).GetPrim()
            root.SetCustomDataByKey("mro:owner", OWNER)
            root.SetCustomDataByKey("mro:layoutHash", plan["config_models_sha256"])
            for sid, spec in plan["cameras"].items():
                representation = lens_representation(spec, mpu)
                spec["usd_lens_representation"] = representation
                represented = representation["authored_usd_camera_units"]
                camera = UsdGeom.Camera.Define(stage, spec["prim_path"])
                xf = UsdGeom.Xformable(camera.GetPrim())
                xf.ClearXformOpOrder()
                xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*[x/mpu for x in spec["position_m"]]))
                q = spec["orientation_wxyz"]
                xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
                xf.SetResetXformStack(True)
                camera.CreateProjectionAttr().Set(UsdGeom.Tokens.perspective)
                camera.CreateFocalLengthAttr().Set(represented["focalLength"])
                camera.CreateHorizontalApertureAttr().Set(represented["horizontalAperture"])
                camera.CreateVerticalApertureAttr().Set(represented["verticalAperture"])
                camera.CreateHorizontalApertureOffsetAttr().Set(0.0)
                camera.CreateVerticalApertureOffsetAttr().Set(0.0)
                camera.CreateFStopAttr().Set(0.0)
                camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1/mpu, 200.0/mpu))
                camera.CreateShutterOpenAttr().Set(0.0)
                camera.CreateShutterCloseAttr().Set(0.0)
                prim = camera.GetPrim()
                prim.SetCustomDataByKey("mro:owner", OWNER)
                prim.SetCustomDataByKey("mro:cameraConfigJson", json.dumps(spec, sort_keys=True))
                world = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                observed_pos = [float(v)*mpu for v in world.ExtractTranslation()]
                observed_forward = list(world.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized())
                expected_forward = rotate_vector(q, [0, 0, -1])
                if math.dist(observed_pos, spec["position_m"]) > 1e-7 or math.dist(observed_forward, expected_forward) > 1e-7:
                    raise RuntimeError("USD camera transform readback failed: " + sid)
                spec["usd_readback"] = {"position_m": observed_pos, "forward_world": observed_forward,
                                        "focal_length": camera.GetFocalLengthAttr().Get(),
                                        "horizontal_aperture": camera.GetHorizontalApertureAttr().Get(),
                                        "vertical_aperture": camera.GetVerticalApertureAttr().Get()}
        plan.update(status="applied_session_layer_only", stage_meters_per_unit=mpu,
                    session_layer_identifier=session.identifier, camera_prims_verified=True)
        return plan
    except Exception:
        session.ImportFromString(previous)
        raise
