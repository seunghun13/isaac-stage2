"""Session-only, noncolliding visual proxies for installed camera stations.

This module neither changes optical camera prims nor creates render products.
Call apply_camera_rig_visuals on Kit's main thread while physics is paused and
no capture/stage edit is running. The caller owns launch/frozen-state checks.
Only that function imports USD; plan_camera_rigs is a stdlib-only CPU helper.

The dimensions are approximate visual geometry, not manufacturer CAD, a
calibrated mount, or a collision/visibility guarantee. Camera local -Z is the
viewing direction; the proxy's front lens center is the actual optical origin.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import re

OWNER = "mro_camera_rig_visuals_v1"
ROOT_PATH = "/MRO_CameraRigs"
CAMERA_ROOT = "/MRO_Cameras"
CAMERA_OWNER = "mro_camera_layout_runtime_v1"
DEFAULT_CAMERA_IDS = tuple("cam_" + str(index).zfill(2) for index in range(1, 13))
MODEL_COLORS = {
    "zed_x_one_4k_wide": [0.10, 0.36, 0.44],
    "flir_bfs_u3_244s8c_kowa_lm12xc": [0.65, 0.39, 0.10],
}
BODY_SIZE_M = [0.15, 0.10, 0.10]
BODY_CENTER_LOCAL_M = [0.0, 0.0, 0.08]
LENS_RADIUS_M = 0.027
LENS_LENGTH_M = 0.030
TRIPOD_FOOT_RADIUS_M = 0.35


def _vec(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(name + " must contain " + str(size) + " numbers")
    if any(isinstance(x, bool) or not isinstance(x, (int, float))
           or not math.isfinite(x) for x in value):
        raise ValueError(name + " must contain finite numbers")
    return [float(x) for x in value]


def _rotate(q, v):
    w, x, y, z = q
    ux, uy, uz = y*v[2]-z*v[1], z*v[0]-x*v[2], x*v[1]-y*v[0]
    vx, vy, vz = y*uz-z*uy, z*ux-x*uz, x*uy-y*ux
    return [v[0]+2*(w*ux+vx), v[1]+2*(w*uy+vy), v[2]+2*(w*uz+vz)]


def rod_spec(start_m, end_m, radius_m):
    """Cylinder local +Z -> end-start, with an exact world-space midpoint."""
    start, end = _vec(start_m, 3, "rod start"), _vec(end_m, 3, "rod end")
    if isinstance(radius_m, bool) or not isinstance(radius_m, (int, float)) \
            or not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("rod radius must be positive and finite")
    delta = [b-a for a, b in zip(start, end)]
    length = math.sqrt(sum(x*x for x in delta))
    if length < 1e-8:
        raise ValueError("rod endpoints must be distinct")
    direction = [x/length for x in delta]
    if direction[2] < -1+1e-12:
        q = [0., 1., 0., 0.]
    else:
        # Quaternion [1 + a dot b, a cross b], a = [0,0,1].
        q = [1+direction[2], -direction[1], direction[0], 0.]
        qnorm = math.sqrt(sum(x*x for x in q))
        q = [x/qnorm for x in q]
    return {"start_m": start, "end_m": end,
            "center_m": [(a+b)/2 for a, b in zip(start, end)],
            "orientation_wxyz": q, "length_m": length,
            "radius_m": float(radius_m)}


def validate_existing_owner(owner, layers, session_identifier):
    """Applied to every existing proxy prim, including hidden stations."""
    if owner != OWNER or not layers or any(layer != session_identifier for layer in layers):
        raise ValueError("Existing camera rig subtree is not exclusively our session-only geometry")


def assert_pose_alignment(rows, position_m, orientation_wxyz, *, tolerance=1e-7):
    """Check all rotation axes, translation, and affine matrix entries."""
    if not isinstance(rows, (list, tuple)) or len(rows) != 4:
        raise ValueError("world matrix must have four rows")
    matrix = [_vec(row, 4, "world matrix row") for row in rows]
    expected = [_rotate(orientation_wxyz, axis)+[0.]
                for axis in ([1.,0.,0.], [0.,1.,0.], [0.,0.,1.])]
    expected.append(list(position_m)+[1.])
    error = max(abs(matrix[r][c]-expected[r][c]) for r in range(4) for c in range(4))
    if error > tolerance:
        raise ValueError("Optical world pose differs from the applied layout (max error " + str(error) + ")")
    return error


def plan_camera_rigs(layout_result, camera_ids=DEFAULT_CAMERA_IDS):
    """Return approximate geometry at exact planned optical poses; no IO/USD."""
    if not isinstance(layout_result, dict) or layout_result.get("owner") != CAMERA_OWNER \
            or layout_result.get("camera_root") != CAMERA_ROOT:
        raise ValueError("Expected camera_layout_runtime result")
    cameras = layout_result.get("cameras")
    if not isinstance(cameras, dict) or not 1 <= len(cameras) <= 12:
        raise ValueError("Layout must contain 1 through 12 cameras")
    if not isinstance(camera_ids, (list, tuple)) or not 1 <= len(camera_ids) <= 12 \
            or any(not isinstance(cid, str) for cid in camera_ids) \
            or len(set(camera_ids)) != len(camera_ids):
        raise ValueError("camera_ids must be a unique nonempty camera ID list")
    fingerprint = layout_result.get("config_models_sha256")
    if not isinstance(fingerprint, str) or not re.fullmatch(r"[0-9a-f]{64}", fingerprint):
        raise ValueError("Applied camera layout hash is missing")
    rigs = {}
    for cid in camera_ids:
        if not re.fullmatch(r"cam_(?:0[1-9]|1[0-2])", cid) or cid not in cameras:
            raise ValueError("Unknown or unsupported camera station: " + cid)
        spec = cameras[cid]
        if spec.get("prim_path") != CAMERA_ROOT + "/" + cid:
            raise ValueError("Camera path differs from its owned station ID")
        model = spec.get("model")
        if model not in MODEL_COLORS:
            raise ValueError("Unsupported visual proxy camera model: " + str(model))
        position = _vec(spec.get("position_m"), 3, "camera position")
        q = _vec(spec.get("orientation_wxyz"), 4, "camera orientation")
        if abs(sum(x*x for x in q)-1.) > 1e-8:
            raise ValueError("Camera orientation must be a unit quaternion")
        if not 0.5 <= position[2] <= 2.5:
            raise ValueError("This tripod proxy supports optical heights 0.5 through 2.5 m")
        # Keep the tripod upright. Only the camera + head follow optical tilt.
        mounting_offset = _rotate(q, [0., -BODY_SIZE_M[1]/2, BODY_CENTER_LOCAL_M[2]])
        mount = [position[i]+mounting_offset[i] for i in range(3)]
        hub = [mount[0], mount[1], position[2]-0.25]
        if mount[2] <= hub[2]+0.01:
            raise ValueError("Camera tilt does not fit the upright tripod proxy")
        feet = [[hub[0]+TRIPOD_FOOT_RADIUS_M*math.cos(i*2*math.pi/3),
                 hub[1]+TRIPOD_FOOT_RADIUS_M*math.sin(i*2*math.pi/3), 0.018]
                for i in range(3)]
        rods = {"center_column": rod_spec(hub, mount, .014)}
        rods.update({"leg_"+str(i+1): rod_spec(foot, hub, .012)
                     for i, foot in enumerate(feet)})
        # Braces make the sub-meter footprint recognizable without scaling it up.
        brace_hub = [hub[0], hub[1], min(.45, hub[2]*.45)]
        rods["lower_column"] = rod_spec(brace_hub, hub, .009)
        for i, foot in enumerate(feet):
            rods["brace_"+str(i+1)] = rod_spec(
                brace_hub,
                [foot[j]+.35*(hub[j]-foot[j]) for j in range(3)], .005)
        rigs[cid] = {
            "camera_id": cid, "camera_prim_path": spec["prim_path"], "model": model,
            "rig_prim_path": ROOT_PATH + "/" + cid,
            "optical_position_world_m": position, "optical_orientation_wxyz": q,
            "optical_forward_world": _rotate(q, [0.,0.,-1.]),
            "body_size_local_m": list(BODY_SIZE_M),
            "body_center_local_m": list(BODY_CENTER_LOCAL_M),
            "lens_front_center_local_m": [0.,0.,0.],
            "lens_radius_m": LENS_RADIUS_M, "lens_length_m": LENS_LENGTH_M,
            "mount_position_world_m": mount, "tripod_hub_world_m": hub,
            "tripod_feet_world_m": feet, "tripod_rods": rods,
            "model_accent_display_color": list(MODEL_COLORS[model]),
            "label_anchor_world_m": [position[0], position[1], position[2]+.23],
            "label_text": cid + (" · ZED" if model.startswith("zed_") else " · FLIR"),
        }
    result = {"schema_version": 1, "owner": OWNER, "rig_root": ROOT_PATH,
              "source_layout_sha256": fingerprint, "selected_camera_ids": list(camera_ids),
              "visible_rig_count": len(rigs), "rigs": rigs,
              "status": "planned_not_applied", "semantic_visual_proxy": True,
              "manufacturer_cad": False, "physical_mount_calibration_claim": False,
              "collisions_enabled": False, "rigid_bodies_created": 0,
              "render_products_created": 0, "sensor_render_products_modified": False,
              "optics_modified": False, "floor_reference_world_z_m": 0.,
              "in_scene_labels_created": False, "cctv_pixel_visibility_verified": False,
              "visibility_note": "Approximate life-size proxies may occupy only a few CCTV pixels; label anchors are metadata for an optional 2D overlay, not a guarantee of visibility."}
    result["geometry_plan_sha256"] = hashlib.sha256(
        json.dumps(result, sort_keys=True, allow_nan=False).encode()).hexdigest()
    return result


def apply_camera_rig_visuals(stage, layout_result, *, camera_ids=DEFAULT_CAMERA_IDS):
    """Build only our visual proxies; preserve all camera properties exactly.

    Must be invoked on the Kit main thread while paused, with no active capture.
    Existing unselected OWN station visuals become invisible. Foreign or asset
    authored rig content causes an error. No layer is saved, and no timeline,
    render product, sensor, or physics API is called.
    """
    plan = plan_camera_rigs(layout_result, camera_ids)
    from pxr import Gf, Sdf, Usd, UsdGeom

    session = stage.GetSessionLayer()
    if session is None or not session.permissionToEdit \
            or stage.GetEditTarget().GetLayer() != session \
            or stage.GetRootLayer().permissionToSave:
        raise ValueError("Expected protected assets and an editable active session layer")
    if str(UsdGeom.GetStageUpAxis(stage)) != "Z" \
            or abs(UsdGeom.GetStageMetersPerUnit(stage)-1.) > 1e-10:
        raise ValueError("Camera rig world coordinates require a Z-up meter stage")
    if layout_result.get("status") != "applied_session_layer_only" \
            or layout_result.get("camera_prims_verified") is not True \
            or layout_result.get("session_layer_identifier") != session.identifier:
        raise ValueError("Camera layout must already be applied and verified in this session")
    source_root = stage.GetPrimAtPath(CAMERA_ROOT)
    if not source_root or source_root.GetCustomDataByKey("mro:owner") != CAMERA_OWNER \
            or source_root.GetCustomDataByKey("mro:layoutHash") != plan["source_layout_sha256"]:
        raise ValueError("Applied camera subtree identity/hash does not match")

    def matrix_rows(prim):
        value = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        return [[float(value[r][c]) for c in range(4)] for r in range(4)]

    def camera_snapshot():
        result = {}
        for cid, spec in layout_result["cameras"].items():
            prim = stage.GetPrimAtPath(spec["prim_path"])
            if not prim or not prim.IsA(UsdGeom.Camera) \
                    or prim.GetCustomDataByKey("mro:owner") != CAMERA_OWNER \
                    or any(s.layer != session for s in prim.GetPrimStack()):
                raise ValueError("Camera is missing or not our session-only camera: " + cid)
            # Cover every attribute, including optics, transform, and time samples.
            attributes = {str(attr.GetName()): {
                "default": repr(attr.Get()),
                "time_samples": [[t, repr(attr.Get(t))] for t in attr.GetTimeSamples()],
                "connections": [str(p) for p in attr.GetConnections()],
            } for attr in prim.GetAttributes()}
            if any(value["time_samples"] for name, value in attributes.items()
                   if name.startswith("xformOp:")):
                raise ValueError("Static camera rig setup does not support animated camera poses")
            result[cid] = {"world_matrix": matrix_rows(prim), "attributes": attributes}
        return result

    cameras_before = camera_snapshot()
    for cid, rig in plan["rigs"].items():
        assert_pose_alignment(cameras_before[cid]["world_matrix"],
                              rig["optical_position_world_m"], rig["optical_orientation_wxyz"])
    existing = stage.GetPrimAtPath(ROOT_PATH)
    if existing:
        for prim in Usd.PrimRange(existing):
            validate_existing_owner(prim.GetCustomDataByKey("mro:owner"),
                [s.layer.identifier for s in prim.GetPrimStack()], session.identifier)
            if prim.IsInstance() or prim.HasAuthoredReferences() or prim.HasPayload() \
                    or any("Physics" in str(schema) or "Physx" in str(schema)
                           for schema in prim.GetAppliedSchemas()):
                raise ValueError("Existing camera rigs contain references, instances, or physics")
    backup = Sdf.Layer.CreateAnonymous("mro_camera_rigs_backup") if existing else None
    if backup is not None and not Sdf.CopySpec(session, ROOT_PATH, backup, ROOT_PATH):
        raise RuntimeError("Could not preserve prior owned rig visual specifications")

    def owned(prim):
        prim.SetCustomDataByKey("mro:owner", OWNER)
        prim.SetCustomDataByKey("mro:semanticVisualProxy", True)
        return prim

    def transform(prim, position, q=None, scale=None):
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*position))
        if q is not None:
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(q[0], Gf.Vec3d(*q[1:])))
        if scale is not None:
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))
        return xf

    def color(shape, rgb):
        shape.CreateDisplayColorAttr().Set([Gf.Vec3f(*rgb)])
        shape.CreateDisplayOpacityAttr().Set([1.])

    def cube(path, position, size, rgb):
        shape = UsdGeom.Cube.Define(stage, path)
        owned(shape.GetPrim())
        shape.CreateSizeAttr().Set(1.)
        transform(shape.GetPrim(), position, scale=size)
        color(shape, rgb)
        return shape

    def cylinder(path, position, radius, height, rgb, q=None):
        shape = UsdGeom.Cylinder.Define(stage, path)
        owned(shape.GetPrim())
        shape.CreateAxisAttr().Set(UsdGeom.Tokens.z)
        shape.CreateRadiusAttr().Set(radius)
        shape.CreateHeightAttr().Set(height)
        transform(shape.GetPrim(), position, q=q)
        color(shape, rgb)
        return shape

    try:
        with Usd.EditContext(stage, session):
            root = owned(UsdGeom.Xform.Define(stage, ROOT_PATH).GetPrim())
            root.SetCustomDataByKey("mro:layoutHash", plan["source_layout_sha256"])
            root.SetCustomDataByKey("mro:geometryPlanHash", plan["geometry_plan_sha256"])
            # Root may not inherit a prior transform from our own session edits.
            root_xf = UsdGeom.Xformable(root)
            if root_xf.GetOrderedXformOps():
                raise ValueError("Camera rig root unexpectedly has transform operations")
            root_xf.SetResetXformStack(True)
            UsdGeom.Imageable(root).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
            hidden = []
            for child in list(root.GetChildren()):
                if child.GetName() not in plan["selected_camera_ids"]:
                    UsdGeom.Imageable(child).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
                    hidden.append(str(child.GetName()))
            for cid, rig in plan["rigs"].items():
                path = rig["rig_prim_path"]
                if stage.GetPrimAtPath(path):
                    stage.RemovePrim(path)
                station = owned(UsdGeom.Xform.Define(stage, path).GetPrim())
                UsdGeom.Imageable(station).CreateVisibilityAttr().Set(UsdGeom.Tokens.inherited)
                station.SetCustomDataByKey("mro:cameraId", cid)
                station.SetCustomDataByKey("mro:cameraPrimPath", rig["camera_prim_path"])
                station.SetCustomDataByKey("mro:model", rig["model"])
                optical_path = path + "/OpticalFrame"
                optical = owned(UsdGeom.Xform.Define(stage, optical_path).GetPrim())
                transform(optical, rig["optical_position_world_m"], rig["optical_orientation_wxyz"])
                cube(optical_path+"/Housing", BODY_CENTER_LOCAL_M, BODY_SIZE_M, [.14,.15,.16])
                cube(optical_path+"/ModelAccent", [0., .0505, .085], [.075,.002,.055],
                     rig["model_accent_display_color"])
                cylinder(optical_path+"/LensBarrel", [0.,0.,LENS_LENGTH_M/2],
                         LENS_RADIUS_M, LENS_LENGTH_M, [.055,.06,.065])
                # The front center is the optical origin; all material stays at
                # local z >= 0, behind the camera's -Z viewing direction.
                cylinder(optical_path+"/LensGlass", [0.,0.,.0005],
                         LENS_RADIUS_M*.84, .001, [.04,.10,.14])
                tripod = owned(UsdGeom.Xform.Define(stage, path+"/Tripod").GetPrim())
                for name, rod in rig["tripod_rods"].items():
                    cylinder(str(tripod.GetPath())+"/"+name, rod["center_m"],
                             rod["radius_m"], rod["length_m"], [.32,.34,.36],
                             rod["orientation_wxyz"])
                for i, foot in enumerate(rig["tripod_feet_world_m"]):
                    cylinder(path+"/Tripod/Foot_"+str(i+1), foot, .028, .025, [.10,.11,.12])
                cube(path+"/Tripod/Head", rig["mount_position_world_m"], [.055,.055,.03], [.10,.11,.12])
                station.SetCustomDataByKey("mro:rigSpecJson", json.dumps(rig, sort_keys=True, allow_nan=False))

        if camera_snapshot() != cameras_before:
            raise RuntimeError("Camera optical or transform properties changed during proxy setup")
        verified = {}
        for cid, rig in plan["rigs"].items():
            prim = stage.GetPrimAtPath(rig["rig_prim_path"]+"/OpticalFrame")
            rows = matrix_rows(prim)
            error = assert_pose_alignment(rows, rig["optical_position_world_m"], rig["optical_orientation_wxyz"])
            source_rows = cameras_before[cid]["world_matrix"]
            error_source = max(abs(rows[r][c]-source_rows[r][c]) for r in range(4) for c in range(4))
            if error_source > 1e-7:
                raise RuntimeError("Rig optical frame is not coincident with its camera")
            station_visibility = str(UsdGeom.Imageable(
                stage.GetPrimAtPath(rig["rig_prim_path"])).ComputeVisibility())
            if station_visibility != str(UsdGeom.Tokens.inherited):
                raise RuntimeError("Selected camera rig has inherited invisible state")
            verified[cid] = {"world_from_optical_frame_row_matrix": rows,
                             "max_error_vs_plan": error,
                             "max_error_vs_camera": error_source,
                             "visibility": station_visibility}
        shape_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROOT_PATH)):
            validate_existing_owner(prim.GetCustomDataByKey("mro:owner"),
                [s.layer.identifier for s in prim.GetPrimStack()], session.identifier)
            if any("Physics" in str(schema) or "Physx" in str(schema)
                   for schema in prim.GetAppliedSchemas()):
                raise RuntimeError("Camera proxy unexpectedly has a physics schema")
            if prim.IsA(UsdGeom.Gprim):
                shape_count += 1
        result = copy.deepcopy(plan)
        result.update(status="applied_session_layer_only", session_layer_identifier=session.identifier,
                      cameras_unchanged_verified=True, optical_frames_verified=True,
                      noncolliding_geometry_verified=True, usd_readback=verified,
                      hidden_unselected_owned_rig_ids=sorted(hidden),
                      total_proxy_gprim_count_including_hidden=shape_count,
                      camera_properties_sha256=hashlib.sha256(json.dumps(
                          cameras_before, sort_keys=True, allow_nan=False).encode()).hexdigest())
        return result
    except BaseException:
        # Restore ONLY our visual subtree, never the entire live session layer.
        with Usd.EditContext(stage, session):
            stage.RemovePrim(ROOT_PATH)
            if backup is not None and not Sdf.CopySpec(backup, ROOT_PATH, session, ROOT_PATH):
                raise RuntimeError("Owned camera visual subtree rollback failed")
        raise
