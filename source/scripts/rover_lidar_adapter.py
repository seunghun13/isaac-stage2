"""Owned native RTX rotary LiDAR approximating VLP-16 coverage on Husky A200.

No vendor VLP-16 USD is assumed: create one OmniLidar in the current session
layer using the installed 5.0 Core schema. This is an explicitly approximate
16-beam profile, not calibrated Velodyne firing/packet emulation. No World,
physics step, ROS node, mass, collision, or global RTX setting is owned here.

The reviewed drone module supplies ONLY pure bounded serialization helpers.
Native GMO get_data is never called here. An optional already-owned CPU packet
can be parsed separately; its timestamp remains in the native clock domain.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

from drone_lidar_adapter import confined_path, parse_gmo_packet, summarize_payload

PROFILE = 'MRO_VLP16_SPEC_APPROX_16ch_10hz_1800cols_v1'
DIRECT = 'IsaacExtractRTXSensorPointCloudNoAccumulator'
MOUNT_TRANSLATION_M = (0.0, 0.0, 0.60)
OWNER_KEY = 'mro:rover_lidar_owner'
MAX_SAMPLE_BYTES = 32 * 1024 * 1024
MAX_TOTAL_SAVED_BYTES = 128 * 1024 * 1024


def profile_attributes():
    """Fresh mutable values; explicit types match installed NVIDIA 5.0 USDA.

    Column rate 18000 / rotation 10 = 1800 columns/rev (0.2 degrees).
    Beams in a column fire together in this approximation. Do not interpret
    these zeros as real VLP-16 factory firing offsets or external time sync.
    """
    core = 'omni:sensor:Core:'
    values = {
        'omni:sensor:modelName': ('String', PROFILE),
        'omni:sensor:tickRate': ('Float', 10.0),
        core+'scanType': ('Token', 'ROTARY'),
        core+'rotationDirection': ('Token', 'CW'),
        core+'rayType': ('Token', 'IDEALIZED'),
        core+'nearRangeM': ('Float', 0.3),
        core+'farRangeM': ('Float', 100.0),
        core+'rangeResolutionM': ('Float', 0.002),
        core+'rangeAccuracyM': ('Float', 0.0),
        core+'azimuthErrorMean': ('Float', 0.0),
        core+'azimuthErrorStd': ('Float', 0.0),
        core+'elevationErrorMean': ('Float', 0.0),
        core+'elevationErrorStd': ('Float', 0.0),
        core+'maxReturns': ('UInt', 1),
        core+'scanRateBaseHz': ('UInt', 10),
        core+'reportRateBaseHz': ('UInt', 18000),
        core+'numberOfEmitters': ('UInt', 16),
        core+'numberOfChannels': ('UInt', 16),
        core+'skipDroppingInvalidPoints': ('Bool', True),
        core+'intensityProcessing': ('Token', 'NORMALIZATION'),
        core+'intensityMappingType': ('Token', 'LINEAR'),
    }
    emitter = core+'emitterState:s001:'
    values[emitter+'elevationDeg'] = ('FloatArray', [float(-15+2*i) for i in range(16)])
    # The installed native Core requires IDs in 1..numberOfChannels; this is
    # also the convention in the reviewed NVIDIA OS1 USDA (not zero-based).
    values[emitter+'channelId'] = ('UIntArray', list(range(1, 17)))
    values[emitter+'fireTimeNs'] = ('UIntArray', [0]*16)
    for name in ('azimuthDeg', 'distanceCorrectionM', 'focalDistM', 'focalSlope',
                 'horOffsetM', 'reportRateDiv', 'vertOffsetM'):
        values[emitter+name] = ('FloatArray', [0.0]*16)
    return values


def _json_readback(value):
    if isinstance(value, (str, bool, int, float)) or value is None:
        return value
    return [_json_readback(v) for v in value]


def _profile_matches(actual, expected):
    # Sdf Float readback is binary32. Tolerate only normal binary32 rounding;
    # this is configuration readback, not a sensor/GT temporal tolerance.
    if isinstance(expected, float):
        import struct
        return actual == struct.unpack('<f', struct.pack('<f', expected))[0]
    if isinstance(expected, list):
        return (isinstance(actual, list) and len(actual) == len(expected)
                and all(_profile_matches(a, b) for a, b in zip(actual, expected)))
    return type(actual) is type(expected) and actual == expected


def summarize_direct_payload(direct, reference, *, include_arrays=False, max_points=65536):
    """Pure observation summary; never manufactures points or clock equality."""
    import numpy as np
    if not isinstance(direct, dict):
        raise TypeError('Native direct pointcloud output must be a dict')
    report, arrays = summarize_payload({DIRECT: direct, 'ReferenceTime': reference},
                                      include_arrays=include_arrays, max_points=max_points)
    points = direct.get('data')
    valid = np.zeros(0, dtype=bool)
    if points is not None:
        # DIRECT may expose shape (0,) before its first render. Preserve that
        # exact empty native array and wait for returns; never reshape a
        # malformed nonempty payload to make it appear to be an XYZ cloud.
        if (not isinstance(points, np.ndarray) or
                (points.size != 0 and (points.dtype.kind != 'f'
                 or points.ndim != 2 or points.shape[1] != 3))):
            raise ValueError('Native point data must be empty or a floating N by 3 ndarray; '
                             'type=%s dtype=%s shape=%s' % (
                                 type(points).__name__, getattr(points, 'dtype', None),
                                 getattr(points, 'shape', None)))
        if points.size:
            valid = np.isfinite(points).all(axis=1) & np.any(points != 0, axis=1)
    report.update(point_count=0 if points is None or points.size == 0 else int(points.shape[0]),
                  finite_nonzero_points=int(np.count_nonzero(valid)),
                  point_return_observed=bool(np.any(valid)),
                  raw_native_timestamps_preserved=True,
                  world_clock_mapping_verified=False,
                  per_point_time_mapping_verified=False,
                  full_scan_completeness_verified=False,
                  point_frame='native DIRECT annotator frame; native transform metadata preserved',
                  point_frame_is_body_frame=False,
                  native_gmo_read_performed=False)
    if isinstance(reference, dict) and 'referenceTimeNumerator' in reference:
        numerator = int(reference['referenceTimeNumerator'])
        denominator = int(reference.get('referenceTimeDenominator', 0))
        if denominator <= 0:
            raise ValueError('Native ReferenceTime denominator must be positive')
        report['native_render_reference'] = [numerator, denominator]
    return report, arrays


class RoverLidarAdapter:
    """Create before first PLAY; read only this sensor; caller owns motion.

    create() -> initialize() -> sample(). set_enabled(False) gates only this
    RenderProduct while paused, keeping the installed visual mount. close()
    releases only tracked references and an exact owner-marked mount.
    """
    def __init__(self, stage, body_path, output_root, *, mount_translation_m=MOUNT_TRANSLATION_M):
        self.stage = stage
        self.body_path = str(body_path).rstrip('/')
        if not self.body_path.startswith('/') or not self.body_path or '..' in self.body_path.split('/'):
            raise ValueError('An absolute audited rigid-body path is required')
        if tuple(mount_translation_m) != MOUNT_TRANSLATION_M:
            raise ValueError('Only reviewed provisional body-relative mount (0,0,0.60)m is supported')
        self.output_root = confined_path(output_root)
        self.mount_path = self.body_path + '/MroRoverLidarMount'
        self.sensor_path = self.mount_path + '/Sensor'
        self.owner_token = uuid.uuid4().hex
        self.sensor = None
        self.annotators = {}
        self.created = self.closed = False
        self.enabled = True
        self.counter = self.saved_bytes = 0
        self._core = None
        self.creation_report = None

    def _assert_owner(self):
        prim = self.stage.GetPrimAtPath(self.mount_path)
        if not prim.IsValid() or prim.GetCustomDataByKey(OWNER_KEY) != self.owner_token:
            raise RuntimeError('Rover LiDAR mount ownership changed; refusing mutation/read')
        return prim

    def create(self):
        if self.created or self.closed:
            raise RuntimeError('Rover LiDAR creation is one-shot')
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        from isaacsim.sensors.rtx import LidarRtx
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing() or abs(float(timeline.get_current_time())) > 1e-12:
            raise RuntimeError('Fresh context before first PLAY is required')
        if omni.usd.get_context().get_stage() != self.stage:
            raise RuntimeError('Supplied stage is not the current Kit stage')
        body = self.stage.GetPrimAtPath(self.body_path)
        if not body.IsValid() or not body.IsActive() or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('An active, audited native rigid body is required')
        if self.stage.GetPrimAtPath(self.mount_path).IsValid():
            raise RuntimeError('Existing mount must not be overwritten')
        if abs(float(UsdGeom.GetStageMetersPerUnit(self.stage))-1.0) > 1e-12:
            raise RuntimeError('Mount coordinates require a metre stage')
        self._core = _isaacsim_core_nodes.acquire_interface()
        try:
            with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                mount = UsdGeom.Xform.Define(self.stage, self.mount_path)
                mount.AddTranslateOp().Set(Gf.Vec3d(*MOUNT_TRANSLATION_M))
                mount.GetPrim().SetCustomDataByKey(OWNER_KEY, self.owner_token)
                self.created = True
                native = self.stage.DefinePrim(self.sensor_path, 'OmniLidar')
                native.SetMetadata('apiSchemas', Sdf.TokenListOp.Create(prependedItems=['OmniSensorGenericLidarCoreAPI']))
                if not native.HasAPI('OmniSensorGenericLidarCoreAPI'):
                    raise RuntimeError('Installed native LiDAR Core schema is unavailable')
                readback = {}
                for name, (type_name, value) in profile_attributes().items():
                    attr = native.CreateAttribute(name, getattr(Sdf.ValueTypeNames, type_name), custom=False)
                    if not attr.Set(value):
                        raise RuntimeError('Native attribute authoring failed: '+name)
                    actual = _json_readback(attr.Get())
                    if not _profile_matches(actual, value):
                        raise RuntimeError('Native profile readback differs: '+name)
                    readback[name] = actual
                native.SetCustomDataByKey('mro:profile_fidelity', 'VLP16-spec approximation; not factory calibration')
                # LidarRtx wraps this existing native prim and creates its RP;
                # it never enters the default remote OS1 asset creation branch.
                self.sensor = LidarRtx(prim_path=self.sensor_path,
                    name='mro_rover_vlp16_'+self.owner_token[:8],
                    translation=np.zeros(3), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
                rp_path = str(self.sensor.get_render_product_path())
                for name in (DIRECT, 'ReferenceTime'):
                    anno = rep.AnnotatorRegistry.get_annotator(name)
                    self.annotators[name] = anno
                    anno.attach([rp_path])
                # Provisional arch: 0.300m above observed body top ~0.2514m.
                # Proxy is visual only; no automatic empty-link 1kg or collider.
                def cube(name, dimensions, position):
                    obj = UsdGeom.Cube.Define(self.stage, self.mount_path+'/'+name)
                    obj.CreateSizeAttr(1.0)
                    obj.AddTranslateOp().Set(Gf.Vec3d(*position))
                    obj.AddScaleOp().Set(Gf.Vec3f(*dimensions))
                    obj.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.12, 0.14)])
                    obj.GetPrim().SetCustomDataByKey('mro:visual_proxy_only', True)
                for sign, name in ((-1, 'ArchLeft'), (1, 'ArchRight')):
                    cube(name, (0.03, 0.03, 0.30), (0.0, sign*0.20, -0.1986))
                cube('ArchTop', (0.12, 0.43, 0.015), (0.0, 0.0, -0.0486))
                # A small pedestal closes the visual gap to housing bottom.
                cube('Pedestal', (0.065, 0.065, 0.01275), (0.0, 0.0, -0.037725))
                housing = UsdGeom.Cylinder.Define(self.stage, self.mount_path+'/HousingProxy')
                housing.CreateRadiusAttr(0.05165)
                housing.CreateHeightAttr(0.0717)
                housing.CreateAxisAttr('Z')
                housing.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.0045))
                housing.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.24, 0.25)])
                housing.GetPrim().SetCustomDataByKey('mro:visual_proxy_only', True)
                physical = [str(p.GetPath()) for p in Usd.PrimRange(mount.GetPrim()) if
                    p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.MassAPI)
                    or p.HasAPI(UsdPhysics.CollisionAPI)]
                if physical:
                    raise RuntimeError('Sensor descendants unexpectedly own physics: '+repr(physical))
                self.creation_report = dict(schema='mro_rover_lidar_creation_v1',
                    owner_token=self.owner_token, body_path=self.body_path, mount_path=self.mount_path,
                    sensor_path=self.sensor_path, render_product_path=rp_path,
                    profile=PROFILE, native_sensor_type='OmniLidar', profile_readback=readback,
                    native_attribute_profile_sha256=hashlib.sha256(json.dumps(readback, sort_keys=True,
                        separators=(',', ':'), allow_nan=False).encode()).hexdigest(),
                    external_asset_dependencies=[], vendor_vlp16_asset_used=False,
                    profile_is_factory_calibrated=False, firing_order_is_factory_pattern=False,
                    simultaneous_beams_per_column=True, nominal_rays_per_second=288000,
                    nominal_scan_rate_hz=10, horizontal_fov_deg=360, vertical_fov_deg=[-15, 15],
                    mount_translation_base_m=list(MOUNT_TRANSLATION_M),
                    mount_quaternion_base_wxyz=[1, 0, 0, 0],
                    mount_basis='Husky base_link, provisional above 0.300m visual arch',
                    visual_proxy_only=True, added_physics_or_mass=False, tracking_center_changed=False,
                    lidar_payload_mass_modeled=False, enabled=True, annotators=list(self.annotators),
                    native_gmo_reader_attached=False, world_clock_mapping_verified=False)
            return dict(self.creation_report)
        except BaseException:
            self.close()
            raise

    async def initialize(self):
        if not self.created or self.closed or self.sensor is None:
            raise RuntimeError('Create the rover LiDAR first')
        self._assert_owner()
        return dict(initialized=True, simulation_advanced=False,
                    method='direct owned annotators; no World reset/callback installation')

    def set_enabled(self, enabled):
        if type(enabled) is not bool:
            raise TypeError('enabled must be Boolean')
        if not self.created or self.closed or self.sensor is None:
            raise RuntimeError('Rover LiDAR is inactive')
        import omni.timeline
        if omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError('Pause physics before changing rover sensor enable state')
        self._assert_owner()
        texture = self.sensor._render_product.hydra_texture
        before = bool(texture.get_updates_enabled())
        texture.set_updates_enabled(enabled)
        actual = bool(texture.get_updates_enabled())
        if actual is not enabled:
            texture.set_updates_enabled(before)
            raise RuntimeError('Owned RenderProduct enable readback failed')
        self.enabled = enabled
        return dict(enabled=enabled, prior_render_updates=before, render_updates_enabled=actual,
                    only_owned_render_product_changed=True, physics_reset=False)

    def sample(self, *, include_arrays=False, save=False, max_points=65536):
        if not self.created or self.closed or self.sensor is None:
            raise RuntimeError('Rover LiDAR is inactive')
        self._assert_owner()
        if not self.enabled:
            return dict(enabled=False, available=False, measurement=None, reason='rover LiDAR disabled',
                        native_gmo_read_performed=False, world_clock_mapping_verified=False)
        import numpy as np
        import omni.timeline
        from pxr import Usd, UsdGeom
        self.counter += 1
        start = time.monotonic_ns()
        direct = self.annotators[DIRECT].get_data()
        reference = self.annotators['ReferenceTime'].get_data()
        result, arrays = summarize_direct_payload(direct, reference,
            include_arrays=include_arrays, max_points=max_points)
        result.update(schema='mro_rover_lidar_sample_v1', enabled=True,
            sample_index=self.counter, sensor_path=self.sensor_path, profile=PROFILE,
            read_start_monotonic_ns=start, read_end_monotonic_ns=time.monotonic_ns(),
            timeline_s_at_read=float(omni.timeline.get_timeline_interface().get_current_time()),
            sample_semantics='latest native direct return; not a proven whole revolution')
        if 'native_render_reference' in result:
            event_s = float(self._core.get_sim_time_at_time(tuple(result['native_render_reference'])))
            result['native_render_event_sim_time_s'] = event_s if math.isfinite(event_s) else None
            result['render_reference_lookup_is_per_point_time_mapping'] = False
        matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
            self.stage.GetPrimAtPath(self.sensor_path))
        result['sensor_usd_world_matrix_at_read'] = [[float(matrix[i][j]) for j in range(4)] for i in range(4)]
        result['usd_world_matrix_is_scan_timestamp_pose'] = False
        if save:
            if self.saved_bytes + result['raw_bytes'] + 1024*1024 > MAX_TOTAL_SAVED_BYTES:
                raise RuntimeError('Bounded rover LiDAR archive budget exhausted')
            out = confined_path(self.output_root)
            out.mkdir(parents=True, exist_ok=True)
            out = confined_path(out, must_exist=True)
            dest = confined_path(out/('rover_lidar_%06d_%s.npz' % (self.counter, uuid.uuid4().hex[:12])))
            with dest.open('xb') as stream:
                np.savez(stream, **arrays)
                stream.flush()
                os.fsync(stream.fileno())
            size = dest.stat().st_size
            self.saved_bytes += size
            if size > MAX_SAMPLE_BYTES + 1024*1024:
                raise RuntimeError('Archive exceeds bound; evidence preserved')
            result['saved_native_arrays'] = dict(path=str(dest), bytes=size,
                sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),
                total_adapter_saved_bytes=self.saved_bytes,
                format='uncompressed NPZ, exact native arrays and dtypes')
        json.dumps(result, allow_nan=False)
        return result

    @staticmethod
    def parse_owned_cpu_packet(raw, *, max_points=65536):
        """Only parse bytes caller already safely owns; never fetch native GMO."""
        return parse_gmo_packet(raw, max_points=max_points)

    def close(self):
        if self.closed:
            return dict(closed=True, already_closed=True)
        errors = []
        for name, anno in list(self.annotators.items()):
            try:
                anno.detach()
                del self.annotators[name]
            except Exception as exc:
                errors.append(name+': '+repr(exc))
        if self.sensor is not None:
            try:
                self.sensor.detach_all_writers()
                self.sensor.detach_all_annotators()
                if self.sensor._render_product is not None:
                    self.sensor._render_product.destroy()
                    self.sensor._render_product = None
                    self.sensor._render_product_path = None
                self.sensor = None
            except Exception as exc:
                errors.append('owned render product: '+repr(exc))
        if self.created:
            from pxr import Usd
            prim = self.stage.GetPrimAtPath(self.mount_path)
            if prim.IsValid() and prim.GetCustomDataByKey(OWNER_KEY) == self.owner_token:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    if not self.stage.RemovePrim(self.mount_path):
                        errors.append('Owned mount removal returned false')
            elif prim.IsValid():
                errors.append('Owner changed; foreign mount preserved')
        self.closed = not errors
        if self.closed:
            self.enabled = False
        return dict(closed=self.closed, errors=errors, physics_reset=False)
