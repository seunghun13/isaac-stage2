"""Owned, body-mounted Isaac Sim 5.0 RTX LiDAR (no World or ROS ownership).

Only imports Isaac APIs when create() is called inside the reviewed Kit process.
The OS1 USD must already be staged locally with its dependencies. There is no
implicit network download, JSON-to-generic-sensor fallback, play, reset or step.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid


PROJECT_ROOT = Path('/mnt/DATA/workspace/ws_minho/mro_1')
PROFILE = 'OS1_REV6_32ch10hz512res'
DEFAULT_ASSET = PROJECT_ROOT / 'runtime/assets/sensors/Ouster/OS1/OS1.usd'
MOUNT_TRANSLATION_M = (0.0, 0.0, 0.107)
MAX_SAMPLE_BYTES = 32 * 1024 * 1024
MAX_TOTAL_SAVED_BYTES = 256 * 1024 * 1024
MAX_ARRAY_ELEMENTS = 1_000_000
DIRECT = 'IsaacExtractRTXSensorPointCloudNoAccumulator'
FULL_SCAN = 'IsaacCreateRTXLidarScanBuffer'
GMO = 'GenericModelOutput'


def confined_path(value, *, must_exist=False):
    """Resolve symlinks before any write; project root itself is not a target."""
    root = PROJECT_ROOT.resolve(strict=True)
    target = Path(value).resolve(strict=must_exist)
    if target == root or not target.is_relative_to(root):
        raise ValueError('LiDAR path must resolve strictly inside mro_1')
    return target


def _plain_scalar(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        return {'nonfinite': repr(value)}
    if hasattr(value, 'item'):
        return _plain_scalar(value.item())
    raise TypeError('Unsupported metadata type: ' + type(value).__name__)


def parse_gmo_packet(raw, *, max_points=65536):
    """Pure bounded parser for the measured SDK 5.0 GMO v1/NONE CPU layout.

    No native decoder, ctypes, pointer dereference, optional-property getter or
    GPU copy occurs here. Header offsets and array order are pinned to the
    captured SDK1.0.0 packet with NONE auxiliary data. Structural invariants
    refuse unsupported layouts. Stored pointer VALUES are only compared for
    contiguous-array strides; they are never addresses passed to any API.
    """
    import struct
    import numpy as np
    if not isinstance(raw, np.ndarray) or raw.dtype != np.dtype('uint8'):
        raise TypeError('GenericModelOutput must provide a CPU uint8 numpy buffer')
    if raw.size == 0:
        return {'available': False, 'num_elements': 0, 'reason': 'empty native GMO buffer'}
    if not raw.flags.c_contiguous or raw.nbytes < 264 or raw.nbytes > MAX_SAMPLE_BYTES:
        raise ValueError('GMO packet is noncontiguous, undersized or over byte bound')
    flat = raw.reshape(-1)
    header = flat[:264].tobytes()
    u32 = lambda offset: struct.unpack_from('<I', header, offset)[0]
    u64 = lambda offset: struct.unpack_from('<Q', header, offset)[0]
    if u32(0) != 0x4E474D4F or [u32(i) for i in (4, 8, 12)] != [1, 0, 0]:
        raise ValueError('Unsupported GMO magic/version; parser only supports measured 1.0.0')
    size, count = u64(16), u32(24)
    if not 0 <= count <= max_points or not 264 <= size <= flat.nbytes:
        raise ValueError('GMO declared size/count exceeds bounded available buffer')
    if u32(200) != 0 or u32(204) != 1 or u64(256) != 0:
        raise ValueError('Only measured NONE auxiliary / LIDAR layout is supported')
    # The measured packet has the combined frame/motion word zero. Do not guess
    # the ABI of subfields when a different reference/motion mode is requested.
    if u32(28) != 0 or u32(48) not in (0, 1) or u32(52) != 0:
        raise ValueError('Unsupported frame/motion/coordinate/output layout')
    minimum = 264 + count * 21
    aligned = (minimum + 7) // 8 * 8
    if size not in (minimum, aligned):
        raise ValueError('GMO NONE packed byte count differs from header + 21 bytes per point')
    pointers = [u64(i) for i in range(208, 256, 8)]
    if count and (pointers[0] == 0 or any(pointers[i] - pointers[0] != i * count * 4 for i in range(6))):
        raise ValueError('GMO stored pointer strides do not match serialized array order')
    packet = flat[:size].copy()
    if size > minimum and np.any(packet[minimum:] != 0):
        raise ValueError('Unexpected nonzero NONE packet alignment padding')
    def vector(index, dtype):
        return np.frombuffer(packet, dtype=np.dtype(dtype), count=count,
                             offset=264 + index * count * 4).copy()
    offsets = vector(0, '<i4')
    x, y, z, scalar = (vector(i, '<f4') for i in range(1, 5))
    flags = vector(5, 'u1')
    base_ns = u64(40)
    if count and (base_ns + int(offsets.min()) < 0 or base_ns + int(offsets.max()) >= 2 ** 64):
        raise ValueError('Native GMO timestamp plus signed offset exceeds uint64')
    times = np.fromiter((base_ns + int(v) for v in offsets), dtype=np.uint64, count=count)
    # Numeric values are from the class-only installed SDK inspection sdk_02:
    # CoordsType.CARTESIAN=0, SPHERICAL=1; ElementFlags.VALID=64.
    coords = 'CARTESIAN' if u32(48) == 0 else 'SPHERICAL'
    if coords == 'SPHERICAL':
        az, el = np.deg2rad(x.astype(np.float64)), np.deg2rad(y.astype(np.float64))
        xyz = np.column_stack((z * np.cos(el) * np.cos(az),
                               z * np.cos(el) * np.sin(az), z * np.sin(el))).astype(np.float32)
        derivation = 'spherical azimuth/elevation degrees + range metres; native frame'
    else:
        xyz = np.column_stack((x, y, z))
        derivation = 'native Cartesian coordinates copied'
    keep = (flags & 64) == 64
    def frame_at(offset):
        return {'timestamp_ns': u64(offset),
                'orientation_xyzw': np.frombuffer(header, dtype='<f4', count=4, offset=offset + 8).copy(),
                'position_m': np.frombuffer(header, dtype='<f4', count=3, offset=offset + 24).copy()}
    return {
        'available': True, 'num_elements': count, 'packet_timestamp_ns': base_ns,
        'frame_id': u64(32), 'coords_type': coords,
        'frame_motion_raw_word': u32(28), 'frame_motion_profile': 'measured sensor-frame word zero; no alternate ABI inferred',
        'packet_bytes': size, 'available_buffer_bytes': int(flat.nbytes),
        'packet_sha256': hashlib.sha256(packet.tobytes()).hexdigest(),
        'raw_packet_u8': packet.reshape(1, -1), 'gmo_version_from_header': [1, 0, 0],
        'serialized_header_bytes': 264, 'parser': 'bounded contiguous bytes; no native decoder or pointer dereference',
        'aux_type': 'NONE', 'auxiliary_property_access': 'none',
        'model_to_app_transform': np.frombuffer(header, dtype='<f4', count=16, offset=56).copy().reshape(4, 4),
        'frameStart': frame_at(120), 'frameEnd': frame_at(160),
        'native': {'x': x, 'y': y, 'z': z, 'scalar': scalar, 'flags': flags,
                   'timeOffsetNs': offsets, 'point_timestamp_ns': times},
        'point_timestamp_rule': 'native uint64 packet.timestampNs + native signed int32 point.timeOffsetNs',
        'world_epoch_mapping_verified': False,
        'time_offset_min_ns': int(offsets.min()) if count else None,
        'time_offset_max_ns': int(offsets.max()) if count else None,
        'point_timestamp_min_ns': int(times.min()) if count else None,
        'point_timestamp_max_ns': int(times.max()) if count else None,
        'valid_flag_value': 64, 'valid_element_count': int(np.count_nonzero(keep)),
        'cartesian_derivation': derivation, 'cartesian_all_native_frame_m': xyz,
        'valid_indices': np.flatnonzero(keep).astype(np.int64), 'data': xyz[keep].copy(),
        'point_timestamp_ns_valid': times[keep].copy(), 'scalar_valid': scalar[keep].copy(),
    }


def summarize_payload(value, *, include_arrays=False, max_points=65536):
    """Copy native arrays, retain integer timestamps, and enforce finite bounds.

    Returned arrays are never truncated or converted to floating timestamps.
    JSON preview values encode NaN/Inf with explicit markers; saved NPZ retains
    exact native dtypes and bit patterns. No object/pickle arrays are allowed.
    """
    import numpy as np
    if not 1 <= max_points <= 65536:
        raise ValueError('max_points must be between 1 and 65536')
    arrays = {}
    total_bytes = 0
    fields = 0

    def visit(item, path, depth=0):
        nonlocal total_bytes, fields
        fields += 1
        if fields > 256 or depth > 10:
            raise ValueError('LiDAR payload nesting/field bound exceeded')
        if isinstance(item, np.ndarray):
            if item.dtype.hasobject or item.dtype.kind not in 'biuf':
                raise TypeError('Only numeric native arrays may be recorded')
            limit = MAX_SAMPLE_BYTES if path == '/' + GMO + '/raw_packet_u8' else MAX_ARRAY_ELEMENTS
            if item.size > limit:
                raise ValueError('LiDAR array element bound exceeded')
            if item.ndim >= 1 and item.shape[0] > max_points:
                raise ValueError('LiDAR array row bound exceeded; no silent truncation')
            total_bytes += int(item.nbytes)
            if total_bytes > MAX_SAMPLE_BYTES:
                raise ValueError('LiDAR native payload byte bound exceeded')
            native = np.array(item, copy=True, order='C')
            key = 'array_%03d' % len(arrays)
            arrays[key] = native
            report = {
                'array_key': key, 'native_field_path': path,
                'dtype': str(native.dtype), 'shape': list(native.shape),
                'nbytes': int(native.nbytes),
                'sha256': hashlib.sha256(native.tobytes(order='C')).hexdigest(),
            }
            if native.dtype.kind in 'f' and native.size:
                report['nonfinite_elements'] = int(np.count_nonzero(~np.isfinite(native)))
            if 'timestamp' in path.lower():
                report['native_unit_from_node_documentation'] = 'ns'
                report['integer_timestamp_dtype'] = native.dtype.kind in 'iu'
                if native.size and native.dtype.kind in 'iu':
                    report['native_min_ns'] = int(native.min())
                    report['native_max_ns'] = int(native.max())
                    report['native_span_ns'] = int(native.max()) - int(native.min())
                    report['native_zero_count'] = int(np.count_nonzero(native == 0))
                report['epoch_mapping_to_world'] = 'unverified; raw timestamps preserved'
                if path.startswith('/' + FULL_SCAN + '/'):
                    report['usable_per_point_timestamp'] = False
                    report['sdk_5_0_timestamp_layout_issue'] = (
                        'C++ supplies int32 timeOffsetNs; OGN exposes uint64. '
                        'The accumulated buffer omits source packet base times; '
                        'do not interpret this exposed array as absolute point times.')
            if native.ndim == 2 and native.shape[-1] == 3:
                points = native.astype(np.float64, copy=False)
                valid = np.isfinite(points).all(axis=1)
                nonzero = np.linalg.norm(np.where(np.isfinite(points), points, 0.0), axis=1) > 0
                report['finite_nonzero_rows'] = int(np.count_nonzero(valid & nonzero))
                if np.any(valid & nonzero):
                    lengths = np.linalg.norm(points[valid & nonzero], axis=1)
                    report['coordinate_norm_min_m'] = float(lengths.min())
                    report['coordinate_norm_max_m'] = float(lengths.max())
            if include_arrays:
                def json_array(v):
                    if isinstance(v, list):
                        return [json_array(x) for x in v]
                    return _plain_scalar(v)
                report['values'] = json_array(native.tolist())
            return report
        if isinstance(item, dict):
            if len(item) > 128:
                raise ValueError('LiDAR dictionary field bound exceeded')
            return {str(k): visit(v, path + '/' + str(k), depth + 1) for k, v in item.items()}
        if isinstance(item, (list, tuple)):
            if len(item) > 256:
                raise ValueError('Unexpected long native sequence; expected ndarray')
            return [visit(v, path + '/' + str(i), depth + 1) for i, v in enumerate(item)]
        return _plain_scalar(item)

    report = visit(value, '')
    return {'native': report, 'array_count': len(arrays), 'raw_bytes': total_bytes}, arrays


class DroneLidarAdapter:
    """One body-fixed OS1 with independently owned direct/scan-buffer readers.

    Example (inside Kit; root prepares physics separately)::

        lidar = DroneLidarAdapter(stage, '/MRO/drone_1/base_link', output_dir,
            sensor_asset_path='/mnt/DATA/workspace/ws_minho/mro_1/runtime/assets/sensors/Ouster/OS1/OS1.usd')
        lidar.create()                 # before first PLAY; no simulation step
        await lidar.initialize()       # attachment validation; no World reset
        # root starts continuous PLAY and advances/render steps
        result = lidar.sample(save=True)
        lidar.close()                  # before root destroys its World

    The example body path is illustrative: pass the actual audited rigid body.
    """

    def __init__(self, stage, body_path, output_root, *, sensor_asset_path=None,
                 enable_native_packet_parser=False):
        self.stage = stage
        self.body_path = str(body_path).rstrip('/')
        if not self.body_path.startswith('/') or '..' in self.body_path.split('/'):
            raise ValueError('An absolute USD rigid-body prim path is required')
        self.output_root = confined_path(output_root)
        self.asset_path = confined_path(sensor_asset_path or DEFAULT_ASSET)
        if type(enable_native_packet_parser) is not bool:
            raise TypeError('enable_native_packet_parser must be Boolean')
        self.enable_native_packet_parser = enable_native_packet_parser
        self.mount_path = self.body_path + '/MroLidarMount'
        self.model_path = self.mount_path + '/SensorModel'
        self.sensor_path = None
        self.sensor = None
        self.annotators = {}
        self.created = False
        self.closed = False
        self.counter = 0
        self.saved_bytes = 0
        self.owner_token = uuid.uuid4().hex
        self._previous_scan_digest = None
        self._rep = None
        self._core = None
        self.creation_report = None

    def create(self):
        if self.created or self.closed:
            raise RuntimeError('LiDAR adapter creation is one-shot')
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        from isaacsim.sensors.rtx import LidarRtx
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
        timeline = omni.timeline.get_timeline_interface()
        if timeline.is_playing() or abs(float(timeline.get_current_time())) > 1e-12:
            raise RuntimeError('Create LiDAR before the first PLAY in a fresh context')
        if omni.usd.get_context().get_stage() != self.stage:
            raise RuntimeError('LidarRtx uses the current USD stage; supplied stage differs')
        body = self.stage.GetPrimAtPath(self.body_path)
        if not body.IsValid() or not body.IsActive():
            raise ValueError('LiDAR parent body is absent/inactive')
        if self.stage.GetPrimAtPath(self.mount_path).IsValid():
            raise RuntimeError('Refusing to reuse or overwrite an existing LiDAR mount')
        if abs(UsdGeom.GetStageMetersPerUnit(self.stage) - 1.0) > 1e-12:
            raise RuntimeError('LiDAR mounting coordinates require a metre stage')
        asset = confined_path(self.asset_path, must_exist=True)
        if not asset.is_file() or asset.suffix.lower() not in ('.usd', '.usda', '.usdc'):
            raise ValueError('An already staged local Ouster USD is required')
        self._rep = rep
        self._core = _isaacsim_core_nodes.acquire_interface()
        session = self.stage.GetSessionLayer()
        try:
            with Usd.EditContext(self.stage, session):
                mount = UsdGeom.Xform.Define(self.stage, self.mount_path)
                mount.AddTranslateOp().Set(Gf.Vec3d(*MOUNT_TRANSLATION_M))
                mount.GetPrim().SetCustomDataByKey('mro:lidar_owner', self.owner_token)
                self.created = True
                model = UsdGeom.Xform.Define(self.stage, self.model_path).GetPrim()
                # Author selection before composition so the asset's default
                # 128-channel payload is never requested as an intermediate.
                model.GetVariantSets().AddVariantSet('sensor').SetVariantSelection(PROFILE)
                if not model.GetReferences().AddReference(str(asset)):
                    raise RuntimeError('Local Ouster asset reference failed')
                variants = model.GetVariantSets().GetVariantSet('sensor')
                if PROFILE not in variants.GetVariantNames() or not variants.SetVariantSelection(PROFILE):
                    raise RuntimeError('OS1 asset lacks the exact reviewed sensor variant')
                children = list(Usd.PrimRange(model))
                native = [p for p in children if p.GetTypeName() == 'OmniLidar']
                if len(native) != 1 or not native[0].HasAPI('OmniSensorGenericLidarCoreAPI'):
                    raise RuntimeError('Expected exactly one native configured OmniLidar')
                expected = {'omni:sensor:Core:scanType': 'ROTARY',
                            'omni:sensor:tickRate': 10.0,
                            'omni:sensor:Core:scanRateBaseHz': 10,
                            'omni:sensor:Core:reportRateBaseHz': 5120,
                            'omni:sensor:Core:numberOfChannels': 32}
                profile_readback = {name: native[0].GetAttribute(name).Get() for name in expected}
                if profile_readback != expected:
                    raise RuntimeError('Composed OS1 profile readback differs: ' + repr(profile_readback))
                keep_invalid = native[0].GetAttribute('omni:sensor:Core:skipDroppingInvalidPoints')
                if keep_invalid.IsValid():
                    keep_invalid.Set(True)  # Match installed IsaacSensorCreateRtxLidar.do().
                physics = [str(p.GetPath()) for p in children if
                           p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.MassAPI)]
                if physics:
                    raise RuntimeError('Sensor asset has unreviewed physical links: ' + repr(physics))
                self.sensor_path = str(native[0].GetPath())
                xf = UsdGeom.Xformable(native[0])
                if xf.GetResetXformStack():
                    raise RuntimeError('Sensor resets its transform stack; body mounting would fail')
                transform = Gf.Transform(xf.GetLocalTransformation())
                if max(abs(float(v) - 1.0) for v in transform.GetScale()) > 1e-8:
                    raise RuntimeError('Expected an unscaled native sensor coordinate frame')
                quat = transform.GetRotation().GetQuat()
                q = [quat.GetReal(), *quat.GetImaginary()]
                self.sensor = LidarRtx(
                    prim_path=self.sensor_path, name='mro_drone_os1_' + self.owner_token[:8],
                    translation=np.asarray(transform.GetTranslation(), dtype=float),
                    orientation=np.asarray(q, dtype=float),
                )
                # Wrapping an existing OmniLidar avoids the constructor's remote
                # OS1 download path and preserves its verified native variant.
                rp_path = self.sensor.get_render_product_path()
                for name, params in (
                    (DIRECT, None),
                    (FULL_SCAN, {'outputTimestamp': True, 'outputDistance': True,
                                 'outputIntensity': True, 'outputObjectId': True,
                                 'outputEmitterId': True}),
                    ('ReferenceTime', None),
                    (GMO, None),
                ):
                    if name == GMO:
                        anno = rep.AnnotatorRegistry.get_annotator(name, device='cpu')
                    else:
                        anno = rep.AnnotatorRegistry.get_annotator(name, init_params=params) if params else rep.AnnotatorRegistry.get_annotator(name)
                    self.annotators[name] = anno
                    anno.attach([rp_path])
                renderable = [p for p in children if p.GetTypeName() in ('Mesh', 'Cube', 'Cylinder', 'Sphere')]
                proxy = False
                if not renderable:
                    cylinder = UsdGeom.Cylinder.Define(self.stage, self.mount_path + '/HousingProxy')
                    cylinder.CreateRadiusAttr(0.043)
                    cylinder.CreateHeightAttr(0.073)
                    cylinder.CreateAxisAttr('Z')
                    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.042))
                    cylinder.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])
                    cylinder.GetPrim().SetCustomDataByKey('mro:visual_proxy_only', True)
                    proxy = True
                self.creation_report = {
                    'owner_token': self.owner_token, 'mount_path': self.mount_path,
                    'body_path': self.body_path, 'sensor_path': self.sensor_path,
                    'body_relative_mount_m': list(MOUNT_TRANSLATION_M),
                    'body_relative_mount_quaternion_wxyz': [1, 0, 0, 0],
                    'mount_basis': 'MRS M690 base_link reference, provisional; not TrackingCenter',
                    'sensor_local_translation_in_asset_m': [float(v) for v in transform.GetTranslation()],
                    'sensor_local_quaternion_in_asset_wxyz': [float(v) for v in q],
                    'profile': PROFILE, 'sensor_type': 'OmniLidar',
                    'profile_readback': profile_readback,
                    'skip_dropping_invalid_points': keep_invalid.Get() if keep_invalid.IsValid() else None,
                    'local_asset': str(asset), 'asset_sha256': hashlib.sha256(asset.read_bytes()).hexdigest(),
                    'render_product_path': str(rp_path),
                    'housing_visual_proxy': proxy, 'added_physics_or_mass': False,
                    'tracking_center_changed': False, 'native_point_timestamp_unit': 'ns',
                    'world_clock_mapping_verified': False,
                    'full_scan_timestamp_field_valid': False,
                    'point_time_source': 'per-packet GenericModelOutput timestampNs + signed timeOffsetNs',
                    'native_packet_parser_enabled': self.enable_native_packet_parser,
                    'scan_buffer_pause_resume_known_issue': True,
                    'annotators': list(self.annotators),
                }
            return dict(self.creation_report)
        except BaseException:
            self.close()
            raise

    async def initialize(self):
        # Annotator readers are directly owned here; LidarRtx.initialize() would
        # install callbacks and BaseSensor physics dependencies we do not need.
        if not self.created or self.closed or self.sensor is None:
            raise RuntimeError('Create the LiDAR before initialization')
        return {'initialized': True, 'simulation_advanced': False,
                'method': 'direct native annotator reads; no sensor callback/World reset'}

    def sample(self, *, include_arrays=False, save=False, max_points=65536):
        if not self.created or self.closed or self.sensor is None:
            raise RuntimeError('LiDAR adapter is inactive')
        import numpy as np
        import omni.timeline
        from pxr import Usd, UsdGeom
        self.counter += 1
        read_start_ns = time.monotonic_ns()
        payload = {name: anno.get_data() for name, anno in self.annotators.items()}
        if GMO in payload:
            raw = payload[GMO]
            try:
                if self.enable_native_packet_parser:
                    payload[GMO] = parse_gmo_packet(raw, max_points=max_points)
                else:
                    evidence = {'available': False, 'parser_enabled': False,
                                'reason': 'raw-only capture until packet/header and basic-field decoder review'}
                    if isinstance(raw, np.ndarray) and raw.dtype == np.uint8 and raw.flags.c_contiguous and raw.nbytes <= MAX_SAMPLE_BYTES:
                        packet = raw.reshape(-1).copy()
                        evidence.update(raw_packet_available=True, raw_packet_u8=packet.reshape(1, -1),
                                        packet_bytes=int(packet.nbytes), header_hex=packet[:128].tobytes().hex())
                        if packet.size >= 28:
                            header = packet[:28].tobytes()
                            evidence['header_magic_uint32'] = int.from_bytes(header[:4], 'little')
                            evidence['header_declared_size_uint64'] = int.from_bytes(header[16:24], 'little')
                            evidence['header_declared_elements_uint32'] = int.from_bytes(header[24:28], 'little')
                    else:
                        evidence.update(raw_packet_available=False, output_type=str(type(raw)))
                    payload[GMO] = evidence
            except Exception as exc:
                evidence = {'available': False, 'parse_error': repr(exc),
                            'raw_pointer_address_used_as_parser_input': False}
                if isinstance(raw, np.ndarray) and raw.dtype == np.uint8 and raw.flags.c_contiguous and raw.nbytes <= MAX_SAMPLE_BYTES:
                    evidence.update(raw_packet_u8=raw.reshape(1, -1).copy(),
                                    packet_bytes=int(raw.nbytes),
                                    header_hex=raw.reshape(-1)[:128].tobytes().hex())
                payload[GMO] = evidence
        result, arrays = summarize_payload(payload, include_arrays=include_arrays, max_points=max_points)
        result.update({'sample_index': self.counter, 'sensor_path': self.sensor_path,
                       'profile': PROFILE, 'read_start_monotonic_ns': read_start_ns,
                       'read_end_monotonic_ns': time.monotonic_ns(),
                       'timeline_s_at_read': float(omni.timeline.get_timeline_interface().get_current_time()),
                       'timeline_playing_at_read': bool(omni.timeline.get_timeline_interface().is_playing()),
                       'world_clock_mapping_verified': False,
                       'sample_semantics': 'latest native buffers; a full rotary scan spans acquisition time'})
        result['clouds'] = {
            'direct': self._cloud_report(result['native'].get(DIRECT, {}), DIRECT),
            'full_scan_buffer': self._cloud_report(result['native'].get(FULL_SCAN, {}), FULL_SCAN),
            'native_packet': self._cloud_report(result['native'].get(GMO, {}), GMO),
        }
        ref = payload.get('ReferenceTime', {})
        if isinstance(ref, dict) and 'referenceTimeNumerator' in ref and 'referenceTimeDenominator' in ref:
            numerator, denominator = int(ref['referenceTimeNumerator']), int(ref['referenceTimeDenominator'])
            if denominator > 0:
                result['native_render_reference'] = [numerator, denominator]
                result['native_render_event_sim_time_s'] = float(self._core.get_sim_time_at_time((numerator, denominator)))
        # This pose is the sensor's current USD pose, not a replacement for the
        # scan-buffer-provided end-of-scan transform or per-point GT history.
        matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.sensor_path))
        result['sensor_usd_world_matrix_at_read'] = [[float(matrix[i][j]) for j in range(4)] for i in range(4)]
        result['usd_world_matrix_is_scan_timestamp_pose'] = False
        scan_items = [(key, a) for key, a in arrays.items()
                      if self._array_path(result['native'], key).startswith('/' + FULL_SCAN + '/')]
        if scan_items:
            digest = hashlib.sha256()
            for key, array in scan_items:
                digest.update(key.encode('ascii'))
                digest.update(array.tobytes(order='C'))
            signature = digest.hexdigest()
            result['scan_buffer_identical_to_previous_read'] = None if self._previous_scan_digest is None else signature == self._previous_scan_digest
            self._previous_scan_digest = signature
        if save:
            if result['raw_bytes'] > MAX_SAMPLE_BYTES or self.saved_bytes + result['raw_bytes'] + 1024 * 1024 > MAX_TOTAL_SAVED_BYTES:
                raise RuntimeError('Bounded LiDAR save budget exhausted')
            out = confined_path(self.output_root)
            out.mkdir(parents=True, exist_ok=True)
            out = confined_path(out, must_exist=True)
            # UUID + exclusive open means no dataset or prior sample overwrite.
            destination = out / ('lidar_%06d_%s.npz' % (self.counter, uuid.uuid4().hex[:12]))
            destination = confined_path(destination)
            with destination.open('xb') as stream:
                np.savez(stream, **arrays)
                stream.flush()
                os.fsync(stream.fileno())
            size = destination.stat().st_size
            if size > MAX_SAMPLE_BYTES + 1024 * 1024:
                raise RuntimeError('Native archive unexpectedly exceeded bound; file preserved')
            self.saved_bytes += size
            result['saved_native_arrays'] = {'path': str(destination), 'bytes': size,
                'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
                'format': 'uncompressed NPZ; original native numeric dtypes',
                'total_adapter_saved_bytes': self.saved_bytes}
        # Strict serialization check without silently allowing NaN in reports.
        json.dumps(result, allow_nan=False)
        return result

    @staticmethod
    def _cloud_report(tree, source):
        """Convenience aliases retain the native field path/NPZ array key."""
        leaves = []
        def collect(value):
            if isinstance(value, dict):
                if 'array_key' in value:
                    leaves.append(value)
                else:
                    for child in value.values():
                        collect(child)
            elif isinstance(value, list):
                for child in value:
                    collect(child)
        collect(tree)
        points = [x for x in leaves if x['native_field_path'].endswith('/data')
                  and len(x['shape']) == 2 and x['shape'][1] == 3]
        timestamps = [x for x in leaves if 'timestamp' in x['native_field_path'].lower()]
        return {'native_annotator': source,
                'point_count': points[0]['shape'][0] if len(points) == 1 else 0,
                'finite_nonzero_points': points[0].get('finite_nonzero_rows', 0) if len(points) == 1 else 0,
                'point_array': {k: v for k, v in points[0].items() if k != 'values'} if len(points) == 1 else None,
                'timestamp_arrays': [{k: v for k, v in x.items() if k != 'values'} for x in timestamps],
                'timestamp_arrays_valid_per_point': False if source == FULL_SCAN else None,
                'point_count_requires_native_N_by_3_data': True,
                'full_scan_complete_flag_inferred': False}

    @staticmethod
    def _array_path(tree, key):
        if isinstance(tree, dict):
            if tree.get('array_key') == key:
                return tree.get('native_field_path', '')
            for value in tree.values():
                found = DroneLidarAdapter._array_path(value, key)
                if found:
                    return found
        elif isinstance(tree, list):
            for value in tree:
                found = DroneLidarAdapter._array_path(value, key)
                if found:
                    return found
        return ''

    def close(self):
        if self.closed:
            return {'closed': True, 'already_closed': True}
        errors = []
        for name, annotator in list(self.annotators.items()):
            try:
                annotator.detach()
            except Exception as exc:
                errors.append(name + ': ' + repr(exc))
        self.annotators.clear()
        if self.sensor is not None:
            try:
                # The SDK destructor owns these exact resources. Clear its
                # reference after explicit destroy to prevent double-destruction.
                self.sensor.detach_all_writers()
                self.sensor.detach_all_annotators()
                rp = self.sensor._render_product
                if rp is not None:
                    rp.destroy()
                    self.sensor._render_product = None
                    self.sensor._render_product_path = None
            except Exception as exc:
                errors.append('render_product: ' + repr(exc))
            self.sensor = None
        if self.created:
            from pxr import Usd
            prim = self.stage.GetPrimAtPath(self.mount_path)
            if prim.IsValid() and prim.GetCustomDataByKey('mro:lidar_owner') == self.owner_token:
                with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
                    if not self.stage.RemovePrim(self.mount_path):
                        errors.append('Owned mount removal returned false')
            elif prim.IsValid():
                errors.append('Mount owner token changed; prim preserved')
        self.closed = not errors
        return {'closed': self.closed, 'errors': errors}
