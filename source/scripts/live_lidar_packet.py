"""Read the existing Isaac 5.0 CPU GMO packet for a live ROS PointCloud2.

No sensor, render product, native decoder, filesystem write or World operation.
The existing pure parser keeps XYZ and native point times from one packet.
Callers publish the returned bytes themselves; this module has no ROS imports.

Example in the already prepared Kit context::

    lidar_bundle = read_existing_adapter(
        runtime.lidar, sensor_frame_id='drone_lidar',
        clock_audit={'world_time_s_at_observation': float(world.current_time)})

``header_timestamp_ns`` is a decimal string of the native packet reference,
not a claim that it is World time, the first point time or acquisition start.
The exact uint64 point time is split over two ROS UINT32 fields because
sensor_msgs/PointField has no UINT64 datatype. Recombine using integer math:
``native_time_ns_lo | (native_time_ns_hi << 32)``.
"""
from __future__ import annotations

import base64
import hashlib
import json
import time

import numpy as np

from drone_lidar_adapter import GMO, parse_gmo_packet


MAX_POINTS = 65536
MAX_CLOCK_AUDIT_BYTES = 65536
POINT_STEP = 32
NATIVE_CLOCK_DOMAIN = 'isaac_rtx_gmo_native_timestamp_ns'
POINT_FIELDS = (
    ('x', 0, 7, '<f4'),
    ('y', 4, 7, '<f4'),
    ('z', 8, 7, '<f4'),
    ('scalar', 12, 7, '<f4'),
    ('native_time_ns_lo', 16, 6, '<u4'),
    ('native_time_ns_hi', 20, 6, '<u4'),
    ('native_time_offset_ns', 24, 5, '<i4'),
    ('native_element_index', 28, 6, '<u4'),
)
RECORD_DTYPE = np.dtype({
    'names': [f[0] for f in POINT_FIELDS],
    'formats': [f[3] for f in POINT_FIELDS],
    'offsets': [f[1] for f in POINT_FIELDS],
    'itemsize': POINT_STEP,
})


def _strict_clock_audit(value):
    if value is None:
        return {}
    if not isinstance(value, dict) or any(not isinstance(k, str) for k in value):
        raise TypeError('clock_audit must be a strict JSON object with string keys')
    text = json.dumps(value, allow_nan=False, separators=(',', ':'))
    if len(text.encode('utf-8')) > MAX_CLOCK_AUDIT_BYTES:
        raise ValueError('clock_audit exceeds bounded metadata size')
    return json.loads(text)


def _frame_json(frame):
    return {
        'timestamp_ns': str(int(frame['timestamp_ns'])),
        'orientation_xyzw': frame['orientation_xyzw'].tolist(),
        'position_m': frame['position_m'].tolist(),
    }


def packet_to_pointcloud2(raw_cpu_u8, *, sensor_frame_id,
                         clock_audit=None, max_points=MAX_POINTS):
    """Encode all VALID elements of a single native packet, without truncation.

    Coordinates retain the parser's native sensor frame. Header frame poses and
    model_to_app_transform are preserved as audit evidence, not reapplied or
    replaced by a current USD pose. Do not merge these times with direct or
    accumulated scan-buffer points. Invalid ABI/clock bounds raise; callers
    must preserve the error instead of publishing a previous packet as new.

    Native nanoseconds in JSON are decimal strings to remain exact through
    JavaScript intermediaries. PointCloud2 numeric fields are fixed binary.
    Caller clock observations remain separate and are not a calibration.
    """
    if not isinstance(sensor_frame_id, str) or not sensor_frame_id or len(sensor_frame_id) > 512:
        raise ValueError('sensor_frame_id must be a nonempty bounded string')
    if type(max_points) is not int or not 1 <= max_points <= MAX_POINTS:
        raise ValueError('max_points must be an integer in 1..65536')
    observed = _strict_clock_audit(clock_audit)
    parsed = parse_gmo_packet(raw_cpu_u8, max_points=max_points)
    if not parsed['available']:
        return {
            'available': False, 'frame_id': sensor_frame_id,
            'header_timestamp_ns': None, 'pointcloud2': None,
            'audit': {'clock_domain': NATIVE_CLOCK_DOMAIN,
                      'reason': parsed.get('reason', 'native packet unavailable'),
                      'caller_clock_observations': observed},
        }

    base = int(parsed['packet_timestamp_ns'])
    start = int(parsed['frameStart']['timestamp_ns'])
    end = int(parsed['frameEnd']['timestamp_ns'])
    all_times = parsed['native']['point_timestamp_ns']
    if not np.array_equal(parsed['model_to_app_transform'], np.eye(4, dtype=np.float32)):
        raise ValueError('Nonidentity model_to_app transform is outside this native optical-frame profile')
    if end < start or (all_times.size and
                       (int(all_times.min()) < start or int(all_times.max()) > end)):
        raise ValueError('Native point times are outside their own packet frame interval')
    indices = parsed['valid_indices']
    xyz = parsed['data']
    times = parsed['point_timestamp_ns_valid']
    offsets = parsed['native']['timeOffsetNs'][indices]
    scalars = parsed['scalar_valid']
    count = len(indices)
    if xyz.shape != (count, 3) or any(len(a) != count for a in (times, offsets, scalars)):
        raise ValueError('Native point coordinates/times/indices have inconsistent lengths')
    if not np.array_equal(times, all_times[indices]):
        raise ValueError('Native valid point timestamps differ from source element indices')

    records = np.empty(count, dtype=RECORD_DTYPE)
    for axis, col in zip(('x', 'y', 'z'), range(3)):
        records[axis] = xyz[:, col]
    records['scalar'] = scalars
    records['native_time_ns_lo'] = (times & np.uint64(0xffffffff)).astype(np.uint32)
    records['native_time_ns_hi'] = (times >> np.uint64(32)).astype(np.uint32)
    records['native_time_offset_ns'] = offsets
    records['native_element_index'] = indices.astype(np.uint32)
    encoded = records.tobytes(order='C')
    if len(encoded) != count * POINT_STEP:
        raise AssertionError('Unexpected point record byte size')
    result = {
        'available': True,
        'frame_id': sensor_frame_id,
        'header_timestamp_ns': str(base),
        'pointcloud2': {
            'height': 1, 'width': count,
            'fields': [{'name': name, 'offset': offset, 'datatype': datatype, 'count': 1}
                       for name, offset, datatype, _ in POINT_FIELDS],
            'is_bigendian': False,
            'point_step': POINT_STEP, 'row_step': len(encoded),
            'is_dense': bool(np.isfinite(xyz).all()),
            'data_base64': base64.b64encode(encoded).decode('ascii'),
            'data_sha256': hashlib.sha256(encoded).hexdigest(),
        },
        'audit': {
            'source': GMO,
            'clock_domain': NATIVE_CLOCK_DOMAIN,
            'header_timestamp_semantics': 'native packet timestampNs reference; not asserted as acquisition start, first point or World time',
            'native_packet_timestamp_ns': str(base),
            'native_frame_id': str(int(parsed['frame_id'])),
            'frame_start': _frame_json(parsed['frameStart']),
            'frame_end': _frame_json(parsed['frameEnd']),
            'model_to_app_transform': parsed['model_to_app_transform'].tolist(),
            'frame_motion_raw_word': int(parsed['frame_motion_raw_word']),
            'coordinates': parsed['cartesian_derivation'],
            'coordinate_frame_contract': 'native OmniLidar optical sensor frame; caller frame_id must name this frame, not World or drone base_link',
            'native_model_to_app_is_identity': True,
            'world_or_body_transform_applied': False,
            'per_point_motion_compensation_applied': False,
            'external_tf_calibration_verified_by_helper': False,
            'native_coords_type': parsed['coords_type'],
            'native_point_count': int(parsed['num_elements']),
            'valid_point_count': count,
            'valid_flag_value': int(parsed['valid_flag_value']),
            'native_point_min_ns': str(int(times.min())) if count else None,
            'native_point_max_ns': str(int(times.max())) if count else None,
            'all_element_point_min_ns': str(int(all_times.min())) if all_times.size else None,
            'all_element_point_max_ns': str(int(all_times.max())) if all_times.size else None,
            'all_native_points_within_native_frame': True,
            'point_time_formula': 'native packet uint64 timestampNs + same element signed int32 timeOffsetNs',
            'point_time_binary_reconstruction': 'uint64(native_time_ns_lo) | (uint64(native_time_ns_hi) << 32)',
            'packet_sha256': parsed['packet_sha256'],
            'packet_bytes': int(parsed['packet_bytes']),
            'available_buffer_bytes': int(parsed['available_buffer_bytes']),
            'parser': parsed['parser'],
            'world_clock_mapping_verified': False,
            'caller_clock_observations': observed,
            'point_source_matches_direct_or_full_scan': 'not asserted; no points from those annotators were read',
            'sampling_semantics': 'one current native packet; not a complete accumulated rotary scan',
            'scalar_semantics': 'native scalar copied; no calibrated intensity model asserted',
        },
    }
    json.dumps(result, allow_nan=False)
    return result


def read_existing_adapter(adapter, *, sensor_frame_id,
                          clock_audit=None, max_points=MAX_POINTS):
    """Read only the prepared adapter's already attached CPU GMO annotator.

    Does not call adapter.sample(), alter its counters, save NPZ, or read the
    unmatched direct/fullscan annotators. Parent controls the next render tick.
    Read timestamps are host monotonic observations, not sensor acquisition.
    """
    if not adapter.created or adapter.closed:
        raise RuntimeError('Existing LiDAR adapter is inactive')
    if GMO not in adapter.annotators:
        raise RuntimeError('Existing adapter has no CPU GenericModelOutput annotator')
    begin = time.monotonic_ns()
    raw = adapter.annotators[GMO].get_data()
    after_native_read = time.monotonic_ns()
    result = packet_to_pointcloud2(raw, sensor_frame_id=sensor_frame_id,
                                  clock_audit=clock_audit, max_points=max_points)
    result['audit']['host_monotonic_read_start_ns'] = str(begin)
    result['audit']['host_monotonic_native_read_end_ns'] = str(after_native_read)
    result['audit']['host_monotonic_encode_end_ns'] = str(time.monotonic_ns())
    return result
