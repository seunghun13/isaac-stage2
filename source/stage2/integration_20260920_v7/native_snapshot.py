"""Stage2 frozen raw snapshot parser; protected original remains unchanged."""
import hashlib
MAX_SAMPLE_BYTES = 32 * 1024 * 1024
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
        'aux_type': 'NONE', 'unused_aux_pointer_nonzero': u64(256) != 0, 'auxiliary_property_access': 'none',
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

def read_snapshot(self, raw):
    import struct
    from pxr import UsdGeom
    from record_lidar import rows
    header=raw.reshape(-1)[:264].tobytes()
    empty_header=len(header)==264 and struct.unpack_from('<I',header,0)[0]==0x4E474D4F and struct.unpack_from('<I',header,24)[0]==0
    try:parsed={'available':False,'reason':'native zero-element packet'} if empty_header else parse_gmo_packet(raw)
    except Exception as exc:
        raise ValueError(str(exc)+'; raw GMO header='+raw.reshape(-1)[:264].tobytes().hex()) from exc
    ref=self.ref.get_data()
    rawsim=self.sim.get_data()
    current=UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.metadata['sensor_path']))
    metadata={'immutable_single_raw_snapshot':True,'unused_aux_pointer_nonzero':bool(parsed.get('unused_aux_pointer_nonzero',False)),'available':parsed['available'],'reference':{k:int(v) for k,v in ref.items()},
              'reference_sim_s':float(self.core.get_sim_time_at_time((int(ref['referenceTimeNumerator']),int(ref['referenceTimeDenominator'])))),
              'simulation_annotator':rawsim,'sensor_current_usd_matrix_row':rows(current)}
    if empty_header:
        metadata.update(frame_id=struct.unpack_from('<Q',header,32)[0],packet_timestamp_ns=struct.unpack_from('<Q',header,40)[0],
                        num_elements=0,raw_header_hex=header.hex(),
                        frameStart={'timestamp_ns':str(struct.unpack_from('<Q',header,120)[0])},
                        frameEnd={'timestamp_ns':str(struct.unpack_from('<Q',header,160)[0])})
    if parsed['available']:
        for name in ('frame_id','packet_timestamp_ns','packet_bytes','num_elements','valid_element_count','point_timestamp_min_ns','point_timestamp_max_ns','coords_type'):
            metadata[name]=parsed[name]
        for name in ('frameStart','frameEnd'):
            f=parsed[name]
            metadata[name]={'timestamp_ns':str(f['timestamp_ns']),'position_m':f['position_m'].tolist(),'orientation_xyzw':f['orientation_xyzw'].tolist()}
    return metadata,parsed
