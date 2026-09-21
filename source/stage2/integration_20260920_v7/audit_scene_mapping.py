"""Independent reconstruction from bag mapping and immutable native packets."""
import collections,hashlib,json
import numpy as np
from native_snapshot import parse_gmo_packet

def audit_mapping(case,states,events,bag_rows,session,out):
    profiles=json.loads((case/'runtime_profiles.json').read_text())
    assert profiles==session['runtime_profiles']
    calibration=session['scene_time_mapping_contract']
    assert calibration['resolution_hz']==60 and not calibration['fine_global_beam_time_verified']
    keys=('fire_time_ns','max_returns','channels','emitters','report_rate_hz','scan_rate_hz','tick_rate_hz','motion_compensation')
    for sensor,p in profiles.items():
        core={k:p[k] for k in keys};assert core==calibration['profiles'][sensor]
        assert hashlib.sha256(json.dumps(core,sort_keys=True,separators=(',',':')).encode()).hexdigest()==p['profile_sha256']
    selected={f for s in states for f in s['selected_raw_files'].values()}
    mapped={r['raw_file']:r for r in bag_rows if r['type']=='lidar'}
    assert set(mapped)=={e['raw_file'] for e in events if 'pointcloud' in e}
    intervals=collections.defaultdict(list);points=[];checks=collections.Counter()
    for e in events:
        if 'pointcloud' not in e:continue
        b=mapped[e['raw_file']];assert b['scene_time_mapping']==e['scene_time_mapping']
        for k in ('raw_file','raw_sha256','native_header_ns','observed_clock','native','pointcloud'):
            assert b[k]==e[k],k
        assert b['selected_for_completed_state']==(e['raw_file'] in selected)
        assert b['zero_valid_return_packet']==e['zero_valid_return_packet']==(e['pointcloud']['width']==0)
        if e['zero_valid_return_packet']:assert e['raw_file'] not in selected
        raw=np.load(case/e['raw_file'],allow_pickle=False)
        assert hashlib.sha256(raw.tobytes()).hexdigest()==e['raw_sha256']
        p=parse_gmo_packet(raw);profile=profiles[e['sensor']];h=b['scene_time_mapping']
        offset=p['native']['timeOffsetNs'].astype(np.int64)
        channel=(np.arange(len(offset))//profile['max_returns'])%profile['channels']
        fire=np.asarray(profile['fire_time_ns'],np.int64)[channel]
        group=offset-fire;group_rows=group.reshape(-1,profile['max_returns']*profile['channels'])
        assert np.all(np.ptp(group_rows,axis=1)==0)
        native=int(p['packet_timestamp_ns']);emission=native+offset[p['valid_indices']]
        reference=native+group[p['valid_indices']]
        assert np.array_equal(emission.astype(np.uint64),p['point_timestamp_ns_valid'])
        start=int(p['frameStart']['timestamp_ns']);end=int(p['frameEnd']['timestamp_ns'])
        assert h['schema']=='stage2_scene_step_mapping_v1' and h['native_packet_time_ns']==str(native)
        assert (h['native_frame_start_ns'],h['native_frame_end_ns'])==(str(start),str(end))
        assert (h['native_emission_min_ns'],h['native_emission_max_ns'])==((str(int(emission.min())),str(int(emission.max()))) if len(emission) else (None,None))
        assert (h['derived_group_reference_min_ns'],h['derived_group_reference_max_ns'])==((str(int(reference.min())),str(int(reference.max()))) if len(reference) else (None,None))
        assert h['observed_scene_step']==e['observed_clock']['step']
        assert h['observed_scene_stamp_ns']==str(round(e['observed_clock']['global_s']*1e9))
        assert h['runtime_profile_sha256']==profile['profile_sha256']
        assert h['firing_groups_checked']==len(group_rows) and h['firing_pattern_max_residual_ns']==0
        inside=int(((reference>=start)&(reference<=end)).sum())
        assert h['valid_points']==len(reference) and h['group_reference_inside_own_interval']==inside
        assert h['group_reference_outside_own_interval']==len(reference)-inside
        assert h['native_emission_times_unchanged'] and not h['fine_global_beam_emission_verified'] and not h['content_decode_in_this_aircraft_capture']
        intervals[e['sensor']].append((start,end,e['observed_clock']['step']))
        checks['packet_mapping_fields_recomputed']+=1;checks['firing_groups']+=len(group_rows)
        checks['zero_valid_return_packets']+=int(not len(reference))
        if e['raw_file'] in selected:points.append((e,reference))
    statistics=collections.defaultdict(collections.Counter);rows=[]
    arrays={sensor:np.asarray(v,np.int64) for sensor,v in intervals.items()}
    for e,t in points:
        iv=arrays[e['sensor']]
        # Find all temporally intersecting intervals without using the expected step.
        iv=iv[(iv[:,0]<=t.max())&(iv[:,1]>=t.min())]
        candidates={int(step):((iv[iv[:,2]==step,0,None]<=t)&(t<=iv[iv[:,2]==step,1,None])).any(axis=0) for step in np.unique(iv[:,2])}
        multiplicity=sum(candidates.values(),np.zeros(len(t),np.int32))
        current=candidates.get(e['observed_clock']['step'],np.zeros(len(t),bool))
        counts={'packets':1,'points':len(t),'unique_current_step':int(((multiplicity==1)&current).sum()),
           'only_other_step':int(((multiplicity==1)&~current).sum()),'ambiguous_multiple_steps':int((multiplicity>1).sum()),'no_candidate':int((multiplicity==0).sum())}
        statistics[e['sensor']].update(counts)
        rows.append(dict(sensor=e['sensor'],raw_file=e['raw_file'],global_step=e['observed_clock']['step'],stamp_ns=e['stamp_ns'],**counts))
    result={'schema':'stage2_scene_step_mapping_audit_v1','checks':dict(checks),'selected':{k:dict(v) for k,v in statistics.items()},
      'selected_scene_step_mapping_pass':all(v['points']==v['unique_current_step'] and v['packets']==len(states) for v in statistics.values()) and set(statistics)=={'drone','rover'},
      'raw_native_times_unchanged':True,'direct_lidar_content_decode_this_case':False,'prior_calibration_evidence':calibration['evidence'],
      'resolution_hz':60,'fine_global_beam_emission_verified':False,'full30s_all_sensor_content_sync_verified':False}
    (out/'scene_mapping_points.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (out/'SCENE_MAPPING.json').write_text(json.dumps(result,indent=2))
    return result
