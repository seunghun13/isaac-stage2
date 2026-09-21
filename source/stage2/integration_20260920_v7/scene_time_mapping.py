"""Explicit scene-step evidence alongside unchanged native emission times."""
import hashlib,json
import numpy as np

SCHEMA='stage2_scene_step_mapping_v1'
PROFILE_KEYS=('fire_time_ns','max_returns','channels','emitters','report_rate_hz','scan_rate_hz','tick_rate_hz','motion_compensation')

def profile_hash(profile):
    return hashlib.sha256(json.dumps({k:profile[k] for k in PROFILE_KEYS},sort_keys=True,separators=(',',':')).encode()).hexdigest()

def read_profiles(stage,readers,calibration):
    profiles={}
    for sensor,r in readers.items():
        prim=stage.GetPrimAtPath(r.metadata['sensor_path'])
        attributes={a.GetName():{'type':str(a.GetTypeName()),'value_string':str(a.Get())} for a in prim.GetAttributes()}
        fire={a.GetName():[int(x) for x in a.Get()] for a in prim.GetAttributes() if a.GetName().endswith(':fireTimeNs') and a.Get() is not None}
        assert len(fire)==1,(sensor,fire)
        def attr(name):
            a=prim.GetAttribute(name);assert a.IsValid() and a.Get() is not None,(sensor,name)
            return a.Get()
        p={'sensor_path':r.metadata['sensor_path'],'attributes':attributes,'fire_time_ns':next(iter(fire.values())),'fire_attribute':next(iter(fire)),
           'max_returns':int(attr('omni:sensor:Core:maxReturns')),'channels':int(attr('omni:sensor:Core:numberOfChannels')),
           'emitters':int(attr('omni:sensor:Core:numberOfEmitters')),'report_rate_hz':int(attr('omni:sensor:Core:reportRateBaseHz')),
           'scan_rate_hz':int(attr('omni:sensor:Core:scanRateBaseHz')),'tick_rate_hz':float(attr('omni:sensor:tickRate')),
           'motion_compensation':str(attr('omni:sensor:Core:outputMotionCompensationState'))}
        assert len(p['fire_time_ns'])==p['channels']
        assert all(p[k]==calibration['profiles'][sensor][k] for k in PROFILE_KEYS),'Unsupported runtime LiDAR profile'
        p['profile_sha256']=profile_hash(p);profiles[sensor]=p
    return profiles

def packet_mapping(parsed,profile,observed_clock):
    offsets=parsed['native']['timeOffsetNs'].astype(np.int64)
    pattern=np.repeat(np.asarray(profile['fire_time_ns'],dtype=np.int64),profile['max_returns'])
    assert len(offsets)%len(pattern)==0
    residual=np.ptp(offsets.reshape(-1,len(pattern))-pattern,axis=1)
    assert (residual==0).all(),'Native offsets disagree with measured runtime firing profile'
    emission=parsed['point_timestamp_ns_valid'].astype(np.int64)
    reference=emission-pattern[parsed['valid_indices']%len(pattern)]
    start=int(parsed['frameStart']['timestamp_ns']);end=int(parsed['frameEnd']['timestamp_ns'])
    inside=(reference>=start)&(reference<=end)
    return {'schema':SCHEMA,'native_packet_time_ns':str(parsed['packet_timestamp_ns']),
      'native_frame_start_ns':str(start),'native_frame_end_ns':str(end),
      'native_emission_min_ns':str(int(emission.min())) if len(emission) else None,'native_emission_max_ns':str(int(emission.max())) if len(emission) else None,
      'derived_group_reference_min_ns':str(int(reference.min())) if len(reference) else None,'derived_group_reference_max_ns':str(int(reference.max())) if len(reference) else None,
      'runtime_profile_sha256':profile['profile_sha256'],'firing_groups_checked':len(residual),'firing_pattern_max_residual_ns':int(residual.max()),
      'valid_points':len(emission),'group_reference_inside_own_interval':int(inside.sum()),'group_reference_outside_own_interval':int((~inside).sum()),
      'observed_scene_step':int(observed_clock['step']),'observed_scene_stamp_ns':str(round(observed_clock['global_s']*1e9)),
      'temporal_resolution':'60Hz scene step; no sub-step beam emission certification',
      'derived_formula':'native_packet_time_ns + signed timeOffsetNs - runtime fireTimeNs[channel]',
      'channel_formula':'(original_element_index // max_returns) % channels',
      'all_intervals_ambiguity_check':'requires independent complete-bag audit',
      'native_emission_times_unchanged':True,'fine_global_beam_emission_verified':False,'content_decode_in_this_aircraft_capture':False}
