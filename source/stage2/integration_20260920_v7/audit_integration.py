"""Independent actual-bag RGB QR decode and raw payload/clock inventory audit."""
import collections,hashlib,json,sqlite3,struct,sys,traceback,math
from pathlib import Path
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores,get_typestore
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');sys.path[:0]=[str(M/'stage2/integration_20260920_v7'),str(M/'stage1/collection_20260918_v1'),str(M/'scripts')]
from integration_common import topics
from native_snapshot import parse_gmo_packet
from record_runtime import pointcloud
import cv2
case=Path(sys.argv[1]);out=Path(sys.argv[2]);bag=case/'bag';store=get_typestore(Stores.ROS2_HUMBLE)
def lines(p):return [json.loads(x) for x in p.read_text().splitlines()]
def save(n,v):(out/n).write_text(json.dumps(v,indent=2,allow_nan=False))
def ns(s):return s.sec*10**9+s.nanosec
def fingerprint(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024**2),b''):h.update(b)
 return {'bytes':p.stat().st_size,'mtime_ns':p.stat().st_mtime_ns,'sha256':h.hexdigest()}
spec=json.loads((case/'spec.json').read_text());producer=json.loads((case/'producer_report.json').read_text());assert producer['status']=='complete' and not producer['cleanup_errors']
states=lines(case/'states.jsonl');images=lines(case/'images.jsonl');events=lines(case/'native_events.jsonl');pub=lines(case/'published.jsonl');sub=lines(case/'receipts.jsonl')
pubindex={(r['key'],r['sequence_on_topic']):r for r in pub};subindex={(r['key'],r['sequence_on_topic']):r for r in sub};assert len(pubindex)==len(pub) and set(pubindex)==set(subindex)
imageindex={(r['camera_id'],r['stamp_ns']):r for r in images};eventindex={(r['sensor'],r['native_header_ns']):r for r in events if 'pointcloud' in r};assert len(eventindex)==sum('pointcloud' in r for r in events)
stateindex={r['stamp_ns']:r for r in states};stepindex={(r['epoch'],r['clock']['step']):r['stamp_ns'] for r in states}
assert len(states)==spec['ticks']+1 and len(stateindex)==len(states) and len(stepindex)==len(states)
assert [r['clock']['step'] for r in states]==list(range(states[0]['clock']['step'],states[0]['clock']['step']+len(states)))
assert all(b['stamp_ns']>a['stamp_ns'] for a,b in zip(states,states[1:]))
assert all(abs((b['stamp_ns']-a['stamp_ns'])-1e9/60)<2 for a,b in zip(states,states[1:]))
route_errors=[];turn_states=0
for state in states:
 t=state['route_s'];z=producer['scene']['rover_center_z_m'];rp=state['robots']['rover']['gt_center']['position']
 leg=min(int(t//7),3);phase=t-7*leg
 corners=[(2,0),(2,6),(8,6),(8,0),(2,0)]
 a=np.array(corners[leg]);b=np.array(corners[leg+1]);fraction=min(phase/6,1.)
 expected=[*(a+(b-a)*fraction),z];vel=[*((b-a)/6 if phase<6 and t<28 else np.zeros(2)),0.]
 yaw=90-90*leg-90*min(max(phase-6,0),1.)
 turn_states+=int(6<=phase<7 and t<28)
 route_errors.append(math.dist(rp,expected));assert np.allclose(state['robots']['rover']['velocity_world_m_s'],vel,atol=1e-10,rtol=0)
 q=state['robots']['rover']['gt_center']['quaternion_wxyz'];actual_yaw=math.degrees(math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2)))
 assert abs((actual_yaw-yaw+180)%360-180)<1e-7
 if t<4.8335:expected_d=[5,0,.1665+t]
 elif t<14.8335:expected_d=[5,t-4.8335,5]
 elif t<24.8335:expected_d=[5,24.8335-t,5]
 elif t<29.667:expected_d=[5,0,29.8335-t]
 else:expected_d=[5,0,.1665]
 route_errors.append(math.dist(state['robots']['drone']['gt_center']['position'],expected_d))
assert max(route_errors)<1e-7
if spec['ticks']==300:assert turn_states==60
if spec['ticks']==1800:assert turn_states==240
dbs=list(bag.glob('*.db3'));assert len(dbs)==1;db=dbs[0];before=fingerprint(db);yamlbefore=fingerprint(bag/'metadata.yaml')
keys={v[0]:k for k,v in topics().items()};counts=collections.Counter();points=collections.Counter();outside=collections.Counter();metadata_errors=[];mapping_rows=[];completions=[];clockstamps=[];gtchecks=0;imagechecks=0;cloudchecks=0;info_checks=0;wire_exact=0;padding=0;session=None
max_camera_error=0;max_lidar_ref_error=0
with Reader(bag) as reader:
 assert {c.topic:c.msgtype for c in reader.connections}==dict(topics().values())
 for conn,stamp,raw in reader.messages():
  k=keys[conn.topic];seq=counts[k];counts[k]+=1;h=hashlib.sha256(raw).hexdigest();pr=pubindex[(k,seq)];assert h==subindex[(k,seq)]['sha256']
  if h==pr['sha256']:wire_exact+=1
  else:
   # DDS pads serialized payloads to a four-byte wire boundary. Prove every
   # byte originally emitted by the publisher is exact, then inspect suffix.
   end=pr['bytes'];assert 0<len(raw)-end<=3 and all(x==0 for x in raw[end:])
   assert hashlib.sha256(raw[:end]).hexdigest()==pr['sha256'];padding+=1
  msg=store.deserialize_cdr(raw,conn.msgtype)
  if k=='session':session=json.loads(msg.data)
  elif k=='clock':clockstamps.append(ns(msg.clock))
  elif k=='mapping':
   v=json.loads(msg.data);mapping_rows.append(v)
   if v['type']=='completion' and 'selected_raw_files' in v:completions.append(v)
  elif k=='static':
   assert len(msg.transforms)==7 and len({t.child_frame_id for t in msg.transforms})==7
   for t in msg.transforms:
    expected=next(v for v in producer['static_transforms'] if v['child']==t.child_frame_id)
    assert t.header.frame_id==expected['parent'] and ns(t.header.stamp)==producer['origin']['stamp_ns']
    tr=t.transform.translation;q=t.transform.rotation
    assert np.allclose([tr.x,tr.y,tr.z],expected['position'],atol=1e-10,rtol=0) and np.allclose([q.w,q.x,q.y,q.z],expected['quaternion_wxyz'],atol=1e-10,rtol=0)
  elif k.endswith('_waypoints'):assert json.loads(msg.data)==producer['trajectories'][k.split('_')[0]]
  elif k.endswith('_gt_pose'):
   robot=k.split('_')[0];r=stateindex[ns(msg.header.stamp)]['robots'][robot]['gt_center'];assert msg.header.frame_id=='world'
   assert np.allclose([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z],r['position'],atol=1e-10,rtol=0)
   q=[msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z];assert np.allclose(q,r['quaternion_wxyz'],atol=1e-10,rtol=0) and abs(np.linalg.norm(q)-1)<1e-9;gtchecks+=1
  elif k.endswith('_gt_velocity'):
   robot=k.split('_')[0];r=stateindex[ns(msg.header.stamp)]['robots'][robot]['velocity_world_m_s'];assert np.allclose([msg.vector.x,msg.vector.y,msg.vector.z],r,atol=1e-10,rtol=0);gtchecks+=1
  elif k.endswith('_lidar'):
   robot=k.split('_')[0];row=eventindex[(robot,ns(msg.header.stamp))];original=np.load(case/row['raw_file'],allow_pickle=False);assert hashlib.sha256(original.tobytes()).hexdigest()==row['raw_sha256']
   parsed=parse_gmo_packet(original);pc,body=pointcloud(parsed);assert body==msg.data.tobytes() and msg.width==pc['width'] and msg.point_step==32 and msg.row_step==len(body) and msg.header.frame_id==robot+'_1/lidar'
   assert [(f.name,f.offset,f.datatype,f.count) for f in msg.fields]==[(f['name'],f['offset'],f['datatype'],1) for f in pc['fields']]
   points[robot]+=msg.width;outside[robot]+=pc['valid_points_outside_native_frame'];cloudchecks+=1
   err=abs(round(row['native']['reference_sim_s']*1e9)-row['stamp_ns']);max_lidar_ref_error=max(max_lidar_ref_error,err)
  elif k.endswith('_image'):
   cid=k[:-6];stampns=ns(msg.header.stamp);row=imageindex[(cid,stampns)];assert msg.encoding=='rgb8' and msg.step==msg.width*3 and msg.header.frame_id==cid+'/optical'
   data=msg.data.tobytes();assert hashlib.sha256(data).hexdigest()==row['source_sha256'];assert stampns in stateindex
   actualnative=row['native_simulation_time'];actualnative=float(actualnative.get('simulationTime') if isinstance(actualnative,dict) else actualnative)
   err=max(abs(round(row['reference_sim_s']*1e9)-stampns),abs(round(actualnative*1e9)-stampns));max_camera_error=max(max_camera_error,err)
   rgb=np.frombuffer(data,np.uint8).reshape(msg.height,msg.width,3)
   imagechecks+=1
   if row['index'] in (0,120,180,900,spec['ticks']):
    preview=cv2.cvtColor(cv2.resize(rgb,(round(msg.width*min(1,1200/msg.width)),round(msg.height*min(1,1200/msg.width)))),cv2.COLOR_RGB2BGR);assert cv2.imwrite(str(out/(cid+'_'+str(row['index'])+'.png')),preview)
   if imagechecks%12==0:save('progress.json',{'images_checked':imagechecks,'expected_images':len(images),'messages':sum(counts.values())})
  elif k.endswith('_info'):
   cid=k[:-5];assert (cid,ns(msg.header.stamp)) in imageindex;cal=next(c for c in producer['cameras'] if c['camera_id']==cid)
   ck=cal['k'];assert msg.header.frame_id==cid+'/optical' and msg.distortion_model=='plumb_bob'
   assert (msg.width,msg.height)==(cal['width'],cal['height']) and np.allclose(msg.k,ck) and np.array_equal(msg.r,np.eye(3).reshape(-1)) and np.array_equal(msg.d,np.zeros(5))
   assert np.allclose(msg.p,[ck[0],0.,ck[2],0.,0.,ck[4],ck[5],0.,0.,0.,1.,0.]) and msg.binning_x==msg.binning_y==0
   assert msg.roi.x_offset==msg.roi.y_offset==msg.roi.height==msg.roi.width==0 and not msg.roi.do_rectify;info_checks+=1
assert counts==collections.Counter(json.loads((case/'publisher_done.json').read_text())['counts'])==collections.Counter(json.loads((case/'subscriber_done.json').read_text())['counts'])
assert clockstamps==[r['stamp_ns'] for r in states] and imagechecks==info_checks==len(images) and gtchecks==4*len(states) and cloudchecks==len(eventindex)
assert [r['selected_raw_files'] for r in completions]==[r['selected_raw_files'] for r in states]
assert [r['completion_evidence'] for r in completions]==[r['completion_evidence'] for r in states]
assert [r['first_completion_selected'] for r in completions]==[r['first_completion_selected'] for r in states]
cycle_summary={robot:{'first_completion_missing_states':0,'second_completion_missing_states':0,'selected_states':0,'raw_nonempty_by_cycle':collections.Counter()} for robot in ('drone','rover')}
for state in states:
    own=[e for e in events if e['index']==state['index']]
    assert all(e['observed_clock']==state['clock'] for e in own)
    assert {e['completion_cycle'] for e in own}<={1,2}
    chosen={robot:path for cycle in state['completion_evidence'] for robot,path in cycle['last_nonempty'].items()}
    assert chosen==state['selected_raw_files']
    assert [v['cycle'] for v in state['completion_evidence']]==[1,2]
    for robot in cycle_summary:
        c=cycle_summary[robot];c['selected_states']+=1
        c['first_completion_missing_states']+=int(robot not in state['first_completion_selected'])
        c['second_completion_missing_states']+=int(robot not in state['completion_evidence'][1]['last_nonempty'])
        for e in own:
            if e['sensor']==robot and 'pointcloud' in e and e['pointcloud']['width']>0:c['raw_nonempty_by_cycle'][e['completion_cycle']]+=1
raw_checked=0;rejected_checked=[];empty_diagnostic_headers=0
for event in events:
    original=np.load(case/event['raw_file'],allow_pickle=False)
    assert hashlib.sha256(original.tobytes()).hexdigest()==event['raw_sha256'];raw_checked+=1
    # Zero-element headers are retained diagnostics, not decoded point arrays.
    # This mirrors the pre-existing Reader empty-header contract without
    # relaxing a single structural guard for any nonempty point packet.
    if struct.unpack_from('<I',original,24)[0]==0:
        assert original.nbytes>=264 and struct.unpack_from('<I',original,0)[0]==0x4E474D4F
        assert 264<=struct.unpack_from('<Q',original,16)[0]<=original.nbytes
        assert event['native']['available'] is False and event['native']['num_elements']==0
        assert 'pointcloud' not in event and not event.get('rejected_native_packet')
        assert event['native']['raw_header_hex']==original[:264].tobytes().hex()
        empty_diagnostic_headers+=1
        continue
    try: parsed=parse_gmo_packet(original)
    except (ValueError,TypeError):
        assert event.get('rejected_native_packet') and 'pointcloud' not in event
        rejected_checked.append(event['raw_file'])
    else: assert not event.get('rejected_native_packet')
for state in states:
    for cycle in state['completion_evidence']:
        own=[e for e in events if e['index']==state['index'] and e['completion_cycle']==cycle['cycle']]
        assert cycle['last_nonempty']=={e['sensor']:e['raw_file'] for e in own if 'pointcloud' in e and e['pointcloud']['width']>0}
        assert cycle['rejected_native_packets']==sum(bool(e.get('rejected_native_packet')) for e in own)
selected={r['raw_file'] for r in events if 'pointcloud' in r};assert all(set(r['selected_raw_files'])=={'drone','rover'} and set(r['selected_raw_files'].values())<=selected for r in states)
assert session['source_hashes'] and session['spec']==spec and session['scene']['boards']=={}
assert spec['qr_boards_enabled'] is False and session['scene']['qr_boards_enabled'] is False
assert session['qr_verification']['image_content_time_verified'] is None
assert producer['qr_absence_initial']['active_fixture_paths']==producer['qr_absence_final']['active_fixture_paths']==[]
assert session['qr_absence_initial']==producer['qr_absence_initial']
assert not any(t['child'].endswith('/qr') for t in producer['static_transforms'])
layer_text=(case/'recording_session.usda').read_text()
assert not any(token in layer_text for token in ('ValidationTimeBoard_', 'Stage1ValidationMarkers', 'Stage2ValidationMarkers', 'ValidationColorPlane'))
assert session['validation_provenance']==json.loads((M/'stage2/integration_20260920_v7/VALIDATION_PROVENANCE.json').read_text())
assert session['recording_session_sha256']==hashlib.sha256((case/'recording_session.usda').read_bytes()).hexdigest()
for rel,sha in session['source_hashes'].items():assert hashlib.sha256((M/rel).read_bytes()).hexdigest()==sha,rel
assert max_camera_error==0
con=sqlite3.connect(db.as_uri()+'?mode=ro&immutable=1',uri=True);assert con.execute('pragma quick_check').fetchone()[0]=='ok';con.close();after=fingerprint(db);assert after==before and fingerprint(bag/'metadata.yaml')==yamlbefore
result={'status':'pass','scope':'QR-free actual-aircraft collection; camera/GT/global metadata and calibrated60Hz LiDAR scene-step mapping; no direct QR content evidence or fine global beam-time certification','case':str(case),'bag':str(bag),'db_fingerprint':before,'db_preserved':True,'topic_counts':dict(counts),'messages':sum(counts.values()),'image_source_hash_checks':imagechecks,'camera_info_checks':info_checks,'gt_readback_checks':gtchecks,'clock_steps':len(states),'sim_duration_s':(states[-1]['stamp_ns']-states[0]['stamp_ns'])/1e9,'route_interval_s':[states[0]['route_s'],states[-1]['route_s']],'raw_lidar_payload_checks':cloudchecks,'valid_points':dict(points),'points_outside_own_native_frame':dict(outside),'selected_states':len(completions),'camera_metadata_max_error_ns':max_camera_error,'lidar_reference_max_error_ns':max_lidar_ref_error,'qr_verification':{'status':'not_performed_qr_removed','valid_qr_total':None,'wrong_qr_total':None},'camera_image_content_time_verified':None,'qr_absence_verified':True,'native_point_acquisition_global_time_verified':False,'moving_lidar_content_time_verified':False,'all_sensor_acquisition_sync_verified':False,'publisher_dds_exact_cdr':wire_exact,'publisher_dds_string_padding_only':padding,'dds_bag_exact_cdr':sum(counts.values()),'source_hashes_embedded':len(session['source_hashes']),'static_tf_count':7,'sqlite_quick_check':'ok'}
result['publisher_dds_trailing_zero_padding_only']=result.pop('publisher_dds_string_padding_only')
result['route_motion_check']={'maximum_position_error_m':max(route_errors),'rover_stationary_turn_state_count':turn_states,'criterion':'motion compliance only; NEVER a sensor timing criterion','actual_samples':[{k:r[k] for k in ['clock','route_s','robots']} for i,r in enumerate(states) if i in (0,120,150,180,spec['ticks'])]}
result['native_header_minus_global_observation_ns']={robot:{'min':min(r['native_header_ns']-r['stamp_ns'] for r in events if r['sensor']==robot and 'pointcloud' in r),'max':max(r['native_header_ns']-r['stamp_ns'] for r in events if r['sensor']==robot and 'pointcloud' in r)} for robot in ('drone','rover')}
result['all_native_raw_snapshot_hash_checks']=raw_checked;result['zero_element_diagnostic_headers_checked']=empty_diagnostic_headers;result['rejected_native_packets']={'count':len(rejected_checked),'files':rejected_checked,'retained_raw':True,'included_as_pointcloud':False};result['completion_cycle_checks']=cycle_summary;result['two_fixed_completion_cycles_verified']=True
from audit_scene_mapping import audit_mapping
result['scene_mapping']=audit_mapping(case,states,events,mapping_rows,session,out)
result['zero_valid_return_packets']=sum(bool(e.get('zero_valid_return_packet')) for e in events)
result['zero_valid_return_semantics']='retained raw and width0 PointCloud2, nullable valid-point time extrema, excluded only from nonempty selection; native frame interval remains in all-interval search'
result['agreed_scene_step_sync_pass']=bool(result['scene_mapping']['selected_scene_step_mapping_pass'] and max_camera_error==0 and max_lidar_ref_error==0)
save('AUDIT.json',result);print(json.dumps({k:v for k,v in result.items() if k!='qr'}))
