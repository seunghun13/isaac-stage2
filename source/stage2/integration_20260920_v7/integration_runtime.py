"""Native 18-topic integrated acquisition with original raw GMO retention."""
import asyncio,hashlib,json,os,struct,time,traceback
import numpy as np
from integration_common import M,S,C,CAMERAS,SIZES,safe,save,validate_spec
from record_runtime import Emitter,pointcloud
from record_probe import plain
from native_snapshot import read_snapshot
from scene_time_mapping import read_profiles,packet_mapping
from integration_scene import Scene,pose,rows,rover_reader
from noqr_guard import assert_qr_free_stage

async def execute(case,req):
    import carb,omni.kit.app,omni.usd,omni.timeline,omni.replicator.core as rep
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from isaacsim.core.simulation_manager import SimulationManager as SM
    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
    from record_lidar import Reader
    from record_render import ManualRendering
    import stage1_scene
    spec=validate_spec(json.loads((case/'spec.json').read_text()));assert spec['launch_id']==os.environ['MRO_LAUNCH_ID']
    stage=omni.usd.get_context().get_stage();timeline=omni.timeline.get_timeline_interface();assert not timeline.is_playing()
    stage.SetEditTarget(stage.GetSessionLayer());original=stage.GetSessionLayer().ExportToString()
    from omni.kit.viewport.utility import get_active_viewport
    viewport=get_active_viewport();old_view=viewport.updates_enabled;viewport.updates_enabled=False
    products=[];readers={};manual=None;emitter=None;streams=[];report={'status':'running','spec':spec,'native_point_acquisition_global_time_verified':False}
    started=time.monotonic();core=_isaacsim_core_nodes.acquire_interface()
    try:
        settings=carb.settings.get_settings()
        for k,v in {'/app/hydraEngine/waitIdle':True,'/app/updateOrder/checkForHydraRenderComplete':1000,'/rtx/ecoMode/enabled':False,'/rtx/post/tonemap/autoExposure/enabled':False,'/app/asyncRendering':False,'/app/asyncRenderingLowLatency':False,'/rtx/materialDb/syncLoads':True,'/rtx/hydra/materialSyncLoads':True}.items():settings.set(k,v)
        assert UsdGeom.GetStageMetersPerUnit(stage)==1. and UsdGeom.GetStageUpAxis(stage)=='Z'
        scene=Scene(stage,spec['epoch']);observed=stage1_scene.inspect(stage,0);assert not observed['enabled_rigid_bodies'] and not observed['active_physics_scenes']
        cams=[];transforms=[]
        for cid in CAMERAS:
            meta=observed['cameras'][cid];w,h=SIZES[cid];cam=UsdGeom.Camera(stage.GetPrimAtPath('/MRO_Cameras/'+cid));f=float(cam.GetFocalLengthAttr().Get())
            fx=f/float(cam.GetHorizontalApertureAttr().Get())*w;fy=f/float(cam.GetVerticalApertureAttr().Get())*h
            usd=Gf.Matrix4d(*[x for row in meta['T_world_from_usd_camera_row'] for x in row]);optical=Gf.Matrix4d(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1)*usd
            c={'camera_id':cid,'width':w,'height':h,'k':[fx,0.,w/2,0.,fy,h/2,0.,0.,1.],'T_world_optical_row':rows(optical),'original_camera':meta};cams.append(c)
            transforms.append(dict(parent='world',child=cid+'/optical',**pose(optical)))
            p=rep.create.render_product('/MRO_Cameras/'+cid,(w,h));annotations=[rep.AnnotatorRegistry.get_annotator(n) for n in ('rgb','ReferenceTime','IsaacReadSimulationTime')];annotations[-1].initialize(resetOnStop=False)
            for a in annotations:a.attach([p])
            products.append((c,p,annotations))
        readers={'drone':Reader(stage),'rover':rover_reader(stage)}
        for robot,r in readers.items():
            for parent,child,key in [('tracking_center','base_link','T_center_from_base_row'),('base_link','lidar','T_base_from_sensor_row')]:
                m=Gf.Matrix4d(*[x for row in r.metadata[key] for x in row]);transforms.append(dict(parent=robot+'_1/'+parent,child=robot+'_1/'+child,**pose(m)))
        assert len(transforms)==7
        physics=UsdPhysics.Scene.Define(stage,'/Stage2Clock');physics.CreateGravityMagnitudeAttr(0.);api=PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim());api.CreateTimeStepsPerSecondAttr(60);api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP')
        SM.set_default_physics_scene('/Stage2Clock');SM.set_physics_sim_device('cpu');await omni.kit.app.get_app().next_update_async();SM.initialize_physics();timeline.pause()
        await rep.orchestrator.step_async(delta_time=0.,rt_subframes=1,pause_timeline=True)
        manual=ManualRendering([p for _,p,_ in products]+[r.product for r in readers.values()]);seen={k:{} for k in readers};events=[];current_state=None;event_index=0;rawbytes=0
        rawdir=case/'raw';rawdir.mkdir();event_log=(case/'native_events.jsonl').open('x');state_log=(case/'states.jsonl').open('x');image_log=(case/'images.jsonl').open('x');streams=[event_log,state_log,image_log]
        def clock():return {'global_s':float(SM.get_simulation_time()),'step':int(SM.get_num_physics_steps())}
        def frame(path):
            nonlocal event_index,rawbytes
            if current_state is None:return
            for robot,r in readers.items():
                if path!=str(r.product.path):continue
                raw=np.array(r.gmo.get_data(),copy=True).reshape(-1);assert raw.nbytes>=264
                declared=struct.unpack_from('<Q',raw,16)[0];assert 264<=declared<=raw.nbytes;raw=raw[:declared]
                fid=struct.unpack_from('<Q',raw,32)[0];ns=struct.unpack_from('<Q',raw,40)[0];sha=hashlib.sha256(raw.tobytes()).hexdigest();key=(fid,ns,sha)
                if key in seen[robot]:return
                name=f'{robot}_{event_index:06d}.npy';np.save(rawdir/name,raw,allow_pickle=False);rawbytes+=raw.nbytes;assert rawbytes<2*1024**3
                error=None
                try:meta,parsed=read_snapshot(r,raw)
                except (ValueError,TypeError) as exc:
                    # Preserve incomplete callback snapshots as rejected raw evidence.
                    # Never publish structurally invalid native data as a point cloud.
                    error=repr(exc);parsed={'available':False}
                    meta={'available':False,'immutable_single_raw_snapshot':True,'parse_error':error}
                row=dict(current_state,type='lidar',event_index=event_index,sensor=robot,native_header_ns=ns,native_frame_id=fid,native=plain(meta),raw_file='raw/'+name,raw_sha256=sha,native_point_acquisition_global_time_verified=False,observed_clock=clock(),read_phase='NEW_FRAME_callback',rejected_native_packet=error is not None)
                body=b''
                if parsed['available']:
                    assert parsed['packet_sha256']==sha
                    pc,body=pointcloud(parsed);row['pointcloud']=pc
                    row['zero_valid_return_packet']=pc['width']==0
                    row['scene_time_mapping']=packet_mapping(parsed,runtime_profiles[robot],row['observed_clock'])
                event_log.write(json.dumps(plain(row),allow_nan=False)+'\n');event_log.flush();events.append((row,body));seen[robot][key]=(row,body);event_index+=1
                return row,body
        def guarded_frame(path):
            if manual.error:return
            try:frame(path)
            except Exception:
                save(case/'callback_error.json',{'traceback':traceback.format_exc(),'path':path});raise
        manual.after_frame=guarded_frame
        async def render():
            assert time.monotonic()-started<spec['wall_timeout_s'],'finite acquisition wall bound'
            if (case/'abort_requested.json').exists():raise RuntimeError('abort requested')
            fut=manual.next_complete();await omni.kit.app.get_app().next_update_async();await asyncio.wait_for(fut,90)
            await rep.orchestrator.step_async(delta_time=0.,rt_subframes=1,pause_timeline=True);manual.check()
        scene.move(spec['route_offset_s'])
        for i in range(30):
            await render()
            if i%5==0:save(case/'progress.json',{'phase':'warmup','index':i})
        calibration=json.loads((C/'CALIBRATION.json').read_text());runtime_profiles=read_profiles(stage,readers,calibration)
        save(case/'runtime_profiles.json',runtime_profiles)
        origin=clock();origin['stamp_ns']=round(origin['global_s']*1e9)
        source_hashes=json.loads((C/'SOURCE_MANIFEST.json').read_text())
        for rel,sha in source_hashes.items():assert hashlib.sha256((M/rel).read_bytes()).hexdigest()==sha,rel
        report.update(qr_absence_initial=assert_qr_free_stage(stage),qr_verification={'status':'not_performed_qr_removed','image_content_time_verified':None},validation_provenance=json.loads((C/'VALIDATION_PROVENANCE.json').read_text()),runtime_profiles=runtime_profiles,scene_time_mapping_contract=calibration,origin=origin,cameras=cams,static_transforms=transforms,scene=scene.metadata,lidars={k:r.metadata for k,r in readers.items()},source_hashes=source_hashes,
          trajectories={'drone':{'speed_m_s':1,'points':[[5,0,.1665],[5,0,5],[5,10,5],[5,0,5],[5,0,.1665]],'yaw_deg':180},'rover':{'speed_m_s':1,'points':[[2,0,scene.rover_z],[2,6,scene.rover_z],[8,6,scene.rover_z],[8,0,scene.rover_z],[2,0,scene.rover_z]],'turn_duration_s':1,'turn_angle_deg':-90}},
          packet_selection_rule='last structurally valid nonempty frozen callback snapshot per sensor within TWO fixed completed cycles; rejected raw diagnostics retained; all valid nonempty snapshots published, no content input',completion_cycles=2,
          time_contract={'header_image_gt_clock':'observed native global seconds rounded to ns','header_lidar':'unchanged native packet timestamp; not global acquisition time','qr':'absent; no direct content time decode in this capture','bag_record_time':'DDS callback wall receipt','route_offset_s':spec['route_offset_s'],'propulsion':False,'lidar_moving_content_time_verified':False,'scene_step_mapping_schema':'stage2_scene_step_mapping_v1','scene_step_resolution_hz':60,'exact_global_beam_time_verified':False},state_count=0,image_count=0,lidar_count=0,raw_event_count=0)
        stage.GetSessionLayer().Export(str(case/'recording_session.usda'));report['recording_session_sha256']=hashlib.sha256((case/'recording_session.usda').read_bytes()).hexdigest()
        emitter=Emitter(json.loads((case/'publisher_ready.json').read_text())['port']);emitter.offer(dict(report,type='config'));save(case/'configuration.json',plain(report))
        for i in range(spec['ticks']+1):
            if i:SM.step(render=False)
            now=clock();assert now['step']==origin['step']+i and now['step']<65535
            route_s=spec['route_offset_s']+now['global_s']-origin['global_s'];robots=scene.move(route_s)
            current_state={'index':i,'stamp_ns':round(now['global_s']*1e9),'clock':now,'route_s':route_s,'epoch':spec['epoch']};events.clear()
            completion_evidence=[];first_selected={}
            for cycle in range(1,3):
                current_state['completion_cycle']=cycle
                before_events=len(events);await render();assert clock()==now
                cycle_events=events[before_events:]
                last_in_cycle={h['sensor']:h['raw_file'] for h,body in cycle_events if body}
                completion_evidence.append({'cycle':cycle,'event_count':len(cycle_events),'rejected_native_packets':sum(bool(h.get('rejected_native_packet')) for h,body in cycle_events),'nonempty_counts':{robot:sum(h['sensor']==robot and bool(body) for h,body in cycle_events) for robot in readers},'last_nonempty':last_in_cycle,'clock_after':clock()})
                if cycle==1:first_selected=dict(last_in_cycle)
            selected={robot:path for cycle in completion_evidence for robot,path in cycle['last_nonempty'].items()}
            assert set(selected)==set(readers),('no completed nonempty packet',i,selected)
            state=dict(current_state,type='state',robots=robots,selected_raw_files=selected,first_completion_selected=first_selected,completion_evidence=completion_evidence)
            state_log.write(json.dumps(state)+'\n');state_log.flush();emitter.offer(state);report['state_count']+=1
            for h,body in events:
                h['selected_for_completed_state']=h['raw_file']==selected.get(h['sensor'])
                if 'pointcloud' in h:emitter.offer(h,body);report['lidar_count']+=1
                else:emitter.offer(dict(h,type='completion',zero_element_raw_packet=not h.get('rejected_native_packet',False)))
            report['raw_event_count']+=len(events)
            report['rejected_native_packet_count']=report.get('rejected_native_packet_count',0)+sum(bool(h.get('rejected_native_packet')) for h,body in events)
            emitter.offer(dict(current_state,type='completion',selected_raw_files=selected,selection_rule=report['packet_selection_rule'],first_completion_selected=first_selected,completion_evidence=completion_evidence))
            if i%4==0:
                for c,p,(rgb,ref,native) in products:
                    pixels=np.ascontiguousarray(np.asarray(rgb.get_data())[:,:,:3]);assert pixels.dtype==np.uint8 and pixels.shape==(c['height'],c['width'],3)
                    body=pixels.tobytes();rawref=plain(ref.get_data());reference_s=float(core.get_sim_time_at_time((int(rawref['referenceTimeNumerator']),int(rawref['referenceTimeDenominator']))))
                    h=dict(current_state,type='image',camera_id=c['camera_id'],reference=rawref,reference_sim_s=reference_s,native_simulation_time=plain(native.get_data()),source_sha256=hashlib.sha256(body).hexdigest())
                    image_log.write(json.dumps(h)+'\n');image_log.flush();emitter.offer(h,body);report['image_count']+=1
            current_state=None
            if i%20==0:save(case/'progress.json',{'phase':'recording','index':i,'ticks':spec['ticks'],'route_s':route_s,'images':report['image_count'],'raw_events':report['raw_event_count']})
        emitter.offer({'type':'end','state_count':report['state_count'],'image_count':report['image_count'],'lidar_count':report['lidar_count']});emitter.finish()
        report.update(qr_absence_final=assert_qr_free_stage(stage),status='complete',final_clock=clock(),queue_peak_bytes=emitter.peak,raw_bytes=rawbytes,wall_s=time.monotonic()-started)
    except BaseException:
        report.update(status='failed',traceback=traceback.format_exc());raise
    finally:
        timeline.pause();errors=[]
        if manual:report['render_events']=manual.events;manual.close()
        for c,p,anns in products:
            for a in anns:
                try:a.detach([p.path])
                except Exception as exc:errors.append(repr(exc))
        for r in readers.values():
            for a in r.annotators:
                try:a.detach([r.product.path])
                except Exception as exc:errors.append(repr(exc))
        for c,p,anns in products:
            try:p.destroy()
            except Exception as exc:errors.append(repr(exc))
        for r in readers.values():
            try:r.product.destroy();r.closed=True
            except Exception as exc:errors.append(repr(exc))
        for f in streams:f.close()
        if emitter and emitter.thread.is_alive():
            try:emitter.finish()
            except Exception as exc:errors.append(repr(exc))
        stage.GetSessionLayer().Clear();stage.GetSessionLayer().ImportFromString(original);timeline.set_current_time(0.);viewport.updates_enabled=old_view
        report.update(cleanup_errors=errors,session_layer_restored=True);save(case/'producer_report.json',plain(report))
    return {'status':report['status'],'case':str(case),'wall_s':report['wall_s']}
