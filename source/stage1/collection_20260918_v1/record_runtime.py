"""QR-free Stage1 acquisition; retain the pilot clock and rendering sequence."""
import asyncio,hashlib,json,os,socket,threading,time,traceback
import numpy as np
from record_common import M,S,C,CAMERAS,SIZES,MAX_QUEUE_BYTES,safe,save,frame_parts,validate_spec
from record_probe import plain

def assert_qr_free_stage(stage):
    """Reject active validation fixtures left in a reused scene."""
    for prim in stage.Traverse():
        if str(prim.GetPath())=='/Stage1ValidationMarkers' or prim.GetName().startswith('ValidationTimeBoard_'):
            raise ValueError('QR fixture remains; a fresh QR-free Stage1 scene is required')

class Emitter:
    def __init__(self,port):
        from collections import deque
        self.sock=socket.create_connection(('127.0.0.1',port),timeout=15);self.sock.settimeout(120)
        self.items=deque();self.condition=threading.Condition();self.bytes=0;self.peak=0;self.error=None;self.closed=False;self.sent=0
        def work():
            try:
                while True:
                    with self.condition:
                        while not self.items and not self.closed:self.condition.wait(.2)
                        if not self.items:break
                        parts,size=self.items.popleft()
                    for part in parts:self.sock.sendall(part)
                    with self.condition:
                        self.bytes-=size;self.sent+=1;self.condition.notify_all()
            except BaseException as exc:
                with self.condition:self.error=repr(exc);self.condition.notify_all()
            finally:self.sock.close()
        self.thread=threading.Thread(target=work,daemon=True);self.thread.start()
    def offer(self,h,body=b''):
        parts=frame_parts(plain(h),body);size=sum(map(len,parts));deadline=time.monotonic()+125
        with self.condition:
            while self.bytes+size>MAX_QUEUE_BYTES:
                if self.error:raise RuntimeError(self.error)
                if time.monotonic()>deadline:raise TimeoutError('Bounded IPC backpressure')
                # Block this simulation thread: asynchronous idle would create unobserved renders.
                self.condition.wait(.1)
            if self.error:raise RuntimeError(self.error)
            self.items.append((parts,size));self.bytes+=size;self.peak=max(self.peak,self.bytes);self.condition.notify_all()
    def finish(self):
        with self.condition:self.closed=True;self.condition.notify_all()
        self.thread.join(150)
        if self.thread.is_alive() or self.error:raise RuntimeError('IPC finish: '+str(self.error))

def pointcloud(parsed):
    from live_lidar_packet import RECORD_DTYPE,POINT_FIELDS
    indices=parsed['valid_indices'];xyz=parsed['data'];times=parsed['point_timestamp_ns_valid'];count=len(indices)
    assert count<=65536 and xyz.shape==(count,3)
    assert np.array_equal(parsed['model_to_app_transform'],np.eye(4,dtype=np.float32))
    assert np.array_equal(times,parsed['native']['point_timestamp_ns'][indices])
    records=np.empty(count,dtype=RECORD_DTYPE)
    for col,axis in enumerate(('x','y','z')):records[axis]=xyz[:,col]
    records['scalar']=parsed['scalar_valid']
    records['native_time_ns_lo']=(times&np.uint64(0xffffffff)).astype(np.uint32)
    records['native_time_ns_hi']=(times>>np.uint64(32)).astype(np.uint32)
    records['native_time_offset_ns']=parsed['native']['timeOffsetNs'][indices]
    records['native_element_index']=indices.astype(np.uint32)
    body=records.tobytes();start=int(parsed['frameStart']['timestamp_ns']);end=int(parsed['frameEnd']['timestamp_ns'])
    return {'width':count,'row_step':len(body),'is_dense':bool(np.isfinite(xyz).all()),
            'fields':[{'name':n,'offset':o,'datatype':d} for n,o,d,_ in POINT_FIELDS],
            'valid_points_outside_native_frame':int(np.count_nonzero((times<start)|(times>end))),
            'point_time_adjusted':False,'coordinates':parsed['cartesian_derivation'],
            'model_to_app_transform':parsed['model_to_app_transform'].tolist(),
            'native_packet_sha256':parsed['packet_sha256'],'data_sha256':hashlib.sha256(body).hexdigest()},body

async def run(stage,timeline,viewport,request):
    import carb,omni.kit.app,omni.replicator.core as rep
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
    from record_lidar import Reader
    from record_render import ManualRendering
    import stage1_scene,waypoint_motion
    case=safe(request['case']);spec=validate_spec(json.loads((case/'spec.json').read_text()))
    assert spec['launch_id']==os.environ['MRO_LAUNCH_ID'] and not timeline.is_playing()
    assert_qr_free_stage(stage)
    run_dir=safe(os.environ['MRO_VIEW_RUN'])
    with (run_dir/'collection_consumed.json').open('x') as f:json.dump({'case':str(case)},f)
    original=stage.GetSessionLayer().ExportToString();old_view=viewport.updates_enabled
    products=[];lidar=None;manual=None;emitter=None;core=_isaacsim_core_nodes.acquire_interface()
    report={'status':'running','spec':spec,'started_wall_ns':str(time.time_ns()),'lidar_acquisition_time_verified':False}
    try:
        viewport.updates_enabled=False
        settings=carb.settings.get_settings()
        for k,v in {'/app/hydraEngine/waitIdle':True,'/app/updateOrder/checkForHydraRenderComplete':1000,
                    '/rtx/ecoMode/enabled':False,'/rtx/post/tonemap/autoExposure/enabled':False}.items():settings.set(k,v)
        observed=stage1_scene.inspect(stage,0);assert not observed['enabled_rigid_bodies'] and not observed['active_physics_scenes']
        cams=[];transforms=[]
        def pose(matrix):
            q=matrix.ExtractRotationQuat();return {'position':list(matrix.ExtractTranslation()),'quaternion_wxyz':[q.GetReal(),*q.GetImaginary()]}
        for cid in CAMERAS:
            meta=observed['cameras'][cid];w,h=SIZES[cid] if spec['resolution']=='native' else ((665,575) if cid=='cam_02' else (480,270))
            camera=UsdGeom.Camera(stage.GetPrimAtPath('/MRO_Cameras/'+cid));f=float(camera.GetFocalLengthAttr().Get())
            fx=f/float(camera.GetHorizontalApertureAttr().Get())*w;fy=f/float(camera.GetVerticalApertureAttr().Get())*h
            usd=Gf.Matrix4d(*[x for row in meta['T_world_from_usd_camera_row'] for x in row])
            optical=Gf.Matrix4d(1,0,0,0,0,-1,0,0,0,0,-1,0,0,0,0,1)*usd
            cams.append({'camera_id':cid,'width':w,'height':h,'k':[fx,0.,w/2,0.,fy,h/2,0.,0.,1.],
                         'T_world_optical_row':stage1_scene.matrix_rows(optical),'original_camera':meta})
            transforms.append(dict(parent='world',child=cid+'/optical',**pose(optical)))
        route=json.loads((C/'waypoints.json').read_text());trajectory=waypoint_motion.prepare(route)
        drone=UsdGeom.Xformable(stage.GetPrimAtPath('/World/drone_1'));drone.ClearXformOpOrder()
        move=drone.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'collection');move.Set(Gf.Vec3d(0))
        drone.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble,'collection').Set(180.);drone.SetResetXformStack(True)
        tc=stage.GetPrimAtPath('/World/drone_1/base_link/TrackingCenter')
        zero=UsdGeom.XformCache().GetLocalToWorldTransform(tc).ExtractTranslation()
        move.Set(Gf.Vec3d(*route['waypoints_m'][0])-zero)
        physics=UsdPhysics.Scene.Define(stage,'/Stage1CollectionClock');physics.CreateGravityMagnitudeAttr(0.)
        api=PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim());api.CreateTimeStepsPerSecondAttr(60)
        api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP')
        SimulationManager.set_default_physics_scene('/Stage1CollectionClock');SimulationManager.set_physics_sim_device('cpu')
        for cam in cams:
            p=rep.create.render_product('/MRO_Cameras/'+cam['camera_id'],(cam['width'],cam['height']))
            annotations=[rep.AnnotatorRegistry.get_annotator(n) for n in ('rgb','ReferenceTime','IsaacReadSimulationTime')]
            annotations[-1].initialize(resetOnStop=False)
            for a in annotations:a.attach([p])
            products.append((cam,p,annotations))
        lidar=Reader(stage)
        for parent,child,key in [('drone_1/tracking_center','drone_1/base_link','T_center_from_base_row'),('drone_1/base_link','drone_1/lidar','T_base_from_sensor_row')]:
            matrix=Gf.Matrix4d(*[x for row in lidar.metadata[key] for x in row]);transforms.append(dict(parent=parent,child=child,**pose(matrix)))
        await omni.kit.app.get_app().next_update_async();SimulationManager.initialize_physics();timeline.pause()
        for k,v in {'/app/asyncRendering':False,'/app/asyncRenderingLowLatency':False,
                    '/rtx/materialDb/syncLoads':True,'/rtx/hydra/materialSyncLoads':True}.items():settings.set(k,v)
        report['synchronous_render_settings']={k:settings.get(k) for k in ('/app/asyncRendering','/app/asyncRenderingLowLatency','/rtx/materialDb/syncLoads','/rtx/hydra/materialSyncLoads')}
        await rep.orchestrator.step_async(delta_time=0.,rt_subframes=1,pause_timeline=True)
        manual=ManualRendering([p for _,p,_ in products]+[lidar.product])
        lidar_events=[];capture_lidar=False;lidar_seen=set()
        def on_frame(path):
            if not capture_lidar or path!=str(lidar.product.path):return
            meta,parsed=lidar.read()
            if not parsed['available']:return
            key=(int(parsed['frame_id']),int(parsed['packet_timestamp_ns']))
            if key in lidar_seen:return
            lidar_seen.add(key)
            pc,body=pointcloud(parsed)
            assert len(lidar_events)<128
            lidar_events.append((meta,pc,body,int(parsed['packet_timestamp_ns'])))
        manual.after_frame=on_frame
        async def render():
            if (case/'abort_requested.json').exists():raise RuntimeError('Acquisition abort requested')
            await rep.orchestrator.step_async(delta_time=0.,rt_subframes=1,pause_timeline=True)
            manual.check()
        def clock():return {'global_s':float(SimulationManager.get_simulation_time()),'step':int(SimulationManager.get_num_physics_steps())}
        for warm in range(30):
            await render()
            if warm%5==0:save(case/'progress.json',{'phase':'warmup','index':warm,'total':30})
        origin=clock();origin['stamp_ns']=round(origin['global_s']*1e9)
        report.update(cameras=cams,trajectory=trajectory,trajectory_config=route,origin=origin,static_transforms=transforms,
              lidar=lidar.metadata,scene_markers={'enabled':False},drone_board={'enabled':False},
              qr_verification={'status':'not_performed_qr_removed','image_content_time_verified':None},
              time_contract={'camera_header':'observed SimulationManager global seconds rounded to ns; native camera annotators retained; image-content time not checked',
                             'lidar_header':'unaltered native packet timestampNs; not global acquisition time',
                             'bag_record_time':'DDS callback wall time','route_t0_global_s':origin['global_s'],
                             'qr_value':None,'no_flight_dynamics':True,
                             'camera_stride':4,'physics_hz':60,'pointcloud':'valid native elements, original native indices/times; no deskew'},
              state_count=0,image_count=0,lidar_count=0,lidar_unavailable=0)
        stage.GetSessionLayer().Export(str(case/'recording_session.usda'))
        emitter=Emitter(json.loads((case/'publisher_ready.json').read_text())['port']);emitter.offer(dict(report,type='config'))
        save(case/'producer_report.json',plain(report))
        previous=origin
        for i in range(spec['ticks']+1):
            if i:SimulationManager.step(render=False)
            current=clock();assert current['step']==origin['step']+i
            if i:assert abs(current['global_s']-previous['global_s']-1/60)<1e-7
            route_s=current['global_s']-origin['global_s'];sample=waypoint_motion.sample(trajectory,route_s)
            move.Set(Gf.Vec3d(*sample['position_m'])-zero)
            lidar_events.clear();capture_lidar=True
            complete=manual.next_complete()
            await omni.kit.app.get_app().next_update_async()
            await asyncio.wait_for(complete,60)
            manual.check()
            await render();capture_lidar=False;assert clock()==current
            stamp=round(current['global_s']*1e9);gt=pose(UsdGeom.XformCache().GetLocalToWorldTransform(tc))
            assert np.linalg.norm(np.array(gt['position'])-sample['position_m'])<1e-7
            base={'index':i,'stamp_ns':stamp,'clock':current,'route_s':route_s,'epoch':spec['epoch']}
            emitter.offer(dict(base,type='state',gt_center=gt,velocity_world_m_s=sample['velocity_m_s']))
            report['state_count']+=1
            for meta,pc,body,packet_ns in lidar_events:
                emitter.offer(dict(base,type='lidar',native_header_ns=packet_ns,pointcloud=pc,
                                   native=meta,lidar_acquisition_time_verified=False,
                                   observation_phase='native NEW_FRAME before frozen camera completion; observation is not acquisition certification'),body);report['lidar_count']+=1
            if not lidar_events:report['lidar_unavailable']+=1
            if i%4==0:
                for cam,p,(rgb,ref,native) in products:
                    pixels=np.ascontiguousarray(np.asarray(rgb.get_data())[:,:,:3])
                    assert pixels.dtype==np.uint8 and pixels.shape==(cam['height'],cam['width'],3)
                    body=pixels.tobytes();cid=cam['camera_id'];rawref=plain(ref.get_data())
                    ref_s=float(core.get_sim_time_at_time((int(rawref['referenceTimeNumerator']),int(rawref['referenceTimeDenominator']))))
                    h=dict(base,type='image',camera_id=cid,reference=rawref,reference_sim_s=ref_s,native_simulation_time=plain(native.get_data()))
                    if spec['mode']=='smoke':h['source_sha256']=hashlib.sha256(body).hexdigest()
                    emitter.offer(h,body);report['image_count']+=1
            previous=current
            if i%20==0:
                save(case/'progress.json',{'phase':'recording','index':i,'ticks':spec['ticks'],'global_s':current['global_s'],
                     'lidar_packets':report['lidar_count'],'images':report['image_count'],'queue_bytes':emitter.bytes})
                save(case/'producer_report.json',plain(report))
        emitter.offer({'type':'end','state_count':report['state_count'],'image_count':report['image_count'],'lidar_count':report['lidar_count']});emitter.finish()
        report.update(status='complete',final_clock=clock(),queue_peak_bytes=emitter.peak,ipc_records=emitter.sent)
    except BaseException as exc:
        report.update(status='failed',error=repr(exc),traceback=traceback.format_exc());raise
    finally:
        timeline.pause();errors=[]
        if manual:report['render_events']=manual.events;manual.close()
        for cam,p,annotations in products:
            for index,a in enumerate(annotations):
                try:a.detach([p.path])
                except Exception as exc:errors.append({'camera':cam['camera_id'],'annotator':index,'error':repr(exc)})
        if lidar:
            for index,a in enumerate(lidar.annotators):
                try:a.detach([lidar.product.path])
                except Exception as exc:errors.append({'lidar_annotator':index,'error':repr(exc)})
        for cam,p,annotations in products:
            try:p.destroy()
            except Exception as exc:errors.append(repr(exc))
        if lidar:
            try:lidar.product.destroy();lidar.closed=True
            except Exception as exc:errors.append(repr(exc))
        if emitter and emitter.thread.is_alive():
            try:emitter.finish()
            except Exception as exc:errors.append(repr(exc))
        stage.GetSessionLayer().Clear();stage.GetSessionLayer().ImportFromString(original)
        timeline.set_current_time(0.);viewport.updates_enabled=old_view
        report.update(cleanup_errors=errors,session_layer_restored=True,finished_wall_ns=str(time.time_ns()))
        save(case/'producer_report.json',plain(report))
    return {'status':report['status'],'case':str(case),'lidar_acquisition_time_verified':False}
