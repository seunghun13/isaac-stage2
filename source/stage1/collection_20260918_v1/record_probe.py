"""Finite raw LiDAR clock probe in a fresh, owned Stage1 session."""
import asyncio
import json
import os
from pathlib import Path
import time
import traceback

S=Path('/mnt/DATA/workspace/ws_minho/mro_1/stage1')

def plain(value):
    import numpy as np
    if isinstance(value,dict):return {k:plain(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [plain(v) for v in value]
    if isinstance(value,np.ndarray):return plain(value.tolist())
    if isinstance(value,np.generic):return plain(value.item())
    if isinstance(value,int) and abs(value)>2**53:return str(value)
    return value

def save(path,value):
    assert path.resolve()==path and path.is_relative_to(S)
    b=json.dumps(plain(value),allow_nan=False).encode()
    temp=path.with_suffix('.tmp');assert temp.resolve()==temp
    temp.write_bytes(b);temp.replace(path)

async def run(stage,timeline,viewport,request):
    import carb,omni.kit.app,omni.replicator.core as rep
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema
    from isaacsim.core.simulation_manager import SimulationManager
    from record_lidar import Reader
    import stage1_scene
    case=Path(request['case']);assert case.resolve()==case and case.is_relative_to(S/'outputs')
    run=Path(os.environ['MRO_VIEW_RUN'])
    with (run/'collection_consumed.json').open('x') as f:json.dump({'case':str(case)},f)
    original=stage.GetSessionLayer().ExportToString();old_view=viewport.updates_enabled
    reader=None;manual=None;report={'status':'running','rows':[],'request':request,'started_wall_ns':str(time.time_ns())}
    try:
        assert not timeline.is_playing()
        observed=stage1_scene.inspect(stage,0)
        assert not observed['enabled_rigid_bodies'] and not observed['active_physics_scenes']
        viewport.updates_enabled=False
        settings=carb.settings.get_settings()
        for k,v in {'/app/hydraEngine/waitIdle':True,'/app/updateOrder/checkForHydraRenderComplete':1000,'/rtx/ecoMode/enabled':False}.items():settings.set(k,v)
        physics=UsdPhysics.Scene.Define(stage,'/Stage1CollectionClock');physics.CreateGravityMagnitudeAttr(0.)
        api=PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim())
        api.CreateTimeStepsPerSecondAttr(60);api.CreateEnableGPUDynamicsAttr(False);api.CreateBroadphaseTypeAttr('MBP')
        SimulationManager.set_default_physics_scene('/Stage1CollectionClock');SimulationManager.set_physics_sim_device('cpu')
        reader=Reader(stage);report['lidar']=reader.metadata
        await omni.kit.app.get_app().next_update_async()
        SimulationManager.initialize_physics();timeline.pause()
        def clock():return {'global_s':float(SimulationManager.get_simulation_time()),'step':int(SimulationManager.get_num_physics_steps()),'timeline_s':timeline.get_current_time()}
        render_delta=float(request.get('render_delta_s',0.))
        method=request.get('render_method','replicator')
        assert method in ('replicator','app_update')
        assert render_delta in (0.,1/60.)
        report['render_delta_s']=render_delta
        report['render_method']=method
        if method=='app_update':
            from record_render import ManualRendering
            manual=ManualRendering([reader.product])
        async def render(delta=None):
            if (case/'abort_requested.json').exists():raise RuntimeError('owned probe abort')
            if method=='app_update':await omni.kit.app.get_app().next_update_async()
            else:await rep.orchestrator.step_async(delta_time=render_delta if delta is None else delta,rt_subframes=1,pause_timeline=True)
            if manual:manual.check()
        for _ in range(20):await render()
        report['origin']=clock()
        def observe(role,index):
            meta,parsed=reader.read()
            row={'role':role,'index':index,'clock':clock(),'lidar':meta,'wall_ns':str(time.time_ns())}
            report['rows'].append(plain(row))
            save(case/'probe_report.json',report)
            save(case/'progress.json',{'phase':role,'index':index,'native_frame':meta.get('frame_id'),'global_s':clock()['global_s']})
        observe('initial',0)
        for i in range(1,61):
            SimulationManager.step(render=False)
            await render()
            observe('advance',i)
            if i==20:
                await asyncio.sleep(1.0);observe('wall_pause_sensor_enabled',i)
                await render(0.);observe('frozen_render_sensor_enabled',i)
            if i==40:
                reader.enabled(False)
                await asyncio.sleep(1.0);observe('wall_pause_sensor_disabled',i)
                reader.enabled(True)
                await render(0.);observe('frozen_render_after_enable',i)
        report.update(status='complete',final_clock=clock(),finished_wall_ns=str(time.time_ns()))
    except BaseException as exc:
        report.update(status='failed',error=repr(exc),traceback=traceback.format_exc())
        raise
    finally:
        timeline.pause()
        if manual:
            report['render_events']=manual.events
            manual.close()
        report['cleanup_errors']=reader.close() if reader else []
        stage.GetSessionLayer().Clear();stage.GetSessionLayer().ImportFromString(original)
        timeline.set_current_time(0.);viewport.updates_enabled=old_view
        report['session_layer_restored']=True
        save(case/'probe_report.json',report)
    return {'case':str(case),'status':report['status'],'observations':len(report['rows'])}
