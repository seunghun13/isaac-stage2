"""Isolated Stage2 QR-free capture workbench."""
import asyncio, datetime, json, os, sys, traceback
from pathlib import Path
import carb, omni.kit.app, omni.timeline, omni.usd
M=Path('/mnt/DATA/workspace/ws_minho/mro_1')
S=M/'stage2'
RUN=Path(os.environ['MRO_VIEW_RUN']).resolve(strict=True)
assert RUN.is_relative_to(S/'runtime/run')
sys.path[:0]=[str(S/'integration_20260920_v7'),str(S/'scripts'),str(M/'stage1/collection_20260918_v1'),str(M/'stage1/scripts'),str(M/'scripts'),str(M)]
from mro_runtime.paths import atomic_replace_json, read_json
def status(**v):
    atomic_replace_json(M,RUN/'stage_status.json',dict(v,utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),launch_id=os.environ['MRO_LAUNCH_ID']))
async def main():
    app=omni.kit.app.get_app(); timeline=omni.timeline.get_timeline_interface()
    try:
        timeline.stop()
        settings=carb.settings.get_settings()
        for k,v in {'/app/file/ignoreUnsavedOnExit':True,'/omni/replicator/captureOnPlay':False,'/persistent/omni/replicator/captureOnPlay':False,'/rtx/rendermode':'RaytracedLighting','/rtx/post/aa/op':3}.items(): settings.set(k,v)
        # Let Kit finish extension startup before creating the diagnostic stage.
        for _ in range(20): await app.next_update_async()
        assert await omni.usd.get_context().open_stage_async(str(S/'integration_20260920_v7/scene.usda'))
        for _ in range(10): await app.next_update_async()
        status(status='stage2_open',recording=False,timeline_playing=False,integrated_scene=True)
        consumed=False
        while app.is_running():
            await app.next_update_async()
            if consumed or not (RUN/'request.json').exists(): continue
            req=read_json(M,RUN/'request.json'); consumed=True
            assert req['action']=='stage2_integrated' and req['expected_launch_id']==os.environ['MRO_LAUNCH_ID']
            case=Path(req['case']).resolve(strict=True)
            assert case.is_relative_to(S/'outputs')
            status(status='integrated_recording',recording=True,timeline_playing=False,case=str(case))
            try:
                from integration_runtime import execute
                result=await execute(case,req)
            except Exception:
                result={'status':'failed','traceback':traceback.format_exc()}
            atomic_replace_json(M,case/'result.json',result)
            atomic_replace_json(M,RUN/'response.json',dict(request_id=req['request_id'],result=result))
            timeline.pause()
            status(status='integration_complete',recording=False,timeline_playing=False,case=str(case),result_status=result['status'])
    except Exception:
        status(status='bootstrap_failed',recording=False,timeline_playing=False,traceback=traceback.format_exc())
asyncio.ensure_future(main())
