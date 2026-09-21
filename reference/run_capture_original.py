import json,sys,uuid
from ops import call,D
label=sys.argv[1];assert label.isalnum();ticks=int(sys.argv[2]);offset=int(sys.argv[3]);assert (ticks,offset) in [(60,0),(300,4),(1800,0)]
uid=uuid.uuid4().hex
spec={'schema':'stage2_integration_v7','completion_cycles':2,'ticks':ticks,'route_offset_s':offset,'epoch':int(uid[:8],16),'resolution':'native','camera_stride':4,'physics_hz':60,'wall_timeout_s':5400 if ticks==1800 else 1800,'qr_boards_enabled':False,'color_plane_enabled':False}
src='LABEL='+repr('integrated7_noqr_'+label+'_'+uid[:10])+'\nSPEC='+repr(spec)+'\nUID='+repr(uid)+'\n'+'''
from pathlib import Path
import json,sys,subprocess,hashlib
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');S=M/'stage2';C=S/'integration_20260920_v7';sys.path[:0]=[str(S/'scripts'),str(M),str(M/'scripts')]
from stage2_storage_budget import check_storage_budget
cfg=json.loads((S/'config/viewing_session.json').read_text());launch=cfg['consumed_by_launch_id'];run=S/'runtime/run'/launch
assert json.loads((run/'stage_status.json').read_text())['status']=='stage2_open' and not (run/'request.json').exists()
assert C.resolve(strict=True)==C and (S/'outputs').resolve(strict=True)==S/'outputs'
for rel,sha in json.loads((C/'SOURCE_MANIFEST.json').read_text()).items():assert hashlib.sha256((M/rel).read_bytes()).hexdigest()==sha,rel
if SPEC['ticks']==1800:
 readiness=json.loads((S/'manifests/noqr_stage2_30s_readiness.json').read_text())
 assert readiness['release_sha256']==hashlib.sha256((C/'SOURCE_MANIFEST.json').read_bytes()).hexdigest() and readiness['short_regression_pass'] and readiness['replay_pass'] and readiness['within_original_budget']
 assert readiness['qr_free_final_collection_authorized'] and readiness['accepted_scope']=='global_metadata_and_calibrated_60hz_scene_step'
 assert readiness['qr_removal_regression_pass'] and not readiness['fine_global_beam_emission_verified']
budget=check_storage_budget(M,incoming_bytes=(70 if SPEC['ticks']==1800 else 16)*1024**3)
case=S/'outputs'/LABEL;case.mkdir(mode=0o700)
for d in ['home','tmp','cache','logs']:(case/d).mkdir()
spec=dict(SPEC,case=str(case),launch_id=launch);(case/'spec.json').write_text(json.dumps(spec));(case/'budget.json').write_text(json.dumps(budget))
req={'action':'stage2_integrated','request_id':UID,'case':str(case),'expected_launch_id':launch};(case/'request.json').write_text(json.dumps(req))
env={'PATH':'/usr/bin:/bin','HOME':str(case/'home'),'TMPDIR':str(case/'tmp'),'PYTHONDONTWRITEBYTECODE':'1','PYTHONNOUSERSITE':'1','HISTFILE':'','LANG':'C.UTF-8','LC_ALL':'C.UTF-8'}
with (case/'driver.log').open('xb') as log:
 p=subprocess.Popen(['/usr/bin/python3','-B',str(C/'integration_driver.py'),str(case)],cwd=case,env=env,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
req['driver_pid']=p.pid;print(json.dumps(req))
'''
print(json.dumps(call(src,label+'_request.json')))
