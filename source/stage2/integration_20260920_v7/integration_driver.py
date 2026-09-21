"""Finite CPU workers owned by Popen; request issued only after DDS discovery."""
import json,os,subprocess,sys,time,traceback
from pathlib import Path
M=Path('/mnt/DATA/workspace/ws_minho/mro_1');case=Path(sys.argv[1]).resolve(strict=True);out=case
assert case.is_relative_to(M/'stage2/outputs')
sys.path[:0]=[str(M)]
from mro_runtime.paths import atomic_replace_json
req=json.loads((case/'request.json').read_text());run=M/'stage2/runtime/run'/req['expected_launch_id']
bridge=M/'issacsim/exts/isaacsim.ros2.bridge/humble'
env={'PATH':'/usr/bin:/bin','HOME':str(out/'home'),'TMPDIR':str(out/'tmp'),'XDG_CACHE_HOME':str(out/'cache'),'XDG_CONFIG_HOME':str(out/'home'),'XDG_DATA_HOME':str(out/'home'),'ROS_HOME':str(out/'home'),'ROS_LOG_DIR':str(out/'logs'),'PYTHONDONTWRITEBYTECODE':'1','PYTHONNOUSERSITE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','PYTHONPATH':str(M/'runtime/tools/rosbags-0.11.5-py311/site')+':'+str(bridge/'rclpy')+':'+str(M/'issacsim/exts/omni.pip.compute/pip_prebundle')+':'+str(M/'scripts'),'LD_LIBRARY_PATH':str(bridge/'lib')+':'+str(M/'issacsim/kit/python/lib'),'CUDA_VISIBLE_DEVICES':'','HISTFILE':'','LANG':'C.UTF-8','LC_ALL':'C.UTF-8','ROS_DISTRO':'humble','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp','ROS_DOMAIN_ID':'83','FASTRTPS_DEFAULT_PROFILES_FILE':str(M/'stage1/validation_20260918_v1/fastdds_local_udp.xml'),'FASTDDS_DEFAULT_PROFILES_FILE':str(M/'stage1/validation_20260918_v1/fastdds_local_udp.xml'),'RMW_FASTRTPS_USE_QOS_FROM_XML':'1','RMW_FASTRTPS_PUBLICATION_MODE':'SYNCHRONOUS'}
workers=[];logs=[];report={'status':'starting','request':req,'workers':[]}
try:
 for role,ready in [('subscribe','subscriber_ready.json'),('publish','publisher_ready.json')]:
  log=(out/(role+'.log')).open('xb');logs.append(log)
  args=['/usr/bin/bwrap','--ro-bind','/','/','--ro-bind','/dev/shm','/dev/shm','--bind',str(out),str(out),'--proc','/proc','--unshare-pid','--die-with-parent','--new-session','--chdir',str(out),'--',str(M/'issacsim/kit/python/bin/python3'),'-B',str(M/'stage2/integration_20260920_v7/integration_worker.py'),role,str(case)]
  p=subprocess.Popen(args,cwd=out,env=env,stdout=log,stderr=subprocess.STDOUT);workers.append(p);report['workers'].append({'role':role,'pid':p.pid})
  end=time.monotonic()+60
  while not (out/ready).exists():
   assert p.poll() is None,role+' exited';assert time.monotonic()<end,role+' readiness timeout';time.sleep(.2)
 assert json.loads((run/'stage_status.json').read_text())['status']=='stage2_open' and not (run/'request.json').exists()
 atomic_replace_json(M,run/'request.json',req);report['status']='running';atomic_replace_json(M,out/'driver_progress.json',report)
 end=time.monotonic()+json.loads((case/'spec.json').read_text())['wall_timeout_s']+600
 while any(p.poll() is None for p in workers):
  assert time.monotonic()<end,'transport deadline';assert not (case/'abort_requested.json').exists(),'aborted'
  for p in workers:
   if p.poll() not in [None,0]:raise RuntimeError('transport worker failed')
  time.sleep(.5)
 pub=json.loads((out/'publisher_done.json').read_text());sub=json.loads((out/'subscriber_done.json').read_text())
 assert pub['status']==sub['status']=='complete' and pub['counts']==sub['counts']
 report.update(status='complete',counts=pub['counts'])
except BaseException:
 report.update(status='failed',traceback=traceback.format_exc())
 atomic_replace_json(M,case/'abort_requested.json',{'reason':'transport failed'})
finally:
 for p in workers:
  if p.poll() is None:
   p.terminate()
   try:p.wait(timeout=8)
   except subprocess.TimeoutExpired:p.kill();p.wait(timeout=8)
 for log in logs:log.close()
 report['returncodes']=[p.poll() for p in workers];atomic_replace_json(M,out/'driver_report.json',report)
