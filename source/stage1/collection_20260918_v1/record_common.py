"""Bounded Stage1 recording protocol; no simulation or ROS imports."""
import json,os,struct
from pathlib import Path

M=Path('/mnt/DATA/workspace/ws_minho/mro_1')
S=M/'stage1'
C=S/'collection_20260918_v1'
CAMERAS=('cam_01','cam_02','cam_03')
SIZES={'cam_01':(3840,2160),'cam_02':(5320,4600),'cam_03':(3840,2160)}
MAX_BODY=80*1024**2
MAX_HEADER=2*1024**2
MAX_QUEUE_BYTES=256*1024**2

def safe(value):
    p=Path(value)
    if not p.is_absolute() or p.resolve()!=p or not p.is_relative_to(S):raise ValueError('Stage1 path required')
    return p

def save(p,value):
    p=safe(p);tmp=safe(p.with_name(p.name+'.tmp.'+str(os.getpid())))
    b=json.dumps(value,allow_nan=False,separators=(',',':')).encode()
    with tmp.open('xb') as f:f.write(b)
    os.replace(tmp,p)

def frame_parts(header,body=b''):
    if not isinstance(body,bytes) or len(body)>MAX_BODY:raise ValueError('Bounded immutable body required')
    data=json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if len(data)>MAX_HEADER:raise ValueError('Header byte bound')
    return struct.pack('<QI',len(body)+len(data)+4,len(data)),data,body

def receive(sock):
    def exact(n):
        b=bytearray(n);v=memoryview(b);at=0
        while at<n:
            got=sock.recv_into(v[at:])
            if not got:raise EOFError('Truncated recording IPC')
            at+=got
        return b
    total=struct.unpack('<Q',exact(8))[0]
    if not 6<=total<=MAX_BODY+MAX_HEADER+4:raise ValueError('Record bound')
    size=struct.unpack('<I',exact(4))[0]
    if not 2<=size<=MAX_HEADER or size>total-4 or total-4-size>MAX_BODY:raise ValueError('Header/body bound')
    return json.loads(exact(size)),exact(total-4-size)

def topics():
    d={'session':('/stage1/session','std_msgs/msg/String'),
       'waypoints':('/stage1/drone/waypoints','std_msgs/msg/String'),
       'static':('/tf_static','tf2_msgs/msg/TFMessage'),
       'clock':('/clock','rosgraph_msgs/msg/Clock'),
       'gt_pose':('/stage1/drone/gt_pose','geometry_msgs/msg/PoseStamped'),
       'gt_velocity':('/stage1/drone/gt_velocity','geometry_msgs/msg/Vector3Stamped'),
       'lidar':('/stage1/drone/lidar/points_raw','sensor_msgs/msg/PointCloud2'),
       'mapping':('/stage1/time_mapping','std_msgs/msg/String')}
    for c in CAMERAS:
        d[c+'_image']=('/stage1/cameras/'+c+'/image_raw','sensor_msgs/msg/Image')
        d[c+'_info']=('/stage1/cameras/'+c+'/camera_info','sensor_msgs/msg/CameraInfo')
    assert len(d)==14
    return d

def validate_spec(x):
    if x.get('schema')!='stage1_recording_v1':raise ValueError('Recording spec schema')
    if x['mode'] not in ('smoke','pilot','route'):raise ValueError('Mode')
    if type(x['ticks']) is not int or x['ticks'] not in (12,60,300,1800):raise ValueError('Bounded tick count')
    if x['camera_stride']!=4 or x['physics_hz']!=60:raise ValueError('Fixed first collection schedule')
    if x['resolution'] not in ('native','probe'):raise ValueError('Resolution')
    if type(x['wall_timeout_s']) is not int or not 180<=x['wall_timeout_s']<=7200:raise ValueError('Finite wall bound')
    if x['color_plane_enabled'] is not False or x['lidar_enabled'] is not True:raise ValueError('Required sensor scope')
    if x.get('qr_boards_enabled') is not False:raise ValueError('This version requires explicit qr_boards_enabled=false')
    if not 0<=x['epoch']<2**32:raise ValueError('Epoch')
    safe(x['case'])
    return x
