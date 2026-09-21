"""Bounded Stage2 recording protocol; no simulation or ROS imports."""
import json,os,struct
from pathlib import Path

M=Path('/mnt/DATA/workspace/ws_minho/mro_1')
S=M/'stage2'
C=S/'integration_20260920_v7'
CAMERAS=('cam_01','cam_02','cam_03')
SIZES={'cam_01':(3840,2160),'cam_02':(5320,4600),'cam_03':(3840,2160)}
MAX_BODY=80*1024**2
MAX_HEADER=2*1024**2
MAX_QUEUE_BYTES=256*1024**2

def safe(value):
    p=Path(value)
    if not p.is_absolute() or p.resolve()!=p or not p.is_relative_to(S):raise ValueError('Stage2 path required')
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
    d={'session':('/stage2/session','std_msgs/msg/String'),'static':('/tf_static','tf2_msgs/msg/TFMessage'),
       'clock':('/clock','rosgraph_msgs/msg/Clock'),'mapping':('/stage2/time_mapping','std_msgs/msg/String')}
    for robot in ('drone','rover'):
        for key,suffix,typ in [('waypoints','waypoints','std_msgs/msg/String'),('gt_pose','gt_pose','geometry_msgs/msg/PoseStamped'),('gt_velocity','gt_velocity','geometry_msgs/msg/Vector3Stamped'),('lidar','lidar/points_raw','sensor_msgs/msg/PointCloud2')]:
            d[robot+'_'+key]=('/stage2/'+robot+'/'+suffix,typ)
    for c in CAMERAS:
        d[c+'_image']=('/stage2/cameras/'+c+'/image_raw','sensor_msgs/msg/Image')
        d[c+'_info']=('/stage2/cameras/'+c+'/camera_info','sensor_msgs/msg/CameraInfo')
    assert len(d)==18
    return d

def validate_spec(x):
    assert x['schema']=='stage2_integration_v7' and x['completion_cycles']==2
    assert (x['ticks'],x['route_offset_s']) in [(60,0),(300,4),(1800,0)]
    assert x['camera_stride']==4 and x['physics_hz']==60 and x['resolution']=='native'
    assert x['qr_boards_enabled'] is False and x['color_plane_enabled'] is False
    assert 180<=x['wall_timeout_s']<=5400 and 0<=x['epoch']<2**32
    safe(x['case']);return x
