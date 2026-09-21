"""Native rclpy publishing and independent raw-CDR rosbag2 recording."""
import array,hashlib,json,os,socket,sys,time,traceback
from integration_common import safe,save,receive,topics,validate_spec

def main():
    role,directory=sys.argv[1:3];case=safe(directory)
    spec=validate_spec(json.loads((case/'spec.json').read_text()))
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2,PointField
    from geometry_msgs.msg import PoseStamped,Vector3Stamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    classes={c.__name__:c for c in (Image,CameraInfo,PointCloud2,PoseStamped,Vector3Stamped,TFMessage,Clock,String)}
    specs=topics()
    def qos(key):
        return QoSProfile(depth=2 if key.endswith('_image') else 128,reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL if (key in ('static','session') or key.endswith('_waypoints')) else DurabilityPolicy.VOLATILE)
    rclpy.init(args=[]);node=rclpy.create_node('stage2_integrated_'+role)
    counts={k:0 for k in specs};report={'status':'running','role':role,'pid':os.getpid(),'counts':counts}
    def stamp(m,ns,frame):
        m.header.stamp.sec,m.header.stamp.nanosec=divmod(int(ns),10**9);m.header.frame_id=frame
    try:
        if role=='publish':
            pubs={k:node.create_publisher(classes[t.split('/')[-1]],topic,qos(k)) for k,(topic,t) in specs.items()}
            deadline=time.monotonic()+60
            while not all(p.get_subscription_count() for p in pubs.values()):
                if time.monotonic()>deadline:raise TimeoutError('DDS discovery')
                rclpy.spin_once(node,timeout_sec=.05)
            listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(240)
            save(case/'publisher_ready.json',{'port':listener.getsockname()[1],'discovered':{k:p.get_subscription_count() for k,p in pubs.items()}})
            conn,peer=listener.accept();assert peer[0]=='127.0.0.1';conn.settimeout(180)
            config,body=receive(conn);assert config['type']=='config' and not body
            publish_log=(case/'published.jsonl').open('x')
            from rclpy.serialization import serialize_message
            def publish(k,msg):
                raw=serialize_message(msg);pubs[k].publish(raw)
                publish_log.write(json.dumps({'key':k,'sequence_on_topic':counts[k],'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest(),'sha256':hashlib.sha256(raw).hexdigest(),'prefix_sha256':hashlib.sha256(raw[:8+int.from_bytes(raw[4:8],'little')]).hexdigest() if isinstance(msg,String) else None})+'\n')
                publish_log.flush();counts[k]+=1
            def string(k,item):
                msg=String();msg.data=json.dumps(item,allow_nan=False,separators=(',',':'));publish(k,msg)
            string('session',config)
            for robot in ('drone','rover'):string(robot+'_waypoints',config['trajectories'][robot])
            transforms=[]
            for item in config['static_transforms']:
                msg=TransformStamped();stamp(msg,config['origin']['stamp_ns'],item['parent']);msg.child_frame_id=item['child']
                msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=item['position']
                msg.transform.rotation.w,msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z=item['quaternion_wxyz']
                transforms.append(msg)
            msg=TFMessage();msg.transforms=transforms;publish('static',msg)
            calibrations={c['camera_id']:c for c in config['cameras']}
            while True:
                h,body=receive(conn);kind=h['type']
                if kind=='end':
                    assert not body;report['producer_end']=h;break
                if kind=='state':
                    assert not body;ns=h['stamp_ns']
                    msg=Clock();msg.clock.sec,msg.clock.nanosec=divmod(ns,10**9);publish('clock',msg)
                    for robot in ('drone','rover'):
                        msg=PoseStamped();stamp(msg,ns,'world');p=h['robots'][robot]['gt_center']
                        msg.pose.position.x,msg.pose.position.y,msg.pose.position.z=p['position']
                        msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z=p['quaternion_wxyz'];publish(robot+'_gt_pose',msg)
                        msg=Vector3Stamped();stamp(msg,ns,'world')
                        msg.vector.x,msg.vector.y,msg.vector.z=h['robots'][robot]['velocity_world_m_s'];publish(robot+'_gt_velocity',msg)
                elif kind=='image':
                    c=h['camera_id'];cal=calibrations[c];w,hh=cal['width'],cal['height']
                    assert len(body)==w*hh*3
                    if h.get('source_sha256'):assert hashlib.sha256(body).hexdigest()==h['source_sha256']
                    msg=Image();stamp(msg,h['stamp_ns'],c+'/optical');msg.width=w;msg.height=hh;msg.encoding='rgb8';msg.step=w*3
                    msg.data=array.array('B',body);publish(c+'_image',msg)
                    info=CameraInfo();stamp(info,h['stamp_ns'],c+'/optical');info.width=w;info.height=hh;info.distortion_model='plumb_bob';info.d=[0.]*5
                    k=cal['k'];info.k=k;info.r=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.p=[k[0],0.,k[2],0.,0.,k[4],k[5],0.,0.,0.,1.,0.];publish(c+'_info',info)
                elif kind=='lidar':
                    meta=h['pointcloud'];assert len(body)==meta['row_step']
                    msg=PointCloud2();stamp(msg,h['native_header_ns'],h['sensor']+'_1/lidar')
                    msg.height=1;msg.width=meta['width'];msg.point_step=32;msg.row_step=len(body);msg.is_bigendian=False;msg.is_dense=meta['is_dense']
                    msg.fields=[PointField(name=f['name'],offset=f['offset'],datatype=f['datatype'],count=1) for f in meta['fields']]
                    msg.data=array.array('B',body);publish(h['sensor']+'_lidar',msg)
                elif kind=='completion':assert not body
                else:raise ValueError('Unexpected record '+str(kind))
                h['publisher_wall_ns']=str(time.time_ns());string('mapping',h)
                rclpy.spin_once(node,timeout_sec=0)
            publish_log.close();report.update(status='complete');save(case/'publisher_done.json',report)
            deadline=time.monotonic()+180
            while not (case/'subscriber_done.json').exists() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
            conn.close();listener.close()
        elif role=='subscribe':
            from rosbags.rosbag2 import Writer
            from rosbags.typesys import Stores,get_typestore
            from rosbags.interfaces import Qos,QosHistory,QosReliability,QosDurability,QosLiveliness,QosTime
            writer=Writer(case/'bag',version=8);writer.open()
            store=get_typestore(Stores.ROS2_HUMBLE)
            def offered(k):
                infinite=QosTime(9223372036,854775807)
                return [Qos(QosHistory.KEEP_LAST,2 if k.endswith('_image') else 128,QosReliability.RELIABLE,
                            QosDurability.TRANSIENT_LOCAL if (k in ('static','session') or k.endswith('_waypoints')) else QosDurability.VOLATILE,
                            infinite,infinite,QosLiveliness.AUTOMATIC,infinite,False)]
            connections={k:writer.add_connection(topic,t,typestore=store,offered_qos_profiles=offered(k)) for k,(topic,t) in specs.items()}
            receipts=(case/'receipts.jsonl').open('x')
            def callback(k):
                def receive_raw(raw):
                    wall=time.time_ns();mono=time.monotonic_ns();writer.write(connections[k],wall,raw);counts[k]+=1
                    receipts.write(json.dumps({'key':k,'sequence_on_topic':counts[k]-1,'wall_ns':str(wall),'mono_ns':str(mono),'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()},separators=(',',':'))+'\n')
                return receive_raw
            subs=[node.create_subscription(classes[t.split('/')[-1]],topic,callback(k),qos(k),raw=True) for k,(topic,t) in specs.items()]
            save(case/'subscriber_ready.json',{'pid':os.getpid(),'record_time':'actual DDS callback wall time'})
            done=None;drain=None;deadline=time.monotonic()+spec['wall_timeout_s']+360
            try:
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.05)
                    if done is None and (case/'publisher_done.json').exists():
                        done=json.loads((case/'publisher_done.json').read_text());drain=time.monotonic()+180
                    if done and counts==done['counts']:break
                    if drain and time.monotonic()>drain:break
                    if (case/'abort_requested.json').exists() and not done:break
            finally:writer.close();receipts.close()
            report.update(status='complete' if done and done['status']=='complete' and counts==done['counts'] else 'loss_or_failure',expected=done['counts'] if done else None)
            save(case/'subscriber_done.json',report)
        else:raise ValueError(role)
    except BaseException as exc:
        report.update(status='failed',error=repr(exc),traceback=traceback.format_exc());save(case/(role+'_error.json'),report);raise
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
