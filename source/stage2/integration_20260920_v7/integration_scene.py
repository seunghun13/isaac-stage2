"""Stage2 session-only USD motion. No propulsion, wheel drive, or pose timing inference."""
import json,math
import numpy as np
from integration_common import M,C

def sample(robot,t,z):
    t=max(0.,float(t))
    if robot=='drone':
        pts=[(5,0,.1665),(5,0,5),(5,10,5),(5,0,5),(5,0,.1665)]
        for a,b in zip(pts,pts[1:]):
            a=np.array(a);b=np.array(b);d=np.linalg.norm(b-a)
            if t<d:return (a+(b-a)*t/d).tolist(),((b-a)/d).tolist(),180.
            t-=d
        return list(pts[-1]),[0.,0.,0.],180.
    pts=[(2,0,z),(2,6,z),(8,6,z),(8,0,z),(2,0,z)]
    for i,(a,b) in enumerate(zip(pts,pts[1:])):
        yaw=90.-90*i;a=np.array(a);b=np.array(b)
        if t<6:return (a+(b-a)*t/6).tolist(),((b-a)/6).tolist(),yaw
        t-=6
        if t<1:return b.tolist(),[0.,0.,0.],yaw-90*t
        t-=1
    return list(pts[-1]),[0.,0.,0.],-270.

def pose(m):
    q=m.ExtractRotationQuat();return {'position':list(m.ExtractTranslation()),'quaternion_wxyz':[q.GetReal(),*q.GetImaginary()]}
def rows(m):return [[float(m[i][j]) for j in range(4)] for i in range(4)]

class Scene:
    def __init__(self,stage,epoch):
        from pxr import Gf,Usd,UsdGeom,UsdPhysics
        from noqr_guard import assert_qr_free_stage
        assert_qr_free_stage(stage)
        self.stage=stage;self.ops={};self.centers={};self.boards={}
        stage.SetEditTarget(stage.GetSessionLayer())
        for layer in stage.GetUsedLayers():
            if layer.realPath:layer.SetPermissionToSave(False)
        stage.OverridePrim('/World/rover_1').SetActive(True)
        rover=stage.GetPrimAtPath('/World/rover_1');assert rover and rover.IsActive()
        for name in ('Physics','Sensors','Sensor'):
            v=rover.GetVariantSets().GetVariantSet(name)
            if 'None' in v.GetVariantNames():v.SetVariantSelection('None')
        for p in list(Usd.PrimRange(rover)):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(p).CreateRigidBodyEnabledAttr(False)
            if p.IsA(UsdPhysics.Joint):UsdPhysics.Joint(p).CreateJointEnabledAttr(False)
            if any(x in p.GetTypeName().lower() for x in ('lidar','imu')):p.SetActive(False)
        definition=json.loads((M/'config/tracking_centers.json').read_text())['definitions']['rover_1']
        tc=UsdGeom.Xform.Define(stage,definition['prim_path']);tc.ClearXformOpOrder()
        tc.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'stage2center').Set(Gf.Vec3d(*definition['offset_base_m']))
        tc.GetPrim().SetCustomDataByKey('stage2:definition_sha256',definition['definition_sha256'])
        self.rover_z=float(UsdGeom.XformCache(Usd.TimeCode(0)).GetLocalToWorldTransform(tc.GetPrim()).ExtractTranslation()[2])
        assert .1<self.rover_z<.5,self.rover_z
        for robot in ('drone','rover'):
            root=UsdGeom.Xformable(stage.GetPrimAtPath('/World/'+robot+'_1'));root.ClearXformOpOrder();root.SetResetXformStack(True)
            tr=root.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'stage2motion');yaw=root.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble,'stage2motion')
            self.ops[robot]=(tr,yaw);self.centers[robot]=stage.GetPrimAtPath('/World/'+robot+'_1/base_link/TrackingCenter')
        self.move(0.)
        active=[p.GetName() for p in stage.GetPrimAtPath('/World').GetChildren() if p.IsActive() and p.GetName().startswith(('drone_','rover_'))]
        assert sorted(active)==['drone_1','rover_1'],active
        assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get() for p in stage.Traverse())
        self.metadata={'rover_center_z_m':self.rover_z,'rover_height_source':'original fixed TrackingCenter USD readback at time 0 before Stage2 movement',
            'active_robots':active,'propulsion':False,'trajectory_velocity_is_physical':False,'rigid_bodies_enabled':False,'qr_lidar_occlusion_possible':False,'qr_boards_enabled':False,
            'boards':{k:b.metadata for k,b in self.boards.items()},'rover_center_definition':definition}
    def move(self,t):
        from pxr import Gf,UsdGeom
        out={}
        for robot,(tr,yaw) in self.ops.items():
            p,v,a=sample(robot,t,self.rover_z);yaw.Set(a);tr.Set(Gf.Vec3d(0))
            zero=UsdGeom.XformCache().GetLocalToWorldTransform(self.centers[robot]).ExtractTranslation();tr.Set(Gf.Vec3d(*p)-zero)
            actual=pose(UsdGeom.XformCache().GetLocalToWorldTransform(self.centers[robot]));assert math.dist(actual['position'],p)<1e-7
            out[robot]={'gt_center':actual,'velocity_world_m_s':v,'yaw_deg':a}
        return out

def rover_reader(stage):
    import omni.replicator.core as rep
    from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
    from pxr import Gf,Sdf,UsdGeom
    from record_lidar import Reader
    from rover_lidar_adapter import profile_attributes,PROFILE
    r=Reader.__new__(Reader);r.stage=stage;r.closed=False
    mount=UsdGeom.Xform.Define(stage,'/World/rover_1/base_link/Stage2LidarMount');mount.AddTranslateOp().Set(Gf.Vec3d(0,0,.60))
    sensor=stage.DefinePrim(str(mount.GetPath())+'/Sensor','OmniLidar')
    sensor.SetMetadata('apiSchemas',Sdf.TokenListOp.Create(prependedItems=['OmniSensorGenericLidarCoreAPI']))
    for name,(typ,value) in profile_attributes().items():assert sensor.CreateAttribute(name,getattr(Sdf.ValueTypeNames,typ),custom=False).Set(value)
    sensor.CreateAttribute('omni:sensor:Core:auxOutputType',Sdf.ValueTypeNames.Token,custom=False).Set('NONE')
    cache=UsdGeom.XformCache();base=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/rover_1/base_link'));center=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/rover_1/base_link/TrackingCenter'));native=cache.GetLocalToWorldTransform(sensor)
    r.metadata={'sensor_path':str(sensor.GetPath()),'profile':PROFILE,'frame_id':'rover_1/lidar','T_base_from_sensor_row':rows(native*base.GetInverse()),'T_center_from_base_row':rows(base*center.GetInverse()),'profile_readback':{a.GetName():str(a.Get()) for a in sensor.GetAttributes()}}
    r.product=rep.create.render_product(r.metadata['sensor_path'],(128,128));r.gmo=rep.AnnotatorRegistry.get_annotator('GenericModelOutput',device='cpu');r.ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime');r.sim=rep.AnnotatorRegistry.get_annotator('IsaacReadSimulationTime');r.sim.initialize(resetOnStop=False)
    r.annotators=[r.gmo,r.ref,r.sim]
    for a in r.annotators:a.attach([r.product])
    r.core=_isaacsim_core_nodes.acquire_interface();return r
