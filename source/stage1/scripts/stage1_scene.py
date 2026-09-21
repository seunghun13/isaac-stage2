"""Stage1 scene with authored waypoint motion and the approved native Ouster mount."""
from pathlib import Path
import json,math,hashlib
M=Path('/mnt/DATA/workspace/ws_minho/mro_1')
S=M/'stage1'

def path(rel):
    p=S/rel
    if not p.resolve().is_relative_to(S.resolve(strict=True)) or p.is_symlink():
        raise ValueError('Stage1 output path escaped')
    return p

def write_json(rel,value):
    p=path(rel);p.parent.mkdir(parents=True,exist_ok=True)
    tmp=p.with_suffix(p.suffix+'.tmp');assert not tmp.is_symlink()
    tmp.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf-8');tmp.replace(p)

def matrix_rows(m):return [[float(m[i][j]) for j in range(4)] for i in range(4)]

def inspect(stage,t_seconds=0.):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics
    tc=Usd.TimeCode(t_seconds*stage.GetTimeCodesPerSecond())
    cache=UsdGeom.XformCache(tc)
    cams={}
    for cid in ('cam_01','cam_02','cam_03'):
        camera=UsdGeom.Camera(stage.GetPrimAtPath('/MRO_Cameras/'+cid))
        prim=camera.GetPrim();matrix=cache.GetLocalToWorldTransform(prim)
        cfg=json.loads(prim.GetCustomDataByKey('mro:cameraConfigJson'))
        forward=list(matrix.TransformDir(Gf.Vec3d(0,0,-1)).GetNormalized())
        cams[cid]={'model':cfg['model'],'position_m':list(matrix.ExtractTranslation()),
            'forward_world':forward,'upward_tilt_deg':math.degrees(math.asin(forward[2])),
            'orientation_usd_wxyz':cfg['orientation_wxyz'],'resolution_px':cfg['resolution_px'],
            'intrinsics':cfg['intrinsics'],'T_world_from_usd_camera_row':matrix_rows(matrix),
            'lens':{'focalLength':camera.GetFocalLengthAttr().Get(),'horizontalAperture':camera.GetHorizontalApertureAttr().Get(),'verticalAperture':camera.GetVerticalApertureAttr().Get(),'fStop':camera.GetFStopAttr().Get()}}
    center=stage.GetPrimAtPath('/World/drone_1/base_link/TrackingCenter')
    cm=cache.GetLocalToWorldTransform(center)
    return {'schema':'mro_stage1_scene_readback_v1','simulation_time_s':t_seconds,
        'time_codes_per_second':stage.GetTimeCodesPerSecond(),'meters_per_unit':UsdGeom.GetStageMetersPerUnit(stage),
        'cameras':cams,'drone_center_position_m':list(cm.ExtractTranslation()),
        'drone_center_matrix_usd_row':matrix_rows(cm),
        'active_robots':[p.GetName() for p in stage.GetPrimAtPath('/World').GetChildren() if p.GetName().startswith(('drone_','rover_')) and p.IsActive()],
        'active_sensor_prims':[str(p.GetPath()) for p in stage.Traverse() if any(x in p.GetTypeName().lower() for x in ('imu','lidar'))],
        'enabled_rigid_bodies':[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(p).GetRigidBodyEnabledAttr().Get()],
        'active_physics_scenes':[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]}

def build(stage):
    from pxr import Gf,Sdf,Usd,UsdGeom,UsdPhysics
    import camera_layout_runtime as layout
    import camera_rig_visuals as rigs
    import waypoint_motion as motion
    if Path(stage.GetRootLayer().realPath).resolve()!=path('stages/scene.usda').resolve():
        raise RuntimeError('Only new Stage1 root may be edited')
    for layer in stage.GetUsedLayers():
        if layer.realPath and not Path(layer.realPath).resolve().is_relative_to(S):layer.SetPermissionToSave(False)
    if path('manifests/scene_build.json').exists():
        previous=json.loads(path('manifests/scene_build.json').read_text())
        for rel,key in [('stages/scene.usda','scene_sha256'),('stages/overrides.usda','override_sha256')]:
            assert hashlib.sha256(path(rel).read_bytes()).hexdigest()==previous[key],rel
        for rel,sha in previous['input_hashes'].items():assert hashlib.sha256(path(rel).read_bytes()).hexdigest()==sha,rel
        for check in previous['waypoint_checks']:
            assert math.dist(inspect(stage,check['time_s'])['drone_center_position_m'],check['position_m'])<1e-7
        return previous
    session=stage.GetSessionLayer();stage.SetEditTarget(session)
    removed=[]
    for child in list(stage.GetPrimAtPath('/World').GetChildren()):
        name=child.GetName()
        if name.startswith('rover_') or (name.startswith('drone_') and name!='drone_1') or name in ('Waypoints','PhysicsScene'):
            stage.OverridePrim(child.GetPath()).SetActive(False);removed.append(str(child.GetPath()))
    # Explicitly mask all old cameras and runtime sensor branches in this new composition.
    for prim in list(stage.Traverse()):
        if any(s in prim.GetTypeName().lower() for s in ('lidar','imu')):
            stage.OverridePrim(prim.GetPath()).SetActive(False)
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI):UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        elif prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
    drone=stage.GetPrimAtPath('/World/drone_1');assert drone and drone.IsActive()
    physics=drone.GetVariantSets().GetVariantSet('Physics')
    if 'None' in physics.GetVariantNames():physics.SetVariantSelection('None')
    for name in ('Sensors','Sensor'):
        variant=drone.GetVariantSets().GetVariantSet(name)
        if 'None' in variant.GetVariantNames():variant.SetVariantSelection('None')
    definition=json.loads(path('config/tracking_center.json').read_text())
    base=stage.GetPrimAtPath(definition['reference_body_prim']);assert base
    center=UsdGeom.Xform.Define(stage,definition['prim_path'])
    center.ClearXformOpOrder();center.AddTranslateOp().Set(Gf.Vec3d(*definition['offset_base_m']))
    center.GetPrim().SetCustomDataByKey('stage1:definition_sha256',definition['definition_sha256'])
    # A distinct op order overrides the legacy animation without editing its layers.
    xf=UsdGeom.Xformable(drone);xf.ClearXformOpOrder()
    move=xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble,'stage1')
    yaw=xf.AddRotateZOp(UsdGeom.XformOp.PrecisionDouble,'stage1');yaw.Set(180.)
    move.Set(Gf.Vec3d(0,0,0));xf.SetResetXformStack(True)
    cache=UsdGeom.XformCache(Usd.TimeCode(0))
    zero_center=cache.GetLocalToWorldTransform(center.GetPrim()).ExtractTranslation()
    route=json.loads(path('config/waypoints.json').read_text())
    trajectory=motion.prepare(route)
    # Using the observed complete center transform handles any imported base offset.
    starts=trajectory['arrival_times_s']
    for t,point in zip(starts,route['waypoints_m']):
        move.Set(Gf.Vec3d(*[point[i]-zero_center[i] for i in range(3)]),Usd.TimeCode(t*60.))
    move.Set(Gf.Vec3d(*[route['waypoints_m'][0][i]-zero_center[i] for i in range(3)]))
    config=json.loads(path('config/camera_layout.json').read_text());models=json.loads(path('config/camera_models.json').read_text())
    stage.GetRootLayer().SetPermissionToSave(False)
    result=layout.apply_layout(stage,config,models)
    rigresult=rigs.apply_camera_rig_visuals(stage,result,camera_ids=tuple(result['cameras']))
    import sys
    collection=S/'collection_20260918_v1'
    assert collection.resolve(strict=True)==collection
    if str(collection) not in sys.path:sys.path.insert(0,str(collection))
    from record_lidar import author
    lidar=author(stage)
    # Camera samples are not acquired here; this is a persistent scene composition.
    stage.SetTimeCodesPerSecond(60.);stage.SetFramesPerSecond(60.);stage.SetStartTimeCode(0.);stage.SetEndTimeCode(starts[-1]*60.)
    stage.SetInterpolationType(Usd.InterpolationTypeLinear)
    checks=[]
    times=sorted(set([0.,starts[-1]]+starts+[(a+b)/2 for a,b in zip(starts,starts[1:])]))
    for t in times:
        observed=inspect(stage,t);expected=motion.sample(trajectory,t)
        error=math.dist(observed['drone_center_position_m'],expected['position_m'])
        assert error<1e-7,(t,error)
        assert observed['active_robots']==['drone_1']
        assert observed['active_sensor_prims']==[lidar['sensor_path']] and not observed['enabled_rigid_bodies'] and not observed['active_physics_scenes']
        assert all(abs(c['upward_tilt_deg']-3)<1e-6 for c in observed['cameras'].values())
        checks.append({'time_s':t,'center_error_m':error,'position_m':observed['drone_center_position_m'],'velocity_world_m_s':expected['velocity_m_s']})
    override=path('stages/overrides.usda');assert not override.exists()
    session.Export(str(override))
    root=stage.GetRootLayer();root.SetPermissionToSave(True);root.subLayerPaths=[str(override),str(M/'hangar/scene.usda')]
    root.timeCodesPerSecond=60.;root.framesPerSecond=60.;root.startTimeCode=0.;root.endTimeCode=starts[-1]*60.;root.Save()
    session.Clear();stage.SetEditTarget(session);root.SetPermissionToSave(False)
    reopened=Usd.Stage.Open(str(path('stages/scene.usda')))
    reopencheck=inspect(reopened,0.)
    assert reopencheck['active_robots']==['drone_1'] and len(reopencheck['cameras'])==3
    assert reopencheck['active_sensor_prims']==[lidar['sensor_path']]
    for check in checks:
        assert math.dist(inspect(reopened,check['time_s'])['drone_center_position_m'],check['position_m'])<1e-7
    report={'status':'scene_built','scene':str(path('stages/scene.usda')),'base_scene_read_only':str(M/'hangar/scene.usda'),
        'removed_from_new_composition':removed,'new_camera_id_scope':'stage1 only: cam01 front ZED,cam02 middle FLIR,cam03 rear ZED',
        'layout':result,'rigs':rigresult,'trajectory':trajectory,'center_definition':definition,
        'waypoint_checks':checks,'reopened_readback':reopencheck,'motion_kind':'USD linear timeline animation; no forces/wheels/physics solver',
        'recorder_implemented':True,'recorder_validated':False,'lidar_enabled':True,'lidar':lidar,
        'input_hashes':{rel:hashlib.sha256(path(rel).read_bytes()).hexdigest() for rel in ('config/waypoints.json','config/tracking_center.json','config/camera_layout.json','config/camera_models.json')},
        'scene_sha256':hashlib.sha256(path('stages/scene.usda').read_bytes()).hexdigest(),'override_sha256':hashlib.sha256(override.read_bytes()).hexdigest()}
    write_json('manifests/scene_build.json',report)
    return report
