"""Stage1 Ouster authoring and raw packet readout; no clock correction."""
import hashlib
from pathlib import Path

ASSET=Path('/mnt/DATA/workspace/ws_minho/mro_1/runtime/assets/sensors/Ouster/OS1/OS1.usd')
PROFILE='OS1_REV6_32ch10hz512res'
MOUNT='/World/drone_1/base_link/Stage1LidarMount'

def rows(matrix):
    return [[float(matrix[i][j]) for j in range(4)] for i in range(4)]

def author(stage):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    assert ASSET.resolve(strict=True)==ASSET
    model_path=MOUNT+'/SensorModel'
    mount=stage.GetPrimAtPath(MOUNT)
    if not mount:
        mount=UsdGeom.Xform.Define(stage,MOUNT)
        mount.AddTranslateOp().Set(Gf.Vec3d(0,0,.107))
        mount.GetPrim().SetCustomDataByKey('stage1:lidar_profile',PROFILE)
        model=UsdGeom.Xform.Define(stage,model_path).GetPrim()
        model.GetVariantSets().AddVariantSet('sensor').SetVariantSelection(PROFILE)
        assert model.GetReferences().AddReference(str(ASSET))
    else:
        assert mount.GetCustomDataByKey('stage1:lidar_profile')==PROFILE
    model=stage.GetPrimAtPath(model_path)
    assert model.GetVariantSets().GetVariantSet('sensor').GetVariantSelection()==PROFILE
    sensors=[p for p in Usd.PrimRange(model) if p.GetTypeName()=='OmniLidar']
    assert len(sensors)==1
    sensor=sensors[0]
    expected={'omni:sensor:Core:scanType':'ROTARY','omni:sensor:tickRate':10.0,
              'omni:sensor:Core:scanRateBaseHz':10,'omni:sensor:Core:reportRateBaseHz':5120,
              'omni:sensor:Core:numberOfChannels':32}
    actual={name:sensor.GetAttribute(name).Get() for name in expected}
    assert actual==expected,actual
    auxiliary=sensor.GetAttribute('omni:sensor:Core:auxOutputType')
    assert auxiliary.IsValid()
    auxiliary.Set('NONE')
    assert auxiliary.Get()=='NONE'
    keep=sensor.GetAttribute('omni:sensor:Core:skipDroppingInvalidPoints')
    if keep.IsValid():keep.Set(True)
    assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) or p.HasAPI(UsdPhysics.MassAPI) for p in Usd.PrimRange(model))
    assert not UsdGeom.Xformable(sensor).GetResetXformStack()
    cache=UsdGeom.XformCache()
    base=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/drone_1/base_link'))
    center=cache.GetLocalToWorldTransform(stage.GetPrimAtPath('/World/drone_1/base_link/TrackingCenter'))
    native=cache.GetLocalToWorldTransform(sensor)
    return {'sensor_path':str(sensor.GetPath()),'mount_path':MOUNT,'profile':PROFILE,
            'profile_readback':actual,'aux_output_type':auxiliary.Get(),'asset':str(ASSET),'asset_sha256':hashlib.sha256(ASSET.read_bytes()).hexdigest(),
            'T_base_from_sensor_row':rows(native*base.GetInverse()),
            'T_center_from_base_row':rows(base*center.GetInverse()),
            'frame_id':'drone_1/lidar','native_coordinate_axes':'ISO8855 +X forward,+Y left,+Z up',
            'added_physics_or_mass':False,'global_acquisition_mapping_verified':False}

class Reader:
    def __init__(self,stage):
        import omni.replicator.core as rep
        from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
        self.stage=stage
        self.metadata=author(stage)
        self.product=rep.create.render_product(self.metadata['sensor_path'],(128,128))
        self.gmo=rep.AnnotatorRegistry.get_annotator('GenericModelOutput',device='cpu')
        self.ref=rep.AnnotatorRegistry.get_annotator('ReferenceTime')
        self.sim=rep.AnnotatorRegistry.get_annotator('IsaacReadSimulationTime')
        self.sim.initialize(resetOnStop=False)
        self.annotators=[self.gmo,self.ref,self.sim]
        for a in self.annotators:a.attach([self.product])
        self.core=_isaacsim_core_nodes.acquire_interface()
        self.closed=False

    def enabled(self,value):
        self.product.hydra_texture.set_updates_enabled(value)

    def read(self):
        import struct
        from drone_lidar_adapter import parse_gmo_packet
        from pxr import UsdGeom
        raw=self.gmo.get_data()
        header=raw.reshape(-1)[:264].tobytes()
        empty_header=len(header)==264 and struct.unpack_from('<I',header,0)[0]==0x4E474D4F and struct.unpack_from('<I',header,24)[0]==0
        try:parsed={'available':False,'reason':'native zero-element packet'} if empty_header else parse_gmo_packet(raw)
        except Exception as exc:
            raise ValueError(str(exc)+'; raw GMO header='+raw.reshape(-1)[:264].tobytes().hex()) from exc
        ref=self.ref.get_data()
        rawsim=self.sim.get_data()
        current=UsdGeom.XformCache().GetLocalToWorldTransform(self.stage.GetPrimAtPath(self.metadata['sensor_path']))
        metadata={'available':parsed['available'],'reference':{k:int(v) for k,v in ref.items()},
                  'reference_sim_s':float(self.core.get_sim_time_at_time((int(ref['referenceTimeNumerator']),int(ref['referenceTimeDenominator'])))),
                  'simulation_annotator':rawsim,'sensor_current_usd_matrix_row':rows(current)}
        if empty_header:
            metadata.update(frame_id=struct.unpack_from('<Q',header,32)[0],packet_timestamp_ns=struct.unpack_from('<Q',header,40)[0],
                            num_elements=0,raw_header_hex=header.hex(),
                            frameStart={'timestamp_ns':str(struct.unpack_from('<Q',header,120)[0])},
                            frameEnd={'timestamp_ns':str(struct.unpack_from('<Q',header,160)[0])})
        if parsed['available']:
            for name in ('frame_id','packet_timestamp_ns','packet_bytes','num_elements','valid_element_count','point_timestamp_min_ns','point_timestamp_max_ns','coords_type'):
                metadata[name]=parsed[name]
            for name in ('frameStart','frameEnd'):
                f=parsed[name]
                metadata[name]={'timestamp_ns':str(f['timestamp_ns']),'position_m':f['position_m'].tolist(),'orientation_xyzw':f['orientation_xyzw'].tolist()}
        return metadata,parsed

    def close(self):
        if self.closed:return []
        errors=[]
        for a in self.annotators:
            try:a.detach([self.product.path])
            except Exception as exc:errors.append(repr(exc))
        try:self.product.destroy()
        except Exception as exc:errors.append(repr(exc))
        self.closed=not errors
        return errors
