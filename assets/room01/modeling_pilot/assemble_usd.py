"""Create the standalone mesh modeling preview; original scene is untouched."""
from pathlib import Path
import sys,json
from pxr import Usd,UsdGeom,UsdLux,Sdf,Gf,UsdRender
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];sys.path.insert(0,str(ROOT/'scripts'))
from configure_scene_preview import add_review_setup
stage=Usd.Stage.CreateNew(str(HERE/'room01_modeling_pilot.usda'));world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim());UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1.)
stage.DefinePrim('/World/ModeledParts').GetReferences().AddReference('modeled_parts.usdc')
light=UsdLux.DomeLight.Define(stage,'/World/PilotAmbient');light.CreateIntensityAttr(.7);light.CreateColorAttr(Gf.Vec3f(.93,.96,1))
sun=UsdLux.DistantLight.Define(stage,'/World/PilotDaylight');sun.CreateIntensityAttr(1.2);sun.CreateAngleAttr(18);UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-30,20,25))
for name,location,size,intensity,rotation in [
 ('DeskSoftbox',(.85,-1.95,.6),(1.2,1.2),12.,(0.,0.,0.)),
 ('RoomSoftbox',(.0,.0,.8),(2.,2.),5.,(0.,0.,0.)),
 ('WindowFill',(.8,.25,.25),(3.,3.),3.,(90.,0.,0.))]:
 lamp=UsdLux.RectLight.Define(stage,'/World/'+name);lamp.CreateWidthAttr(size[0]);lamp.CreateHeightAttr(size[1]);lamp.CreateIntensityAttr(intensity);xf=UsdGeom.Xformable(lamp);xf.AddTranslateOp().Set(Gf.Vec3d(*location));xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
frames=json.loads((ROOT/'data/room01/transforms.json').read_text())['frames'];add_review_setup(stage,frames)
for prim in stage.Traverse():
 if prim.IsA(UsdGeom.Camera):
  for attr,value in {'exposure':0.,'exposure:fStop':1.,'exposure:iso':0.,'exposure:responsivity':1.,'exposure:time':1.}.items():prim.CreateAttribute(attr,Sdf.ValueTypeNames.Float).Set(value)
  prim.CreateAttribute('omni:rtx:autoExposure:enabled',Sdf.ValueTypeNames.Bool).Set(False)
data=dict(stage.GetRootLayer().customLayerData);data['cameraSettings']['boundCamera']='/World/ReviewTable';data['pilotNote']='Geometry prototype: desktop, adjacent panel and window frame; approximate dimensions/materials; exterior not reconstructed; no physics.';stage.GetRootLayer().customLayerData=data
UsdRender.Settings.Get(stage,'/Render/Settings').GetCameraRel().SetTargets([Sdf.Path('/World/ReviewTable')])
stage.GetRootLayer().Save()
(ROOT/'reports/room01_modeling_pilot/assembly.json').write_text(json.dumps({'scene':str(HERE/'room01_modeling_pilot.usda'),'source_scene_modified':False,'units':'meters','up_axis':'Z','default_camera':'ReviewTable','modeled_meshes':15,'background':'trimmed source scan mesh','approximate_geometry':True},indent=2))
print('PILOT_SCENE_CREATED',flush=True)
