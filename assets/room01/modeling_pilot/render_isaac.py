"""Headless Isaac check of the isolated Blender modeling pilot."""
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[2]
OUTPUT=ROOT/'reports/room01_modeling_pilot'
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'multi_gpu':False,'width':736,'height':552,'renderer':'RealTimePathTracing','anti_aliasing':4,'enable_crashreporter':False,'extra_args':['--enable','isaacsim.replicator.nurec_utils']})
try:
 import omni.usd,carb,numpy as np
 from PIL import Image
 from pxr import UsdGeom
 from isaacsim.replicator.nurec_utils.rendering_setup import setup_for_rendering
 from isaacsim.replicator.nurec_utils.render import RenderTargetFactory,CameraRenderer
 context=omni.usd.get_context();assert context.open_stage(str(HERE/'room01_modeling_pilot.usda'))
 stage=context.get_stage();ok,*rest=setup_for_rendering(stage);assert ok,rest
 stage.SetEditTarget(stage.GetSessionLayer())
 meshes=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and str(p.GetPath()).startswith('/World/ModeledParts')]
 assert len(meshes)>10,meshes
 factory=RenderTargetFactory(False,resolution=(736,552));results=[]
 for name in ['ReviewTable','ReviewRoom']:
  path='/World/'+name;target=factory.create(stage,name,camera_path=path)
  renderer=CameraRenderer.open(stage,name,app,target,warmup_steps=500,force_identity_exposure=False)
  matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path));q=matrix.ExtractRotationQuat();pose=[*matrix.ExtractTranslation(),*q.GetImaginary(),q.GetReal()]
  rgb=renderer.render_at_pose(pose)
  print('RGB_STATS',name,None if rgb is None else [float(rgb.min()),float(rgb.max()),float(rgb.mean()),float(rgb.std())],flush=True)
  if rgb is not None:Image.fromarray(rgb).save(OUTPUT/(name+'.png'))
  assert rgb is not None and rgb.std()>2
  Image.fromarray(rgb).save(OUTPUT/(name+'.png'));results.append({'camera':path,'shape':list(rgb.shape),'std':float(rgb.std())});renderer.close()
 out={'result':'pass','native_renderer':'Isaac Sim 6.0.1','modeled_meshes':len(meshes),'renders':results,'aa':carb.settings.get_settings().get('/rtx/post/aa/op')}
 (OUTPUT/'verification.json').write_text(json.dumps(out,indent=2));print('MODELING_PILOT_RENDERED',json.dumps(out),flush=True)
except BaseException:
 import traceback
 error=traceback.format_exc();(OUTPUT/'render_error.txt').write_text(error);print(error,flush=True)
 raise
finally:app.close()
