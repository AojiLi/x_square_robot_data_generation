import bpy,sys,json,math,time
from pathlib import Path
from mathutils import Vector
from bpy_extras.object_utils import world_to_camera_view
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];OUT=ROOT/'reports/room01_full_room_model';scene=bpy.context.scene
for c in bpy.data.collections:
 if c.name.startswith('99 '):c.hide_viewport=True;c.hide_render=True
 else:c.hide_viewport=False;c.hide_render=False
# Correct the broad window fill orientation; all other geometry stays unchanged.
bpy.data.objects['Window_soft_fill'].rotation_euler.x=-math.pi/2
scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=256;scene.cycles.use_adaptive_sampling=True;scene.cycles.adaptive_threshold=.015;scene.cycles.adaptive_min_samples=32;scene.cycles.sample_clamp_indirect=3;scene.cycles.use_denoising=False;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.threads_mode='FIXED';scene.render.threads=20
manifest=json.loads((OUT/'review_cameras.json').read_text());checks=[]
for item in manifest:
 scene.render.resolution_x=736;scene.render.resolution_y=552;scene.render.pixel_aspect_x=1;scene.render.pixel_aspect_y=item['fx']/item['fy'];cam=bpy.data.objects[item['name']];errors=[]
 for p in [Vector((.1,.05,-1)),Vector((-.2,.1,-2)),Vector((.3,-.2,-1.5))]:
  xy=world_to_camera_view(scene,cam,cam.matrix_world@p);actual=(xy.x*736,(1-xy.y)*552);expected=(item['fx']*p.x/-p.z+item['cx'],item['cy']-item['fy']*p.y/-p.z);errors.extend([abs(a-b) for a,b in zip(actual,expected)])
 checks.append({'camera':item['name'],'max_projection_error_px':max(errors)})
(OUT/'camera_projection_checks.json').write_text(json.dumps(checks,indent=2));print('CAMERA_PROJECTION',max(x['max_projection_error_px'] for x in checks),flush=True)
requested=sys.argv[sys.argv.index('--')+1:] if '--' in sys.argv else ['Overview_cutaway','ReviewChairs','ReviewWorkbench','ReviewTable']
for name in requested:
 for key in ['02 West wall','04 Entry door and glazed partition','06 Ceiling tiles and services']:bpy.data.collections[key].hide_render=(name=='Overview_cutaway')
 bpy.data.collections['15 Exterior photographic context - unmeasured'].hide_render=(name=='Overview_cutaway')
 scene.camera=bpy.data.objects[name]
 if name=='Overview_cutaway':scene.cycles.samples=512;scene.render.resolution_x=1280;scene.render.resolution_y=960;scene.render.pixel_aspect_y=1
 else:
  scene.cycles.samples=256;scene.render.resolution_x=736;scene.render.resolution_y=552;scene.render.pixel_aspect_y=float(scene.camera.get('pixel_aspect_y',1))
 scene.render.filepath=str(OUT/f'{name}.png');start=time.time();bpy.ops.render.render(write_still=True);print('RENDERED',name,round(time.time()-start,2),flush=True)
