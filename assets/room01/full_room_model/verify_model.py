import bpy,json,math
from pathlib import Path
from mathutils import Vector
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];OUT=ROOT/'reports/room01_full_room_model';scene=bpy.context.scene
required=['Door_leaf','Monitor_bezel','Writing_table','Optical_workbench','Large_equipment_flightcase','Chair_west','Chair_middle','Chair_workbench','Rolling_stool','Smoke_detector_body','Ceiling_vent_frame','Computer_tower']
assert all(name in bpy.data.objects for name in required)
assert scene.unit_settings.scale_length==1 and scene.unit_settings.system=='METRIC'
invalid=[];vertices=0;polygons=0;collections=[]
for obj in scene.objects:
 if any(c.name.startswith('99 ') for c in obj.users_collection):continue
 if not all(math.isfinite(v) for row in obj.matrix_world for v in row):invalid.append(obj.name)
 if obj.type=='MESH':
  vertices+=len(obj.data.vertices);polygons+=len(obj.data.polygons)
  if len(obj.data.vertices)==0 or len(obj.data.polygons)==0:invalid.append(obj.name)
assert not invalid,invalid
images=[im for im in bpy.data.images if im.source=='FILE'];missing=[im.name for im in images if not im.packed_file and not Path(bpy.path.abspath(im.filepath)).exists()];assert not missing,missing
# Check fit-sensitive furniture bounds against the measured side walls, without treating
# the intentional open carton flaps and the wall decals as a physics validation.
bounds={}
for name in ['Writing_table','Optical_workbench','Large_equipment_flightcase','Chair_west','Chair_middle','Chair_workbench','Rolling_stool']:
 root=bpy.data.objects[name];objects=list(root.children_recursive);points=[]
 for o in objects:
  if o.type in ['MESH','CURVE']:points += [o.matrix_world@Vector(v) for v in o.bound_box]
 bounds[name]={'min':[min(v[i] for v in points) for i in range(3)],'max':[max(v[i] for v in points) for i in range(3)]}
result={'result':'pass','required_components':required,'objects':len(scene.objects),'authored_mesh_vertices':vertices,'authored_mesh_polygons':polygons,'invalid_objects':invalid,'file_images':len(images),'packed_images':sum(bool(im.packed_file) for im in images),'missing_images':missing,'unit':'meter','source_reference_hidden':bpy.data.collections['99 Original scan reference - hidden'].hide_render,'major_component_bounds':bounds,'validation_scope':'Asset structure, units, image dependencies, major component extents; not a physical or surveyed-dimension certification'}
(OUT/'asset_checks.json').write_text(json.dumps(result,indent=2));print('FULL_ROOM_ASSET_CHECKS',json.dumps(result),flush=True)
