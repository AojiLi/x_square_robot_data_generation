"""Small editable Blender modeling study, in the existing scan's metric frame."""
import bpy, json, math
from pathlib import Path
from mathutils import Vector
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
fit=json.loads((ROOT/'reports/room01_modeling_pilot/geometry_fit.json').read_text())
bpy.ops.object.select_all(action='SELECT'); bpy.ops.object.delete(use_global=False)
scene=bpy.context.scene; scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1

def material(name, color, rough=.5, metallic=0):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
 p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*color,1);p.inputs['Roughness'].default_value=rough;p.inputs['Metallic'].default_value=metallic
 return m
white=material('Desk warm gray laminate',(.58,.61,.59),.48)
edge=material('Desk dark edge band',(.10,.12,.12),.4)
metal=material('Window dark aluminum',(.055,.064,.07),.36,.55)
trim=material('Cable slot rim',(.32,.34,.32),.3,.25)
dark=material('Cable slot recessed insert',(.07,.085,.08),.5)
glass=material('Glass placeholder',(.53,.70,.78),.12)
for mat in [white,edge,glass]:
 shader=mat.node_tree.nodes.get('Principled BSDF');shader.inputs['Roughness'].default_value=.92;shader.inputs['Specular IOR Level'].default_value=0


def box(name,center,size,mat,bevel=0,angle=0):
 bpy.ops.mesh.primitive_cube_add(size=1,location=center);o=bpy.context.object;o.name=name;o.dimensions=size;o.rotation_euler.z=angle
 bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
 o.data.materials.append(mat)
 if bevel:
  b=o.modifiers.new('Manufactured edge radius','BEVEL');b.width=bevel;b.segments=4
  o.data.use_auto_smooth=True
  o.modifiers.new('Face normals','WEIGHTED_NORMAL')
 return o
z=fit['table_top_z'];xmin,xmax=.727,1.342;ymin,ymax=-2.61,-1.386
def rounded_panel(name,top,thickness,mat):
 r=.055;pts=[]
 for (cx,cy),a0 in [((xmax-r,ymax-r),0),((xmin+r,ymax-r),math.pi/2),((xmin+r,ymin+r),math.pi),((xmax-r,ymin+r),3*math.pi/2)]:
  for k in range(13):
   a=a0+k*math.pi/24;pts.append((cx+r*math.cos(a),cy+r*math.sin(a)))
 n=len(pts);verts=[(x,y,h) for h in [top-thickness,top] for x,y in pts]
 faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update();o=bpy.data.objects.new(name,mesh);scene.collection.objects.link(o);o.data.materials.append(mat)
 b=o.modifiers.new('Soft manufactured edge','BEVEL');b.width=.002;b.segments=3
 return o
board=rounded_panel('DeskTop',z,.024,white)
rounded_panel('DeskEdgeBand',z-.023,.015,edge)
box('DeskBackPanel',(1.37,-2.02,.34),(.035,1.40,2.20),white,.002)

# Capsule-shaped cable opening, position inferred by calibrated photo rays.
def capsule(name,cx,cy,zcenter,width,length,height,mat=None):
 r=width/2;straight=length/2-r;points=[]
 for center, start in [(straight,0),(-straight,math.pi)]:
  for k in range(25):
   a=start+k*math.pi/24;points.append((cx+r*math.cos(a),cy+center+r*math.sin(a)))
 # Build instead using semicircle ends oriented along Y.
 points=[]
 for center,start in [(straight,0),(-straight,math.pi)]:
  for k in range(25):
   a=start+k*math.pi/24;points.append((cx+r*math.cos(a),cy+center+r*math.sin(a)))
 n=len(points);verts=[(x,y,zcenter+dz) for dz in [-height/2,height/2] for x,y in points]
 faces=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 mesh=bpy.data.meshes.new(name);mesh.from_pydata(verts,[],faces);mesh.update();o=bpy.data.objects.new(name,mesh);scene.collection.objects.link(o)
 if mat:o.data.materials.append(mat)
 return o
# Outer ring has a physically modeled opening; its exact profile is an approximation.
cx,cy=1.221,-1.922
cutter=capsule('CableSlotCut',cx,cy,z,.043,.179,.12)
mod=board.modifiers.new('Cable pass through','BOOLEAN');mod.operation='DIFFERENCE';mod.object=cutter
bpy.context.view_layer.objects.active=board;bpy.ops.object.modifier_apply(modifier=mod.name)
bpy.data.objects.remove(cutter,do_unlink=True)
ring=capsule('CableSlotRim',cx,cy,z+.001,.055,.192,.005,trim)
inner=capsule('CableSlotRimCut',cx,cy,z,.041,.176,.03)
mod=ring.modifiers.new('Recess','BOOLEAN');mod.operation='DIFFERENCE';mod.object=inner
bpy.context.view_layer.objects.active=ring;bpy.ops.object.modifier_apply(modifier=mod.name);bpy.data.objects.remove(inner,do_unlink=True)
capsule('CableSlotInset',cx,cy,z-.012,.041,.176,.002,dark)
# Window plane fitted from scan points; divisions approximated from a calibrated photo.
nx,ny=fit['window_normal_xy'];offset=fit['window_offset'];ax,ay=ny,-nx;angle=math.atan2(ay,ax)
def window_box(name,u,h,su,sh,depth,mat):
 return box(name,(nx*offset+ax*u,ny*offset+ay*u,h),(su,depth,sh),mat,.004,angle)
lo,hi=-1.43,1.53;left,right=.05,3.12
for i,u in enumerate([left,1.059,2.714,right]):window_box(f'WindowMullion_{i}',u,(lo+hi)/2,.062,hi-lo,.075,metal)
for i,h in enumerate([lo,hi]):window_box(f'WindowRail_{i}',(left+right)/2,h,right-left,.065,.075,metal)
window_box('WindowLeftCrossRail',(.05+1.059)/2,-.32,1.059-.05,.048,.065,metal)
for i,(a,b) in enumerate(zip([left,1.059,2.714],[1.059,2.714,right])):
 window_box(f'GlassPane_{i}',(a+b)/2,(lo+hi)/2,b-a-.063,hi-lo-.065,.008,glass)
scene['pilot_scope']='Approximate desk top and window frame only; no surveyed CAD accuracy, no exterior reconstruction, no physics authored.'
bpy.ops.object.select_all(action='SELECT')
# The local distribution has no built-in USD exporter; export evaluated Blender meshes
# and recreate the equivalent UsdGeom meshes using the installed OpenUSD SDK.
depsgraph=bpy.context.evaluated_depsgraph_get();parts=[]
for obj in list(scene.objects):
 if obj.type!='MESH':continue
 evaluated=obj.evaluated_get(depsgraph);mesh=evaluated.to_mesh();mat=obj.data.materials[0]
 shader=mat.node_tree.nodes.get('Principled BSDF')
 parts.append({'name':obj.name,'points':[list(evaluated.matrix_world@v.co) for v in mesh.vertices],
  'faces':[list(f.vertices) for f in mesh.polygons], 'color':list(mat.diffuse_color[:3]),
  'roughness':shader.inputs['Roughness'].default_value,'metallic':shader.inputs['Metallic'].default_value})
 evaluated.to_mesh_clear()
(HERE/'modeled_parts_geometry.json').write_text(json.dumps(parts))
# Keep the source scan available as a hidden alignment reference in the editable file.
existing=set(bpy.data.objects)
bpy.ops.import_scene.gltf(filepath=str(HERE.parent/'scan_original.glb'))
reference=bpy.data.collections.new('SCAN REFERENCE - toggle to inspect');scene.collection.children.link(reference)
for obj in set(bpy.data.objects)-existing:
 for coll in list(obj.users_collection):coll.objects.unlink(obj)
 reference.objects.link(obj)
reference.hide_render=True;reference.hide_viewport=True
bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'room01_modeling_pilot.blend'))
print('PILOT_BLENDER_DONE',len(scene.objects),flush=True)
