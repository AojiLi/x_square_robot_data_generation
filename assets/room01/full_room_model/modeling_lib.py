"""Blender geometry and materials for the photo-grounded room model."""
import bpy,math,random
from mathutils import Vector,Matrix
from pathlib import Path
CURRENT=None

def collection(name):
 global CURRENT
 CURRENT=bpy.data.collections.new(name);bpy.context.scene.collection.children.link(CURRENT);return CURRENT

def link(obj,name,mat=None):
 obj.name=name
 for c in list(obj.users_collection):c.objects.unlink(obj)
 CURRENT.objects.link(obj)
 if mat:obj.data.materials.append(mat)
 return obj

def material(name,color,rough=.6,metal=0):
 m=bpy.data.materials.new(name);m.diffuse_color=(*color,1);m.use_nodes=True
 p=m.node_tree.nodes.get('Principled BSDF');p.inputs['Base Color'].default_value=(*color,1);p.inputs['Roughness'].default_value=rough;p.inputs['Metallic'].default_value=metal
 return m

def noise_bump(mat,scale=150,strength=.12,distance=.001):
 n=mat.node_tree.nodes;l=mat.node_tree.links;p=n.get('Principled BSDF');noise=n.new('ShaderNodeTexNoise');noise.inputs['Scale'].default_value=scale;noise.inputs['Detail'].default_value=2
 bump=n.new('ShaderNodeBump');bump.inputs['Strength'].default_value=strength;bump.inputs['Distance'].default_value=distance;l.new(noise.outputs['Fac'],bump.inputs['Height']);l.new(bump.outputs['Normal'],p.inputs['Normal'])

def mesh(name,verts,faces,mat=None,smooth=False):
 d=bpy.data.meshes.new(name);d.from_pydata(verts,[],faces);d.update();o=bpy.data.objects.new(name,d);CURRENT.objects.link(o)
 if mat:d.materials.append(mat)
 if smooth:
  for f in d.polygons:f.use_smooth=True
 return o

def bevel(o,width=.006,segments=3):
 if width:
  b=o.modifiers.new('Edge radius','BEVEL');b.width=width;b.segments=segments
  if o.type=='MESH':
   o.data.use_auto_smooth=True;w=o.modifiers.new('Surface normals','WEIGHTED_NORMAL');w.keep_sharp=True
 return o

def box(name,center,size,mat=None,r=.004,rot=(0,0,0)):
 bpy.ops.mesh.primitive_cube_add(size=1,location=center);o=link(bpy.context.object,name,mat);o.dimensions=size;o.rotation_euler=rot;bpy.ops.object.transform_apply(location=False,rotation=False,scale=True);return bevel(o,r)

def cylinder(name,a,b,r,mat=None,r2=None,vertices=20):
 a,b=Vector(a),Vector(b);d=b-a;bpy.ops.mesh.primitive_cone_add(vertices=vertices,radius1=r,radius2=r if r2 is None else r2,depth=d.length,location=(a+b)/2)
 o=link(bpy.context.object,name,mat);o.rotation_euler=d.to_track_quat('Z','Y').to_euler()
 for p in o.data.polygons:p.use_smooth=True
 return bevel(o,min(.002,r*.15),2)

def tube(name,points,r,mat=None,cyclic=False,res=2):
 data=bpy.data.curves.new(name,'CURVE');data.dimensions='3D';data.resolution_u=1;data.bevel_depth=r;data.bevel_resolution=res
 s=data.splines.new('POLY');s.points.add(len(points)-1)
 for p,v in zip(s.points,points):p.co=(*v,1)
 s.use_cyclic_u=cyclic;o=bpy.data.objects.new(name,data);CURRENT.objects.link(o)
 if mat:data.materials.append(mat)
 return o

def smooth_tube(name,points,r,mat=None):
 data=bpy.data.curves.new(name,'CURVE');data.dimensions='3D';data.resolution_u=10;data.bevel_depth=r;data.bevel_resolution=2
 s=data.splines.new('BEZIER');s.bezier_points.add(len(points)-1)
 for p,v in zip(s.bezier_points,points):p.co=v;p.handle_left_type='AUTO';p.handle_right_type='AUTO'
 o=bpy.data.objects.new(name,data);CURRENT.objects.link(o)
 if mat:data.materials.append(mat)
 return o

def rounded_panel(name,center,size,mat,corner=.04,edge=.003):
 x,y,z=center;w,d,t=size;r=min(corner,w/2-.001,d/2-.001);pts=[]
 for (cx,cy),start in [((w/2-r,d/2-r),0),((-w/2+r,d/2-r),math.pi/2),((-w/2+r,-d/2+r),math.pi),((w/2-r,-d/2+r),3*math.pi/2)]:
  for k in range(9):
   a=start+k*math.pi/16;pts.append((x+cx+r*math.cos(a),y+cy+r*math.sin(a)))
 n=len(pts);vs=[(px,py,z+dz) for dz in [-t/2,t/2] for px,py in pts];fs=[tuple(range(n-1,-1,-1)),tuple(range(n,2*n))]+[(i,(i+1)%n,(i+1)%n+n,i+n) for i in range(n)]
 return bevel(mesh(name,vs,fs,mat),edge)

def torus(name,center,major,minor,mat,rotation=(0,0,0)):
 bpy.ops.mesh.primitive_torus_add(major_radius=major,minor_radius=minor,major_segments=40,minor_segments=8,location=center,rotation=rotation);o=link(bpy.context.object,name,mat)
 for p in o.data.polygons:p.use_smooth=True
 return o

def text_obj(name,text,location,size,mat,rotation=(0,0,0),align='CENTER',font=None):
 d=bpy.data.curves.new(name,'FONT');d.body=text;d.align_x=align;d.align_y='CENTER';d.size=size;d.extrude=.00008
 if font:d.font=font
 o=bpy.data.objects.new(name,d);CURRENT.objects.link(o);o.location=location;o.rotation_euler=rotation;d.materials.append(mat);return o

def parent_new(before,name,location=(0,0,0),angle=0):
 empty=bpy.data.objects.new(name,None);CURRENT.objects.link(empty)
 for o in set(bpy.data.objects)-before-{empty}:o.parent=empty
 empty.location=location;empty.rotation_euler.z=angle;return empty

def image_material(name,path,emission=0):
 m=material(name,(1,1,1),.85);nodes=m.node_tree.nodes;links=m.node_tree.links;p=nodes.get('Principled BSDF');tex=nodes.new('ShaderNodeTexImage');tex.image=bpy.data.images.load(str(path),check_existing=True)
 links.new(tex.outputs['Color'],p.inputs['Base Color'])
 if emission:
  links.new(tex.outputs['Color'],p.inputs['Emission Color']);p.inputs['Emission Strength'].default_value=emission
 return m

def quad_uv(name,points,uv,mat):
 o=mesh(name,points,[(0,1,2,3)],mat);layer=o.data.uv_layers.new(name='UVMap')
 for loop in o.data.loops:layer.data[loop.index].uv=uv[loop.vertex_index]
 return o

def batch_rods(name,lines,r,mat,sides=6):
 vs=[];fs=[]
 for line in lines:
  start=len(vs)
  for j,p in enumerate(line):
   tang=Vector(line[min(j+1,len(line)-1)])-Vector(line[max(0,j-1)]);tang.normalize();u=tang.cross(Vector((0,1,0)))
   if u.length<.01:u=tang.cross(Vector((1,0,0)))
   u.normalize();v=tang.cross(u)
   for k in range(sides):vs.append(tuple(Vector(p)+r*(u*math.cos(2*math.pi*k/sides)+v*math.sin(2*math.pi*k/sides))))
  for j in range(len(line)-1):
   for k in range(sides):a=start+j*sides+k;b=start+j*sides+(k+1)%sides;fs.append((a,b,b+sides,a+sides))
 return mesh(name,vs,fs,mat,True)
