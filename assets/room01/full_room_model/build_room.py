"""Detailed room reconstruction from the supplied Polycam photos and metric mesh.
Run with the project-local Blender launcher. No existing room asset is overwritten.
"""
import bpy,sys,math,json,random,numpy as np
from pathlib import Path
from mathutils import Vector,Matrix
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2];OUT=ROOT/'reports/room01_full_room_model';sys.path.insert(0,str(HERE))
import modeling_lib as G
from modeling_lib import *
from seating import chair,stool
random.seed(28)
bpy.ops.object.select_all(action='SELECT');bpy.ops.object.delete(use_global=False)
for c in list(bpy.data.collections):
 if c.name!='Collection':bpy.data.collections.remove(c)
scene=bpy.context.scene;scene.unit_settings.system='METRIC';scene.unit_settings.scale_length=1
FLOOR=-1.466;CEILING=1.190;X0=-2.426;X1=2.339;Y0=-2.530;Y1=1.570;COLUMN_X=1.346;COLUMN_Y=-1.463
footprint=[(X0,Y0),(COLUMN_X,Y0),(COLUMN_X,COLUMN_Y),(X1,COLUMN_Y),(X1,Y1),(X0,Y1)]
frames=json.loads((ROOT/'data/room01/transforms.json').read_text())['frames']
M={}
for key,color,rough,metal in [
 ('wall',(.72,.745,.735),.85,0),('white',(.71,.735,.715),.62,0),('black',(.014,.019,.022),.60,0),('fabric',(.021,.027,.03),.88,0),('arm',(.065,.073,.077),.58,0),
 ('chrome',(.56,.61,.64),.25,.9),('aluminum',(.35,.41,.44),.36,.8),('dark_metal',(.065,.080,.089),.42,.65),('rubber',(.009,.012,.015),.85,0),
 ('cardboard',(.39,.255,.137),.9,0),('cardboard_light',(.50,.35,.20),.9,0),('paper',(.8,.81,.78),.9,0),('red',(.55,.032,.028),.55,0),('blue',(.008,.28,.58),.47,0),('yellow',(.72,.52,.025),.5,0),('green',(.012,.37,.18),.58,0),('purple',(.17,.055,.43),.48,0),('cable',(.008,.01,.012),.62,0),('packing_blue',(.11,.47,.59),.78,0),('tape',(.50,.36,.18),.45,0)]:M[key]=material(key,color,rough,metal)
noise_bump(M['fabric'],420,.25,.0008);noise_bump(M['wall'],220,.06,.0003);noise_bump(M['cardboard'],160,.17,.0007);noise_bump(M['white'],340,.05,.0003)
# Carpet tiles have alternating fiber orientation, as seen in the scan.
carpets=[]
for k in range(4):
 m=material('Carpet_tile_'+str(k),(.10,.11,.115),.96);n=m.node_tree.nodes;l=m.node_tree.links;p=n.get('Principled BSDF');tc=n.new('ShaderNodeTexCoord');mapping=n.new('ShaderNodeMapping');mapping.inputs['Rotation'].default_value[2]=(k%2)*math.pi/2;l.new(tc.outputs['Generated'],mapping.inputs['Vector'])
 wave=n.new('ShaderNodeTexWave');wave.wave_type='BANDS';wave.bands_direction='X';wave.inputs['Scale'].default_value=20;wave.inputs['Distortion'].default_value=.6;l.new(mapping.outputs['Vector'],wave.inputs['Vector'])
 ramp=n.new('ShaderNodeValToRGB');ramp.color_ramp.elements[0].color=([.018,.050,.032,.075][k],[.022,.055,.036,.08][k],[.025,.060,.040,.084][k],1);ramp.color_ramp.elements[1].color=([.115,.24,.17,.30][k],[.12,.25,.175,.31][k],[.125,.255,.18,.315][k],1);l.new(wave.outputs['Color'],ramp.inputs['Fac']);l.new(ramp.outputs['Color'],p.inputs['Base Color'])
 bump=n.new('ShaderNodeBump');bump.inputs['Strength'].default_value=.24;bump.inputs['Distance'].default_value=.0012;l.new(wave.outputs['Color'],bump.inputs['Height']);l.new(bump.outputs['Normal'],p.inputs['Normal']);carpets.append(m)
wood=material('Pale_vertical_woodgrain',(.50,.46,.37),.65);n=wood.node_tree.nodes;l=wood.node_tree.links;tc=n.new('ShaderNodeTexCoord');mp=n.new('ShaderNodeMapping');mp.inputs['Scale'].default_value=(25,25,.25);l.new(tc.outputs['Generated'],mp.inputs['Vector']);tex=n.new('ShaderNodeTexNoise');tex.inputs['Scale'].default_value=4;l.new(mp.outputs['Vector'],tex.inputs['Vector']);ramp=n.new('ShaderNodeValToRGB');ramp.color_ramp.elements[0].color=(.36,.335,.28,1);ramp.color_ramp.elements[1].color=(.65,.60,.50,1);l.new(tex.outputs['Fac'],ramp.inputs['Fac']);l.new(ramp.outputs['Color'],n.get('Principled BSDF').inputs['Base Color'])
glass=material('Clear_window_glass',(.96,.98,1.0),.010);p=glass.node_tree.nodes.get('Principled BSDF');p.inputs['Transmission Weight'].default_value=1;p.inputs['IOR'].default_value=1.45;glass.use_screen_refraction=True
window_frame=material('Window_powdercoat',(.08,.095,.10),.36,.45)
# Room shell. All geometry uses the scan's original world coordinates.
floor_coll=collection('01 Floor and carpet tiles')
mesh('Floor_slab',[(x,y,FLOOR-.025) for x,y in footprint],[(0,1,2,3,4,5)],M['black'])
def clip_tile(xa,xb,ya,yb):
 if xa>=X1 or ya>=Y1 or xb<=xa or yb<=ya:return []
 if xb<=COLUMN_X or ya>=COLUMN_Y:return [(xa,xb,ya,yb)]
 if xa>=COLUMN_X:return [(xa,xb,max(ya,COLUMN_Y),yb)] if yb>COLUMN_Y else []
 parts=[(xa,COLUMN_X,ya,yb)]
 if yb>COLUMN_Y:parts.append((COLUMN_X,xb,COLUMN_Y,yb))
 return parts
for ix in range(10):
 for iy in range(9):
  a=X0+ix*.50;b=Y0+iy*.50
  for j,(xa,xb,ya,yb) in enumerate(clip_tile(a,min(a+.5,X1),b,min(b+.5,Y1))):
   box(f'Carpet_{ix:02d}_{iy:02d}_{j}',((xa+xb)/2,(ya+yb)/2,FLOOR-.006),(xb-xa-.0015,yb-ya-.0015,.012),carpets[(ix+iy)%2+2*((ix*3+iy)%3==0)],.001)
west=collection('02 West wall')
box('West_wall',(X0-.06,(Y0+Y1)/2,(FLOOR+CEILING)/2),(.12,Y1-Y0,CEILING-FLOOR),M['wall'],.001)
east=collection('03 East wall and inset')
box('East_wall',(X1+.06,(COLUMN_Y+Y1)/2,(FLOOR+CEILING)/2),(.12,Y1-COLUMN_Y,CEILING-FLOOR),M['wall'],.001)
box('Inset_wall',(COLUMN_X+.06,(Y0+COLUMN_Y)/2,(FLOOR+CEILING)/2),(.12,COLUMN_Y-Y0,CEILING-FLOOR),M['wall'],.001)
box('Inset_return',((COLUMN_X+X1)/2,COLUMN_Y-.06,(FLOOR+CEILING)/2),(X1-COLUMN_X,.12,CEILING-FLOOR),M['wall'],.001)
# Black skirting around the plaster walls.
for i,(a,b) in enumerate(zip(footprint,footprint[1:]+footprint[:1])):
 if i in [0,4]:continue
 length=math.dist(a,b);center=((a[0]+b[0])/2,(a[1]+b[1])/2,FLOOR+.042)
 box(f'Skirting_{i}',center,(length,.014,.084),M['black'],.001,rot=(0,0,math.atan2(b[1]-a[1],b[0]-a[0])))
entry=collection('04 Entry door and glazed partition')
# Door opening and fixed wood sidelight, measured from photo 54 on y=-2.53.
for x in [X0,-2.30,-1.30,-.88,COLUMN_X]:box('Partition_upright', (x,Y0,(FLOOR+CEILING)/2),(.042,.074,CEILING-FLOOR),M['black'],.003)
for z in [FLOOR+.025,CEILING-.028]:box('Partition_crossbar',((X0+COLUMN_X)/2,Y0,z),(COLUMN_X-X0,.075,.052),M['black'],.003)
box('Door_leaf',(-1.80,Y0-.002,(FLOOR+CEILING)/2),(.958,.038,CEILING-FLOOR-.058),wood,.004)
box('Wood_sidelight',(-1.09,Y0-.006,(FLOOR+CEILING)/2),(.38,.035,CEILING-FLOOR-.055),wood,.002)
box('Door_edge_filler',((X0-2.30)/2,Y0-.006,(FLOOR+CEILING)/2),(-2.30-X0-.03,.035,CEILING-FLOOR-.06),wood,.002)
for z in [FLOOR+.36,CEILING-.40]:cylinder('Door_hinge',(-2.276,Y0+.021,z-.045),(-2.276,Y0+.021,z+.045),.010,M['chrome'])
cylinder('Handle_rose',(-1.445,Y0+.020,-.515),(-1.445,Y0+.034,-.515),.026,M['chrome'])
cylinder('Handle_neck',(-1.445,Y0+.03,-.515),(-1.445,Y0+.083,-.515),.009,M['chrome'])
cylinder('Door_lever',(-1.445,Y0+.083,-.515),(-1.570,Y0+.083,-.515),.008,M['chrome'])
cylinder('Keyhole_rose',(-1.445,Y0+.022,-.584),(-1.445,Y0+.030,-.584),.019,M['chrome'])
box('Keyhole',(-1.445,Y0+.031,-.584),(.004,.001,.015),M['black'],.001)
# Full-height venetian blind partition.
for a,b in [(-.86,.24),(.27,COLUMN_X-.025)]:
 box('Partition_glass',((a+b)/2,Y0-.015,(FLOOR+CEILING)/2),(b-a,.006,CEILING-FLOOR-.065),glass,0)
 for j in range(104):
  z=FLOOR+.04+j*.0248
  box('Blind_slat',((a+b)/2,Y0+.015,z),(b-a-.014,.025,.0015),M['aluminum'],.0004,rot=(math.radians(-22),0,0))
 for x in [a+.13,b-.13]:cylinder('Blind_cord',(x,Y0+.025,FLOOR+.02),(x,Y0+.025,CEILING-.05),.0012,M['paper'],vertices=8)
box('Blind_center_mullion',(.255,Y0,(FLOOR+CEILING)/2),(.026,.076,CEILING-FLOOR),M['black'],.002)
box('Thermostat_body',(-1.107,Y0+.050,-.090),(.092,.022,.083),M['paper'],.005)
box('Thermostat_LCD',(-1.107,Y0+.063,-.084),(.053,.003,.031),material('LCD',(.14,.20,.16),.5),.001)
text_obj('Thermostat_digits','22.0',(-1.107,Y0+.066,-.084),.013,M['black'],rotation=(math.pi/2,0,math.pi))
box('Switch_plate',(-1.023,Y0+.045,-.09),(.070,.014,.082),M['paper'],.004)
windows=collection('05 Windows and hardware')
# Primary plane recovered from frame bars, not from the outdoor LiDAR returns.
mullions=[X0,-1.77,-.48,.79,X1]
for i,x in enumerate(mullions):box(f'Window_mullion_{i}',(x,Y1,(FLOOR+CEILING)/2),(.06,.08,CEILING-FLOOR),window_frame,.004)
for z in [FLOOR+.035,CEILING-.03]:box('Window_outer_rail',((X0+X1)/2,Y1,z),(X1-X0,.085,.060),window_frame,.003)
for i,(a,b) in enumerate(zip(mullions[:-1],mullions[1:])):
 box(f'Window_glass_{i}',((a+b)/2,Y1+.005,(FLOOR+CEILING)/2),(b-a-.065,.008,CEILING-FLOOR-.067),glass,0)
 # Recessed seals and lower rails.
 for x in [a+.040,b-.040]:box('Window_vertical_seal',(x,Y1-.035,(FLOOR+CEILING)/2),(.008,.011,CEILING-FLOOR-.09),M['rubber'],.001)
 if i in [1,2]:
  box('Opening_window_lower_rail',((a+b)/2,Y1-.045,-.480),(b-a-.04,.062,.044),window_frame,.002)
  hx=b-.17;cylinder('Window_handle_base',(hx,Y1-.075,-.407),(hx,Y1-.090,-.407),.012,M['aluminum'])
  tube('Window_lever',[(hx,Y1-.090,-.43),(hx,Y1-.11,-.36),(hx+.042,Y1-.11,-.36)],.007,M['aluminum'])
# Ceiling tiles and services.
roof=collection('06 Ceiling tiles and services')
ceiling_mat=material('Acoustic_ceiling',(.64,.655,.635),.92);noise_bump(ceiling_mat,600,.18,.0008)
for ix in range(8):
 for iy in range(7):
  a=X0+ix*.6;b=Y0+iy*.6
  for xa,xb,ya,yb in clip_tile(a,min(a+.6,X1),b,min(b+.6,Y1)):
   box('Ceiling_tile',((xa+xb)/2,(ya+yb)/2,CEILING+.012),(xb-xa-.006,yb-ya-.006,.024),ceiling_mat,.001)
box('Ceiling_vent_frame',(-.171,-1.327,CEILING-.008),(.67,.285,.016),M['white'],.006)
box('Ceiling_vent_recess',(-.171,-1.327,CEILING-.019),(.614,.237,.010),M['black'],.001)
for i in range(16):box('Vent_louver',(-.171,-1.435+i*.0143,CEILING-.024),(.595,.007,.012),M['aluminum'],.001)
cylinder('Smoke_detector_base',(-.319,-.973,CEILING-.006),(-.319,-.973,CEILING-.024),.054,M['white'],vertices=40)
cylinder('Smoke_detector_body',(-.319,-.973,CEILING-.025),(-.319,-.973,CEILING-.062),.045,M['white'],r2=.035,vertices=40)
for i in range(10):
 a=i*2*math.pi/10;box('Smoke_detector_vent',(-.319+.039*math.cos(a),-.973+.039*math.sin(a),CEILING-.042),(.012,.003,.014),M['black'],.001,rot=(0,0,a+math.pi/2))
# Small dome camera and the visible cable beside the window.
collection('07 Wall fixtures and sockets')
cylinder('Wall_camera_mount',(X1-.012,1.40,1.024),(X1-.057,1.40,1.024),.043,M['white'],vertices=32)
bpy.ops.mesh.primitive_uv_sphere_add(segments=24,ring_count=12,location=(X1-.087,1.40,1.014));camd=G.link(bpy.context.object,'Wall_camera_dome',M['white']);camd.scale=(.055,.045,.045)
cylinder('Wall_camera_lens',(X1-.100,1.39,1.004),(X1-.132,1.38,.993),.018,M['black'],vertices=24)
smooth_tube('Camera_cable',[(X1-.10,1.41,1.04),(X1-.022,1.35,1.13),(X1-.015,.64,.77),(X1-.015,.13,.38)],.002,M['paper'])
def outlet(name,center,normal='X'):
 x,y,z=center
 if normal=='X':
  box(name,center,(.014,.086,.073),M['paper'],.004)
  for dy in [-.018,.018]:box(name+'_slot',(x-.008,y+dy,z+.009),(.001,.006,.021),M['black'],.0005)
  box(name+'_ground',(x-.008,y,z-.015),(.001,.009,.017),M['black'],.0005)
 else:
  box(name,center,(.086,.014,.073),M['paper'],.004)
  for dx in [-.018,.018]:box(name+'_slot',(x+dx,y+.008,z+.009),(.006,.001,.021),M['black'],.0005)
for label,position in [('Workbench_socket',(X1-.017,-.85,FLOOR+.235)),('Table_socket',(COLUMN_X-.017,-1.93,FLOOR+.15))]:outlet(label,position)
# Furniture and smaller props are authored below.
# An intermediate checkpoint keeps the architecture inspectable during construction.
bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'room01_full.blend'))
print('ROOM_ARCHITECTURE_READY',len(scene.objects),flush=True)
from furniture import white_table,bench,workstation_props,flight_case,case_top_props,floor_clutter
collection('08 White writing table');white_table(M,FLOOR)
collection('09 Optical workstation');bench(M,FLOOR);workstation_props(M,FLOOR)
collection('10 Seating - three chairs and stool')
chair('Chair_west',(-.565,.395,FLOOR),math.radians(14),M)
chair('Chair_middle',(-.020,.680,FLOOR),math.radians(21),M,curved=True)
chair('Chair_workbench',(.800,.440,FLOOR),math.radians(-45),M)
stool('Rolling_stool',(.345,.430,FLOOR),M)
collection('11 Large equipment flightcase');case_root=flight_case(M,FLOOR)
collection('12 Items on flightcase');case_top_props(M)
collection('13 Cartons toolcase and floor cables');floor_clutter(M,FLOOR)
# Original printed labels are projected onto their reconstructed support plane.
collection('14 Labels from supplied photographs')
def photo_decal(name,frame_idx,pixels,axis,offset,normal_offset=0,ink_only=False):
 f=frames[frame_idx];t=np.array(f['transform_matrix']);verts=[];uv=[]
 for x,y in pixels:
  u=y;v=f['h']-1-x;ray=t[:3,:3]@np.array([(u-f['cx'])/f['fl_x'],(f['cy']-v)/f['fl_y'],-1.]);point=t[:3,3]+ray*((offset-t[axis,3])/ray[axis]);point[axis]+=normal_offset;verts.append(point.tolist());uv.append((x/f['h'],1-y/f['w']))
 material_name='Photo_decal_'+name;path=OUT/f'frame_{frame_idx:03d}.png';mat=image_material(material_name,path)
 if ink_only:
  n=mat.node_tree.nodes;l=mat.node_tree.links;p=n.get('Principled BSDF');tex=next(x for x in n if x.bl_idname=='ShaderNodeTexImage')
  l.remove(p.inputs['Base Color'].links[0]);p.inputs['Base Color'].default_value=(.78,.8,.77,1);bw=n.new('ShaderNodeRGBToBW');l.new(tex.outputs['Color'],bw.inputs[0]);r=n.new('ShaderNodeValToRGB');r.color_ramp.elements[0].position=.49;r.color_ramp.elements[1].position=.76;l.new(bw.outputs[0],r.inputs[0]);l.new(r.outputs['Color'],p.inputs['Alpha']);mat.blend_method='CLIP';mat.shadow_method='CLIP'
 o=quad_uv(name,verts,uv,mat);o['source_frame']=frame_idx;o['source_upright_pixels']=json.dumps(pixels);return o
photo_decal('Flightcase_handwritten_label',340,[(294,682),(567,677),(570,820),(298,824)],1,.417,-.001)
photo_decal('Flightcase_shipping_label',340,[(26,685),(169,686),(224,858),(67,863)],1,.417,-.001)
photo_decal('Flightcase_care_icons',332,[(440,153),(735,159),(735,486),(430,457)],1,.417,-.001,True)
photo_decal('Flightcase_green_sticker',332,[(281,79),(333,65),(348,137),(296,153)],1,.417,-.001)
# A distant photo card provides the visible exterior without claiming measured exterior geometry.
ext=collection('15 Exterior photographic context - unmeasured')
extmat=image_material('Exterior_facade_reference',HERE/'textures/exterior_facade_photo.png',.5)
card=quad_uv('Distant_photo_facade',[(-.3,8,-7),(3.2,8,-7),(3.2,8,2),(-.3,8,2)],[(0,0),(1,0),(1,1),(0,1)],extmat)
card.visible_shadow=False;card.visible_diffuse=False;card['role']='Photographic window background; external distance is illustrative, not scanned geometry'
# Calibrated cameras, each using the same upright 4:3 crop as earlier diagnostics.
collection('16 Review cameras')
roll=np.array([[0.,-1.,0.,0.],[1.,0.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
camera_manifest=[]
for name,idx in [('ReviewWindow',4),('ReviewDoor',54),('ReviewChairs',77),('ReviewStorage',85),('ReviewTable',100),('ReviewWorkbench',193),('ReviewChairDetail',247),('ReviewBenchFront',394),('ReviewCeiling',440),('ReviewWest',448)]:
 f=frames[idx];width=f['h'];height=width*3//4;fx,fy=f['fl_y'],f['fl_x'];cx=f['h']-1-f['cy'];cy=f['cx']-(f['w']-height)//2
 data=bpy.data.cameras.new(name);o=bpy.data.objects.new(name,data);G.CURRENT.objects.link(o);o.matrix_world=Matrix((np.array(f['transform_matrix'])@roll).tolist());data.lens=35.;data.sensor_fit='HORIZONTAL';data.sensor_width=35*width/fx;data.shift_x=(width/2-cx)/width;data.shift_y=(cy-height/2)/width*(fx/fy);data.clip_start=.025;data.clip_end=100
 o['source_frame']=idx;o['source_image']=f['file_path'];o['calibrated_resolution']=[width,height];o['pixel_aspect_y']=fx/fy
 camera_manifest.append({'name':name,'frame':idx,'resolution':[width,height],'fx':fx,'fy':fy,'cx':cx,'cy':cy,'matrix_world':[list(v) for v in o.matrix_world]})
 if (OUT/f'frame_{idx:03d}.png').exists():
  data.show_background_images=True;bg=data.background_images.new();bg.image=bpy.data.images.load(str(OUT/f'frame_{idx:03d}.png'),check_existing=True);bg.alpha=.35;bg.display_depth='BACK';bg.frame_method='CROP'
def look_camera(name,position,target,lens=32,ortho=None):
 data=bpy.data.cameras.new(name);o=bpy.data.objects.new(name,data);G.CURRENT.objects.link(o);o.location=position;o.rotation_euler=(Vector(target)-o.location).to_track_quat('-Z','Y').to_euler();data.lens=lens;data.clip_start=.03;data.clip_end=100
 if ortho:data.type='ORTHO';data.ortho_scale=ortho
 return o
hero=look_camera('Overview_cutaway',(-6.7,-8.1,4.5),(-.05,-.4,-.35),ortho=7.8)
look_camera('Interior_wide',(-1.23,-2.12,.25),(.32,.61,-.45),lens=21)
(OUT/'review_cameras.json').write_text(json.dumps(camera_manifest,indent=2))
# Soft daylight plus broad interior fill. These are review lights, not a light-field calibration.
collection('17 Lighting')
world=bpy.data.worlds.new('Room daylight');scene.world=world;world.use_nodes=True;world.node_tree.nodes['Background'].inputs[0].default_value=(.62,.72,.85,1);world.node_tree.nodes['Background'].inputs[1].default_value=.38
for name,loc,energy,size,rotation in [('Ceiling_soft_fill',(0,-.4,1.10),170,3.7,(0,0,0)),('Window_soft_fill',(-.1,1.42,.38),125,3.5,(-math.pi/2,0,0)),('Storage_fill',(-1.5,-.8,.8),50,1.6,(0,0,0))]:
 d=bpy.data.lights.new(name,'AREA');d.energy=energy;d.specular_factor=0.0;d.shape='DISK';d.size=size;o=bpy.data.objects.new(name,d);G.CURRENT.objects.link(o);o.location=loc;o.rotation_euler=rotation;o.visible_glossy=False
sun=bpy.data.lights.new('Sunlight','SUN');sun.energy=1.6;sun.angle=.12;o=bpy.data.objects.new('Sunlight',sun);G.CURRENT.objects.link(o);o.rotation_euler=(math.radians(-55),0,math.radians(20))
# Keep the original scan as a switchable alignment reference, hidden in final views.
reference=collection('99 Original scan reference - hidden');before=set(bpy.data.objects);bpy.ops.import_scene.gltf(filepath=str(HERE.parent/'scan_original.glb'))
for o in set(bpy.data.objects)-before:
 for c in list(o.users_collection):c.objects.unlink(o)
 reference.objects.link(o)
reference.hide_render=True;reference.hide_viewport=True
scene.render.engine='CYCLES';scene.cycles.samples=32;scene.cycles.use_denoising=False;scene.cycles.max_bounces=6;scene.cycles.transparent_max_bounces=8;scene.render.resolution_x=1280;scene.render.resolution_y=960;scene.render.resolution_percentage=100;scene.render.image_settings.file_format='PNG';scene.render.film_transparent=False
scene.eevee.use_gtao=True;scene.eevee.gtao_distance=.3;scene.eevee.use_ssr=True;scene.eevee.use_ssr_refraction=True;scene.eevee.taa_render_samples=128
scene.view_settings.view_transform='AgX';scene.view_settings.exposure=0;scene.view_settings.gamma=1;scene.camera=hero
scene['model_scope']='Complete observed room reconstructed as editable geometry: architecture, three office chairs, stool, two tables, equipment, case, cartons, floor clutter. Small unseen profiles are approximate.'
scene['source_coordinate_system']='Original scan world, meters, Z-up';scene['room_floor_z']=FLOOR;scene['room_ceiling_z']=CEILING;scene['exterior_context']='Photo card only; no measured exterior geometry';scene['physics_status']='Visual model; collisions and robot placement not authored'
# Viewport opens as an architectural cutaway; all omitted walls remain in named collections.
for coll in [roof,entry,west,ext]:coll.hide_viewport=True
for o in bpy.context.view_layer.objects:o.select_set(False)
for screen in bpy.data.screens:
 for area in screen.areas:
  if area.type=='VIEW_3D':
   space=area.spaces.active;space.shading.type='MATERIAL';space.shading.use_scene_lights=True;space.shading.use_scene_world=True;space.shading.color_type='MATERIAL';space.shading.show_cavity=True;space.overlay.show_floor=False;space.region_3d.view_location=Vector((0,-.4,-.45));space.region_3d.view_rotation=Vector((-6,-8,5)).to_track_quat('Z','Y');space.region_3d.view_distance=8.3
bpy.ops.file.pack_all();bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'room01_full.blend'))
manifest={'blender_version':bpy.app.version_string,'objects':len(scene.objects),'mesh_objects':sum(o.type=='MESH' for o in scene.objects),'curve_objects':sum(o.type=='CURVE' for o in scene.objects),'collections':[c.name for c in bpy.data.collections], 'footprint':footprint,'floor_z':FLOOR,'ceiling_z':CEILING,'three_chair_centers':[[-.565,.395],[ -.020,.680],[.800,.440]],'stool_center':[.345,.430],'source_photo_count':len(frames),'reference_scan_hidden':True,'dimensions_status':'Approximate fits to source scan and photographed landmarks','blend_file':str(HERE/'room01_full.blend')}
(OUT/'model_manifest.json').write_text(json.dumps(manifest,indent=2));print('FULL_ROOM_BUILT',json.dumps(manifest),flush=True)
