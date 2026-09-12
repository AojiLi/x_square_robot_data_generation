import bpy,math,random
from modeling_lib import *

def ellipsoid(name,center,size,mat,segments=24):
 bpy.ops.mesh.primitive_uv_sphere_add(segments=segments,ring_count=12,location=center);o=link(bpy.context.object,name,mat);o.scale=size
 for p in o.data.polygons:p.use_smooth=True
 return o

def cup(name,center,r,h,mat):
 x,y,z=center;vs=[];fs=[];N=40
 for rad,zz in [(r*.88,z),(r,z+h),(r-.002,z+h),(r*.88-.002,z+.003)]:
  for i in range(N):a=2*math.pi*i/N;vs.append((x+rad*math.cos(a),y+rad*math.sin(a),zz))
 for j in range(3):
  for i in range(N):fs.append((j*N+i,j*N+(i+1)%N,(j+1)*N+(i+1)%N,(j+1)*N+i))
 fs.append(tuple(range(3*N,4*N)));return mesh(name,vs,fs,mat,True)

def tape_roll(name,center,r,width,mat,core):
 x,y,z=center;torus(name+'_body',(x,y,z+width/2),r-width*.3,width*.3,mat);torus(name+'_core',(x,y,z+width/2),r-width*.6,.002,core)

def carton(name,center,size,M,opened=False,yaw=0,tilt=(0,0,0)):
 before=set(bpy.data.objects);w,d,h=size;t=.004;mat=M['cardboard'];z=h/2
 box(name+'_base',(0,0,t/2),(w,d,t),mat,.001)
 for side in [-1,1]:
  box(name+'_wall_x',(side*(w/2-t/2),0,z),(t,d,h),mat,.001)
  box(name+'_wall_y',(0,side*(d/2-t/2),z),(w,t,h),mat,.001)
 if opened:
  for side in [-1,1]:
   # Real flaps with their hinge along the top edge.
   o=box(name+'_flap_x',(0,0,0),(w*.46,d,.003),M['cardboard_light'],.0005)
   o.location=(side*(w/2+w*.18),0,h-.015);o.rotation_euler.y=side*math.radians(35)
   o=box(name+'_flap_y',(0,0,0),(w,d*.40,.003),mat,.0005);o.location=(0,side*(d/2+d*.14),h+.035);o.rotation_euler.x=-side*math.radians(45)
 else:
  box(name+'_lid',(0,0,h),(w,d,.004),mat,.001)
  box(name+'_packing_tape',(0,0,h+.0028),(.043,d+.003,.001),M['tape'],.0002)
  for side in [-1,1]:box(name+'_tape_end',(0,side*(d/2+.001),h-.07),(.043,.001,.14),M['tape'],.0002)
 root=parent_new(before,name,center,yaw);root.rotation_euler.x=tilt[0];root.rotation_euler.y=tilt[1];return root

def white_table(M,F):
 before=set(bpy.data.objects);z=-.743;x0,x1=.727,1.342;y0,y1=-2.514,-1.360
 board=rounded_panel('Writing_table_top',((x0+x1)/2,(y0+y1)/2,z-.012),(x1-x0,y1-y0,.024),M['white'],.055,.002)
 for mod in list(board.modifiers):bpy.context.view_layer.objects.active=board;bpy.ops.object.modifier_apply(modifier=mod.name)
 cut=rounded_panel('Grommet_cutter',(1.221,-1.922,z),(.043,.178,.12),M['black'],.021,0)
 b=board.modifiers.new('Cable opening','BOOLEAN');b.operation='DIFFERENCE';b.object=cut;b.solver='EXACT';bpy.context.view_layer.objects.active=board;bpy.ops.object.modifier_apply(modifier=b.name);bpy.data.objects.remove(cut,do_unlink=True)
 rounded_panel('Writing_table_edge',((x0+x1)/2,(y0+y1)/2,z-.027),(x1-x0+.002,y1-y0+.002,.010),M['dark_metal'],.055,.001)
 ring=rounded_panel('Writing_table_grommet_rim',(1.221,-1.922,z+.001),(.055,.190,.004),M['aluminum'],.026,0)
 cut=rounded_panel('Grommet_rim_cutter',(1.221,-1.922,z),(.041,.173,.06),M['black'],.020,0)
 b=ring.modifiers.new('Recess','BOOLEAN');b.object=cut;b.operation='DIFFERENCE';bpy.context.view_layer.objects.active=ring;bpy.ops.object.modifier_apply(modifier=b.name);bpy.data.objects.remove(cut,do_unlink=True)
 rounded_panel('Writing_table_slot_insert',(1.221,-1.922,z-.012),(.041,.172,.003),M['dark_metal'],.020,0)
 for x in [x0+.038,x1-.038]:
  for y in [y0+.040,y1-.040]:
   box('Writing_table_leg',(x,y,(F+z-.026)/2),(.043,.043,z-F-.026),M['white'],.014)
   box('Writing_table_foot',(x,y,F+.008),(.041,.041,.015),M['white'],.006)
 for y in [y0+.044,y1-.044]:box('Writing_table_apron',((x0+x1)/2,y,z-.056),(x1-x0-.06,.025,.05),M['white'],.004)
 # Visible red marker outlines and light scratches, as in the source photographs.
 red=M['red']
 pts=[(1.019+.031*math.cos(a),-1.929+.026*math.sin(a),z+.00025) for a in [2*math.pi*i/70 for i in range(71)]];tube('Table_marker_circle',pts,.00035,red)
 for y in [-2.41,-1.55]:tube('Table_marker_rectangle',[(.750,y,z+.0004),(.766,y,z+.0004),(.766,y+.024,z+.0004),(.750,y+.024,z+.0004),(.750,y,z+.0004)],.00045,red)
 for j in range(9):
  x=random.uniform(.85,1.25);y=random.uniform(-2.45,-1.45);tube('Laminate_scuff',[(x,y,z+.0001),(x+.017,y+.003,z+.0001),(x+.023,y+.008,z+.0001)],.00016,M['arm'],res=1)
 root=parent_new(before,'Writing_table');root['source_photo_frames']='100,108,115,124,139,162';return root

def bench(M,F):
 before=set(bpy.data.objects);x0,x1=1.280,2.045;y0,y1=-.346,.846;z=-.667
 optical=material('Optical_breadboard_tapped_grid',(.43,.46,.48),.37,.6);n=optical.node_tree.nodes;l=optical.node_tree.links;p=n.get('Principled BSDF');tc=n.new('ShaderNodeTexCoord');sep=n.new('ShaderNodeSeparateXYZ');l.new(tc.outputs['Generated'],sep.inputs[0]);terms=[]
 for axis,scale in [('X',(x1-x0)/.025),('Y',(y1-y0)/.025)]:
  mul=n.new('ShaderNodeMath');mul.operation='MULTIPLY';mul.inputs[1].default_value=scale;l.new(sep.outputs[axis],mul.inputs[0]);fract=n.new('ShaderNodeMath');fract.operation='FRACT';l.new(mul.outputs[0],fract.inputs[0]);sub=n.new('ShaderNodeMath');sub.operation='SUBTRACT';sub.inputs[1].default_value=.5;l.new(fract.outputs[0],sub.inputs[0]);sq=n.new('ShaderNodeMath');sq.operation='MULTIPLY';l.new(sub.outputs[0],sq.inputs[0]);l.new(sub.outputs[0],sq.inputs[1]);terms.append(sq)
 add=n.new('ShaderNodeMath');add.operation='ADD';l.new(terms[0].outputs[0],add.inputs[0]);l.new(terms[1].outputs[0],add.inputs[1]);less=n.new('ShaderNodeMath');less.operation='LESS_THAN';less.inputs[1].default_value=.010;l.new(add.outputs[0],less.inputs[0]);mix=n.new('ShaderNodeMixRGB');mix.blend_type='MIX';mix.inputs[1].default_value=(.43,.46,.47,1);mix.inputs[2].default_value=(.006,.009,.012,1);l.new(less.outputs[0],mix.inputs[0]);l.new(mix.outputs[0],p.inputs['Base Color']);bump=n.new('ShaderNodeBump');bump.invert=True;bump.inputs['Strength'].default_value=.6;bump.inputs['Distance'].default_value=.0017;l.new(less.outputs[0],bump.inputs['Height']);l.new(bump.outputs['Normal'],p.inputs['Normal'])
 rounded_panel('Optical_table_top',((x0+x1)/2,(y0+y1)/2,z-.020),(x1-x0,y1-y0,.04),optical,.012,.002)
 box('Optical_table_body',((x0+x1)/2,(y0+y1)/2,z-.092),(x1-x0-.03,y1-y0-.035,.145),M['black'],.004)
 for x in [x0+.07,x1-.07]:
  for y in [y0+.065,y1-.065]:
   box('Optical_table_leg',(x,y,(F+z-.145)/2),(.135,.135,z-F-.145),M['dark_metal'],.012)
   cylinder('Optical_table_level_foot',(x,y,F+.007),(x,y,F+.050),.045,M['chrome'],r2=.030)
 for zbar in [F+.18,F+.40]:
  box('Workbench_crossbrace',(x0+.075,(y0+y1)/2,zbar),(.055,y1-y0-.12,.045),M['dark_metal'],.004)
 for y in [y0+.07,y1-.07]:box('Workbench_depth_brace',((x0+x1)/2,y,F+.22),(x1-x0-.12,.05,.045),M['dark_metal'],.004)
 root=parent_new(before,'Optical_workbench');root['source_photo_frames']='193,200,394,401,409,417';return root

def keyboard(name,center,M,yaw=math.pi/2):
 before=set(bpy.data.objects);rounded_panel(name+'_shell',(0,0,0),(.435,.143,.017),M['black'],.010,.004)
 for row in range(6):
  for col in range(19):
   if row==0 and 5<=col<=10:continue
   x=-.200+col*.0215;y=-.055+row*.021
   box(name+'_key',(x,y,.0125),(.018,.017,.008),M['arm'],.002)
   labels=['ZXCVBNM,./','ASDFGHJKL;','QWERTYUIOP','1234567890','1234567890','1234567890']
   if col<10:text_obj(name+'_legend',labels[row][col],(x,y,.0170),.0055,M['paper'])
 box(name+'_space',( -.025,-.055,.0125),(.115,.017,.008),M['arm'],.002)
 return parent_new(before,name,center,yaw)

def workstation_props(M,F):
 z=-.667
 # Display faces toward -X.
 box('Monitor_bezel',(1.816,.223,-.416),(.035,.568,.342),M['black'],.012)
 screen=material('Powered_off_monitor',(.015,.023,.031),.26,.1)
 box('Monitor_screen',(1.796,.223,-.409),(.001,.538,.307),screen,.004)
 cylinder('Monitor_stalk',(1.861,.22,z+.012),(1.842,.22,-.570),.028,M['black'],vertices=20)
 rounded_panel('Monitor_base',(1.821,.22,z+.010),(.20,.21,.018),M['black'],.06,.004)
 text_obj('Monitor_brand','PHILIPS',(1.794,.223,-.581),.009,M['aluminum'],rotation=(math.pi/2,0,-math.pi/2))
 keyboard('Keyboard',(1.536,.209,z+.012),M)
 ellipsoid('Mouse',(1.447,-.110,z+.025),(.046,.031,.022),M['black']);tube('Mouse_split',[(1.404,-.11,z+.030),(1.447,-.11,z+.046)],.0006,M['rubber'])
 # Black tray, four colored blocks, and cups.
 tray=(1.335,.226,z+.005);w,d,h=.190,.205,.088
 box('Sorting_tray_base',tray,(w,d,.010),M['black'],.002)
 for sign in [-1,1]:box('Sorting_tray_wall_x',(tray[0]+sign*(w/2-.003),tray[1],z+h/2),( .006,d,h),M['black'],.001);box('Sorting_tray_wall_y',(tray[0],tray[1]+sign*(d/2-.003),z+h/2),(w,.006,h),M['black'],.001)
 for i,(dx,dy,mat) in enumerate([(-.047,.045,'green'),(.047,.045,'red'),(-.047,-.045,'red'),(.045,-.045,'blue')]):box('Task_cube_'+str(i),(tray[0]+dx,tray[1]+dy,z+.034),(.055,.055,.055),M[mat],.003)
 cup('Purple_cup',(1.470,-.016,z+.003),.031,.090,M['purple']);cup('Orange_cup',(1.899,-.10,z+.003),.025,.074,material('orange',(.8,.19,.015),.45))
 box('Pin_holder',(1.34,-.057,z+.037),(.066,.075,.072),M['red'],.004)
 for i,mat in enumerate(['blue','yellow','yellow']):cylinder('Task_pin',(1.323+i*.017,-.057,z+.071),(1.323+i*.017,-.057,z+.131),.0065,M[mat],vertices=16)
 # Slotted black rail and accessories near the window edge.
 box('Slotted_rail',(1.645,.712,z+.013),(.555,.071,.025),M['black'],.006)
 for x in [1.445,1.615,1.79]:rounded_panel('Rail_slot',(x,.712,z+.026),(.073,.019,.0015),M['rubber'],.009,0)
 tape_roll('Blue_cable_reel',(1.473,.479,z+.004),.056,.034,M['blue'],M['paper'])
 for i in range(4):torus('White_cable_coil',(1.808,-.208,z+.009+i*.003),.059+i*.003,.0019,M['paper'])
 box('Interface_box',(1.950,-.178,z+.031),(.103,.143,.061),M['black'],.004)
 for y in [-.20,-.15]:cylinder('Interface_connector',(1.897,y,z+.03),(1.888,y,z+.03),.009,M['green'],vertices=16)
 # White retail package, upright near the front edge.
 o=box('White_retail_box',(1.385,-.278,z+.062),(.028,.12,.124),M['paper'],.003);o.rotation_euler.y=.06
 for i in range(11):box('Retail_box_print',(1.369,-.318+i*.006,z+.066),(.0005,.002,.072),M['aluminum'],0)
 # A small clear bottle with red sleeve.
 bottle=material('Translucent_bottle',(.8,.88,.9),.18);bottle.node_tree.nodes.get('Principled BSDF').inputs['Transmission Weight'].default_value=.78
 cylinder('Bottle_body',(1.925,-.025,z+.001),(1.925,-.025,z+.125),.024,bottle,vertices=32)
 cylinder('Bottle_label',(1.925,-.025,z+.016),(1.925,-.025,z+.080),.0245,M['red'],vertices=32)
 cylinder('Bottle_neck',(1.925,-.025,z+.125),(1.925,-.025,z+.164),.010,bottle);cylinder('Bottle_cap',(1.925,-.025,z+.16),(1.925,-.025,z+.174),.011,M['paper'])
 # Tower computer, ventilation, ports, and the headset resting on it.
 box('Computer_tower',(1.716,-.498,F+.289),(.435,.244,.566),M['black'],.014)
 for i in range(12):box('PC_front_vent',(1.496,-.498,F+.10+i*.021),(.002,.165,.008),M['rubber'],.001)
 box('PC_port_panel',(1.493,-.498,F+.478),(.004,.13,.045),M['arm'],.003)
 for y in [-.535,-.497]:box('PC_USB',(1.489,y,F+.480),(.001,.018,.007),M['black'],.001)
 cylinder('PC_power_button',(1.488,-.453,F+.480),(1.483,-.453,F+.480),.006,M['blue'],vertices=16)
 pts=[(1.73+.105*math.cos(a),-.495,F+.625+.095*math.sin(a)) for a in [math.pi*j/40 for j in range(41)]];tube('Headset_headband',pts,.012,M['black'],res=3)
 for x in [1.625,1.835]:box('Headset_earcup',(x,-.495,F+.612),(.055,.071,.088),M['black'],.025)
 # Realistic loose lead paths, each kept as an editable curve.
 for j in range(6):
  smooth_tube('Workbench_signal_cable',[(1.82,-.16+j*.018,z+.05),(2.02,-.27+j*.006,z),(2.03,-.39,F+.60),(1.84,-.47+j*.016,F+.48)],.0020,M['cable'])
 smooth_tube('Monitor_lead',[(1.82,.22,-.55),(1.97,.32,z+.015),(1.98,-.12,z+.01)],.0023,M['cable'])
 smooth_tube('Keyboard_lead',[(1.55,.39,z+.025),(1.66,.5,z+.04),(1.89,.44,z+.01)],.0018,M['cable'])

def flight_case(M,F):
 before=set(bpy.data.objects);x0,x1=-2.315,-.985;y0,y1=.417,1.354;top=-.035;bottom=F+.075
 case=material('Flightcase_black_laminate',(.025,.035,.041),.64);noise_bump(case,230,.09,.0005)
 box('Flightcase_shell',((x0+x1)/2,(y0+y1)/2,(bottom+top)/2),(x1-x0,y1-y0,top-bottom),case,.012)
 # Extruded aluminum edge profiles and riveted corner guards.
 for x in [x0,x1]:
  for y in [y0,y1]:box('Case_vertical_extrusion',(x,y,(bottom+top)/2),(.027,.028,top-bottom+.02),M['aluminum'],.003)
 for z in [bottom,top]:
  for y in [y0,y1]:box('Case_horizontal_rail_x',((x0+x1)/2,y,z),(x1-x0+.025,.031,.026),M['aluminum'],.002)
  for x in [x0,x1]:box('Case_horizontal_rail_y',(x,(y0+y1)/2,z),(.029,y1-y0,.026),M['aluminum'],.002)
 for x in [x0,x1]:
  for y in [y0,y1]:
   for z in [bottom,top]:box('Case_corner_guard',(x,y,z),(.054,.054,.055),M['chrome'],.012)
 for x in np_range(x0+.065,x1-.045,.13):
  for z in [bottom+.008,top-.008]:cylinder('Case_front_rivet',(x,y0-.016,z),(x,y0-.020,z),.0032,M['chrome'],vertices=12)
 for y in np_range(y0+.05,y1-.035,.13):
  for z in [bottom+.012,top-.008]:cylinder('Case_side_rivet',(x1+.015,y,z),(x1+.019,y,z),.0032,M['chrome'],vertices=12)
 # Recessed carrying handles on front and right side.
 box('Case_front_handle_plate',(-1.84,y0-.019,-.65),(.166,.008,.110),M['chrome'],.014)
 box('Case_front_handle_well',(-1.84,y0-.024,-.65),(.126,.002,.071),M['black'],.012)
 cylinder('Case_front_handle',(-1.895,y0-.038,-.655),(-1.785,y0-.038,-.655),.010,M['aluminum'])
 for z in [-.695,-.605]:
  for x in [-1.91,-1.77]:cylinder('Handle_rivet',(x,y0-.024,z),(x,y0-.026,z),.003,M['aluminum'],vertices=12)
 box('Case_side_handle_plate',(x1+.019,.92,-.75),(.008,.164,.104),M['chrome'],.012)
 box('Case_side_handle_well',(x1+.024,.92,-.75),(.002,.126,.064),M['black'],.008)
 cylinder('Case_side_handle',(x1+.038,.865,-.75),(x1+.038,.975,-.75),.01,M['aluminum'])
 for x in [x0+.075,x1-.075]:
  for y in [y0+.075,y1-.075]:
   cylinder('Case_castor_stem',(x,y,F+.046),(x,y,bottom),.015,M['chrome'])
   cylinder('Case_castor_wheel',(x-.024,y,F+.040),(x+.024,y,F+.04),.038,M['black'],vertices=24)
 # The white side logo is geometry, not an oversized pasted photo.
 text_obj('Case_side_XR','XR',(x1+.020,.91,-.40),.19,M['paper'],rotation=(math.pi/2,0,math.pi/2))
 text_obj('Case_side_brand','X SQUARE ROBOT',(x1+.021,.91,-.56),.039,M['paper'],rotation=(math.pi/2,0,math.pi/2))
 root=parent_new(before,'Large_equipment_flightcase');root['source_photo_frames']='46,316,332,340,347,355';return root

def np_range(a,b,step):
 values=[]
 while a<=b:values.append(a);a+=step
 return values

def case_top_props(M):
 z=-.018
 # Gamepad shell, analog sticks, D-pad and colored face buttons.
 before=set(bpy.data.objects)
 rounded_panel('Controller_body',(0,0,.015),(.154,.092,.032),M['white'],.034,.009)
 for side in [-1,1]:
  ellipsoid('Controller_grip',(side*.057,-.045,.011),(.034,.055,.023),M['white'])
  cylinder('Controller_stick',(side*.04,.005,.022),(side*.04,.005,.032),.013,M['black'],vertices=20)
 for x,y in [(.053,.037),(.069,.022),(.053,.009),(.037,.023)]:
  index=[(.053,.037),(.069,.022),(.053,.009),(.037,.023)].index((x,y));cylinder('Controller_button',(x,y,.03),(x,y,.035),.004,M[['yellow','red','blue','green'][index]],vertices=12)
 box('Controller_Dpad_h',(-.055,.024,.034),(.023,.008,.004),M['black'],.001);box('Controller_Dpad_v',(-.055,.024,.034),(.008,.023,.004),M['black'],.001)
 parent_new(before,'Game_controller',(-1.93,.68,z),-.18)
 # Black retail box, opened lid and printed booklet.
 carton('Black_package',(-1.61,.93,z),(.34,.25,.14),{**M,'cardboard':M['black'],'cardboard_light':M['black']},True,-.10)
 box('Equipment_booklet',(-1.46,.66,z+.005),(.20,.15,.008),M['black'],.002,rot=(0,0,.13))
 for i in range(6):box('Booklet_print',(-1.44,.60+i*.009,z+.009),(.11,.002,.0004),M['paper'],0,rot=(0,0,.13))
 carton('Small_top_carton',(-1.14,1.12,z),(.21,.20,.17),M,True,.08)
 box('Tissue_box',(-1.18,.665,z+.067),(.21,.14,.132),M['paper'],.014)
 box('Tissue_box_band',(-1.18,.590,z+.045),(.18,.001,.053),M['green'],.001)
 rounded_panel('Tissue_slot',(-1.18,.665,z+.134),(.094,.022,.001),M['black'],.011,0)
 mesh('Raised_tissue',[(-1.23,.66,z+.132),(-1.20,.66,z+.23),(-1.14,.67,z+.205),(-1.13,.67,z+.135),(-1.18,.68,z+.155)],[(0,1,2,3,4)],M['paper'])
 tape_roll('Blue_tape',(-1.43,.54,z),.040,.030,M['blue'],M['paper']);tape_roll('White_tape',(-1.31,.525,z),.048,.035,M['paper'],M['cardboard'])
 box('Case_top_yellow_cube',(-1.10,.487,z+.038),(.072,.072,.074),M['yellow'],.003)
 for j in range(3):box('Loose_manual',(-2.16,.90+j*.045,z+.003+j*.002),(.21,.16,.002),M['paper'],.0005,rot=(0,0,.14*j))

def floor_clutter(M,F):
 carton('Tall_shipping_carton',(-2.225,-1.555,F),(.335,.455,1.03),M,False,.03)
 carton('Bottom_cardboard_box',(-2.14,-1.12,F),(.46,.40,.195),M,False,-.04)
 carton('Stacked_cardboard_box',(-2.10,-1.07,F+.20),(.34,.32,.34),M,False,.08,tilt=(.13,-.13,0))
 carton('Tilted_open_carton',(-2.11,-.98,F+.48),(.33,.28,.35),M,True,-.10,tilt=(.15,-.25,0))
 # Dark rugged tool case with molded ribs, latches and front carrying handle.
 box('Toolcase_lower',(-2.09,-.603,F+.157),(.47,.745,.30),M['dark_metal'],.025)
 box('Toolcase_lid',(-2.09,-.603,F+.320),(.49,.76,.058),M['dark_metal'],.017)
 for y in [-.876,-.33]:
  box('Toolcase_latch_back',(-1.84,y,F+.27),(.018,.07,.12),M['black'],.004)
  box('Toolcase_silver_latch',(-1.829,y,F+.26),(.007,.049,.065),M['chrome'],.006)
 box('Toolcase_handle_plate',(-1.848,-.603,F+.18),(.012,.175,.11),M['chrome'],.008)
 tube('Toolcase_handle',[(-1.81,-.679,F+.225),(-1.79,-.679,F+.165),(-1.79,-.527,F+.165),(-1.81,-.527,F+.225)],.011,M['chrome'])
 for y in [-.9,-.72,-.49,-.31]:box('Toolcase_front_rib',(-1.852,y,F+.13),(.016,.020,.22),M['black'],.004)
 carton('Tube_cardboard_tray',(-2.095,-.593,F+.354),(.48,.73,.114),M,True,.01)
 text_obj('TUBE_mark','TUBE',(-1.85,-.60,F+.42),.048,M['black'],rotation=(math.pi/2,0,math.pi/2))
 for j in range(3):box('Documents_on_tray',(-2.08,-.66+j*.04,F+.484+j*.003),(.22,.16,.003),M['paper'],.0006,rot=(0,0,.12*j))
 bag=ellipsoid('Gray_postal_bag',(-2.12,-.42,F+.51),(.17,.11,.033),M['arm']);random.seed(5)
 for v in bag.data.vertices:v.co*=random.uniform(.96,1.04)
 carton('Open_electronics_carton',(-2.11,-.015,F),(.48,.59,.58),M,True,-.04)
 box('Power_adapter_in_carton',(-2.05,-.015,F+.47),(.17,.13,.09),M['paper'],.007)
 for j in range(7):
  mat=M['cable'] if j<4 else M['red'] if j==4 else M['blue'];smooth_tube('Carton_loose_wire',[(-2.13+j*.03,-.14,F+.49),(-2.00+j*.022,-.02,F+.70),(-1.80+j*.015,.12,F+.48),(-1.88+j*.017,.15,F+.35)],.002,mat)
 pink=material('Pale_pink_plastic',(.47,.24,.27),.8);ellipsoid('Pink_packing_bag',(-1.79,-1.12,F+.035),(.11,.14,.045),pink)
 # Blue mailing bag, white label, and bubble wrap below the workstation.
 ellipsoid('Blue_floor_mailer',(1.77,-1.05,F+.028),(.16,.12,.036),M['packing_blue'])
 box('Mailer_label',(1.765,-1.05,F+.063),(.085,.045,.0007),M['paper'],.001,rot=(0,0,-.18))
 ellipsoid('Bubble_wrap_under_bench',(1.72,.12,F+.045),(.16,.23,.05),M['paper'])
 # Multiway outlet and finned power supply on the floor.
 before=set(bpy.data.objects);box('Powerstrip_body',(0,0,.024),(.235,.063,.044),M['paper'],.010)
 for x in [-.082,-.028,.028,.082]:
  for dy in [-.011,.011]:box('Powerstrip_contact',(x,dy,.047),(.012,.004,.001),M['black'],.001)
 parent_new(before,'Floor_powerstrip',(.93,-.77,F),-.42)
 box('Finned_power_supply',(.51,-.84,F+.046),(.17,.13,.086),M['aluminum'],.006)
 for i in range(10):box('Power_supply_fin',(.435+i*.0165,-.84,F+.098),(.006,.13,.027),M['chrome'],.001)
 # Editable cable coils and wall-to-device runs, following the visible floor paths.
 for k in range(3):
  points=[]
  for j in range(100):
   a=2*math.pi*j/99;r=.23+k*.022;points.append((1.43+r*math.cos(a),-1.01+.20*math.sin(a),F+.004+k*.0015))
  tube('Blue_Ethernet_loop',points,.0022,M['blue'])
 for j in range(4):smooth_tube('Loose_black_floor_cable',[(1.86,-.3,-.80),(1.66,-.50,F+.05),(1.05,-.67-j*.04,F+.008),(.46,-.70-j*.035,F+.005),(.27,-1.08,F+.009),(.90,-1.26,F+.005)],.0023,M['cable'])
 smooth_tube('White_mains_lead',[(2.322,-.85,F+.23),(2.10,-.97,F+.08),(1.64,-1.2,F+.008),(1.27,-.94,F+.004),(.94,-.77,F+.026)],.0032,M['paper'])
 smooth_tube('White_PC_lead',[(1.66,-.64,F+.55),(1.66,-.74,F+.25),(1.40,-.73,F+.02),(.99,-.81,F+.024)],.003,M['paper'])
 smooth_tube('Writing_table_wall_cable',[(1.33,-1.93,F+.15),(1.25,-1.90,F+.02),(.79,-1.72,F+.005),(.68,-1.25,F+.005),(1.08,-1.04,F+.006)],.003,M['paper'])
 # Folded support stand visible beside the workstation window.
 for j in range(3):cylinder('Folded_tripod_leg',(2.10+j*.022,1.40,F+.02),(1.995+j*.017,1.37,-.42),.010,M['black'])
 box('Tripod_hinge',(2.02,1.38,-.42),(.10,.08,.035),M['black'],.006)
 for z in [-.55,-.78,-1.03]:cylinder('Tripod_lock',(2.00,1.36,z),(1.98,1.33,z),.012,M['black'],vertices=16)
