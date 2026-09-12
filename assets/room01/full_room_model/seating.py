import bpy,math
from modeling_lib import *

def chair(name,center,yaw,M,curved=False):
 before=set(bpy.data.objects);black=M['black'];fabric=M['fabric'];steel=M['chrome'];seat_z=.408 if curved else .435
 # Lift, under-seat mechanism, and five casters.
 cylinder(name+'_gas_lift',(0,0,.13),(0,0,.40),.022,steel)
 cylinder(name+'_lift_sleeve',(0,0,.11),(0,0,.29),.044,black)
 box(name+'_tilt_mechanism',(0,.015,.365),(.21,.22,.062),black,.017)
 for j in range(5):
  a=2*math.pi*j/5+.15;end=(.31*math.cos(a),.31*math.sin(a),.070)
  cylinder(name+f'_spoke_{j}',(.04*math.cos(a),.04*math.sin(a),.15),end,.038,black,r2=.021,vertices=12)
  cylinder(name+f'_caster_pin_{j}',end,(end[0],end[1],.035),.01,steel)
  # Twin castor wheels, with an axle tangential to the spoke.
  ux,uy=-math.sin(a),math.cos(a)
  for k in [-1,1]:
   middle=(end[0]+ux*k*.021,end[1]+uy*k*.021,.034)
   cylinder(name+f'_wheel_{j}_{k}',(middle[0]-ux*.011,middle[1]-uy*.011,middle[2]),(middle[0]+ux*.011,middle[1]+uy*.011,middle[2]),.031,black,vertices=20)
 rounded_panel(name+'_seat',(0,-.012,seat_z),(.49,.475,.082),fabric,.08,.014)
 # Backrest surface: a slightly bowed manufactured mesh, with separate strands.
 def back_point(x,z):
  yn=.205+.055*(z-.49)/.58 + (.025 if curved else .012)*(1-(x/.25)**2)
  return (x,yn,z)
 rows=[]
 for i in range(184):
  z=.470+i*.0025;w=.210+.014*math.sin((z-.5)*math.pi/.5)
  rows.append([back_point(-w+2*w*j/16,z) for j in range(17)])
 for i in range(60):
  x=-.208+i*.416/59;rows.append([back_point(x,.470+.457*j/18) for j in range(19)])
 batch_rods(name+'_woven_back',rows,.00055,black,5)
 # Molded flat back frame, with a rounded inner opening and a real thickness.
 outer=[];inner=[]
 for target,w,h,r in [(outer,.49,.552,.043),(inner,.413,.472,.026)]:
  midz=.696
  for cx,cz,a0 in [(w/2-r,midz+h/2-r,0),(-w/2+r,midz+h/2-r,math.pi/2),(-w/2+r,midz-h/2+r,math.pi),(w/2-r,midz-h/2+r,3*math.pi/2)]:
   for k in range(12):
    a=a0+k*math.pi/22;target.append(back_point(cx+r*math.cos(a),cz+r*math.sin(a)))
 n=len(outer);vs=outer+inner+[(x,y+.029,z) for x,y,z in outer]+[(x,y+.029,z) for x,y,z in inner];fs=[]
 for i in range(n):
  j=(i+1)%n;fs.extend([(i,j,n+j,n+i),(2*n+i,3*n+i,3*n+j,2*n+j),(i,2*n+i,2*n+j,j),(n+i,n+j,3*n+j,3*n+i)])
 bevel(mesh(name+'_molded_back_frame',vs,fs,black),.006,3)
 cylinder(name+'_back_support',(0,.16,.36),(0,.253,.80),.023,black,vertices=12)
 if not curved:box(name+'_lumbar',(0,.211,.602),(.37,.054,.090),fabric,.032)
 # Headrest pad and adjustable brackets.
 cylinder(name+'_head_bracket',(0,.262,.935),(0,.284,1.035),.023,black,vertices=12)
 head=rounded_panel(name+'_headrest',(0,0,0),(.33,.16,.076),fabric,.037,.013)
 head.rotation_euler.x=math.pi/2;head.location=(0,.286,1.06)
 for side in [-1,1]:
  box(name+f'_arm_post_{side}',(side*.30,.052,.515 if curved else .55),(.037,.054,.215 if curved else .255),black,.013)
  rounded_panel(name+f'_arm_pad_{side}',(side*.305,.022,.658 if curved else .704),(.090,.265,.043),M['arm'],.029,.008)
  cylinder(name+f'_arm_brace_{side}',(side*.15,.015,.378),(side*.30,.060,.433),.018,black)
 tube(name+'_adjust_lever',[(.12,0,.355),(.25,-.05,.335),(.29,-.10,.36)],.008,black)
 box(name+'_lever_grip',(.29,-.10,.36),(.052,.044,.017),black,.006)
 root=parent_new(before,name,center,yaw);root['source_photo_frames']='77,247,270,363,479';root['geometry_status']='Photo and scan fitted; profile dimensions approximate'
 return root

def stool(name,center,M):
 before=set(bpy.data.objects);b=M['black'];c=M['chrome']
 cylinder(name+'_lift',(0,0,.12),(0,0,.43),.023,c);cylinder(name+'_sleeve',(0,0,.1),(0,0,.30),.045,b)
 for j in range(5):
  a=2*math.pi*j/5;end=(.29*math.cos(a),.29*math.sin(a),.07);cylinder(name+f'_spoke_{j}',(0,0,.14),end,.035,b,r2=.02)
  ux,uy=-math.sin(a),math.cos(a)
  for k in [-1,1]:
   mid=(end[0]+ux*k*.02,end[1]+uy*k*.02,.033);cylinder(name+f'_wheel_{j}_{k}',(mid[0]-ux*.01,mid[1]-uy*.01,.033),(mid[0]+ux*.01,mid[1]+uy*.01,.033),.03,b)
 cylinder(name+'_seat',(0,0,.425),(0,0,.495),.195,M['fabric'],vertices=64)
 torus(name+'_seat_piping',(0,0,.486),.189,.0025,M['arm'])
 return parent_new(before,name,center,0)
