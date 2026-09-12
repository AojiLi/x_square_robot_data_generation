from pathlib import Path
import json,numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/room01_full_room_model'
s=np.load(OUT/'scan_geometry.npz');p=s['points'];bvh=BVHTree.FromPolygons(p.tolist(),s['triangles'].tolist(),all_triangles=True)
frames=json.loads((ROOT/'data/room01/transforms.json').read_text())['frames']
anchors=[(77,'chair_left_seat',295,795),(77,'chair_right_seat',585,770),(77,'chair_left_head',313,462),(77,'chair_right_head',518,470),
 (394,'workbench_front_left',151,437),(394,'workbench_front_right',654,489),(394,'monitor_top_left',312,185),(394,'monitor_top_right',567,197),(394,'monitor_bottom_center',421,355),
 (46,'case_top_left',287,649),(46,'case_top_right',685,814),(46,'case_front_center',489,866),
 (85,'tall_carton_center',63,695),(85,'toolcase_center',454,884),(85,'open_carton_right',674,807),(85,'small_cartons',209,852),
 (440,'smoke_detector',389,622),(440,'ceiling_grille',211,628)]
anchors += [(247,'chair_workbench_seat',287,655),(247,'chair_workbench_back',211,246),(270,'chair_left_seat_close',272,771),(262,'stool_seat',516,409),(262,'stool_hub',443,727)]
results=[]
for idx,label,x,y in anchors:
 f=frames[idx];t=np.array(f['transform_matrix']);u=y;v=f['h']-1-x;ray=t[:3,:3]@np.array([(u-f['cx'])/f['fl_x'],(f['cy']-v)/f['fl_y'],-1.]);ray/=np.linalg.norm(ray)
 hit,normal,face,dist=bvh.ray_cast(Vector(t[:3,3]),Vector(ray),20)
 item={'label':label,'frame':idx,'upright_pixel':[x,y],'hit':list(hit) if hit else None,'normal':list(normal) if normal else None,'distance':dist};results.append(item);print(label,None if hit is None else np.round(hit,3).tolist())
(OUT/'photo_landmarks.json').write_text(json.dumps(results,indent=2))
