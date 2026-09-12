from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import json,hashlib
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/room01_full_room_model'
font=ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',24)
views=[('ReviewChairs',77,'座椅与窗户'),('ReviewBenchFront',394,'工作台'),('ReviewTable',100,'白色桌子'),('ReviewDoor',54,'门与玻璃隔断'),('ReviewStorage',85,'储物区域'),('ReviewCeiling',440,'吊顶')]
for page in range(2):
 pairs=views[page*3:(page+1)*3];im=Image.new('RGB',(1488,1824),'#f2f4f5');d=ImageDraw.Draw(im);d.text((12,6),'原始照片 · 相同机位裁剪',font=font,fill='#172734');d.text((756,6),'Blender Cycles · 重建模型',font=font,fill='#172734')
 for row,(name,idx,title) in enumerate(pairs):
  photo=Image.open(OUT/f'frame_{idx:03d}.png').convert('RGB').crop((0,220,736,772));render=Image.open(OUT/f'{name}.png').convert('RGB');assert photo.size==render.size==(736,552)
  y=44+row*590;im.paste(photo,(0,y));im.paste(render,(752,y));d.text((12,y+552),title,font=font,fill='#172734')
 im.save(OUT/f'photo_comparison_{page+1}.png')
manifest=json.loads((OUT/'model_manifest.json').read_text());manifest['blend_sha256']=hashlib.sha256((ROOT/'assets/room01/full_room_model/room01_full.blend').read_bytes()).hexdigest();manifest['renders']=['Overview_cutaway']+[name for name,_,_ in views]+['ReviewWorkbench'];manifest['camera_projection_check_scope']='Numerical verification of coordinate/intrinsic conversion, not a scene accuracy score';(OUT/'delivery_manifest.json').write_text(json.dumps(manifest,indent=2))
print('REVIEW_ARTIFACTS_READY')
