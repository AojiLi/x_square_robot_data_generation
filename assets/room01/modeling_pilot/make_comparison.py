from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[3];OUT=ROOT/'reports/room01_modeling_pilot'
font_path=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if not font_path.exists():font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
font=lambda size:ImageFont.truetype(str(font_path),size)
figure=Image.new('RGB',(1764,790),'#f4f5f7');draw=ImageDraw.Draw(figure)
draw.text((18,8),'同机位局部对比 · Isaac Sim 实际输出',font=font(30),fill='#17232d')
for col,label in enumerate(['原始照片','旧场景：高斯','建模样板：网格与简化材质']):draw.text((col*588+10,52),label,font=font(23),fill='#283d4b')
for row,(name,source,box) in enumerate([
 ('ReviewTable','357971838221',(80,200,660,490)),
 ('ReviewRoom','357755788663',(40,0,620,290))]):
 photo=Image.open(ROOT/'data/room01/images'/f'{source}.png').convert('RGB').transpose(Image.Transpose.ROTATE_270).crop((0,220,736,772))
 old=Image.open(ROOT/'reports/room01_blur_diagnosis/fixed_scene_v3'/f'{name}.png').convert('RGB')
 new=Image.open(OUT/f'{name}.png').convert('RGB')
 assert old.size==new.size==photo.size==(736,552)
 for col,im in enumerate([photo,old,new]):figure.paste(im.crop(box),(col*588+4,90+row*305))
draw.text((16,708),'右列另设预览灯光，背景为扫描网格。窗外景色未重建，材质尚未还原。',font=font(23),fill='#283d4b')
draw.text((16,746),'裁剪均为原尺寸像素；清晰边界不等于照片还原度提高。',font=font(22),fill='#283d4b')
figure.save(OUT/'comparison.png')
full=Image.new('RGB',(1488,1180),'#f4f5f7');d=ImageDraw.Draw(full)
for col,label in enumerate(['旧场景：高斯','建模样板：扫描网格背景']):d.text((col*744+10,6),label,font=font(24),fill='#17232d')
for row,name in enumerate(['ReviewTable','ReviewRoom']):
 for col,base in enumerate([ROOT/'reports/room01_blur_diagnosis/fixed_scene_v3',OUT]):full.paste(Image.open(base/f'{name}.png'),(col*744,42+row*564))
full.save(OUT/'full_views.png')
print('COMPARISONS_SAVED')
