import bpy,math
from pathlib import Path
HERE=Path(__file__).resolve().parent
bpy.data.objects['Thermostat_digits'].rotation_euler.z=math.pi
bpy.context.scene.cycles.samples=256;bpy.context.scene.cycles.use_denoising=False
bpy.ops.wm.save_as_mainfile(filepath=str(HERE/'room01_full.blend'))
print('FINAL_BLEND_SAVED',flush=True)
