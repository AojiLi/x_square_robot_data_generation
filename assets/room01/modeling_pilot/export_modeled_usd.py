"""Export evaluated Blender geometry to USD without altering the source scan."""
from pathlib import Path
import json,numpy as np
from pxr import Usd,UsdGeom,UsdShade,Sdf,Gf,Vt
HERE=Path(__file__).resolve().parent
parts=json.loads((HERE/'modeled_parts_geometry.json').read_text());s=Usd.Stage.CreateNew(str(HERE/'modeled_parts.usdc'));root=UsdGeom.Xform.Define(s,'/Pilot');s.SetDefaultPrim(root.GetPrim());UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
for item in parts:
 name=item['name'];m=UsdGeom.Mesh.Define(s,'/Pilot/'+name);p=np.array(item['points'],dtype=np.float32)
 m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(p));m.CreateFaceVertexCountsAttr([len(f) for f in item['faces']]);m.CreateFaceVertexIndicesAttr([v for f in item['faces'] for v in f]);m.CreateSubdivisionSchemeAttr('none');m.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.stack([p.min(0),p.max(0)])))
 material=UsdShade.Material.Define(s,'/Pilot/Materials/'+name);shader=UsdShade.Shader.Define(s,str(material.GetPath())+'/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*item['color']));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(item['roughness']);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(item['metallic']);material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material)
 if name.startswith('Desk') or name.startswith('Glass'):
  shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.92)
  shader.CreateInput('useSpecularWorkflow',Sdf.ValueTypeNames.Int).Set(1)
  shader.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0))
 m.GetPrim().SetCustomData({'source':'Evaluated Blender 4.0.2 mesh; approximate modeling study'})
s.GetRootLayer().Save();print('USD_EXPORTED',len(parts))
