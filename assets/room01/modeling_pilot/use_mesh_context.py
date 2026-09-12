from pathlib import Path
import json,numpy as np
from pxr import Usd,UsdGeom,Sdf,Vt
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
fit=json.loads((ROOT/'reports/room01_modeling_pilot/geometry_fit.json').read_text())
source=Usd.Stage.Open(str(HERE.parent/'scan_mesh.usdc'));out=Usd.Stage.CreateNew(str(HERE/'context_scan_mesh.usdc'))
Sdf.CopySpec(source.GetRootLayer(),'/ScanMesh',out.GetRootLayer(),'/ScanMesh');out.SetDefaultPrim(out.GetPrimAtPath('/ScanMesh'));UsdGeom.SetStageUpAxis(out,'Z');UsdGeom.SetStageMetersPerUnit(out,1.)
removed=0;remaining=0
for prim in out.Traverse():
 if prim.IsA(UsdGeom.Mesh):
  m=UsdGeom.Mesh(prim);p=np.array(m.GetPointsAttr().Get());counts=np.array(m.GetFaceVertexCountsAttr().Get());assert np.all(counts==3)
  indices=np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1,3);centers=p[indices].mean(1);x,y,z=centers.T;n=np.array(fit['window_normal_xy']);along=x*n[1]-y*n[0];distance=x*n[0]+y*n[1]-fit['window_offset']
  rm=((x>.60)&(x<1.46)&(y>-2.77)&(y<-1.28)&(z>fit['table_top_z']-.13)&(z<fit['table_top_z']+.12))|((distance>-.25)&(along>-.15)&(along<3.4)&(z>-1.55)&(z<1.65))
  rm|=((x>1.22)&(x<1.52)&(y>-2.78)&(y<-1.27)&(z>-.85)&(z<1.45))|((distance>-.68)&(along>-.15)&(along<3.4)&(z>-.45)&(z<1.65))
  selected=indices[~rm];m.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(selected),3,np.int32)));m.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(selected.reshape(-1).astype(np.int32)));removed+=int(rm.sum());remaining+=len(selected)
 for attr in prim.GetAttributes():
  if attr.GetTypeName()==Sdf.ValueTypeNames.Asset:
   value=attr.Get()
   if value and value.path.startswith('textures/'):
    attr.Set(Sdf.AssetPath('../'+value.path))
out.GetRootLayer().Save()
s=Usd.Stage.Open(str(HERE/'room01_modeling_pilot.usda'));s.DefinePrim('/World/ScanContext').GetReferences().AddReference('context_scan_mesh.usdc')
d=dict(s.GetRootLayer().customLayerData);d['pilotNote']='Blender geometry study with trimmed original scan mesh context. Plain window infill; exterior and remaining furniture not remodeled. Approximate materials; no physics.';s.GetRootLayer().customLayerData=d;s.GetRootLayer().Save()
(ROOT/'reports/room01_modeling_pilot/context_mesh.json').write_text(json.dumps({'background':'trimmed original textured scan mesh','removed_triangles':removed,'remaining_triangles':remaining,'gaussians_active':False},indent=2))
print('MESH_CONTEXT',removed,remaining)
