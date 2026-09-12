#!/usr/bin/env python3
"""Recover exact STL payloads from the user's existing USD conversion.

Nothing is accepted unless the bytes match the original Git LFS SHA-256. A
downloaded prefix can restore STL headers and signed-zero encodings lost in USD.
Unrecoverable files remain pending normal Git LFS download.
"""
from pathlib import Path
import hashlib
import json
import struct
import xml.etree.ElementTree as ET

import numpy as np
from pxr import Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/robots/quanta_x2"
STORAGE = ROOT / "tools/robot_lfs"


def main():
    manifest = json.loads((ROOT / "reports/room01_reconstruction/robot_lfs_manifest.json").read_text())
    entries = {row["relative"]: row for row in manifest}
    stage = Usd.Stage.Open(str(ASSETS / "quanta_x2_robot.usdc"))
    meshes = [p for p in stage.Traverse(Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
    robot = ET.parse(ASSETS / "quanta_x2_dual_revo2_bridge.urdf").getroot()
    headers = {b"\0"*80, b" "*80}
    for row in manifest:
        oid = row["oid"]
        cache = STORAGE / "objects" / oid[:2] / oid[2:4] / oid
        paths = list((STORAGE / "incomplete").glob(oid+"*"))
        if cache.is_file():
            paths.append(cache)
        for path in paths:
            if path.stat().st_size >= 84:
                with path.open("rb") as stream:
                    headers.add(stream.read(80))
    recovered = []
    for link in robot.findall("link"):
        name = link.get("name")
        usd_name = "_"+name if name[0].isdigit() else name
        for kind in ["visual", "collision"]:
            for source in link.findall(kind):
                mesh_reference = source.find("geometry/mesh")
                if mesh_reference is None:
                    continue
                relative = mesh_reference.get("filename")
                row = entries[relative]
                oid = row["oid"]
                target = STORAGE / "objects" / oid[:2] / oid[2:4] / oid
                if target.exists():
                    continue
                prefixes = list((STORAGE / "incomplete").glob(oid+"*"))
                prefix = max(prefixes, key=lambda p: p.stat().st_size).read_bytes() if prefixes else b""
                candidates = [prefix[:80]] if len(prefix) >= 84 else list(headers)
                anchor = str(stage.GetDefaultPrim().GetPath())+"/"+usd_name+"/"+kind+"s/"
                for prim in meshes:
                    if not str(prim.GetPath()).startswith(anchor):
                        continue
                    mesh = UsdGeom.Mesh(prim)
                    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
                    if not (counts == 3).all() or row["size"] != 84+50*len(counts):
                        continue
                    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
                    vertices = np.asarray(mesh.GetPointsAttr().Get(), dtype=np.float32)
                    normals = np.asarray(mesh.GetNormalsAttr().Get(), dtype=np.float32)
                    encoding = mesh.GetNormalsInterpolation()
                    if encoding == "faceVarying":
                        face_normals = normals.reshape(-1, 3, 3)[:, 0]
                    elif encoding == "vertex":
                        face_normals = normals[indices[:, 0]]
                    elif len(normals) == len(counts):
                        face_normals = normals
                    else:
                        continue
                    records = np.zeros(len(counts), dtype=[("normal", "<f4", 3),
                                                             ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
                    records["normal"] = face_normals
                    records["vertices"] = vertices[indices]
                    body = struct.pack("<I", len(counts))+records.tobytes()
                    for header in candidates:
                        blob = header+body
                        matched = hashlib.sha256(blob).hexdigest() == oid
                        used_prefix = False
                        if not matched and prefix:
                            blob = prefix+blob[len(prefix):]
                            used_prefix = True
                            matched = hashlib.sha256(blob).hexdigest() == oid
                        if matched:
                            target.parent.mkdir(parents=True, exist_ok=True)
                            temporary = target.with_suffix(".recovering")
                            temporary.write_bytes(blob)
                            temporary.replace(target)
                            recovered.append({"file": relative, "sha256": oid, "bytes": len(blob),
                                              "restored_prefix_bytes": len(prefix) if used_prefix else 80})
                            print("Recovered original bytes:", relative, flush=True)
                            break
                    if target.exists():
                        break
    path = ROOT / "reports/room01_reconstruction/robot_mesh_recovery.json"
    existing = json.loads(path.read_text()) if path.exists() else []
    known = {row["sha256"]: row for row in existing}
    known.update({row["sha256"]: row for row in recovered})
    path.write_text(json.dumps(list(known.values()), indent=2)+"\n")
    print("New recovered objects:", len(recovered), "bytes:", sum(row["bytes"] for row in recovered))


if __name__ == "__main__":
    main()
