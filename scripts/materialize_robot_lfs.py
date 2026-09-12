#!/usr/bin/env python3
"""Resolve copied robot LFS pointers from this project's verified object cache."""
import hashlib
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def main():
    report_dir = ROOT / "reports/room01_reconstruction"
    rows = json.loads((report_dir / "robot_lfs_manifest.json").read_text())
    destination = ROOT / "assets/robots/quanta_x2"
    storage = ROOT / "tools/robot_lfs/objects"
    pending, ready = [], []
    for row in rows:
        oid = row["oid"]
        blob = storage / oid[:2] / oid[2:4] / oid
        target = destination / row["relative"]
        if not blob.is_file():
            pending.append(row["relative"])
            continue
        if blob.stat().st_size != row["size"] or hashlib.sha256(blob.read_bytes()).hexdigest() != oid:
            raise ValueError(f"Invalid LFS payload: {row['relative']}")
        if target.stat().st_size < 512:
            pointer = target.read_text()
            if f"oid sha256:{oid}" not in pointer:
                raise ValueError(f"Destination changed unexpectedly: {target}")
            temporary = target.with_name(target.name+".materializing")
            shutil.copyfile(blob, temporary)
            temporary.replace(target)
        if target.stat().st_size != row["size"] or hashlib.sha256(target.read_bytes()).hexdigest() != oid:
            raise ValueError(f"Copied LFS payload mismatch: {row['relative']}")
        ready.append(row["relative"])
    urdf = destination / "quanta_x2_dual_revo2_bridge.urdf"
    robot = ET.parse(urdf).getroot()
    mesh_names = sorted({mesh.get("filename") for mesh in robot.findall(".//mesh")})
    missing, pointers = [], []
    for name in mesh_names:
        path = destination / name
        if not path.is_file():
            missing.append(name)
        elif path.stat().st_size < 512 and path.read_bytes().startswith(b"version https://git-lfs.github.com/spec/v1"):
            pointers.append(name)
    status = {"urdf": str(urdf), "urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest(),
              "robot_name": robot.get("name"), "links": len(robot.findall("link")),
              "joints": len(robot.findall("joint")), "unique_mesh_references": len(mesh_names),
              "lfs_files_materialized": len(ready), "lfs_files_pending": pending,
              "missing_meshes": missing, "mesh_pointers_remaining": pointers,
              "ready_to_load_urdf": not missing and not pointers,
              "verification": "Every materialized resource matches the SHA-256 recorded in its source LFS pointer."}
    (report_dir / "robot_asset_checks.json").write_text(json.dumps(status, indent=2)+"\n")
    print(json.dumps({key: value for key, value in status.items() if key not in
                      ["lfs_files_pending", "missing_meshes", "mesh_pointers_remaining"]}, indent=2))
    print(f"Pending LFS files: {len(pending)}; unresolved mesh references: {len(pointers)+len(missing)}")


if __name__ == "__main__":
    main()
