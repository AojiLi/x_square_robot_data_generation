#!/usr/bin/env python3
"""Re-author only saved camera optics/mounts after editing task_config.json."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pxr import Usd
from room01_sim.camera_assets import author_cameras
from room01_sim.calibration import validate_optics

asset = ROOT / "assets/room01/sim"
config = json.loads((asset / "task_config.json").read_text())
for spec in config["cameras"].values():
    validate_optics(spec)
stage = Usd.Stage.Open(str(asset / "room01_manipulation.usda"))
records = author_cameras(stage, config)
stage.GetRootLayer().Save()
print(json.dumps({name: {key: spec[key] for key in ["resolution", "model", "fx", "fy", "cx", "cy"]}
                  for name, spec in records.items()}, indent=2))
print("Camera changes require a fresh camera/interface check and a new episode before finalizing a delivery.")
