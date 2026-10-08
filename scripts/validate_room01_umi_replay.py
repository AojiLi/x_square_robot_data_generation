#!/usr/bin/env python3
"""Validate source provenance, scene geometry and delivered simulation labels."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pyarrow.parquet as pq
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.umi import load_episode, action_chunks, inverse, pose9, source_fingers_from_q


def validate(args):
    from pxr import Usd, UsdPhysics
    checks = {}
    source = load_episode(args.source)
    manifest = json.loads((args.source / "source_manifest.json").read_text())
    for item in manifest["files"]:
        if hashlib.sha256((args.source / item["local_path"]).read_bytes()).hexdigest() != item["sha256"]:
            raise AssertionError("Source checksum mismatch: "+item["local_path"])
    checks["source_files_sha256"] = len(manifest["files"])
    checks["source_pose_contract"] = source["audit"]
    a, mask = action_chunks(source["source_poses"], source["fingers_deg"])
    np.testing.assert_allclose(a[~mask], source["source_action"][~mask], atol=4e-8)
    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler("xyz", [.4, -.7, 1.1]).as_matrix()
    world[:3, 3] = [1., 2., 3.]
    a2, mask2 = action_chunks(world @ source["source_poses"], source["fingers_deg"])
    np.testing.assert_allclose(a, a2, atol=1e-6)
    np.testing.assert_array_equal(mask, mask2)
    checks["shared_anchor_actions_and_world_invariance"] = True
    stage = Usd.Stage.Open(str(ROOT / "assets/room01/umi_replay/room01_umi_replay.usda"))
    cfg = json.loads((ROOT / "assets/room01/umi_replay/task_config.json").read_text())
    scene_checks = {}
    for name, spec in cfg["task_objects"].items():
        prim = stage.GetPrimAtPath(spec["prim_path"])
        assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
        mass = UsdPhysics.MassAPI(prim).GetMassAttr().Get()
        assert abs(mass-spec["mass_kg"]) < 1e-7
        shapes = [child for child in prim.GetChildren() if child.HasAPI(UsdPhysics.CollisionAPI)]
        assert len(shapes) == (5 if name == "BlackBox" else 1)
        scene_checks[name] = {"mass_kg": mass, "collider_count": len(shapes)}
    assert not stage.GetPrimAtPath("/World/Task/RedCube").IsActive()
    checks["dynamic_scene_objects"] = scene_checks
    replay = json.loads((args.episode / "replay.json").read_text())
    obs = np.load(args.episode / "observations.npz")
    control = np.load(args.episode / "controls.npz")
    for key, array in obs.items():
        assert np.isfinite(array).all(), key
    np.testing.assert_allclose(np.diff(obs["timestamp"]), .1, atol=1e-8)
    np.testing.assert_allclose(np.diff(control["time"]), 1/30, atol=1e-8)
    np.testing.assert_allclose(source_fingers_from_q(obs["joint_positions"]), obs["fingers_deg"], atol=1e-6)
    checks["finite_states_finger_mapping_10hz_observations_30hz_controls"] = True
    if replay["images_recorded"]:
        for camera in replay["camera_names"]:
            assert len(list((args.episode / "images" / camera).glob("*.png"))) == len(obs["timestamp"])
        assert np.max(np.ptp(obs["camera_timestamps"], axis=1)) < 1e-9
        np.testing.assert_allclose(obs["camera_timestamps"][:, 0], obs["simulation_timestamp"], atol=1e-9)
        checks["all_three_native_cameras_synchronized"] = True
    if args.dataset:
        data = pq.read_table(args.dataset / "data/chunk-000/episode_000000.parquet")
        states = np.asarray(data["observation.state"].to_pylist())
        previous = np.concatenate([obs["previous_tool_poses"][None], obs["tool_poses"][:-1]])
        delta = inverse(previous) @ obs["tool_poses"]
        expected = np.concatenate([pose9(delta[:, 0]), pose9(delta[:, 1]), obs["fingers_deg"].reshape(-1, 12)], axis=-1)
        np.testing.assert_allclose(states, expected, atol=2e-6)
        actions, pads = action_chunks(obs["tool_poses"], obs["fingers_deg"])
        np.testing.assert_array_equal(np.asarray(data["action"].to_pylist()), actions)
        np.testing.assert_array_equal(np.asarray(data["action_is_pad"].to_pylist()), pads)
        stats = json.loads((args.dataset / "meta/stats.json").read_text())
        assert stats["action"]["count"] == [int((~pads).sum())]
        provenance = json.loads((args.dataset / "meta/simulation_provenance.json").read_text())
        assert provenance["origin"] == "simulation"
        assert provenance["task_success"] == replay["task_success"]
        if not replay["task_success"]:
            assert not provenance["training_eligible"]
        checks["export_labels_from_achieved_motion_and_padding_stats"] = True
        checks["failure_not_marked_training_success"] = True
    result = {"engineering_checks_pass": True, "task_success": replay["task_success"],
              "episode": str(args.episode.resolve()), "dataset": str(args.dataset.resolve()) if args.dataset else None,
              "checks": checks,
              "limits": {"tcp_measured": False, "scene_registration_measured": False,
                         "camera_extrinsics_measured": False, "wood_friction_measured": False,
                         "real_robot_training_validated": False}}
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--dataset", type=Path)
    parser.add_argument("--source", type=Path, default=ROOT / "data/umi_replay/episode_000000")
    parser.add_argument("--report", type=Path)
    validate(parser.parse_args())
