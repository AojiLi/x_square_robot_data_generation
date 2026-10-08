"""Behavioral checks for stage detection and fixed-motion layout screening."""
import unittest
import csv
import json
import tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from room01_sim.umi_layout import detect_events, identify_objects, obb_overlap, box_candidates, trajectory_hash, stable_warmstarts
from room01_sim.umi import load_episode, action_chunks, pose9


class LayoutTests(unittest.TestCase):
    def test_unused_invalid_raw_prefix_does_not_reject_valid_episode(self):
        import pyarrow as pa
        import pyarrow.parquet as pq
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "source/camera").mkdir(parents=True); (root / "alignment").mkdir()
            raw = []
            poses = np.tile(np.eye(4), (5, 2, 1, 1))
            poses[:, :, 0, 3] = np.arange(5)[:, None]*.01
            for i in range(5):
                row = {"timestamp_ns": 1000+i}
                for s in ("left", "right"):
                    row.update({s+"_pose_valid": int(i > 0), s+"_px": i*.01, s+"_py": 0, s+"_pz": 0,
                                s+"_qx": 0, s+"_qy": 0, s+"_qz": 0, s+"_qw": int(i > 0)})
                raw.append(row)
            with (root / "source/camera/hand_pose_test_true_absolute.csv").open("w") as f:
                writer = csv.DictWriter(f, fieldnames=list(raw[0])); writer.writeheader(); writer.writerows(raw)
            with (root / "alignment/alignment_output_grid.csv").open("w") as f:
                writer = csv.DictWriter(f, fieldnames=["is_output_anchor", "hand_pose_timestamp_ns", "e6_frame_id"]); writer.writeheader()
                writer.writerows({"is_output_anchor": int(i > 1), "hand_pose_timestamp_ns": 1000+i, "e6_frame_id": i} for i in range(1, 5))
            delta = np.eye(4); delta[0, 3] = .01
            state = np.tile(np.r_[pose9(delta), pose9(delta), np.zeros(12)], (3, 1)).astype(np.float32)
            actions, _ = action_chunks(poses[2:], np.zeros((3, 2, 6)))
            pq.write_table(pa.table({"observation.state": state.tolist(), "action": actions.tolist(), "timestamp": [0., .1, .2]}), root / "episode_000007.parquet")
            (root / "source_manifest.json").write_text(json.dumps({"episode_index": 7}))
            loaded = load_episode(root)
            self.assertEqual(loaded["episode_index"], 7)
            self.assertEqual(loaded["source_raw_indices"].tolist(), [2, 3, 4])

    def test_failed_repeat_invalidates_successful_cached_layout(self):
        good = {"status": "complete", "phase": "grasp_probe", "reference_fingerprint": "v1",
                "initial_object_poses": {"block": [0, 0, 0, 0, 0, 0, 1]},
                "grasps": {"block": {"evaluated": True, "passed": True}}}
        bad = {**good, "phase": "record_probe", "grasps": {"block": {"evaluated": True, "passed": False}}}
        self.assertEqual(len(stable_warmstarts([("good", good)], "v1")), 1)
        self.assertEqual(stable_warmstarts([("good", good), ("repeat", bad)], "v1"), [])
        self.assertEqual(stable_warmstarts([("good", good)], "different_model"), [])

    def test_asset_detection_does_not_treat_skin_as_red_cube(self):
        image = np.full((480, 640, 3), 240, np.uint8)
        image[245:310, 270:340] = 20
        image[335:360, 280:305] = [200, 20, 20]
        image[335:360, 330:355] = [20, 100, 200]
        image[335:380, 190:230] = [160, 110, 90]
        self.assertEqual(identify_objects(image)["objects_by_side"], {"left": "RedBig", "right": "Blue"})

    def test_stage_detection_uses_signal_not_episode_timestamps(self):
        times = np.arange(100)/10
        fingers = np.zeros((100, 2, 6))
        fingers[:, :, 1] = 60  # Opposition alone must not count as a grasp.
        fingers[12:32, 1, 2:4] = 30
        fingers[57:79, 0, 2:4] = 20
        first = detect_events(times, fingers)
        shifted = detect_events(times+37., fingers)
        self.assertEqual([e["side"] for e in first], ["right", "left"])
        np.testing.assert_allclose([e["onset_s"]+37 for e in first], [e["onset_s"] for e in shifted])
        self.assertEqual([e["release_index"] for e in first], [32, 79])

    def test_sat_checks_rotated_box_edges(self):
        centers = np.array([[0., 0, 0], [2., 0, 0]])
        rotation = np.repeat(Rotation.from_euler("z", .6).as_matrix()[None], 2, axis=0)
        hit = obb_overlap(centers, rotation, np.array([.5, .2, .2]), np.zeros(3), np.eye(3), np.array([.3, .3, .3]))
        np.testing.assert_array_equal(hit, [True, False])

    def make_trace(self, lifted):
        times = np.linspace(0, 4, 81)
        poses = np.zeros((len(times), 5, 7)); poses[..., 6] = 1.
        names = ["RedBig", "Blue", "Green", "RedSmall", "BlackBox"]
        initial = {}
        for i, y in [(0, -1.87), (1, -1.95)]:
            poses[:, i, 0] = np.interp(times, [0, 1, 2, 4], [.80, .80, .98, .98])
            poses[:, i, 1] = y
            poses[:, i, 2] = np.interp(times, [0, .8, 2.5, 3.3, 4], [.748, 1.02, 1.02, .748, .748]) if lifted else .748
            initial[names[i]] = poses[0, i].tolist()
        events = [{"asset": "RedBig", "release_s": 3.3}, {"asset": "Blue", "release_s": 3.3}]
        return {"object_names": names, "source_time": times, "object_poses": poses}, events, initial

    def test_joint_release_region_can_produce_box_layout(self):
        trace, events, initial = self.make_trace(True)
        candidates, audit = box_candidates(trace, events, initial, limit=2)
        self.assertGreater(len(candidates), 0)
        self.assertEqual(set(candidates[0]["poses"]), {"RedBig", "Blue", "Green", "RedSmall", "BlackBox"})

    def test_low_sweep_rejected_even_when_release_points_fit(self):
        trace, events, initial = self.make_trace(False)
        candidates, audit = box_candidates(trace, events, initial)
        self.assertEqual(candidates, [])
        self.assertGreater(audit["containment_candidates"], 0)
        self.assertGreater(audit["swept_wall_rejections"], 0)

    def test_reference_digest_detects_motion_changes(self):
        data = {"times": np.arange(3.), "targets": np.zeros((3, 26)),
                "hand_targets": np.tile(np.eye(4), (3, 2, 1, 1)), "tool_to_hand_base": np.tile(np.eye(4), (2, 1, 1))}
        before = trajectory_hash(data)
        data["targets"][1, 0] += .001
        self.assertNotEqual(before, trajectory_hash(data))


if __name__ == "__main__":
    unittest.main()
