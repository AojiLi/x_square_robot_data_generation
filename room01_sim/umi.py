"""Auditable UMI v2 trajectory decoding and robot-space replay math.

This module has no Isaac imports. Dataset poses use column-vector transforms;
finger values are measured absolute degrees, not normalized gripper widths.
"""
from pathlib import Path
import csv
import json
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.optimize import least_squares

from room01_sim.kinematics import RobotKinematics, ARM_SUFFIXES, HAND_SUFFIXES, ACTION_JOINTS, rotation

SIDES = ("left", "right")
# Source flex/aux correspond to the 59/90-degree URDF drives respectively.
FINGER_TO_URDF = np.array([1, 0, 2, 3, 4, 5])


def read_csv(path):
    with Path(path).open(newline="") as handle:
        return list(csv.DictReader(handle))


def verify_source_package(directory, require_manifest_files=True):
    directory = Path(directory).resolve()
    manifest_path = directory / "source_manifest.json"
    content = manifest_path.read_bytes(); manifest = json.loads(content)
    files = manifest.get("files", [])
    if require_manifest_files and not files:
        raise ValueError("Source package has no file checksum manifest")
    digest = hashlib.sha256(content)
    for item in files:
        path = (directory / item["local_path"]).resolve()
        if not path.is_relative_to(directory):
            raise ValueError("Source manifest path escapes its episode package")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != item["sha256"]:
            raise ValueError("Source checksum mismatch: "+item["local_path"])
        digest.update(item["local_path"].encode()); digest.update(actual.encode())
    return digest.hexdigest()


def absolute_matrices(rows, side):
    matrices = np.broadcast_to(np.eye(4), (len(rows), 4, 4)).copy()
    matrices[:, :3, 3] = [[float(row[f"{side}_p{axis}"]) for axis in "xyz"] for row in rows]
    matrices[:, :3, :3] = Rotation.from_quat([
        [float(row[f"{side}_q{axis}"]) for axis in "xyzw"] for row in rows
    ]).as_matrix()
    return matrices


def pose9(matrices):
    return np.concatenate([matrices[..., :3, 3], matrices[..., :3, 0], matrices[..., :3, 1]], axis=-1)


def inverse(matrices):
    result = np.zeros_like(matrices)
    result[..., :3, :3] = matrices[..., :3, :3].swapaxes(-1, -2)
    result[..., :3, 3] = -(result[..., :3, :3] @ matrices[..., :3, 3, None])[..., 0]
    result[..., 3, 3] = 1
    return result


def load_episode(directory):
    directory = Path(directory)
    verify_source_package(directory, require_manifest_files=False)
    parquet_files = list(directory.glob("episode_*.parquet"))
    if len(parquet_files) != 1:
        raise ValueError("An episode package must contain exactly one episode Parquet file")
    parquet_path = parquet_files[0]
    cached = directory / "decoded_arrays.npz"
    try:
        import pyarrow.parquet as pq
    except ModuleNotFoundError:
        pq = None
    if pq is not None:
        table = pq.read_table(parquet_path)
        values = {key: table[key].to_pylist() for key in ("observation.state", "action", "timestamp")}
    elif cached.exists():
        # Optional lossless decoding by an existing PyArrow installation. The
        # cache is accepted only for the exact, locally retained Parquet file.
        with np.load(cached, allow_pickle=False) as arrays:
            expected = hashlib.sha256(parquet_path.read_bytes()).hexdigest()
            if str(arrays["parquet_sha256"]) != expected:
                raise ValueError("Decoded-array cache does not match the source Parquet")
            values = {key: arrays[key].copy() for key in ("observation.state", "action", "timestamp")}
    else:
        raise ImportError("Install pyarrow to decode the source Parquet, or provide its SHA-checked decoded-array cache")
    state = np.asarray(values["observation.state"], dtype=float)
    actions = np.asarray(values["action"], dtype=float).reshape(len(state), 50, 30)
    times = np.asarray(values["timestamp"], dtype=float)
    grids = read_csv(directory / "alignment/alignment_output_grid.csv")
    anchors = [row for row in grids if int(row["is_output_anchor"])]
    raw = read_csv(directory / "source/camera/hand_pose_test_true_absolute.csv")
    by_timestamp = {int(row["timestamp_ns"]): i for i, row in enumerate(raw)}
    indices = np.array([by_timestamp[int(row["hand_pose_timestamp_ns"])] for row in anchors])
    if len(indices) != len(state) or not np.allclose(np.diff(times), .1, atol=2e-6):
        raise ValueError("Unexpected episode length or nonuniform 10 Hz grid")
    all_grid_indices = [by_timestamp[int(row["hand_pose_timestamp_ns"])] for row in grids]
    first = all_grid_indices.index(int(indices[0]))
    if first == 0:
        raise ValueError("Missing predecessor for the first recorded state")
    previous_indices = np.r_[all_grid_indices[first-1], indices[:-1]]
    local_offset = np.eye(4); local_offset[0, 3] = -.042
    selected_rows = [raw[i] for i in indices]
    previous_rows = [raw[i] for i in previous_indices]
    for row in selected_rows+previous_rows:
        for side in SIDES:
            if row.get(side+"_pose_valid", "1") in ("0", "false", "False"):
                raise ValueError("A selected trajectory anchor has an invalid absolute pose")
    # Invalid, unused raw prefix/suffix samples must not reject a valid emitted
    # episode; only its exact anchors and predecessors enter reconstruction.
    previous = np.stack([absolute_matrices(previous_rows, side) @ local_offset for side in SIDES], axis=1)
    selected = np.stack([absolute_matrices(selected_rows, side) @ local_offset for side in SIDES], axis=1)
    deltas = inverse(previous) @ selected
    state_error = float(np.max(np.abs(np.concatenate([pose9(deltas[:, 0]), pose9(deltas[:, 1])], axis=-1)-state[:, :18])))
    # Check all non-padded futures; never concatenate overlapping action chunks.
    action_error = 0.
    for t in range(len(state)-1):
        count = min(50, len(state)-t-1)
        delta = inverse(selected[t]) @ selected[t+1:t+1+count]
        expected = np.concatenate([pose9(delta[:, 0]), pose9(delta[:, 1]), state[t+1:t+1+count, 18:]], axis=-1)
        action_error = max(action_error, float(np.max(np.abs(expected-actions[t, :count]))))
    if state_error > 2e-5 or action_error > 2e-5:
        raise ValueError(f"Absolute source does not reproduce training labels: {state_error=}, {action_error=}")
    return {
        "episode_index": int(json.loads((directory / "source_manifest.json").read_text()).get("episode_index", 0)),
        "source_directory": str(directory.resolve()),
        "times": times-times[0], "source_poses": selected, "previous_source_poses": previous[0],
        "fingers_deg": state[:, 18:].reshape(-1, 2, 6), "source_state": state,
        "source_action": actions, "source_raw_indices": indices,
        "source_frame_ids": np.array([int(row["e6_frame_id"]) for row in anchors]),
        "audit": {"state_pose_max_abs_error": state_error, "action_max_abs_error": action_error,
                  "raw_local_origin_offset_m": [-.042, 0., 0.], "frames": len(state),
                  "first_state_predecessor_raw_row": int(previous_indices[0])},
    }


def sample_poses(times, matrices, query_times):
    query_times = np.clip(query_times, times[0], times[-1])
    result = np.broadcast_to(np.eye(4), (len(query_times), 2, 4, 4)).copy()
    for side in range(2):
        for axis in range(3):
            result[:, side, axis, 3] = np.interp(query_times, times, matrices[:, side, axis, 3])
        result[:, side, :3, :3] = Slerp(times, Rotation.from_matrix(matrices[:, side, :3, :3]))(query_times).as_matrix()
    return result


def finger_targets(fingers_deg):
    return np.deg2rad(np.asarray(fingers_deg)[..., FINGER_TO_URDF])


def source_fingers_from_q(q):
    q = np.asarray(q)
    return np.stack([np.rad2deg(q[..., 7:13])[..., FINGER_TO_URDF],
                     np.rad2deg(q[..., 20:26])[..., FINGER_TO_URDF]], axis=-2)


class ArmChain:
    """Small URDF chain used for continuous IK, without traversing both hands."""
    def __init__(self, kinematics, side, fixed_positions, base):
        self.names = [f"{side}_{suffix}_joint" for suffix in ARM_SUFFIXES]
        self.kinematics, self.side = kinematics, side
        self.fixed, self.base = fixed_positions, np.asarray(base)
        parents = {joint["child"]: joint for joint in kinematics.joints.values()}
        self.chain = []
        link = side+"_hand_base_link"
        while link != kinematics.root:
            joint = parents[link]
            self.chain.insert(0, joint)
            link = joint["parent"]
        self.lower = np.array([kinematics.joints[name]["lower"] for name in self.names])
        self.upper = np.array([kinematics.joints[name]["upper"] for name in self.names])

    def forward(self, q):
        frame = self.base.copy()
        values = dict(zip(self.names, q))
        for joint in self.chain:
            frame = frame @ joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                r = np.eye(4)
                r[:3, :3] = rotation(joint["axis"], values.get(joint["name"], self.fixed.get(joint["name"], 0.)))
                frame = frame @ r
        return frame

    def solve(self, target, initial):
        def residual(q):
            frame = self.forward(q)
            return np.r_[frame[:3, 3]-target[:3, 3],
                         .15*Rotation.from_matrix(frame[:3, :3].T @ target[:3, :3]).as_rotvec(),
                         .0001*(q-initial)]
        result = least_squares(residual, np.clip(initial, self.lower+1e-7, self.upper-1e-7),
                               bounds=(self.lower, self.upper), max_nfev=100,
                               ftol=1e-9, xtol=1e-9, gtol=1e-9)
        frame = self.forward(result.x)
        position_error = np.linalg.norm(frame[:3, 3]-target[:3, 3])
        angle_error = np.linalg.norm(Rotation.from_matrix(frame[:3, :3].T @ target[:3, :3]).as_rotvec())
        return result.x, float(position_error), float(angle_error)


def action_chunks(tool_poses, fingers_deg, horizon=50):
    """Recompute training labels from achieved simulation poses at 10 Hz."""
    count = len(tool_poses)
    actions = np.zeros((count, horizon, 30), np.float32)
    padding = np.zeros((count, horizon), bool)
    for t in range(count):
        future = np.minimum(np.arange(t+1, t+1+horizon), count-1)
        delta = inverse(tool_poses[t]) @ tool_poses[future]
        actions[t] = np.concatenate([pose9(delta[:, 0]), pose9(delta[:, 1]), fingers_deg[future].reshape(horizon, 12)], axis=-1)
        padding[t] = np.arange(t+1, t+1+horizon) >= count
    return actions, padding
