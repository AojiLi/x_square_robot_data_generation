"""Automatic initial-state proposals for immutable UMI robot trajectories.

Geometry produces candidates, never a promise of successful physical contact.
Only the separate forward-physics evaluator can accept a layout.
"""
from pathlib import Path
import hashlib
import json
import struct

import numpy as np
from scipy.ndimage import binary_closing, label, median_filter
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from room01_sim.kinematics import RobotKinematics, URDF, HAND_SUFFIXES, ACTION_JOINTS, origin_transform
from room01_sim.umi import ArmChain, finger_targets, inverse, sample_poses

CATALOG = {
    "RedBig": {"size": [.05]*3, "mass_kg": .072, "color": [.62, .025, .018]},
    "Blue": {"size": [.05]*3, "mass_kg": .072, "color": [.015, .32, .65]},
    "Green": {"size": [.05]*3, "mass_kg": .072, "color": [.015, .42, .18]},
    "RedSmall": {"size": [.04]*3, "mass_kg": .042, "color": [.62, .025, .018]},
}
BOX = {"size": [.174, .144, .102], "mass_kg": .110, "wall_m": .002, "bottom_m": .002, "color": [.018]*3}
TABLE_XY = np.array([[.727, 1.342], [-2.514, -1.360]])


def detect_events(times, fingers):
    """Infer hold intervals from non-thumb flexion, with shared thresholds.

    Thumb opposition is excluded from the activity signal because it changes
    during approach even when the other fingers remain open.
    """
    events = []
    for side, name in enumerate(("left", "right")):
        curl = median_filter(np.mean(fingers[:, side, 2:6], axis=1), size=3, mode="nearest")
        low, high = float(np.percentile(curl, 10)), float(curl.max())
        if high-low < 1.5:
            continue
        active = binary_closing(curl > low+.30*(high-low), structure=np.ones(3), border_value=0)
        groups, count = label(active)
        for group in range(1, count+1):
            indices = np.flatnonzero(groups == group)
            if len(indices) < 3 or indices[-1] == len(times)-1:
                continue
            onset = int(indices[0])
            while onset > 0 and curl[onset-1] > low+.10*(high-low):
                onset -= 1
            peak = float(curl[indices].max())
            plateau = int(indices[np.flatnonzero(curl[indices] >= low+.75*(peak-low))[0]])
            release = int(indices[-1]+1)
            candidates = np.unique(np.linspace(max(0, onset-1), min(plateau+1, release-1), 5).round().astype(int))
            events.append({"side": name, "side_index": side, "onset_index": onset,
                           "closed_index": plateau, "release_index": release,
                           "onset_s": float(times[onset]), "closed_s": float(times[plateau]),
                           "release_s": float(times[release]), "candidate_indices": candidates.tolist(),
                           "curl_range_deg": [float(low), peak]})
    return sorted(events, key=lambda event: event["onset_s"])


def identify_objects(rgb):
    """Use first-frame color components for identity only, not metric poses.

    Ambiguous images are rejected rather than silently assigning an asset.
    This detector is scoped to the supplied white-table / black-box task.
    """
    h, w = rgb.shape[:2]
    yy, xx = np.mgrid[:h, :w]
    f = rgb.astype(float)
    center = (xx > .30*w) & (xx < .72*w) & (yy > .4*h) & (yy < .80*h)
    dark, count = label((f.max(axis=-1) < 70) & center)
    boxes = []
    for i in range(1, count+1):
        y, x = np.where(dark == i)
        if len(x) >= 200 and .06*w <= np.ptp(x) <= .25*w and .04*h <= np.ptp(y) <= .25*h:
            cost = len(x)/(1+40*((x.mean()/w-.5)**2+(y.mean()/h-.60)**2))
            boxes.append((cost, [int(x.min()), int(y.min()), int(x.max()), int(y.max())]))
    if not boxes:
        raise ValueError("identity_unresolved: no central black box detected")
    box = max(boxes)[1]
    r, g, b = f.transpose(2, 0, 1)
    roi = (xx > .18*w) & (xx < .82*w) & (yy > box[3]) & (yy < .92*h)
    masks = {"Red": (r > 1.8*g) & (r > 2*b) & (r > 75),
             "Green": (g > 1.3*r) & (g > 1.05*b) & (g > 55),
             "Blue": (b > 1.3*r) & (b > 1.02*g) & (b > 70)}
    components = []
    for color, mask in masks.items():
        groups, count = label(mask & roi)
        for i in range(1, count+1):
            y, x = np.where(groups == i)
            if 60 <= len(x) <= 2000 and np.ptp(x) >= 5 and np.ptp(y) >= 5:
                components.append({"color": color, "area": len(x), "center": [float(x.mean()), float(y.mean())]})
    components = sorted(components, key=lambda c: c["area"], reverse=True)[:2]
    if len(components) != 2:
        raise ValueError("identity_unresolved: expected two outside colored components")
    components.sort(key=lambda c: c["center"][0])
    reference_areas = [c["area"] for c in components if c["color"] != "Red"]
    if reference_areas:
        reference = float(np.mean(reference_areas))
        for c in components:
            if c["color"] == "Red":
                ratio = c["area"]/reference
                c["red_area_ratio"] = ratio
                if not .40 <= ratio <= 1.8:
                    raise ValueError("identity_unresolved: red cube scale is ambiguous")
                c["asset"] = "RedSmall" if ratio < .78 else "RedBig"
            else:
                c["asset"] = c["color"]
    else:
        small, big = sorted(components, key=lambda c: c["area"])
        if big["area"]/small["area"] < 1.3:
            raise ValueError("identity_unresolved: red cube sizes cannot be separated")
        small["asset"], big["asset"] = "RedSmall", "RedBig"
    return {"objects_by_side": dict(zip(("left", "right"), [c["asset"] for c in components])),
            "components": components, "box_pixel_bounds": box,
            "method": "color/relative-scale heuristic; screen-left/right assignment, no position reconstruction"}


def trajectory_hash(trajectory):
    digest = hashlib.sha256()
    for key in ("times", "targets", "hand_targets", "tool_to_hand_base"):
        a = np.ascontiguousarray(trajectory[key], dtype=np.float64)
        digest.update(key.encode()); digest.update(str(a.shape).encode()); digest.update(a.tobytes())
    return digest.hexdigest()


def reference_fingerprint(config, reference_hash, time_scale):
    """Invalidate cached servo traces when physics code, model or setup changes."""
    root = Path(__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for name in ("room01_sim/runtime.py", "room01_sim/control.py", "room01_sim/umi_auto_runtime.py", "scripts/prepare_room01_umi_replay.py",
                 "assets/robots/quanta_x2/quanta_x2_dual_revo2_bridge.urdf",
                 "assets/room01/sim/quanta_x2_physics.usdc", "assets/room01/sim/room_environment.usdc"):
        digest.update((root / name).read_bytes())
    settings = {k: config[k] for k in ("physics_dt", "control_hz", "robot_base_position", "robot_base_yaw_rad", "joint_home_rad", "locked_joint_names")}
    settings.update(trajectory_sha256=reference_hash, time_scale=time_scale, servo_integral=True)
    settings["object_physics"] = {name: {k: spec[k] for k in ("size", "mass_kg", "wall_m", "bottom_m") if k in spec}
                                  for name, spec in config["task_objects"].items()}
    digest.update(json.dumps(settings, sort_keys=True).encode())
    return digest.hexdigest()


def stable_warmstarts(history, fingerprint):
    """A known failed repeat invalidates a previously successful grasp seed."""
    valid = [(path, r) for path, r in history if r.get("status") == "complete"
             and r.get("phase") in ("grasp_probe", "record_probe") and r.get("reference_fingerprint") == fingerprint]
    key = lambda r: json.dumps(r["initial_object_poses"], sort_keys=True)
    failed = {key(r) for _, r in valid if any(g.get("evaluated") and not g["passed"] for g in r.get("grasps", {}).values())}
    result, seen = [], set()
    for path, report in valid:
        token = key(report)
        if token not in failed and token not in seen and report.get("grasps") and all(g["passed"] for g in report["grasps"].values()):
            result.append({"poses": report["initial_object_poses"], "source": str(path)}); seen.add(token)
    return result


def freeze_trajectory(data, config, calibration):
    """One rigid registration and IK pass, completed before any layout search."""
    poses = data["source_poses"]
    axis = np.asarray(calibration["tracking_axes_to_zup"])
    span = axis @ (poses[0, 0, :3, 3]-poses[0, 1, :3, 3])
    if np.linalg.norm(span[:2]) < .1:
        raise ValueError("registration_invalid: initial hand span is too small")
    world = np.eye(4)
    world[:3, :3] = Rotation.from_euler("z", np.arctan2(span[0], span[1])).as_matrix() @ axis
    world[:3, 3] = np.asarray(calibration["initial_hand_midpoint_world"])-world[:3, :3] @ poses[0, :, :3, 3].mean(0)
    tool = np.asarray(calibration["training_tool_to_hand_base"])
    times = np.arange(round(data["times"][-1]*30)+1)/30
    hand = world @ sample_poses(data["times"], poses, times) @ tool
    fingers = np.stack([np.interp(times, data["times"], data["fingers_deg"][:, side, j])
                        for side in range(2) for j in range(6)], -1).reshape(-1, 2, 6)
    kin = RobotKinematics(); base = np.eye(4); base[:3, 3] = config["robot_base_position"]
    targets = np.zeros((len(times), 26)); errors = np.zeros((len(times), 2, 2))
    for side, name in enumerate(("left", "right")):
        chain = ArmChain(kin, name, config["joint_home_rad"], base)
        previous = np.array([config["joint_home_rad"][n] for n in chain.names])
        for i, target in enumerate(hand[:, side]):
            previous, pe, re = chain.solve(target, previous)
            targets[i, side*13:side*13+7] = previous; errors[i, side] = [pe, re]
        targets[:, side*13+7:side*13+13] = finger_targets(fingers[:, side])
    if errors[..., 0].max() > .005 or errors[..., 1].max() > .05:
        raise ValueError(f"unreachable_trajectory: max IK position error {errors[..., 0].max():.4f} m")
    lower = np.array([kin.joints[n]["lower"] for n in ACTION_JOINTS]); upper = np.array([kin.joints[n]["upper"] for n in ACTION_JOINTS])
    limit_error = np.maximum(lower-targets, 0)+np.maximum(targets-upper, 0)
    if limit_error.max() > np.deg2rad(.25):
        raise ValueError("joint_limits: decoded finger angles exceed the robot limits")
    targets = np.clip(targets, lower, upper)
    return {"times": times, "targets": targets, "hand_targets": hand, "tool_to_hand_base": tool,
            "world_from_tracking": world, "source_times": data["times"], "source_poses": poses,
            "source_fingers_deg": data["fingers_deg"], "source_frame_ids": data["source_frame_ids"], "ik_errors": errors,
            "limit_rounding_max_deg": np.asarray(np.rad2deg(limit_error.max()))}


def stl_surface(path, maximum=180, with_normals=False):
    content = Path(path).read_bytes()
    n = struct.unpack("<I", content[80:84])[0]
    if len(content) != 84+50*n:
        raise ValueError(f"Expected binary STL: {path}")
    dtype = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    vertices = np.unique(np.frombuffer(content, dtype=dtype, offset=84)["vertices"].reshape(-1, 3), axis=0)
    hull = ConvexHull(vertices)
    triangles = vertices[hull.simplices]
    points = np.vstack([vertices[hull.vertices], triangles.mean(1), (triangles[:, 0]+triangles[:, 1])/2,
                        (triangles[:, 1]+triangles[:, 2])/2, (triangles[:, 2]+triangles[:, 0])/2])
    points = np.unique(points, axis=0)
    points = points[np.linspace(0, len(points)-1, min(maximum, len(points))).round().astype(int)]
    on_face = abs(points @ hull.equations[:, :3].T+hull.equations[:, 3]) < 1e-5
    normals = on_face.astype(float) @ hull.equations[:, :3]
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    return (points, normals) if with_normals else points


class HandGeometry:
    def __init__(self):
        self.kin = RobotKinematics()
        self.meshes = {}
        self.normals = {}
        for name, link in self.kin.links.items():
            if not any(s in name for s in ("thumb", "index", "middle", "ring", "pinky", "hand_base")):
                continue
            samples, normals = [], []
            for collision in link.findall("collision"):
                mesh = collision.find("geometry/mesh")
                if mesh is None:
                    continue
                p, normal = stl_surface(URDF.parent / mesh.get("filename"), with_normals=True)
                scale = np.array([float(x) for x in mesh.get("scale", "1 1 1").split()])
                t = origin_transform(collision.find("origin"))
                samples.append((t[:3, :3] @ (p*scale).T).T+t[:3, 3])
                normal = normal/scale
                normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
                normals.append(normal @ t[:3, :3].T)
            if samples:
                self.meshes[name] = np.vstack(samples)
                self.normals[name] = np.vstack(normals)

    def at(self, side, finger_deg, hand_matrix, normals=False):
        positions = {f"{side}_{suffix}_joint": v for suffix, v in zip(HAND_SUFFIXES, finger_targets(finger_deg))}
        fk = self.kin.forward(positions)
        root = inverse(fk[side+"_hand_base_link"])
        result = {}
        for name, points in self.meshes.items():
            if name.startswith(side+"_"):
                t = hand_matrix @ root @ fk[name]
                result[name] = self.normals[name] @ t[:3, :3].T if normals else (t[:3, :3] @ points.T).T+t[:3, 3]
        return result


def cube_sdf(points, position, size, yaw):
    r = Rotation.from_euler("z", yaw).as_matrix()
    p = (np.asarray(points)-position) @ r
    q = np.abs(p)-size/2
    return np.linalg.norm(np.maximum(q, 0), axis=-1)+np.minimum(q.max(axis=-1), 0)


def cube_corners(poses, size):
    local = np.array([[x, y, z] for x in [-.5, .5] for y in [-.5, .5] for z in [-.5, .5]])*size
    poses = np.asarray(poses)
    return (Rotation.from_quat(poses[..., 3:]).as_matrix() @ local.T).swapaxes(-1, -2)+poses[..., None, :3]


def grasp_candidates(data, trajectory, event, asset, geometry, count=24):
    side = event["side"]; si = event["side_index"]; size = CATALOG[asset]["size"][0]
    table_z = .723
    initial_cloud = np.vstack(list(geometry.at(side, data["fingers_deg"][0, si], trajectory["hand_targets"][0, si]).values()))
    approach = []
    for i in np.unique(np.linspace(0, max(0, event["onset_index"]-2), 10).round().astype(int)):
        h = sample_poses(trajectory["times"], trajectory["hand_targets"], np.array([data["times"][i]]))[0, si]
        approach.extend(geometry.at(side, data["fingers_deg"][i, si], h).values())
    approach = np.vstack(approach)
    choices = []
    for source_index in event["candidate_indices"]:
        t = data["times"][source_index]
        hand = sample_poses(trajectory["times"], trajectory["hand_targets"], np.array([t]))[0, si]
        cloud = geometry.at(side, data["fingers_deg"][source_index, si], hand)
        normals = geometry.at(side, data["fingers_deg"][source_index, si], hand, normals=True)
        thumb = np.vstack([cloud[side+"_thumb_distal_link"], cloud[side+"_thumb_proximal_link"]])
        thumb_normals = np.vstack([normals[side+"_thumb_distal_link"], normals[side+"_thumb_proximal_link"]])
        fingers = np.vstack([cloud[side+"_"+finger+"_"+segment+"_link"]
                             for finger in ("index", "middle") for segment in ("proximal", "distal")])
        finger_normals = np.vstack([normals[side+"_"+finger+"_"+segment+"_link"]
                                    for finger in ("index", "middle") for segment in ("proximal", "distal")])
        difference = fingers[None]-thumb[:, None]
        midpoint = (fingers[None]+thumb[:, None])/2
        horizontal = np.linalg.norm(difference[..., :2], axis=-1)
        height_gap = np.maximum(table_z+.003-midpoint[..., 2], 0)+np.maximum(midpoint[..., 2]-(table_z+size-.003), 0)
        pair_score = abs(horizontal-(size-.002))+2*height_gap+.2*abs(difference[..., 2])
        pair_score[(horizontal < .5*size) | (horizontal > 1.6*size)] = np.inf
        direction = difference/np.maximum(np.linalg.norm(difference, axis=-1, keepdims=True), 1e-9)
        facing_thumb = np.sum(thumb_normals[:, None]*direction, axis=-1)
        facing_finger = -np.sum(finger_normals[None]*direction, axis=-1)
        opposed = thumb_normals @ finger_normals.T
        pair_score[(facing_thumb < .35) | (facing_finger < .35) | (opposed > -.3)] = np.inf
        thumb_group = np.r_[np.zeros(len(cloud[side+"_thumb_distal_link"]), int), np.ones(len(cloud[side+"_thumb_proximal_link"]), int)]
        finger_group = np.concatenate([np.full(len(cloud[side+"_"+f+"_"+s+"_link"]), int(s == "proximal")) for f in ("index", "middle") for s in ("proximal", "distal")])
        selected = []
        for tg in range(2):
            for fg in range(2):
                scores = np.where((thumb_group[:, None] == tg) & (finger_group[None] == fg), pair_score, np.inf)
                selected.extend(int(i) for i in np.argsort(scores.ravel())[:8] if np.isfinite(scores.ravel()[i]))
        all_points = np.vstack(list(cloud.values()))
        for flat in selected:
            a, b = np.unravel_index(flat, pair_score.shape)
            if not np.isfinite(pair_score[a, b]):
                continue
            center = midpoint[a, b].copy(); center[2] = table_z+size/2+.0002
            yaw = np.arctan2(difference[a, b, 1], difference[a, b, 0]) % (np.pi/2)
            for dx, dy in [(0, 0), (-.006, 0), (.006, 0), (0, -.006), (0, .006)]:
                position = center+np.array([dx, dy, 0.])
                for angle in (yaw, yaw-.15, yaw+.15):
                    pose = np.r_[position, Rotation.from_euler("z", angle).as_quat()]
                    corners = cube_corners(pose, size)
                    if not ((corners[:, :2].min(0) >= TABLE_XY[:, 0]+.001).all() and (corners[:, :2].max(0) <= TABLE_XY[:, 1]-.001).all()):
                        continue
                    if cube_sdf(initial_cloud, position, size, angle).min() < -.001:
                        continue
                    td = float(cube_sdf(thumb, position, size, angle).min())
                    fd = float(cube_sdf(fingers, position, size, angle).min())
                    penetration = float(min(cube_sdf(all_points, position, size, angle).min(), 0))
                    approach_penetration = float(max(-cube_sdf(approach, position, size, angle).min(), 0))
                    score = abs(td)+abs(fd)+5*max(-penetration-.003, 0)+2*max(approach_penetration-.003, 0)+.3*pair_score[a, b]
                    choices.append({"asset": asset, "pose_xyzw": pose.tolist(), "score": float(score),
                                    "contact_family": "thumb_"+("distal", "proximal")[thumb_group[a]]+"/finger_"+("distal", "proximal")[finger_group[b]],
                                    "approach_penetration_m": approach_penetration,
                                    "proposal_source_time_s": float(t), "thumb_gap_m": td, "finger_gap_m": fd,
                                    "opposing_surface_dot": float(opposed[a, b]),
                                    "surface_penetration_m": -penetration})
    # Preserve different contact families instead of spending the whole
    # physical budget on near-duplicates from one local geometric minimum.
    buckets = {key: sorted([c for c in choices if c["contact_family"] == key], key=lambda c: c["score"])
               for key in {c["contact_family"] for c in choices}}
    families = sorted(buckets, key=lambda key: (buckets[key][0]["score"], key))
    result = []
    while len(result) < count and any(buckets.values()):
        for key in families:
            while buckets[key]:
                choice = buckets[key].pop(0); p = np.array(choice["pose_xyzw"])
                if all(np.linalg.norm(p[:2]-np.array(c["pose_xyzw"][:2])) > .003 or abs(np.dot(p[3:], c["pose_xyzw"][3:])) < .997 for c in result):
                    choice["candidate_id"] = len(result); result.append(choice); break
            if len(result) >= count:
                break
    return result


def parked_layout(active_poses):
    """Probe-only parking position, clear of the demonstrated hand workspace."""
    box_pose = np.array([1.20, -2.39, .723+BOX["size"][2]/2, 0, 0, 0, 1.])
    poses = {"BlackBox": box_pose.tolist(), **active_poses}
    remaining = [name for name in CATALOG if name not in active_poses]
    for name, dy in zip(remaining, [-.035, .035]):
        size = CATALOG[name]["size"][0]
        poses[name] = [1.20, -2.39+dy, .723+.002+size/2+.0002, 0, 0, 0, 1]
    return poses


def obb_overlap(centers, rotations, half_sizes, wall_center, wall_rotation, wall_half):
    """15-axis SAT for batches of cubes against one oriented wall."""
    centers = np.asarray(centers); r = rotations.swapaxes(-1, -2) @ wall_rotation
    ar = abs(r)+1e-10
    t = (rotations.swapaxes(-1, -2) @ (wall_center-centers)[..., None])[..., 0]
    a = np.broadcast_to(half_sizes, centers.shape); b = np.asarray(wall_half)
    overlap = np.all(abs(t) <= a+(ar @ b), axis=-1)
    overlap &= np.all(abs((t[..., None, :] @ r)[..., 0, :]) <= b+(a[..., None, :] @ ar)[..., 0, :], axis=-1)
    for i in range(3):
        k, l = (i+1)%3, (i+2)%3
        for j in range(3):
            m, n = (j+1)%3, (j+2)%3
            ra = a[..., k]*ar[..., l, j]+a[..., l]*ar[..., k, j]
            rb = b[m]*ar[..., i, n]+b[n]*ar[..., i, m]
            overlap &= abs(t[..., l]*r[..., k, j]-t[..., k]*r[..., l, j]) <= ra+rb
    return overlap


def box_candidates(trace, events, active_poses, limit=12):
    """Jointly fit one box to both outcomes and reject swept-cube wall hits."""
    names = trace["object_names"]
    corners, sweeps, rotations, sizes = [], [], [], []
    for event in events:
        name = event["asset"]; index = names.index(name); size = CATALOG[name]["size"][0]
        # Use the probe's physically achieved release/settling position.
        end = min(event["release_s"]+.5, trace["source_time"][-1])
        frame = np.argmin(abs(trace["source_time"]-end))
        corners.append(cube_corners(trace["object_poses"][frame, index], size))
        # After release, contact with the container is expected and must be
        # resolved by the final physical replay, not rejected using a no-box fall.
        keep = trace["source_time"] <= event["release_s"]
        poses = trace["object_poses"][keep, index]
        sweeps.append(poses[:, :3]); rotations.append(Rotation.from_quat(poses[:, 3:]).as_matrix()); sizes.append(np.full((len(poses), 3), size/2+.0006))
    landing = np.concatenate(corners)
    centers = np.vstack(sweeps); rotations = np.concatenate(rotations); half_sizes = np.vstack(sizes)
    results = []; containment_count = 0; blocked_count = 0
    for yaw in np.arange(0, np.pi, np.pi/36):
        r = Rotation.from_euler("z", yaw).as_matrix()
        projected = landing @ r
        inner = np.array(BOX["size"][:2])/2-BOX["wall_m"]-.001
        lower = projected[:, :2].max(0)-inner
        upper = projected[:, :2].min(0)+inner
        if np.any(lower > upper):
            continue
        for x in np.linspace(lower[0], upper[0], 5):
            for y in np.linspace(lower[1], upper[1], 5):
                box_center = r @ np.array([x, y, .723+BOX["size"][2]/2])
                pose = np.r_[box_center, Rotation.from_matrix(r).as_quat()]
                bc = cube_corners(pose, np.asarray(BOX["size"]))
                if np.any(bc[:, :2].min(0) < TABLE_XY[:, 0]+.001) or np.any(bc[:, :2].max(0) > TABLE_XY[:, 1]-.001):
                    continue
                # Target objects must begin outside the box.
                if any(np.all(abs((np.array(p[:3])-box_center) @ r)[:2] < np.array(BOX["size"][:2])/2+CATALOG[name]["size"][0]/2) for name, p in active_poses.items()):
                    continue
                containment_count += 1
                length, width, height = BOX["size"]; wall = BOX["wall_m"]
                slabs = [([-length/2+wall/2, 0, .001], [wall/2, width/2, (height-.002)/2]),
                         ([length/2-wall/2, 0, .001], [wall/2, width/2, (height-.002)/2]),
                         ([0, -width/2+wall/2, .001], [(length-2*wall)/2, wall/2, (height-.002)/2]),
                         ([0, width/2-wall/2, .001], [(length-2*wall)/2, wall/2, (height-.002)/2])]
                if any(obb_overlap(centers, rotations, half_sizes, box_center+r@offset, r, np.asarray(half)).any() for offset, half in slabs):
                    blocked_count += 1; continue
                poses = {**active_poses, "BlackBox": pose.tolist()}
                occupied = [((np.asarray(p[:3])-box_center)@r)[:2] for p in [trace["object_poses"][np.argmin(abs(trace["source_time"]-min(e["release_s"]+.5, trace["source_time"][-1]))), names.index(e["asset"])] for e in events]]
                valid = True
                for name in [n for n in CATALOG if n not in active_poses]:
                    size = CATALOG[name]["size"][0]
                    available = inner-size/2-.001
                    points = [np.array([px, py]) for px in np.linspace(-available[0], available[0], 7) for py in np.linspace(-available[1], available[1], 7)]
                    best = max(points, key=lambda p: min(np.linalg.norm(p-q) for q in occupied))
                    if min(np.linalg.norm(best-q) for q in occupied) < .045:
                        valid = False; break
                    occupied.append(best)
                    p = box_center+r@np.r_[best, -height/2+.002+size/2+.0002]
                    poses[name] = np.r_[p, Rotation.from_matrix(r).as_quat()].tolist()
                if valid:
                    results.append({"poses": poses, "yaw_rad": float(yaw), "score": float(np.linalg.norm((lower+upper)/2-[x, y]))})
    results.sort(key=lambda c: c["score"])
    unique = []
    for item in results:
        p = np.array(item["poses"]["BlackBox"])
        if all(np.linalg.norm(p[:2]-np.array(x["poses"]["BlackBox"][:2])) > .004 or abs(item["yaw_rad"]-x["yaw_rad"]) > .05 for x in unique):
            item["candidate_id"] = len(unique); unique.append(item)
            if len(unique) >= limit:
                break
    return unique, {"containment_candidates": containment_count, "swept_wall_rejections": blocked_count,
                    "feasible_geometric_candidates": len(results),
                    "reason": "no_sampled_joint_release_region" if containment_count == 0 else "insufficient_sampled_swept_clearance" if not results else "candidates_ready",
                    "collision_screen": "30 Hz measured cube OBB vs 4 walls; proposal filter only, final physics required"}
