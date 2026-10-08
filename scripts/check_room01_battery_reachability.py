#!/usr/bin/env python3
"""CPU-only URDF IK screen for battery grasp, lift, reorientation and insertion.

No scene mutation or physics is performed. Reachable IK endpoints are neither
collision-free paths nor evidence of contact, grasp stability or insertion.
Run with .venv-usd/bin/python scripts/check_room01_battery_reachability.py.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from room01_sim.kinematics import RobotKinematics, ARM_SUFFIXES, URDF
from room01_sim.umi import ArmChain


def pose(position, rotation=None):
    result = np.eye(4)
    result[:3, 3] = position
    if rotation is not None:
        result[:3, :3] = rotation
    return result


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ik_fit(chain, target, initial, starts=3):
    rng = np.random.default_rng(2916)
    guesses = [initial] + [initial+rng.normal(0, 0.9, 7) for _ in range(starts-1)]
    best = None
    for guess in guesses:
        q, error, angle = chain.solve(target, guess)
        score = error+0.15*angle
        if best is None or score < best[0]:
            best = score, q, error, angle
        if error < 0.0002 and angle < 0.002:
            break
    _, q, error, angle = best
    return {"joint_positions_rad": dict(zip(chain.names, q.tolist())),
            "position_error_m": error, "orientation_error_rad": angle,
            "orientation_error_deg": float(np.degrees(angle)),
            "ik_endpoint_pass": bool(error <= 0.002 and angle <= np.deg2rad(2)),
            "joint_limit_min_margin_rad": float(min(np.min(q-chain.lower), np.min(chain.upper-q))),
            "desired_world_from_hand_base": target.tolist(),
            "achieved_world_from_hand_base": chain.forward(q).tolist()}


def coarse_table_screen(k, config, spec, base, side, solution):
    """Sample centerline capsules only; do not claim a complete collision test."""
    positions = dict(config["joint_home_rad"])
    positions.update(solution["joint_positions_rad"])
    frames = k.forward(positions, base)
    links = [side+"_"+suffix+"_link" for suffix in ARM_SUFFIXES]+[side+"_hand_base_link"]
    points = [frames[name][:3, 3] for name in links]
    samples = np.concatenate([np.linspace(a, b, 12) for a, b in zip(points[:-1], points[1:])])
    table_rotation = np.asarray(spec["frame"]["rotation_world_from_local"])
    table_origin = np.asarray(spec["frame"]["world_origin_m"])
    local = (samples-table_origin) @ table_rotation
    radius = 0.025
    above = ((np.abs(local[:, 0]) <= spec["table"]["length_m"]/2+radius)
             & (np.abs(local[:, 1]) <= spec["table"]["depth_m"]/2+radius))
    minimum = float(local[above, 2].min()-radius-spec["table"]["height_m"]) if np.any(above) else None
    return {"assumed_arm_capsule_radius_m": radius,
            "minimum_sampled_arm_capsule_tabletop_clearance_m": minimum,
            "tabletop_risk_detected": bool(minimum is not None and minimum < 0),
            "scope": "Arm link-origin segments only; excludes hands, robot self-collision, foam and room obstacles"}


def append_trial_screen(report, args, config, spec, k, base):
    """Screen the finalized contact candidate without expanding the search."""
    if not args.trial_candidate.exists():
        return
    document = json.loads(args.trial_candidate.read_text())
    candidate = next(c for c in document["candidates"] if c["candidate_id"] == 15)
    side = candidate["side"]
    chain = ArmChain(k, side, config["joint_home_rad"], base)
    initial = np.array([config["joint_home_rad"][name] for name in chain.names])
    item = config["task_objects"][candidate["object"]]
    original = pose(item["position"], Rotation.from_quat(item["orientation_xyzw"]).as_matrix())
    desired = np.asarray(candidate["world_from_hand_base"])
    relative = np.linalg.inv(original)@desired
    grasp = ik_fit(chain, desired, initial)
    grasp["coarse_table_screen"] = coarse_table_screen(k, config, spec, base, side, grasp)
    q_grasp = np.array([grasp["joint_positions_rad"][name] for name in chain.names])
    table_rotation = np.asarray(spec["frame"]["rotation_world_from_local"])
    table_origin = np.asarray(spec["frame"]["world_origin_m"])
    red_top = spec["table"]["height_m"]+spec["props"]["red_dims_m"][2]
    red_center = np.r_[spec["props"]["red_position_xy_m"], red_top]
    red_rotation = Rotation.from_euler("z", spec["props"]["red_yaw_deg"], degrees=True).as_matrix()
    holes = [table_origin+table_rotation@(red_center+red_rotation@np.array([dx, 0, 0]))
             for dx in (-spec["props"]["hole_pitch_m"], 0, spec["props"]["hole_pitch_m"])]
    lifted = original.copy()
    lifted[2, 3] += 0.10
    lift = ik_fit(chain, lifted@relative, q_grasp)
    lift.update({"phase": "lift_battery_100mm", "desired_object_world_pose": lifted.tolist()})
    q_lift = np.array([lift["joint_positions_rad"][name] for name in chain.names])
    turn_axis = np.cross(original[:3, 0], [0, 0, 1.])
    turn_axis /= np.linalg.norm(turn_axis)
    upright_rotation = Rotation.from_rotvec(turn_axis*np.pi/2).as_matrix()@original[:3, :3]
    screen = {"source_file": str(args.trial_candidate.relative_to(ROOT)),
              "source_sha256": sha(args.trial_candidate), "candidate_id": 15,
              "source_geometry_confirmation": candidate.get("actual_obstacle_confirmation"),
              "source_diagnostic_acceptance": candidate.get("diagnostic_acceptance"),
              "grasp_ik": grasp, "lift_100mm_ik": lift,
              "battery_from_hand_base": relative.tolist(),
              "open_motor_positions_rad": dict(zip(candidate["motor_names"], candidate["open_fingers_rad"])),
              "closed_motor_positions_rad": dict(zip(candidate["motor_names"], candidate["closed_fingers_rad"])),
              "scope": "Endpoints only, assuming a rigid maintained grasp; real contacts, held-object slip and swept collision are not inferred",
              "sequences": []}
    for pullback in (0.0, 0.10):
        near = lifted.copy()
        near[:3, 3] -= table_rotation[:, 1]*pullback
        near_result = ik_fit(chain, near@relative, q_lift)
        near_result.update({"phase": "horizontal_carry_toward_robot", "desired_object_world_pose": near.tolist()})
        q_near = np.array([near_result["joint_positions_rad"][name] for name in chain.names])
        # One bounded refinement of the best coarse region; no unbounded search.
        for yaw in sorted(set(range(-150, 181, 30)) | set(range(15, 46, 5))):
            upright = near.copy()
            upright[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix()@upright_rotation
            turned = ik_fit(chain, upright@relative, q_near)
            turned.update({"phase": "turn_battery_axis_vertical", "desired_object_world_pose": upright.tolist()})
            q_turn = np.array([turned["joint_positions_rad"][name] for name in chain.names])
            for hole_id, center in enumerate(holes):
                above = pose(center+[0, 0, spec["props"]["battery_length_m"]/2+0.035], upright[:3, :3])
                seated = pose(center+[0, 0, spec["props"]["battery_length_m"]/2-spec["props"]["hole_depth_m"]], upright[:3, :3])
                phases = [lift, near_result, turned]
                q = q_turn.copy()
                for name, target in (("above_hole_35mm_tip_clearance", above), ("nominal_seated_13mm_insertion", seated)):
                    result = ik_fit(chain, target@relative, q)
                    result.update({"phase": name, "desired_object_world_pose": target.tolist()})
                    result["coarse_table_screen"] = coarse_table_screen(k, config, spec, base, side, result)
                    phases.append(result)
                    q = np.array([result["joint_positions_rad"][n] for n in chain.names])
                screen["sequences"].append({"horizontal_pullback_toward_robot_m": pullback,
                    "vertical_axis_yaw_deg": yaw, "hole_id_photo_left_to_right": hole_id,
                    "all_ik_endpoints_pass": grasp["ik_endpoint_pass"] and all(p["ik_endpoint_pass"] for p in phases),
                    "max_position_error_m": max(p["position_error_m"] for p in phases),
                    "max_orientation_error_deg": max(p["orientation_error_deg"] for p in phases),
                    "waypoints": phases})
    screen["best_by_hole"] = []
    for hole_id in range(3):
        sequence = min((s for s in screen["sequences"] if s["hole_id_photo_left_to_right"] == hole_id),
                       key=lambda s: (not s["all_ik_endpoints_pass"], s["max_position_error_m"]+0.15*np.deg2rad(s["max_orientation_error_deg"])))
        screen["best_by_hole"].append({key: value for key, value in sequence.items() if key != "waypoints"})
    screen["summary"] = {"grasp_ik_pass": grasp["ik_endpoint_pass"],
                         "lift_100mm_ik_pass": lift["ik_endpoint_pass"],
                         "complete_endpoint_sets_pass": sum(s["all_ik_endpoints_pass"] for s in screen["sequences"]),
                         "collision_free_path_validated": False, "grasp_or_insertion_success_validated": False}
    screen["yaw_search"] = "30 degree global endpoint grid, plus one 5 degree refinement over +15 to +45 degrees; best means best screened, not a proof of global optimality"
    report["trial15_screen"] = screen
    report["summary"][side]["finalized_trial15_screen"] = screen["summary"]
    print("TRIAL15", json.dumps(screen["summary"]), json.dumps(screen["best_by_hole"]), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"assets/room01/battery_task/task_config.json")
    parser.add_argument("--spec", type=Path, default=ROOT/"assets/room01/battery_task/scene_spec.json")
    parser.add_argument("--candidates", type=Path, default=ROOT/"reports/room01_battery_feasibility/grasp_candidates_refined.json")
    parser.add_argument("--trial-candidate", type=Path, default=ROOT/"reports/room01_battery_feasibility/grasp_candidate_trial15.json")
    parser.add_argument("--trial-only", action="store_true", help="Append finalized trial15 screening to a matching existing core report")
    parser.add_argument("--output", type=Path, default=ROOT/"reports/room01_battery_feasibility/reachability.json")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    spec = json.loads(args.spec.read_text())
    geometry = json.loads(args.candidates.read_text())
    k = RobotKinematics()
    base = pose(config["robot_base_position"], Rotation.from_euler("z", config["robot_base_yaw_rad"]).as_matrix())
    if args.trial_only:
        report = json.loads(args.output.read_text())
        for key, path in (("config_sha256", args.config), ("scene_spec_sha256", args.spec),
                          ("urdf_sha256", URDF), ("grasp_candidates_sha256", args.candidates)):
            if report["inputs"][key] != sha(path):
                raise ValueError(f"Core input changed ({key}); rerun without --trial-only")
        append_trial_screen(report, args, config, spec, k, base)
        args.output.write_text(json.dumps(report, indent=2)+"\n")
        return
    home = config["joint_home_rad"]
    chains = {side: ArmChain(k, side, home, base) for side in ("left", "right")}
    initial = {side: np.array([home[n] for n in chain.names]) for side, chain in chains.items()}
    object_poses = {name: pose(item["position"], Rotation.from_quat(item["orientation_xyzw"]).as_matrix())
                    for name, item in config["task_objects"].items()}
    table_rotation = np.asarray(spec["frame"]["rotation_world_from_local"])
    table_origin = np.asarray(spec["frame"]["world_origin_m"])
    base_local = table_rotation.T@(base[:3, 3]-table_origin)
    fk = k.forward(home, base)
    report = {
        "scope": "CPU URDF bounded 7-DOF IK endpoint screening; no dynamics or executed trajectory",
        "inputs": {"config": str(args.config.relative_to(ROOT)), "config_sha256": sha(args.config),
                   "scene_spec_sha256": sha(args.spec), "urdf_sha256": sha(URDF),
                   "grasp_candidates": str(args.candidates.relative_to(ROOT)), "grasp_candidates_sha256": sha(args.candidates)},
        "criteria": {"position_error_m": 0.002, "orientation_error_deg": 2.0,
                     "random_seed": 2916, "ik_initial_guesses": "Home plus two bounded random perturbations; continuation at subsequent waypoints"},
        "locked_joint_positions_rad": {n: home[n] for n in config["locked_joint_names"]},
        "wrist_joints": "Both wrist joints per arm remain active, within URDF limits",
        "grasp_success_validated": False, "insertion_success_validated": False,
        "collision_free_path_validated": False,
        "base_table_relation": {"base_world_m": base[:3, 3].tolist(), "base_table_local_m": base_local.tolist(),
                                "base_center_to_front_table_edge_m": float(-spec["table"]["depth_m"]/2-base_local[1]),
                                "table_center_world_m": table_origin.tolist(),
                                "registration_status": "Photo-estimated, robot-to-table relative transform retained from prior scene"},
        "hand_geometry_measurements": {}, "legacy_handbase_only_probes": [],
        "grasp_candidates": [], "task_waypoint_sequences": [], "base_shift_sensitivity": [],
        "limitations": ["IK feasibility does not establish finger contact or grasp stability.",
                        "Axial battery rotations preserve ideal cylinder geometry but require fresh hand/foam clearance checks.",
                        "Mirrored left proposals are approximations; the two URDF hands differ slightly.",
                        "Only sparse endpoint poses are screened; swept paths, self-collision and room collision are not validated.",
                        "Battery and hole share the nominal 12.5 mm diameter; rigid insertion is not validated by pose reachability.",
                        "Base shifts are counterfactual diagnostics; the robot or scene has not been moved."]}
    for side, object_name in (("left", "BatteryLeft"), ("right", "BatteryRight")):
        inverse = np.linalg.inv(fk[side+"_hand_base_link"])
        links = [side+"_"+finger+"_touch_link" for finger in ("thumb", "index", "middle")]
        report["hand_geometry_measurements"][side] = {
            "basis": "Actual URDF touch-link origins at configured home motor angles; not fingertip surface contacts",
            "touch_origins_in_hand_base_m": {n: (inverse@fk[n])[:3, 3].tolist() for n in links},
            "shoulder_pitch_origin_world_m": fk[side+"_shoulder_pitch_link"][:3, 3].tolist(),
            "battery_center_distance_from_shoulder_m": float(np.linalg.norm(
                object_poses[object_name][:3, 3]-fk[side+"_shoulder_pitch_link"][:3, 3]))}
        # Historical cube offset proves only a palm-down hand-base point is reachable.
        old_rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        old_offset = np.array([0.050, -0.036 if side == "left" else 0.036, 0.087])
        target = pose(object_poses[object_name][:3, 3]-old_rotation@old_offset, old_rotation)
        result = ik_fit(chains[side], target, initial[side])
        result.update({"side": side, "hand_base_to_object_center_local_m": old_offset.tolist(),
                       "interpretation": "Legacy cube grasp offset; does not align fingers or prove battery grasp"})
        report["legacy_handbase_only_probes"].append(result)

    # Use sampled real hand collision geometry proposals. Mirror each right-hand
    # proposal for the left arm, but label it explicitly as unvalidated geometry.
    reflection = np.diag([1., -1., 1., 1.])
    for source in geometry["candidates"]:
        for side, object_name in (("right", "BatteryRight"), ("left", "BatteryLeft")):
            relative = np.asarray(source["battery_from_hand_base"])
            mirrored = side != source["side"]
            if mirrored:
                relative = reflection@relative@reflection
            rolls = list(range(-90, 91, 15))
            if source["candidate_id"] == 1:
                rolls += [5, 6, 7, 7.5, 8, 9, 10]
            for roll in sorted(rolls):
                axial = pose([0, 0, 0], Rotation.from_euler("x", roll, degrees=True).as_matrix())
                adjusted = axial@relative
                target = object_poses[object_name]@adjusted
                result = ik_fit(chains[side], target, initial[side])
                result.update({"candidate_key": f"{side}_source{source['candidate_id']}_roll{roll:+g}",
                    "side": side, "object": object_name, "source_candidate_id": source["candidate_id"],
                    "axial_roll_deg": roll, "mirrored_hand_proposal": mirrored,
                    "geometric_grasp_validated": False,
                    "source_unrolled_dense_geometry_pass": bool(source.get("dense_screen_pass", False)),
                    "battery_from_hand_base": adjusted.tolist(),
                    "hand_base_from_battery": np.linalg.inv(adjusted).tolist(),
                    "open_motor_positions_rad": {n.replace(source["side"], side, 1): value for n, value in zip(source["motor_names"], source.get("open_fingers_rad", source["open_motor_rad"]))},
                    "closed_motor_positions_rad": {n.replace(source["side"], side, 1): value for n, value in zip(source["motor_names"], source.get("closed_fingers_rad", source["closed_motor_rad"]))}})
                result["coarse_table_screen"] = coarse_table_screen(k, config, spec, base, side, result)
                report["grasp_candidates"].append(result)
            # Also test the opposite side of the horizontal cylinder explicitly.
            target = object_poses[object_name]@np.diag([-1., -1., 1., 1.])@relative
            result = ik_fit(chains[side], target, initial[side])
            result.update({"candidate_key": f"{side}_source{source['candidate_id']}_opposite_side",
                           "side": side, "object": object_name, "source_candidate_id": source["candidate_id"],
                           "axial_roll_deg": None, "mirrored_hand_proposal": mirrored,
                           "geometric_grasp_validated": False,
                           "battery_from_hand_base": (np.diag([-1., -1., 1., 1.])@relative).tolist()})
            report["grasp_candidates"].append(result)

    red_xy = np.asarray(spec["props"]["red_position_xy_m"])
    red_rotation = Rotation.from_euler("z", spec["props"]["red_yaw_deg"], degrees=True).as_matrix()
    red_top = spec["table"]["height_m"]+spec["props"]["red_dims_m"][2]
    hole_centers = [table_origin+table_rotation@(np.r_[red_xy, red_top]+red_rotation@np.array([dx, 0, 0]))
                    for dx in (-spec["props"]["hole_pitch_m"], 0, spec["props"]["hole_pitch_m"])]
    report["hole_top_centers_world_m"] = [p.tolist() for p in hole_centers]
    length = spec["props"]["battery_length_m"]
    for side in ("left", "right"):
        pool = [c for c in report["grasp_candidates"] if c["side"] == side and c["axial_roll_deg"] is not None]
        pool.sort(key=lambda c: c["position_error_m"]+0.15*c["orientation_error_rad"])
        print(side, "best grasp", pool[0]["candidate_key"], pool[0]["position_error_m"], pool[0]["orientation_error_deg"], flush=True)
        sequence_pool = [c for c in pool if c["source_unrolled_dense_geometry_pass"]] or pool
        for candidate in sequence_pool[:4]:
            original = object_poses[candidate["object"]]
            relative = np.asarray(candidate["battery_from_hand_base"])
            axis = original[:3, 0]
            rotate_axis = np.cross(axis, [0, 0, 1.])
            rotate_axis /= np.linalg.norm(rotate_axis)
            turn = Rotation.from_rotvec(rotate_axis*np.pi/2).as_matrix()
            lifted = original.copy()
            lifted[2, 3] += 0.12
            vertical = lifted.copy()
            vertical[:3, :3] = turn@original[:3, :3]
            q = np.array([candidate["joint_positions_rad"][n] for n in chains[side].names])
            common = []
            for name, object_pose in (("grasp", original), ("lift_120mm", lifted), ("rotate_battery_axis_90deg_to_vertical", vertical)):
                target = object_pose@relative
                result = ik_fit(chains[side], target, q)
                result["phase"] = name
                result["desired_object_world_pose"] = object_pose.tolist()
                result["coarse_table_screen"] = coarse_table_screen(k, config, spec, base, side, result)
                common.append(result)
                q = np.array([result["joint_positions_rad"][n] for n in chains[side].names])
            for hole_id, center in enumerate(hole_centers):
                for vertical_yaw in range(-150, 181, 30):
                    carry_rotation = Rotation.from_euler("z", vertical_yaw, degrees=True).as_matrix()@vertical[:3, :3]
                    above = pose(center+[0, 0, length/2+0.035], carry_rotation)
                    seated = pose(center+[0, 0, length/2-spec["props"]["hole_depth_m"]], carry_rotation)
                    phases = list(common)
                    q2 = q.copy()
                    for name, object_pose in (("above_hole_35mm_tip_clearance", above), ("nominal_seated_13mm_insertion", seated)):
                        result = ik_fit(chains[side], object_pose@relative, q2)
                        result["phase"] = name
                        result["desired_object_world_pose"] = object_pose.tolist()
                        result["coarse_table_screen"] = coarse_table_screen(k, config, spec, base, side, result)
                        phases.append(result)
                        q2 = np.array([result["joint_positions_rad"][n] for n in chains[side].names])
                    report["task_waypoint_sequences"].append({
                        "candidate_key": candidate["candidate_key"], "side": side,
                        "hole_id_photo_left_to_right": hole_id,
                        "own_outer_hole": hole_id == (0 if side == "left" else 2),
                        "additional_vertical_battery_yaw_deg": vertical_yaw,
                        "all_ik_endpoints_pass": all(p["ik_endpoint_pass"] for p in phases),
                        "max_position_error_m": max(p["position_error_m"] for p in phases),
                        "max_orientation_error_deg": max(p["orientation_error_deg"] for p in phases),
                        "waypoints": phases})
        # Diagnose how a measurement error in base registration changes the
        # same geometric proposal. These configurations are never authored.
        for source in geometry["candidates"][:3]:
            source_id = source["candidate_id"]
            candidate = next(c for c in pool if c["source_candidate_id"] == source_id and c["axial_roll_deg"] == 0)
            for distance in (0, 0.05, 0.10, 0.15):
                shifted_base = base.copy()
                shifted_base[:3, 3] += table_rotation[:, 1]*distance
                shifted_chain = ArmChain(k, side, home, shifted_base)
                result = ik_fit(shifted_chain, np.asarray(candidate["desired_world_from_hand_base"]), initial[side])
                report["base_shift_sensitivity"].append({"candidate_key": candidate["candidate_key"],
                    "side": side, "hypothetical_base_translation_toward_table_m": distance,
                    "position_error_m": result["position_error_m"],
                    "orientation_error_deg": result["orientation_error_deg"], "ik_endpoint_pass": result["ik_endpoint_pass"]})
    report["summary"] = {}
    for side in ("left", "right"):
        candidates = [c for c in report["grasp_candidates"] if c["side"] == side]
        sequences = [s for s in report["task_waypoint_sequences"] if s["side"] == side]
        report["summary"][side] = {
            "grasp_ik_candidates_tested": len(candidates),
            "grasp_ik_endpoints_pass": sum(c["ik_endpoint_pass"] for c in candidates),
            "complete_waypoint_sets_pass": sum(s["all_ik_endpoints_pass"] for s in sequences),
            "grasp_geometry_status": "Source collision-mesh proposals plus unverified cylinder-axis rolls and left-hand mirrors",
            "grasp_contact_or_insertion_success": False}
    append_trial_screen(report, args, config, spec, k, base)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report["summary"], indent=2), flush=True)
    print(args.output, flush=True)


if __name__ == "__main__":
    main()
