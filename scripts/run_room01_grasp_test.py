#!/usr/bin/env python3
"""Exercise a contact-based grasp and optionally record synchronized VLA data."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher
parser = argparse.ArgumentParser()
parser.add_argument("--seconds", type=float, default=10.)
parser.add_argument("--record", action="store_true")
parser.add_argument("--keep-open", action="store_true", help="Keep the GUI open after the test.")
parser.add_argument("--output", type=Path, default=ROOT / "outputs/room01/vla_episodes")
parser.add_argument("--offset", type=float, nargs=3, help="Override the grasp offset in task_config.json.")
parser.add_argument("--closure", type=float, help="Override the closure target in task_config.json.")
parser.add_argument("--report", type=Path, default=ROOT / "reports/room01_sim/grasp_verification.json")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = args.record or not args.headless
from room01_sim.bootstrap import launch
launcher = launch(args)
app = launcher.app
exit_code = 0
try:
    import numpy as np
    from room01_sim.runtime import RoomTaskSim, tensor
    from room01_sim.oracle import GraspTestController
    from room01_sim.episodes import EpisodeWriter
    runtime = RoomTaskSim(app, device=args.device, cameras=args.enable_cameras)
    controller = GraspTestController(runtime, approach_offset=args.offset, close=args.closure)
    writer = EpisodeWriter(args.output, runtime, controller_name="scripted_pose_feedback_grasp") if args.record else None
    log, held, longest_hold = [], 0, 0
    observation = runtime.observe(images=args.record)
    for step in range(round(args.seconds*runtime.config["control_hz"])):
        action = controller.action()
        next_observation, reward, terminated, truncated, info = runtime.step(action, images=args.record)
        if not args.headless and not args.record:
            runtime.sim.render()
        velocity = tensor(runtime.cube.data.root_link_vel_w)[0, :3].cpu().numpy()
        stable = info["object_lifted"] and float(np.linalg.norm(velocity)) < .3
        held = held+1 if stable else 0
        longest_hold = max(longest_hold, held)
        log.append({"time": next_observation["timestamp"], "phase": controller.phase, "lift_m": info["lift_height_m"],
                    "hand_position_error_m": controller.position_error, "cube_position": next_observation["cube_pose_xyzw"][:3].tolist(),
                    "hand_position": next_observation["right_hand_pose_xyzw"][:3].tolist(), "stable_lift": bool(stable)})
        if writer:
            writer.append(observation, info["applied_action"], next_observation, info)
        observation = next_observation
        if step % 30 == 0:
            print("GRASP_STEP", step, json.dumps(log[-1]), flush=True)
        if terminated:
            break
    success = longest_hold >= round(.5*runtime.config["control_hz"])
    exit_code = 0 if success else 2
    report = {"result": "pass" if success else "fail", "success": success, "longest_stable_lift_s": longest_hold/runtime.config["control_hz"],
              "success_definition": "Cube lifted >8cm above tabletop for >=0.5s, within 18cm of a hand, speed below 0.3m/s; no object attachment or gravity disabling.",
              "offset": controller.approach_offset.tolist(), "closure": controller.close, "trajectory": log}
    if writer:
        report["episode"] = str(writer.close(success_label=success, label_method=report["success_definition"]))
    args.report.write_text(json.dumps(report, indent=2))
    print("GRASP_TEST_COMPLETE", json.dumps({k:v for k,v in report.items() if k != "trajectory"}), flush=True)
    if args.keep_open and not args.headless:
        while app.is_running():
            runtime.advance_physics(runtime.substeps, render=True)
    runtime.close()
except Exception as error:
    exit_code = 1
    import traceback
    traceback.print_exc()
    args.report.write_text(json.dumps({"result": "fail", "error": str(error)}, indent=2))
    print("GRASP_TEST_FAILED", str(error), flush=True)
finally:
    app.close(exit_code=exit_code)
