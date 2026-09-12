"""Single-workcell simulator with the same action/state contract used for VLA data.

Import only after starting Isaac Lab through bootstrap.launch().
"""
from pathlib import Path
import json
import math

import numpy as np
import torch
from pxr import Usd, UsdUtils

import omni.usd
import isaaclab.sim as sim_utils
from isaaclab.actuators import IdealPDActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab_physx.physics import PhysxCfg
from isaaclab.sim.utils import open_stage

from room01_sim.kinematics import ACTION_JOINTS, RobotKinematics, quaternion_matrix, quaternion_xyzw
from room01_sim.control import joint_servo

ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets/room01/sim"


def tensor(value):
    if hasattr(value, "torch"):
        return value.torch
    if isinstance(value, torch.Tensor):
        return value
    import warp as wp
    if isinstance(value, wp.array):
        return wp.to_torch(value)
    return torch.as_tensor(value)


def actuator_configs(kinematics, locked_joints=()):
    groups = {name: [] for name in ["arms", "hands", "mimics", "torso", "head", "wheels"]}
    for name, joint in kinematics.joints.items():
        if joint["type"] == "fixed" or name in locked_joints:
            continue
        if joint["mimic"]:
            group = "mimics"
        elif name.startswith("bow_"):
            group = "torso"
        elif name.startswith("head_"):
            group = "head"
        elif "wheel" in name:
            group = "wheels"
        elif any(part in name for part in ["thumb", "index", "middle", "ring", "pinky"]):
            group = "hands"
        else:
            group = "arms"
        groups[group].append(name)
    return {group: IdealPDActuatorCfg(
        joint_names_expr=names,
        stiffness={name: joint_servo(name, group == "mimics")[0] for name in names},
        damping={name: joint_servo(name, group == "mimics")[1] for name in names},
        armature={name: joint_servo(name, group == "mimics")[2] for name in names},
        effort_limit={name: kinematics.joints[name]["effort"] for name in names},
        effort_limit_sim={name: kinematics.joints[name]["effort"] for name in names},
        velocity_limit_sim={name: kinematics.joints[name]["velocity"] for name in names},
    ) for group, names in groups.items() if names}


class RoomTaskSim:
    def __init__(self, application, *, device="cuda:0", cameras=False, seed=0, review_camera=False, diagnostic_self_collision=True, diagnostic_mimic=True, diagnostic_contacts=False):
        self.application = application
        self.config = json.loads((ASSETS / "task_config.json").read_text())
        self.kinematics = RobotKinematics()
        self.frames = json.loads((ASSETS / "robot_frames.json").read_text())
        self.rng = np.random.default_rng(seed)
        stage = open_stage(str(ASSETS / "room01_manipulation.usda"))
        cache = UsdUtils.StageCache.Get()
        omni.usd.get_context().attach_stage_with_callback(cache.Insert(stage).ToLongInt())
        self.stage = stage
        stage.SetEditTarget(stage.GetSessionLayer())
        self.contact_pairs = {}
        self._contact_subscription = None
        if diagnostic_contacts:
            from pxr import PhysxSchema, PhysicsSchemaTools, UsdPhysics
            from omni.physx import get_physx_simulation_interface
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) and str(prim.GetPath()).startswith("/World/Robot/"):
                    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            def on_contacts(headers, data):
                for header in headers:
                    a, b = str(PhysicsSchemaTools.intToSdfPath(header.actor0)), str(PhysicsSchemaTools.intToSdfPath(header.actor1))
                    key = " | ".join(sorted([a, b]))
                    record = self.contact_pairs.setdefault(key, {"max_force_n": 0., "min_separation_m": 0., "events": 0})
                    for index in range(header.contact_data_offset, header.contact_data_offset+header.num_contact_data):
                        record["max_force_n"] = max(record["max_force_n"], float(np.linalg.norm(data[index].impulse))/self.config["physics_dt"])
                        record["min_separation_m"] = min(record["min_separation_m"], float(data[index].separation))
                    record["events"] += 1
            self._contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contacts)
        if not diagnostic_self_collision:
            stage.GetPrimAtPath("/World/Robot/root_joint").GetAttribute("physxArticulation:enabledSelfCollisions").Set(False)
        if not diagnostic_mimic:
            for prim in stage.Traverse():
                for schema in prim.GetAppliedSchemas():
                    if "MimicJoint" in schema:
                        prim.RemoveAppliedSchema(schema)
        self.dt = self.config["physics_dt"]
        self.substeps = round(1/self.config["control_hz"]/self.dt)
        if not math.isclose(self.substeps*self.dt, 1/self.config["control_hz"], abs_tol=1e-9):
            raise ValueError("The camera/control period must contain an integer number of physics steps")
        self.sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=self.dt, device=device, physics=PhysxCfg(enable_scene_query_support=True, enable_external_forces_every_iteration=True),
            render_interval=self.substeps, use_fabric=True,
            render=sim_utils.RenderCfg(antialiasing_mode="DLAA", enable_dlssg=False, enable_shadows=True,
                                       enable_reflections=True, enable_translucency=True, enable_direct_lighting=True,
                                       enable_global_illumination=True, samples_per_pixel=8),
        ))
        self.device = self.sim.device
        robot_cfg = ArticulationCfg(
            prim_path="/World/Robot", spawn=None,
            init_state=ArticulationCfg.InitialStateCfg(
                pos=tuple(self.config["robot_base_position"]), rot=(0., 0., 0., 1.),
                joint_pos={name: value for name, value in self.config["joint_home_rad"].items() if name not in self.config.get("locked_joint_names", [])}, joint_vel={".*": 0.},
            ),
            actuators=actuator_configs(self.kinematics, self.config.get("locked_joint_names", [])),
        )
        self.robot = Articulation(robot_cfg)
        self.cube = RigidObject(RigidObjectCfg(
            prim_path="/World/Task/RedCube", spawn=None,
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(self.config["cube_position"])),
        ))
        self.cameras = {}
        if cameras:
            from room01_sim.sensors import create_cameras
            self.cameras = create_cameras(self.config, self.stage)
        self.review_camera = None
        if review_camera:
            from isaaclab.sensors import Camera, CameraCfg
            self.review_camera = Camera(CameraCfg(prim_path="/World/ReviewSetup", spawn=None, width=1280, height=960,
                                                   data_types=["rgb"], update_period=0., update_latest_camera_pose=True))
        self.sim.reset()
        self.robot.update(self.dt)
        self.cube.update(self.dt)
        self.joint_names = list(self.robot.joint_names)
        self.action_indices = [self.joint_names.index(name) for name in ACTION_JOINTS]
        self.mimic_indices = [(self.joint_names.index(name), self.joint_names.index(joint["mimic"]["joint"]), float(joint["mimic"].get("multiplier", 1.)))
                              for name, joint in self.kinematics.joints.items() if joint["mimic"]]
        self.targets = tensor(self.robot.data.default_joint_pos).clone()
        self.action_lower = np.array([self.kinematics.joints[name]["lower"] for name in ACTION_JOINTS], np.float32)
        self.action_upper = np.array([self.kinematics.joints[name]["upper"] for name in ACTION_JOINTS], np.float32)
        self.action_velocity = np.array([self.kinematics.joints[name]["velocity"] for name in ACTION_JOINTS], np.float32)
        self.elapsed_steps = 0
        self.episode_step = 0
        self.last_action = self.targets[0, self.action_indices].cpu().numpy().copy()
        self.reset(randomize=False)

    @property
    def time(self):
        return self.elapsed_steps*self.dt

    def advance_physics(self, steps, render=False):
        for index in range(steps):
            self.robot.set_joint_position_target_index(target=self.targets)
            # Allocate passive distal-link gravity torque to its real drive motor.
            gravity = tensor(self.robot.data.gravity_compensation_forces).clone()
            for distal, proximal, multiplier in self.mimic_indices:
                gravity[:, proximal] += multiplier*gravity[:, distal]
                gravity[:, distal] = 0.
            self.robot.set_joint_effort_target_index(target=gravity)
            self.robot.write_data_to_sim()
            self.cube.write_data_to_sim()
            self.sim.step(render=render and (index == steps-1))
            self.robot.update(self.dt)
            self.cube.update(self.dt)
            self.elapsed_steps += 1
            if not bool(torch.isfinite(tensor(self.robot.data.joint_pos)).all()):
                raise RuntimeError(f"Nonfinite robot joint state at physics step {self.elapsed_steps}")
        for camera in self.cameras.values():
            camera.update(steps*self.dt)
        if self.review_camera:
            self.review_camera.update(steps*self.dt)

    def reset(self, *, randomize=True, seed=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        q = tensor(self.robot.data.default_joint_pos).clone()
        self.targets.copy_(q)
        self.robot.write_joint_state_to_sim_index(position=q, velocity=torch.zeros_like(q))
        self.robot.reset()
        pose = tensor(self.cube.data.default_root_pose).clone()
        if randomize:
            bounds = np.asarray(self.config["object_spawn_xy_range"])
            pose[0, :2] = torch.tensor(self.rng.uniform(bounds[:, 0], bounds[:, 1]), device=self.device, dtype=torch.float32)
        self.cube.write_root_pose_to_sim_index(root_pose=pose)
        self.cube.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=self.device))
        self.cube.reset()
        self.advance_physics(48)
        self.episode_step = 0
        self.last_action = self.targets[0, self.action_indices].cpu().numpy().copy()
        return self.observe(images=False), {"seed": seed, "reset_settle_seconds": 48*self.dt}

    def observe(self, *, images=True):
        positions = tensor(self.robot.data.joint_pos)[0]
        velocities = tensor(self.robot.data.joint_vel)[0]
        observation = {
            "state": positions[self.action_indices].cpu().numpy().copy(),
            "joint_velocity": velocities[self.action_indices].cpu().numpy().copy(),
            "cube_pose_xyzw": tensor(self.cube.data.root_link_pose_w)[0].cpu().numpy().copy(),
            "timestamp": self.time,
            "language_instruction": self.config["language_instruction"],
        }
        for side in ["left", "right"]:
            frame = self.frames[side+"_hand_base_link"]
            index = self.robot.body_names.index(frame["rigid_body"])
            body = tensor(self.robot.data.body_link_pose_w)[0, index].cpu().numpy()
            matrix = np.eye(4)
            matrix[:3, :3] = quaternion_matrix(body[3:])
            matrix[:3, 3] = body[:3]
            pose = matrix@np.asarray(frame["body_to_frame"])
            observation[side+"_hand_pose_xyzw"] = np.concatenate([pose[:3, 3], quaternion_xyzw(pose[:3, :3])]).astype(np.float32)
        if images and self.cameras:
            from room01_sim.sensors import capture_frames
            observation.update(capture_frames(self))
        return observation

    def step(self, action, *, images=True):
        action = np.asarray(action, dtype=np.float32)
        if action.shape != (len(ACTION_JOINTS),) or not np.isfinite(action).all():
            raise ValueError(f"Action must be {len(ACTION_JOINTS)} finite joint targets in radians")
        clipped = np.clip(action, self.action_lower, self.action_upper)
        max_delta = self.action_velocity/self.config["control_hz"]
        applied = self.last_action+np.clip(clipped-self.last_action, -max_delta, max_delta)
        self.targets[0, self.action_indices] = torch.as_tensor(applied, device=self.device)
        self.last_action = applied.copy()
        self.advance_physics(self.substeps)
        self.episode_step += 1
        obs = self.observe(images=images)
        height = float(obs["cube_pose_xyzw"][2]-self.config["tabletop_z"]-self.config["cube_size"][2]/2)
        dropped = float(obs["cube_pose_xyzw"][2]) < self.config["tabletop_z"]-.15
        hand_distance = min(float(np.linalg.norm(obs[side+"_hand_pose_xyzw"][:3]-obs["cube_pose_xyzw"][:3])) for side in ["left", "right"])
        reward = math.exp(-5*hand_distance)+5*max(0., height)
        lifted = height > .08 and hand_distance < .18
        truncated = self.episode_step >= 30*self.config["control_hz"]
        return obs, reward, dropped, truncated, {"object_lifted": lifted, "lift_height_m": height, "applied_action": applied.copy(), "action_clipped": bool(np.any(np.abs(applied-action) > 1e-5))}

    def close(self):
        self.sim.stop()
