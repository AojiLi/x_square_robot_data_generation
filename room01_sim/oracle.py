"""A bounded pose-feedback controller for exercising real object contacts.

This is a scripted test/data-collection controller, not a trained VLA policy.
It uses simulator object poses to define the task trajectory.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from room01_sim.kinematics import ACTION_JOINTS, ARM_SUFFIXES, quaternion_matrix
from room01_sim.runtime import tensor


def blend(start, end, amount):
    amount = np.clip(amount, 0., 1.)
    smooth = amount*amount*(3-2*amount)
    return (1-smooth)*np.asarray(start)+smooth*np.asarray(end)


class GraspTestController:
    def __init__(self, runtime, *, approach_offset=None, close=None):
        self.runtime = runtime
        parameters = runtime.config["scripted_grasp_parameters"]
        self.approach_offset = np.asarray(parameters["approach_offset_m"] if approach_offset is None else approach_offset, dtype=float)
        self.initial = runtime.observe(images=False)
        self.cube = self.initial["cube_pose_xyzw"][:3].copy()
        self.start = self.initial["right_hand_pose_xyzw"][:3].copy()
        self.grasp = self.cube-self.approach_offset
        self.above = self.grasp+np.array([0., 0., .13])
        self.lift = self.grasp+np.array([0., 0., .16])
        self.rotation = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
        self.close = float(parameters["closure_target_rad"] if close is None else close)
        self.start_time = runtime.time
        self.phase = "settle"
        preparation = runtime.config.get("scripted_grasp_preparation", {})
        self.preparation_seconds = float(preparation.get("duration_s", 0.))
        self.preparation_target = self.initial["state"].copy()
        for name, value in preparation.get("joint_targets_rad", {}).items():
            self.preparation_target[ACTION_JOINTS.index(name)] = value
        self.prepared = False

    def action(self):
        runtime = self.runtime
        elapsed = runtime.time-self.start_time
        if elapsed < self.preparation_seconds:
            self.phase = "orient_for_grasp"
            self.target_position = self.start
            current = runtime.observe(images=False)["right_hand_pose_xyzw"][:3]
            self.position_error = float(np.linalg.norm(self.start-current))
            return blend(self.initial["state"], self.preparation_target, elapsed/self.preparation_seconds)
        if not self.prepared:
            self.start = runtime.observe(images=False)["right_hand_pose_xyzw"][:3].copy()
            self.prepared = True
        elapsed -= self.preparation_seconds
        opening = .08
        if elapsed < .5:
            target = self.start
            self.phase = "settle"
        elif elapsed < 2.:
            target = blend(self.start, self.above, (elapsed-.5)/1.5)
            self.phase = "approach"
        elif elapsed < 3.5:
            target = blend(self.above, self.grasp, (elapsed-2.)/1.5)
            self.phase = "descend"
        elif elapsed < 5.5:
            target = self.grasp
            opening = float(blend(.08, self.close, (elapsed-3.5)/2.))
            self.phase = "close"
        elif elapsed < 7.5:
            target = blend(self.grasp, self.lift, (elapsed-5.5)/2.)
            opening = self.close
            self.phase = "lift"
        else:
            target = self.lift
            opening = self.close
            self.phase = "hold"
        obs = runtime.observe(images=False)
        current_pose = obs["right_hand_pose_xyzw"]
        orientation_error = Rotation.from_matrix(self.rotation@quaternion_matrix(current_pose[3:]).T).as_rotvec()
        error = np.r_[target-current_pose[:3], .15*orientation_error]
        q = tensor(runtime.robot.data.joint_pos)[0].cpu().numpy()
        positions = dict(runtime.config["joint_home_rad"])
        positions.update(zip(runtime.joint_names, q))
        base_pose = tensor(runtime.robot.data.root_link_pose_w)[0].cpu().numpy()
        base = np.eye(4);base[:3, :3] = quaternion_matrix(base_pose[3:]);base[:3, 3] = base_pose[:3]
        jacobian = runtime.kinematics.arm_jacobian("right", positions, base=base)
        jacobian[3:] *= .15
        delta = jacobian.T@np.linalg.solve(jacobian@jacobian.T+.004**2*np.eye(6), .25*error)
        action = runtime.last_action.copy()
        for suffix, change in zip(ARM_SUFFIXES, delta):
            index = ACTION_JOINTS.index("right_"+suffix+"_joint")
            action[index] += np.clip(change, -.045, .045)
        for finger in ["index", "middle", "ring", "pinky"]:
            action[ACTION_JOINTS.index("right_"+finger+"_proximal_joint")] = opening
        action[ACTION_JOINTS.index("right_thumb_metacarpal_joint")] = opening
        action[ACTION_JOINTS.index("right_thumb_proximal_joint")] = min(opening, .8)
        self.target_position = target
        self.position_error = float(np.linalg.norm(target-current_pose[:3]))
        return action
