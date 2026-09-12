"""URDF forward kinematics and bounded pose fitting, independent of Isaac Sim."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/robots/quanta_x2/quanta_x2_dual_revo2_bridge.urdf"


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)+np.sin(angle)*skew+(1-np.cos(angle))*(skew@skew)


def origin_transform(element):
    transform = np.eye(4)
    if element is None:
        return transform
    xyz = [float(x) for x in element.get("xyz", "0 0 0").split()]
    r, p, y = [float(x) for x in element.get("rpy", "0 0 0").split()]
    transform[:3, :3] = rotation([0, 0, 1], y)@rotation([0, 1, 0], p)@rotation([1, 0, 0], r)
    transform[:3, 3] = xyz
    return transform


def quaternion_xyzw(matrix):
    """Return a normalized quaternion for a column-vector rotation matrix."""
    m = np.asarray(matrix, dtype=float)
    values, vectors = np.linalg.eigh(np.array([
        [m[0, 0]-m[1, 1]-m[2, 2], m[0, 1]+m[1, 0], m[0, 2]+m[2, 0], m[2, 1]-m[1, 2]],
        [m[0, 1]+m[1, 0], m[1, 1]-m[0, 0]-m[2, 2], m[1, 2]+m[2, 1], m[0, 2]-m[2, 0]],
        [m[0, 2]+m[2, 0], m[1, 2]+m[2, 1], m[2, 2]-m[0, 0]-m[1, 1], m[1, 0]-m[0, 1]],
        [m[2, 1]-m[1, 2], m[0, 2]-m[2, 0], m[1, 0]-m[0, 1], np.trace(m)],
    ])/3.)
    q = vectors[:, np.argmax(values)]
    return q if q[3] >= 0 else -q


def quaternion_matrix(xyzw):
    x, y, z, w = np.asarray(xyzw)/np.linalg.norm(xyzw)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class RobotKinematics:
    def __init__(self, path=URDF):
        document = ET.parse(path).getroot()
        self.joints = {}
        self.links = {e.get("name"): e for e in document.findall("link")}
        children = set()
        for element in document.findall("joint"):
            limit = element.find("limit")
            mimic = element.find("mimic")
            name = element.get("name")
            self.joints[name] = {
                "name": name, "type": element.get("type"),
                "parent": element.find("parent").get("link"), "child": element.find("child").get("link"),
                "origin": origin_transform(element.find("origin")),
                "axis": np.array([float(x) for x in element.find("axis").get("xyz").split()]) if element.find("axis") is not None else np.array([1., 0., 0.]),
                "lower": float(limit.get("lower", "-3.14159265")) if limit is not None else 0.,
                "upper": float(limit.get("upper", "3.14159265")) if limit is not None else 0.,
                "effort": float(limit.get("effort", "0")) if limit is not None else 0.,
                "velocity": float(limit.get("velocity", "0")) if limit is not None else 0.,
                "mimic": dict(mimic.attrib) if mimic is not None else None,
            }
            children.add(self.joints[name]["child"])
        self.root = next(name for name in self.links if name not in children)

    def expand_mimics(self, positions):
        result = dict(positions)
        for name, joint in self.joints.items():
            mimic = joint["mimic"]
            if mimic:
                result[name] = result.get(mimic["joint"], 0.)*float(mimic.get("multiplier", 1.))+float(mimic.get("offset", 0.))
        return result

    def forward(self, positions, base=None):
        q = self.expand_mimics(positions)
        transforms = {self.root: np.eye(4) if base is None else np.asarray(base)}
        pending = list(self.joints.values())
        while pending:
            ready = [j for j in pending if j["parent"] in transforms]
            if not ready:
                raise ValueError("URDF contains a disconnected joint graph")
            for joint in ready:
                relative = joint["origin"].copy()
                if joint["type"] in {"revolute", "continuous"}:
                    relative[:3, :3] = relative[:3, :3]@rotation(joint["axis"], q.get(joint["name"], 0.))
                transforms[joint["child"]] = transforms[joint["parent"]]@relative
                pending.remove(joint)
        return transforms

    def solve_arm(self, side, target_position, target_rotation, positions, *, base=None):
        from scipy.optimize import least_squares
        from scipy.spatial.transform import Rotation
        names = [f"{side}_{suffix}_joint" for suffix in ARM_SUFFIXES]
        initial = np.array([positions[name] for name in names])
        lower = np.array([self.joints[name]["lower"] for name in names])
        upper = np.array([self.joints[name]["upper"] for name in names])
        target_position = np.asarray(target_position)
        target_rotation = np.asarray(target_rotation)
        def residual(values):
            q = dict(positions);q.update(zip(names, values))
            pose = self.forward(q, base)[side+"_hand_base_link"]
            angle = Rotation.from_matrix(pose[:3, :3].T@target_rotation).as_rotvec()
            return np.r_[pose[:3, 3]-target_position, .15*angle, .0003*(values-initial)]
        result = least_squares(residual, np.clip(initial, lower+1e-6, upper-1e-6), bounds=(lower, upper), max_nfev=300,
                               ftol=1e-10, xtol=1e-10, gtol=1e-10)
        q = dict(positions);q.update(zip(names, result.x))
        pose = self.forward(q, base)[side+"_hand_base_link"]
        error = float(np.linalg.norm(pose[:3, 3]-target_position))
        angle_error = float(np.linalg.norm(Rotation.from_matrix(pose[:3, :3].T@target_rotation).as_rotvec()))
        if not result.success or error > .005 or angle_error > .05:
            raise ValueError(f"IK did not reach target: {side}, position={error:.4f}m, angle={angle_error:.4f}rad")
        return q, {"position_error_m": error, "rotation_error_rad": angle_error, "joint_positions_rad": dict(zip(names, result.x.tolist()))}

    def arm_jacobian(self, side, positions, *, base=None):
        poses = self.forward(positions, base)
        end = poses[side+"_hand_base_link"][:3, 3]
        columns = []
        for suffix in ARM_SUFFIXES:
            joint = self.joints[f"{side}_{suffix}_joint"]
            frame = poses[joint["parent"]]@joint["origin"]
            axis = frame[:3, :3]@joint["axis"]
            columns.append(np.r_[np.cross(axis, end-frame[:3, 3]), axis])
        return np.asarray(columns).T


ARM_SUFFIXES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_roll", "elbow_yaw", "wrist_roll", "wrist_pitch")
HAND_SUFFIXES = ("thumb_metacarpal", "thumb_proximal", "index_proximal", "middle_proximal", "ring_proximal", "pinky_proximal")
ACTION_JOINTS = tuple(f"{side}_{name}_joint" for side in ["left", "right"] for name in ARM_SUFFIXES+HAND_SUFFIXES)


def home_pose():
    positions = {name: 0. for name in RobotKinematics().joints}
    for side, arm in [("left", [1.57, -1.57, -1.57, -1.57, 0., 0., 0.]),
                      ("right", [-1.57, -1.57, 1.57, -1.57, 0., 0., 0.])]:
        positions.update({f"{side}_{suffix}_joint": q for suffix, q in zip(ARM_SUFFIXES, arm)})
    return positions
