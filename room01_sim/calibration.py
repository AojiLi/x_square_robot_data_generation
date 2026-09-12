"""Camera optics validation and projections in ROS optical coordinates.

Calibration values and their physical sensor mapping are separate concerns.
Unknown source extrinsics must never be interpreted as robot-link transforms.
"""
import numpy as np


def validate_optics(spec):
    """Reject incomplete or ambiguous numerical camera models before authoring."""
    width, height = spec["resolution"]
    if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in (width, height)):
        raise ValueError("Camera resolution must contain two positive integers")
    values = [spec[k] for k in ("fx", "fy", "cx", "cy", "near", "far")]
    if not np.isfinite(values).all() or min(values[:2]) <= 0:
        raise ValueError("Camera intrinsics must be finite with positive focal lengths")
    if not 0 < spec["near"] < spec["far"]:
        raise ValueError("Invalid camera clipping range")
    models = {"pinhole": 8, "opencv_pinhole": 8, "fisheye_equidistant": 4, "opencv_fisheye": 4}
    if spec["model"] not in models:
        raise ValueError("Unsupported distortion model: "+spec["model"])
    coefficients = spec.get("distortion_coefficients", [0.] * models[spec["model"]])
    if len(coefficients) != models[spec["model"]] or not np.isfinite(coefficients).all():
        raise ValueError("Invalid distortion coefficient count or values")
    if not np.isfinite(spec["local_position"]).all():
        raise ValueError("Nonfinite camera position")
    q = np.asarray(spec["local_rotation_xyzw"])
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-5:
        raise ValueError("Camera orientation must be a unit xyzw quaternion")


def project_ros(points, spec):
    """Project optical-frame XYZ through the model actually used by RTX.

    Fisheye angles use atan2 so off-axis rays beyond 90 degrees do not fold
    into the front hemisphere. Non-visible pinhole points yield NaN.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.shape[-1] != 3:
        raise ValueError("Expected optical-frame XYZ points")
    x, y, z = np.moveaxis(points, -1, 0)
    if spec["model"] in {"fisheye_equidistant", "opencv_fisheye"}:
        radius = np.hypot(x, y)
        theta = np.arctan2(radius, z)
        k1, k2, k3, k4 = spec.get("distortion_coefficients", [0.] * 4)
        t2 = theta*theta
        distorted = theta*(1+t2*(k1+t2*(k2+t2*(k3+t2*k4))))
        scale = np.divide(distorted, radius, out=np.zeros_like(radius), where=radius > 1e-12)
        u, v = x*scale, y*scale
        valid = np.linalg.norm(points, axis=-1) > 1e-12
        valid &= ~((radius <= 1e-12) & (z <= 0))
    else:
        with np.errstate(divide="ignore", invalid="ignore"):
            xn, yn = x/z, y/z
            r2 = xn*xn+yn*yn
            k1, k2, p1, p2, k3, k4, k5, k6 = spec.get("distortion_coefficients", [0.] * 8)
            denominator = 1+r2*(k4+r2*(k5+r2*k6))
            radial = (1+r2*(k1+r2*(k2+r2*k3)))/denominator
            u = xn*radial+2*p1*xn*yn+p2*(r2+2*xn*xn)
            v = yn*radial+p1*(r2+2*yn*yn)+2*p2*xn*yn
        valid = (z > 0) & (np.abs(denominator) > 1e-12)
    result = np.stack([spec["fx"]*u+spec["cx"], spec["fy"]*v+spec["cy"]], axis=-1)
    return np.where(np.asarray(valid)[..., None], result, np.nan)
