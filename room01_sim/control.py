"""Initial servo parameters, in SI units; tune against real actuator logs."""


def joint_servo(name, mimic=False):
    if any(part in name for part in ["thumb", "index", "middle", "ring", "pinky"]):
        return (0., 0., .0001) if mimic else (4., .04, .0001)
    if "shoulder" in name:
        return 400., 30., .03
    if "elbow" in name:
        return 300., 10., .01
    if "wrist" in name:
        return 150., 3., .003
    if name.startswith("bow_"):
        return 1200., 80., .03
    if name.startswith("head_"):
        return 25., 2., .003
    return 50., 5., .01
