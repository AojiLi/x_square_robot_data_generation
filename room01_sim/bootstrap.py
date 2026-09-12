"""Launch the pinned Isaac Lab runtime without unrelated task extensions."""
from pathlib import Path
import json
import os
import re
import fcntl
import subprocess

ROOT = Path(__file__).resolve().parents[1]
_renderer_lock = None


def launch(args):
    global _renderer_lock
    if getattr(args, "enable_cameras", False):
        lock_path = ROOT / "tools/room01_sim_runtime/camera_render.lock"
        lock_path.parent.mkdir(parents=True, exist_ok=True)
        _renderer_lock = lock_path.open("a+")
        try:
            fcntl.flock(_renderer_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError("A room01 camera-rendering process is already running. Close that preview or wait for its recording before starting another.") from error
        _renderer_lock.seek(0);_renderer_lock.truncate();_renderer_lock.write(str(os.getpid()));_renderer_lock.flush()
        if str(getattr(args, "device", "cuda:0")).startswith("cuda"):
            try:
                free = subprocess.check_output(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"], text=True)
                device_index = int(str(getattr(args, "device", "cuda:0")).split(":")[-1])
                free_mb = int(free.splitlines()[device_index])
                if free_mb < 7000:
                    raise RuntimeError(f"Only {free_mb} MiB GPU memory is free. This three-camera workcell requires at least 7000 MiB free; close another GPU renderer first.")
            except (FileNotFoundError, subprocess.CalledProcessError, ValueError, IndexError):
                pass
    from isaaclab.app import AppLauncher
    lab = Path(os.environ["ISAACLAB_ROOT"])
    target = ROOT / "tools/room01_sim_runtime/experiences"
    target.mkdir(parents=True, exist_ok=True)
    for suffix in ["", ".headless", ".rendering", ".headless.rendering"]:
        template = (lab / "apps" / f"isaaclab.python{suffix}.kit").read_text()
        template = re.sub(r'^"isaaclab_(?:tasks|rl|mimic|contrib)"\s*=.*\n', "", template, flags=re.MULTILINE)
        template = template.replace('"${app}/../source"', json.dumps(str(lab / "source")))
        template = template.replace("isaaclab.python", "room01.python")
        destination = target / f"room01.python{suffix}.kit"
        if not destination.exists() or destination.read_text() != template:
            temporary = target / f"room01.python{suffix}.{os.getpid()}.tmp"
            temporary.write_text(template)
            temporary.replace(destination)
    headless = bool(getattr(args, "headless", False))
    args.visualizer = ["none"] if headless else ["kit"]
    args.visualizer_explicit = True
    suffix = (".headless" if headless else "")+(".rendering" if getattr(args, "enable_cameras", False) else "")
    args.experience = str(target / f"room01.python{suffix}.kit")
    args.kit_args = (getattr(args, "kit_args", "")+f" --ext-folder {target}").strip()
    args.width, args.height = 1280, 960
    args.window_width, args.window_height = 1440, 1000
    return AppLauncher(args)
