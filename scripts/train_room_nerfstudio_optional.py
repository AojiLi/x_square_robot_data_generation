#!/usr/bin/env python3
"""Run a reproducible Splatfacto baseline in the capture's metric coordinates."""

import argparse
import copy
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("CUDA_HOME", str(ROOT / "tools/cuda-12.8"))
os.environ["PATH"] = str(ROOT / "tools/cuda-12.8/bin") + os.pathsep + os.environ.get("PATH", "")
os.environ.setdefault("TORCH_CUDA_ARCH_LIST", "12.0")
os.environ.setdefault("MAX_JOBS", "4")
os.environ.setdefault("OMP_NUM_THREADS", "8")
os.environ.setdefault("WANDB_MODE", "disabled")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=12000)
    parser.add_argument("--run-name", default="baseline_v1")
    parser.add_argument("--load-dir", type=Path)
    args = parser.parse_args()
    import torch
    from nerfstudio.configs.method_configs import method_configs
    from nerfstudio.scripts.train import main as train

    torch.set_num_threads(8)
    torch.set_num_interop_threads(4)
    config = copy.deepcopy(method_configs["splatfacto"])
    config.data = ROOT / "data/room01"
    config.output_dir = ROOT / "outputs"
    config.experiment_name = "room01"
    config.timestamp = args.run_name
    config.max_num_iterations = args.steps
    config.steps_per_save = 2000
    config.steps_per_eval_image = 1000
    config.steps_per_eval_all_images = 4000
    config.steps_per_eval_batch = 0
    config.vis = "tensorboard"
    config.machine.seed = 20260909
    config.machine.num_devices = 1
    config.mixed_precision = False
    if args.load_dir:
        config.load_dir = args.load_dir.resolve()
    datamanager = config.pipeline.datamanager
    datamanager.cache_images = "cpu"
    datamanager.cache_images_type = "uint8"
    dataparser = datamanager.dataparser
    dataparser.data = config.data
    dataparser.downscale_factor = 2
    dataparser.orientation_method = "none"
    dataparser.center_method = "none"
    dataparser.auto_scale_poses = False
    dataparser.scale_factor = 1.0
    dataparser.scene_scale = 4.0
    dataparser.load_3D_points = True
    model = config.pipeline.model
    model.camera_optimizer.mode = "off"
    model.num_downscales = 1
    model.resolution_schedule = 1500
    model.cull_alpha_thresh = 0.01
    model.stop_split_at = min(8000, args.steps)
    model.background_color = "random"
    config.optimizers["means"]["scheduler"].max_steps = args.steps
    print("Starting metric Splatfacto: 439 training / 49 diagnostic evaluation views", flush=True)
    train(config)


if __name__ == "__main__":
    main()
