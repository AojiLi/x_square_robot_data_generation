#!/usr/bin/env python3
"""Train the metric Polycam scene with gsplat's official rasterizer/strategy.

The dataset remains compatible with Nerfstudio. This focused runner avoids the
unrelated dependencies of the complete Nerfstudio application. Core Gaussian
parameterization and optimizer defaults follow gsplat v1.4.0's simple_trainer:
https://github.com/nerfstudio-project/gsplat/blob/v1.4.0/examples/simple_trainer.py
"""

import argparse
import json
import math
import os
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("CUDA_HOME", str(ROOT / "tools/cuda-12.8"))
os.environ["PATH"] = str(ROOT / "tools/cuda-12.8/bin") + os.pathsep + str(ROOT / ".venv/bin") + os.pathsep + os.environ.get("PATH", "")
os.environ.setdefault("TORCH_CUDA_ARCH_LIST", "12.0")
os.environ.setdefault("MAX_JOBS", "4")
os.environ.setdefault("OMP_NUM_THREADS", "8")

import numpy as np
import torch
from PIL import Image, ImageDraw
from plyfile import PlyData, PlyElement
from pytorch_msssim import ssim
from scipy.spatial import cKDTree

from gsplat import rasterization
from gsplat.strategy import DefaultStrategy


def json_write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")


def load_dataset(path, factor=2):
    meta = json.loads((path / "transforms.json").read_text())
    train_names = set(meta["train_filenames"])
    eval_names = set(meta["val_filenames"])
    datasets = {"train": [], "eval": []}
    flip = np.diag([1., -1., -1., 1.])
    for i, frame in enumerate(meta["frames"]):
        name = Path(frame["file_path"]).name
        camera_to_world_cv = np.array(frame["transform_matrix"]) @ flip
        k = np.array([[frame["fl_x"]/factor, 0, frame["cx"]/factor],
                      [0, frame["fl_y"]/factor, frame["cy"]/factor], [0, 0, 1]], dtype=np.float32)
        image_dir = "images" if factor == 1 else f"images_{factor}"
        mask_dir = "masks" if factor == 1 else f"masks_{factor}"
        record = {"frame_index": i, "frame_id": Path(name).stem,
                  "view": torch.tensor(np.linalg.inv(camera_to_world_cv), dtype=torch.float32, device="cuda")[None],
                  "K": torch.from_numpy(k).cuda()[None],
                  "rgb": torch.from_numpy(np.array(Image.open(path / image_dir / name))).pin_memory(),
                  "mask": torch.from_numpy(np.array(Image.open(path / mask_dir / name)) > 0).pin_memory()}
        if frame["file_path"] in train_names:
            record["low_rgb"] = torch.from_numpy(np.array(Image.open(path / f"images_{factor*2}" / name))).pin_memory()
            record["low_mask"] = torch.from_numpy(np.array(Image.open(path / f"masks_{factor*2}" / name)) > 0).pin_memory()
            datasets["train"].append(record)
        elif frame["file_path"] in eval_names:
            datasets["eval"].append(record)
        else:
            raise ValueError("Frame has no assigned split")
    if len(datasets["train"]) != 439 or len(datasets["eval"]) != 49:
        raise ValueError("Unexpected view counts")
    return datasets


def initialize(path):
    vertex = PlyData.read(str(path))["vertex"]
    points = np.column_stack([vertex[key] for key in "xyz"])
    rgb = np.column_stack([vertex[key] for key in ["red", "green", "blue"]]).astype(np.float32)/255.
    distances, _ = cKDTree(points).query(points, k=4, workers=4)
    size = np.sqrt((distances[:, 1:]**2).mean(axis=1)).clip(0.002, 0.2)
    n = len(points)
    values = {
        "means": torch.from_numpy(points.copy()),
        "scales": torch.from_numpy(np.repeat(np.log(size).astype(np.float32)[:, None], 3, axis=1)),
        "quats": torch.randn(n, 4),
        "opacities": torch.full((n,), math.log(0.1/0.9)),
        "sh0": torch.from_numpy(((rgb-0.5)/0.28209479177387814)[:, None, :]),
        "shN": torch.zeros(n, 15, 3),
    }
    return torch.nn.ParameterDict({key: torch.nn.Parameter(value.float().cuda()) for key, value in values.items()})


def optimizers_for(splats):
    rates = {"means": 1.6e-4, "scales": 5e-3, "quats": 1e-3,
             "opacities": 5e-2, "sh0": 2.5e-3, "shN": 2.5e-3/20}
    return {key: torch.optim.Adam([{"params": value, "lr": rates[key], "name": key}], eps=1e-15)
            for key, value in splats.items()}


def render(splats, record, degree=3, low_resolution=False, random_background=False, gradients=False):
    rgb = record["low_rgb"] if low_resolution else record["rgb"]
    height, width = rgb.shape[:2]
    k = record["K"].clone()
    if low_resolution:
        k[:, :2] *= 0.5
    background = torch.rand(1, 3, device="cuda") if random_background else torch.zeros(1, 3, device="cuda")
    colors, alpha, info = rasterization(
        means=splats["means"], quats=splats["quats"], scales=splats["scales"].exp(),
        opacities=splats["opacities"].sigmoid(), colors=torch.cat([splats["sh0"], splats["shN"]], dim=1),
        viewmats=record["view"], Ks=k, width=width, height=height, sh_degree=degree,
        near_plane=0.05, far_plane=100., backgrounds=background,
        packed=False, absgrad=gradients, rasterize_mode="classic",
    )
    return colors[0].clamp(0, 1), info


def compare(prediction, target, mask):
    m = mask[..., None].float()
    mse = ((prediction-target).square()*m).sum()/(3*m.sum()).clamp_min(1)
    psnr = -10*torch.log10(mse.clamp_min(1e-12))
    similarity = ssim((prediction*m).permute(2, 0, 1)[None],
                      (target*m).permute(2, 0, 1)[None], data_range=1., size_average=True)
    return psnr, similarity


@torch.no_grad()
def evaluate(splats, records, output, step, full=False):
    indices = list(range(len(records))) if full else [0, 6, 12, 18, 25, 31, 37, 48]
    rows, previews = [], {}
    for index in indices:
        record = records[index]
        target = record["rgb"].cuda(non_blocking=True).float()/255.
        mask = record["mask"].cuda(non_blocking=True)
        prediction, _ = render(splats, record, degree=min(3, step//1000))
        psnr, similarity = compare(prediction, target, mask)
        row = {"frame_id": record["frame_id"], "frame_index": record["frame_index"],
               "psnr_db": float(psnr), "ssim": float(similarity)}
        rows.append(row)
        if full:
            Image.fromarray((prediction.cpu().numpy()*255).round().astype(np.uint8)).save(output / "eval_renders" / f"{record['frame_id']}.png")
        if index in [0, 12, 25, 37]:
            source = Image.fromarray(record["rgb"].numpy()).transpose(Image.Transpose.ROTATE_270)
            predicted = Image.fromarray((prediction.cpu().numpy()*255).round().astype(np.uint8)).transpose(Image.Transpose.ROTATE_270)
            previews[index] = (source, predicted, row)
    metrics = {"step": step, "evaluation_views": len(rows),
               "resolution": [int(records[0]["rgb"].shape[1]), int(records[0]["rgb"].shape[0])],
               "mean_psnr_db": float(np.mean([row["psnr_db"] for row in rows])),
               "mean_ssim": float(np.mean([row["ssim"] for row in rows])),
               "gaussians": len(splats["means"]), "views": rows,
               "scope": "Same-scan diagnostic views excluded from RGB optimization; scan geometry and poses use all frames."}
    json_write(output / f"eval_{step:06d}.json", metrics)
    if previews:
        w, h = next(iter(previews.values()))[0].size
        panel = Image.new("RGB", (w*4, (h+40)*2), (245, 245, 245))
        draw = ImageDraw.Draw(panel)
        for position, index in enumerate(sorted(previews)):
            source, predicted, row = previews[index]
            x, y = (position % 2)*(2*w), (position//2)*(h+40)
            panel.paste(source, (x, y)); panel.paste(predicted, (x+w, y))
            draw.text((x+6, y+h+5), f"View {row['frame_index']+1}: photo | Gaussian render", fill=(20, 20, 20))
            draw.text((x+6, y+h+21), f"PSNR {row['psnr_db']:.2f} dB, SSIM {row['ssim']:.3f}; step {step}", fill=(20, 20, 20))
        panel.save(output / f"comparison_{step:06d}.jpg", quality=92)
    print(json.dumps({key: value for key, value in metrics.items() if key != "views"}), flush=True)
    return metrics


@torch.no_grad()
def export_ply(splats, path):
    positions = splats["means"].detach().cpu().numpy()
    dc = splats["sh0"].detach().cpu().numpy()[:, 0, :]
    rest = splats["shN"].detach().cpu().numpy().transpose(0, 2, 1).reshape(len(positions), -1)
    rotation = torch.nn.functional.normalize(splats["quats"], dim=-1).detach().cpu().numpy()
    values = np.concatenate([positions, np.zeros_like(positions), dc, rest,
                             splats["opacities"].detach().cpu().numpy()[:, None],
                             splats["scales"].detach().cpu().numpy(), rotation], axis=1)
    if not np.isfinite(values).all():
        raise ValueError("Refusing to export nonfinite Gaussian parameters")
    names = list("xyz") + ["nx", "ny", "nz"] + [f"f_dc_{i}" for i in range(3)]
    names += [f"f_rest_{i}" for i in range(rest.shape[1])]
    names += ["opacity"] + [f"scale_{i}" for i in range(3)] + [f"rot_{i}" for i in range(4)]
    array = np.empty(len(positions), dtype=[(name, "<f4") for name in names])
    for index, name in enumerate(names):
        array[name] = values[:, index]
    PlyData([PlyElement.describe(array, "vertex")], text=False, byte_order="<",
            comments=["Trained Gaussian splats; units meters; right-handed Z-up",
                      "Scalar-first quaternions; log-scales; logit-opacity; SH degree 3"]).write(str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=12000)
    parser.add_argument("--run-name", default="gsplat_baseline_v1")
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--eval-only", action="store_true")
    parser.add_argument("--max-gaussians", type=int, default=600000)
    parser.add_argument("--refine-start", type=int, default=500)
    parser.add_argument("--refine-stop", type=int, default=8000)
    parser.add_argument("--reset-every", type=int, default=3000)
    parser.add_argument("--factor", type=int, choices=[1, 2], default=2)
    args = parser.parse_args()
    data = ROOT / "data/room01"
    output = ROOT / "outputs/room01" / args.run_name
    output.mkdir(parents=True, exist_ok=True)
    (output / "eval_renders").mkdir(exist_ok=True)
    if (output / "checkpoint.pt").exists() and args.resume is None:
        raise SystemExit("Output already has a checkpoint; use --resume or a new --run-name")
    torch.set_num_threads(8)
    torch.set_num_interop_threads(4)
    torch.manual_seed(20260909)
    torch.cuda.manual_seed_all(20260909)
    rng = np.random.default_rng(20260909)
    checkpoint = None
    if args.resume:
        checkpoint = torch.load(args.resume, map_location="cuda", weights_only=False)
        splats = torch.nn.ParameterDict({key: torch.nn.Parameter(value.cuda()) for key, value in checkpoint["splats"].items()})
    else:
        splats = initialize(data / "points3D.ply")
    optimizers = optimizers_for(splats)
    strategy = DefaultStrategy(prune_opa=0.01, grow_grad2d=0.0008, grow_scale3d=0.01,
                               prune_scale3d=0.5, prune_scale2d=0.15, grow_scale2d=0.05,
                               refine_scale2d_stop_iter=4000, refine_start_iter=args.refine_start,
                               refine_stop_iter=min(args.refine_stop, args.steps), reset_every=args.reset_every,
                               refine_every=100, pause_refine_after_reset=539, absgrad=True)
    strategy.check_sanity(splats, optimizers)
    state = strategy.initialize_state(scene_scale=1.0)
    start = 0
    if checkpoint:
        start = checkpoint["step"]
        state = checkpoint["strategy_state"]
        for key, optimizer in optimizers.items():
            optimizer.load_state_dict(checkpoint["optimizers"][key])
        rng.bit_generator.state = json.loads(checkpoint["numpy_rng_json"])
        torch.set_rng_state(checkpoint["torch_rng"].cpu())
        torch.cuda.set_rng_state(checkpoint["cuda_rng"].cpu())
    records = load_dataset(data, factor=args.factor)
    if args.eval_only:
        evaluate(splats, records["eval"], output, start, full=True)
        return
    config = {"backend": "gsplat 1.4.0 official rasterizer and DefaultStrategy, focused local runner",
              "steps": args.steps, "resolution": [992//args.factor, 736//args.factor],
              "warmup_resolution": [496//args.factor, 368//args.factor], "image_downscale_factor": args.factor,
              "full_resolution_from_step": 1500, "train_views": 439, "evaluation_views": 49,
              "initial_gaussians": len(splats["means"]), "coordinate_system": "meters, right-handed Z-up",
              "camera_optimization": False, "depth_loss": False,
              "initialization": "Polycam colored point cloud aligned to corrected cameras",
              "ssim_weight": 0.2, "sh_degree": 3, "seed": 20260909,
              "stop_densification_at_step": min(args.refine_stop, args.steps), "point_budget": args.max_gaussians,
              "refine_start": args.refine_start, "opacity_reset_interval": args.reset_every,
              "resume_source": str(args.resume.resolve()) if args.resume else None,
              "gpu": torch.cuda.get_device_name(), "torch": torch.__version__}
    json_write(output / "run_config.json", config)
    print(f"Starting training: {len(splats['means']):,} Gaussians, 439 train / 49 evaluation views", flush=True)
    begin = time.monotonic()
    order = checkpoint.get("remaining_epoch_order", []) if checkpoint else []
    log = (output / "progress.jsonl").open("a", buffering=1)
    for step in range(start, args.steps):
        if not order:
            order = rng.permutation(len(records["train"])).tolist()
        record = records["train"][order.pop()]
        low = step < 1500
        target = record["low_rgb" if low else "rgb"].cuda(non_blocking=True).float()/255.
        mask = record["low_mask" if low else "mask"].cuda(non_blocking=True)
        prediction, info = render(splats, record, degree=min(3, step//1000), low_resolution=low,
                                  random_background=True, gradients=True)
        strategy.step_pre_backward(splats, optimizers, state, step, info)
        m = mask[..., None].float()
        l1 = ((prediction-target).abs()*m).sum()/(3*m.sum()).clamp_min(1)
        similarity = ssim((prediction*m).permute(2, 0, 1)[None], (target*m).permute(2, 0, 1)[None],
                          data_range=1., size_average=True)
        loss = 0.8*l1+0.2*(1-similarity)
        if not torch.isfinite(loss):
            raise FloatingPointError(f"Nonfinite loss at step {step}")
        loss.backward()
        optimizers["means"].param_groups[0]["lr"] = 1.6e-4 * math.exp(math.log(0.01)*step/args.steps)
        for optimizer in optimizers.values():
            optimizer.step()
            optimizer.zero_grad(set_to_none=True)
        strategy.step_post_backward(splats, optimizers, state, step, info, packed=False)
        if len(splats["means"]) > args.max_gaussians and strategy.refine_stop_iter > step:
            strategy.refine_stop_iter = step
            config["densification_stopped_for_point_budget_at"] = step
            json_write(output / "run_config.json", config)
        if step == start or (step+1) % 100 == 0:
            torch.cuda.synchronize()
            progress = {"step": step+1, "loss": float(loss), "l1": float(l1), "ssim": float(similarity),
                        "gaussians": len(splats["means"]), "elapsed_s": time.monotonic()-begin,
                        "peak_vram_gb": torch.cuda.max_memory_allocated()/1e9}
            log.write(json.dumps(progress)+"\n")
            print(json.dumps(progress), flush=True)
        if (step+1) % 2000 == 0 or step+1 == args.steps:
            saved = {"step": step+1, "splats": {key: value.detach().cpu() for key, value in splats.items()},
                     "optimizers": {key: value.state_dict() for key, value in optimizers.items()},
                     "strategy_state": state, "numpy_rng_json": json.dumps(rng.bit_generator.state),
                     "torch_rng": torch.get_rng_state(), "cuda_rng": torch.cuda.get_rng_state(), "config": config,
                     "remaining_epoch_order": order}
            torch.save(saved, output / "checkpoint.pt")
            evaluate(splats, records["eval"], output, step+1, full=step+1 == args.steps)
    log.close()
    export_ply(splats, output / "room01_gaussians.ply")
    print(f"COMPLETE: {output / 'room01_gaussians.ply'}", flush=True)


if __name__ == "__main__":
    main()
