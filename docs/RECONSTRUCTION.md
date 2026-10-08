# 房间扫描与 Gaussian 重建记录

本页保留原 README 中的重建结果和本地重跑记录。当前默认物理工位请见[仓库首页](../README.md)。下文的 `data/`、checkpoint、原始 ZIP 和部分历史备份仅存在于原机器，未随仓库上传；从 GitHub 克隆不能直接执行依赖这些输入的训练命令。

## 在 Isaac Sim 中查看

重新打开 `assets/room01/room01_scene.usda`。场景保存了原生分辨率 DLAA 配置和默认相机 `ReviewRoom`，也可以在视口相机菜单选择 `ReviewTable`、`ReviewChair` 或 `ReviewFloor`。这些是根据实际扫描相机校正朝向并裁剪后的预览机位。原来的 `ScanCamera` 和完整扫描轨迹仍保留。

已确认重新打开文件后，Isaac 自动加载该渲染配置并选择 `ReviewRoom`。自由移到扫描覆盖较少的视角仍可能看到模糊、缺失或变形。玻璃等区域仍有明显重建误差。

详细检查与修复前后对比见 [Isaac Sim 模糊问题检查](../reports/room01_blur_diagnosis/README.md)。

## 当前主要文件

| 文件/目录 | 内容 |
|---|---|
| `data/room01/transforms.json` | Nerfstudio 格式的 488 个校正视角，439 个训练视角、49 个诊断评估视角 |
| `data/room01/coordinate_transforms.json` | 原始扫描、GLB、PLY 到统一米制 Z-up 世界的完整变换 |
| `outputs/room01/gsplat_fullres_v3/room01_gaussians.ply` | 当前选用的 3,001,853 个 Gaussian，以 992×736 原分辨率细化，含三阶 SH |
| `assets/room01/room01_scene.usda` | 可打开的 USD 视觉场景入口，引用同目录 `gaussians.usdc`，附扫描相机 |
| `assets/room01/scan_mesh.usdc` | 米制 Z-up 扫描网格，含 3 份纹理材质；用于几何审查 |
| `assets/room01/scan_original.glb` | 原始 GLB，保留标准 glTF 坐标习惯，适合导入 Blender |
| `assets/room01/model_manifest.json` | 当前模型、训练分辨率、评估结果与 USD 哈希 |
| `assets/room01/room01_scene_v2.usda` | 旧 v2 模型的可打开备份，引用 `gaussians_v2.usdc` |
| `assets/robots/quanta_x2/quanta_x2_dual_revo2_bridge.urdf` | 从指定目录复制的量子 2 URDF，103 个 link、102 个 joint |
| `reports/room01_reconstruction/robot_asset_checks.json` | 机器人引用资源与 LFS 实体文件的最新完整性状态 |
| `reports/room01_reconstruction/` | 数据、CUDA、USD 检查和机器人资源记录 |

原始 5 个 ZIP 和尺寸截图仍在项目根目录。

## 重建结果

下表是 **gsplat 训练器的保留视角评估，不是 Isaac Sim 截图指标**。

| 版本 | 累计训练步数 | Gaussian 数量 | 评估尺寸 | 49 视角平均 PSNR | 平均 SSIM |
|---|---:|---:|---|---:|---:|
| `gsplat_baseline_v1` | 12,000 | 633,155 | 496×368 | 21.32 dB | 0.7541 |
| `gsplat_refined_v2` | 20,000 | 1,839,569 | 496×368 | 21.87 dB | 0.7679 |
| v2 原分辨率重评估 | 20,000 | 1,839,569 | 992×736 | 21.01 dB | 0.7288 |
| **`gsplat_fullres_v3`** | **28,000** | **3,001,853** | **992×736** | **21.63 dB** | **0.7522** |

![照片与 gsplat v3 渲染对照](../outputs/room01/gsplat_fullres_v3/comparison_028000.jpg)

每组左边为扫描照片，右边为未用于 RGB 训练的 gsplat 视角渲染。展示面板旋转了图像以方便观看；训练和标定没有单独旋转图像。只比较相同评估尺寸：v3 相对 v2 的原分辨率重评估提高了约 0.62 dB / 0.0235 SSIM，属于有限改善。

下面才是当前 USD 在 **Isaac Sim 6.0.1** 中的实际输出：

![Isaac Sim v3 桌面预览](../reports/room01_blur_diagnosis/fixed_scene_v3/ReviewTable.png)

这些是同一扫描内的诊断指标：49 帧未参与 RGB 优化，但 Polycam 的位姿和初始化几何来自整次扫描，不能作为独立实景泛化测试。玻璃、椅面及部分近距离视角仍有模糊与重影。全部逐帧数据保留在各版本 `eval_*.json` 中。

## 已验证与未验证

- 原始 ZIP 完整性、488 帧关联、图像解码、图像金字塔和裁边后的内参一致性检查通过。
- Gaussian CUDA 前向和反向检查通过，训练使用 RTX 5070 Ti Laptop。
- Gaussian 的 PLY 与 USD 采用米为单位，世界坐标为右手系 Z-up。
- USD 已通过 OpenUSD 26.8 原生 `ParticleField3DGaussianSplat` 类型检查，并重新打开检查了数量、单位、轴向及相机投影。
- 原始/校正深度及置信度已保留；本版以 RGB 损失、米制相机位姿及扫描点云初始化训练，没有使用未经配准验证的深度损失。
- `room01_scene.usda` 已通过 Isaac Sim 6.0.1 的实际渲染检查，验证了原生分辨率配置和预览相机绑定。它仍是视觉场景，未加入机器人或 Collision API，尚未进行物理验收。
- 量子 2 的 URDF 和依赖资源已复制并补齐，122 个独立网格引用均能解析为实际 STL，全部 LFS 实体通过原始 SHA-256 校验；`robot_asset_checks.json` 的 `ready_to_load_urdf` 为 `true`。原 URDF 字节保持一致。
- 同一机器人已有的 USD 转换件已合并保存为 `assets/robots/quanta_x2/quanta_x2_robot.usdc`。它保留原转换件的固定底座设置；未据此宣称轮式移动或接触仿真已经通过。

## 本地重跑

训练环境为 `.venv`（Python 3.11），最小依赖见 `requirements-gsplat.txt`，实际安装版本见 `requirements-gsplat-lock.txt`。使用 gsplat 官方 rasterizer 和 `DefaultStrategy`，本项目提供针对 Polycam 数据的精简训练入口。完整 Nerfstudio 是可选路径，其脚本和依赖单独标为 `nerfstudio_optional` / `requirements-nerfstudio-optional.txt`，没有混称为本次实际运行后端。

```bash
.venv/bin/python scripts/check_prepared_scene.py

# 首版；已有输出时请选择新的 run-name，避免覆盖已有 checkpoint
.venv/bin/python scripts/train_gsplat_room.py --steps 12000 --run-name baseline_new

# 本次第二轮细化所用参数
.venv/bin/python scripts/train_gsplat_room.py \
  --steps 20000 --run-name refined_new \
  --resume outputs/room01/gsplat_baseline_v1/checkpoint.pt \
  --max-gaussians 1800000 --refine-start 12500 \
  --refine-stop 18000 --reset-every 50000

# 本次原分辨率细化所用参数
.venv/bin/python scripts/train_gsplat_room.py \
  --steps 28000 --run-name fullres_new \
  --resume outputs/room01/gsplat_refined_v2/checkpoint.pt \
  --factor 1 --max-gaussians 3000000 --refine-start 20500 \
  --refine-stop 25000 --reset-every 50000
```

USD 导出环境为 `.venv-usd`（Python 3.12、NumPy 2.3.1、NVIDIA USD Exchange 3.0.0 / OpenUSD 26.8、`usd-convert-gsplat` 0.1.15）：

```bash
.venv-usd/bin/python -m usd_convert_gsplat \
  -i outputs/room01/gsplat_fullres_v3/room01_gaussians.ply \
  -o assets/room01/gaussians.usdc -n RoomGaussian --up-axis Z
.venv-usd/bin/python scripts/export_scene.py
```

`ScanCamera` 对应扫描第 5 个视角；`CaptureReplay` 保存 488 帧的位姿与逐帧内参，时间轴为相对扫描时间、30 time codes/s。两份 TUM 文件分别明确使用 OpenGL 和 OpenCV 相机轴，TUM 自身不保存内参，调用其他渲染器时必须核对其相机约定。

固定桌面版本现已单独建立在 `assets/room01/sim/`。后续实机对齐应补充相机内外参、底座到桌面的实测变换和物体/执行器参数；原 Gaussian 视觉场景不承担这些物理设置。
