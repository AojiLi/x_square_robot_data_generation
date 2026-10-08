# Quanta X2 电池抓取与插孔仿真

基于 **Isaac Sim / Isaac Lab** 的室内机器人操作场景，包含 Quanta X2 双臂机器人、强脑二代 **Revo2 双手**、白桌及三路 RGB 相机。当前默认任务是从黄海绵上抓取电池，调整姿态后插入固定红海绵的孔中。

仓库提供当前场景资产、控制与采集代码、建模参数和检查记录，供同事复用环境和继续开发控制策略。**抓取并插入尚未验证成功；当前版本还不是可直接训练的完整电池 RL 环境。**

![当前电池抓取与插孔工位](reports/room01_battery_task/ReviewSetup.png)

*实际 Isaac Sim 场景截图（2026-09-16 保存）。机器人和白桌已搬到电脑旁边，左手旁的白色条码盒已移除。*

## 快速开始

### 1. 克隆完整资产

先安装 Git 和 Git LFS，再运行：

```bash
git lfs install
git clone https://github.com/AojiLi/x_square_robot_data_generation.git
cd x_square_robot_data_generation
git lfs pull
```

二进制 USD、机器人网格、Blender 文件和图片/视频使用 **Git LFS**。请保留整个仓库目录结构：电池 USD 层会引用原工位、房间和机器人资产。仅下载单个 USD 文件无法加载完整场景。

### 2. 配置运行环境

本项目使用已安装好的 Isaac Sim / Isaac Lab 环境，仓库不包含这些软件、CUDA 或驱动。

| 项目 | 原机器已验证的版本或配置 |
|---|---|
| 操作系统 | Linux |
| Python | 3.12（仿真运行时） |
| Isaac Sim | 6.0.1.0 |
| Isaac Lab | `v3.0.0-beta2.patch1` |
| 物理步进 | 960 Hz |
| 控制与 RGB 采集 | 30 Hz |
| GPU | NVIDIA GPU；三相机启动检查至少 7000 MiB 空闲显存 |

在安装好 `isaacsim`、`isaaclab`、NumPy、SciPy 和 Pillow 的 Python 环境中，设置本机路径：

```bash
export ROOM01_ISAAC_VENV=/absolute/path/to/python-venv
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab
bash scripts/open_room01_sim.sh
```

`ISAACLAB_ROOT` 必须指向包含 `isaaclab.sh`、`apps/` 和 `source/` 的 Isaac Lab 源码目录。上述路径是占位符，需要替换。沿用原配套 runtime 项目的配置方式和完整说明见[同事运行指南](docs/COLLEAGUE_SETUP.md)。其他版本组合尚未验证。

### 3. 查看或采集场景

以下命令在仓库根目录运行：

```bash
# 打开 GUI，机器人保持复位姿态
bash scripts/open_room01_sim.sh

# 使用桌面俯视相机
bash scripts/open_room01_sim.sh --view /World/ReviewBatteryTop

# 离屏采集头部、左腕和右腕 RGB
bash scripts/open_room01_sim.sh --headless --capture --output outputs/battery_preview
```

采集结果写入指定输出目录。GUI 和离屏检查应依次运行。也可在 Isaac Sim 的 File > Open 中打开 `assets/room01/battery_task/room01_battery_task.usda` 查看模型；控制和相机采集使用上述脚本。

## 当前场景

白桌位于电脑桌旁，两块黄海绵分别承托一节横放的蓝色圆柱电池，固定红海绵位于两者之间。

| 物体 | 当前尺寸 | 建模依据 |
|---|---|---|
| 白桌 | 长 110 cm × 深 57 cm，桌面高 74 cm | 实测 |
| 黄海绵 × 2 | 宽 8 cm × 长 9.8 cm × 高 5.2 cm | 实测 |
| 电池 × 2 | 直径 1.25 cm，长 4.9 cm | 实测并确认圆柱直径 |
| 固定红海绵 | 长 7.2 cm × 宽 4.8 cm × 高 4.8 cm | 实测 |
| 红海绵盲孔 × 3 | 直径 1.25 cm，深 1.3 cm | 实测；中心距约 2 cm 为照片估计 |
| 桌面条形贯穿孔 | 总长 23 cm，直段 15.3 cm，靠墙后边距 9.2 cm | 实测；宽 7.7 cm 按半圆端头推导 |

电池为独立动态刚体，红海绵固定，黄海绵和红海绵目前均按静态刚体近似。三盲孔具有真实孔壁和孔底，桌面开孔贯穿顶板。物体摆放、搬移后的绝对坐标、质量及摩擦等参数仍有估计项。

[场景尺寸与坐标](assets/room01/battery_task/README.md) · [可修改建模参数](assets/room01/battery_task/scene_spec.json) · [实景与仿真对照](reports/room01_battery_task/source_vs_sim.jpg)

## 相机与控制接口

| 相机 | RGB 输出 |
|---|---|
| 头部（E6 右眼） | 1600 × 1200 |
| 左腕 | 640 × 480 |
| 右腕 | 640 × 480 |

![电池工位三路 RGB](reports/room01_battery_task/preview/three_views.jpg)

运行时保留 26 维关节目标控制、观测和物理推进接口。相机内外参的已知项与估计项见[相机检查](reports/room01_camera_calibration/README.md)及[双腕视角说明](reports/room01_wrist_view_correction/README.md)。

电池任务的插入奖励尚未实现：在当前配置下调用 `RoomTaskSim.step()` 会明确报 `NotImplementedError`。后续控制可接入关节目标、观测和物理推进接口；训练 RL 前仍需实现任务奖励、终止条件及接触验证。

## 验证范围

已有记录验证了用户尺寸、网格开孔、实际 PhysX 孔洞、电池自由落稳，以及三路 RGB 非空且同一物理时刻输出。详见[场景检查报告](reports/room01_battery_task/README.md)。2026-10-08 发布前，21 项 CPU 检查通过，USD 文件依赖及 Git LFS 完整性检查通过；当次额外 GPU 预览初始化未完成，不计为新的渲染通过记录。

**目前未验证成功的部分：**

- 机器人通过真实接触抓起电池，并完成插入与松手后的稳定保持。
- 海绵形变、孔壁顺应性、实际摩擦和插入力。
- 搬移后机器人、桌面和相机与实机的精确标定。
- 电池任务的 RL / VLA 策略训练与实机部署。

电池与孔名义直径相同，没有人为扩大孔径。真实任务依赖海绵形变，刚性模型不能据此证明插入可行。换机器后请重新执行检查，历史报告只代表原机器的运行结果。

```bash
source scripts/isaac_env.sh
python scripts/validate_room01_battery_task.py --headless --output outputs/battery_validation
```

## 修改模型

尺寸和布局集中在 `assets/room01/battery_task/scene_spec.json`。修改前先关闭显示该场景的窗口，再使用包含 NumPy、SciPy 和 OpenUSD（`pxr`）的 Python 环境重建：

```bash
python scripts/build_room01_battery_task.py
python -m unittest discover -s tests -v
```

原机器的独立 USD 环境使用 Python 3.12 / OpenUSD 26.8。重建后重新运行物理与相机检查。旧 Blender 审查副本属于原方块工位，不包含最新电池场景更新。

## 仓库结构

| 路径 | 内容 |
|---|---|
| `assets/room01/battery_task/` | 当前电池 USD 场景、尺寸、任务配置与建模来源 |
| `assets/robots/quanta_x2/` | Quanta X2 / Revo2 URDF、网格与 USD |
| `assets/room01/sim/` | 原方块工位、物理机器人及相机配置 |
| `room01_sim/` | 仿真运行时、运动学、相机和资产构建模块 |
| `scripts/` | 启动、构建、检查、采集与回放入口 |
| `tests/` | CPU 几何和数据处理检查 |
| `reports/room01_battery_task/` | 当前场景的检查记录与预览 |
| `docs/` | 换机器运行、资产范围及历史重建说明 |

原始扫描 ZIP、照片批量原件、下载的 UMI 数据、完整 episodes、训练 checkpoint、虚拟环境和工具缓存不随仓库上传。文档中的历史本地路径不表示数据已收录到 Git。

## 其他保留功能

- **原方块工位**：`bash scripts/open_room01_sim.sh --legacy`；`--legacy` 必须是第一个参数。[场景说明](assets/room01/sim/README.md) · [原抓取演示](reports/room01_sim/three_camera_grasp.mp4)。
- **UMI 双臂回放与 LeRobot 导出**：需要额外演示数据；双块入盒尚未通过物理验证。[回放说明](assets/room01/umi_replay/README.md) · [自动初始化与筛选](assets/room01/umi_auto/README.md)。
- **扫描与 Gaussian 视觉重建**：属于独立视觉场景，默认操作工位使用物理场景资产。[重建结果与重跑记录](docs/RECONSTRUCTION.md)。

机器人资产来源及完整性见[机器人说明](assets/robots/quanta_x2/README.md)。第三方软件和资产遵循各自许可，本仓库未为它们重新指定许可。完整收录范围见[仓库说明](docs/REPOSITORY.md)。
