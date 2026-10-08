# 同事克隆与运行当前电池环境

## 获取完整资产

在 Linux 上安装 Git LFS 后运行：

```bash
git lfs install
git clone https://github.com/AojiLi/x_square_robot_data_generation.git
cd x_square_robot_data_generation
git lfs pull
```

机器人网格、二进制 USD 和截图通过 LFS 提供。下载 ZIP 或仅取得 LFS 指针不能运行完整场景。当前场景是 `assets/room01/battery_task/room01_battery_task.usda`，它引用仓库内原工位和房间资产，因此请保留整个目录结构。

## 选择已安装的运行环境

本机验证版本：Python 3.12、Isaac Sim 6.0.1.0、Isaac Lab `v3.0.0-beta2.patch1`（commit `ffff603eafc6b74264a5261cc0183d6a65390d78`）。需要 NVIDIA GPU 和相应驱动；三路相机启动时会检查至少 7000 MiB 空闲显存。软件运行时未打包到本仓库。

如已安装同版本 Isaac Sim / Isaac Lab，并在 Python 环境中安装好 `isaacsim`、`isaaclab`、NumPy、SciPy 和 Pillow，配置：

```bash
export ROOM01_ISAAC_VENV=/absolute/path/to/python-venv
export ISAACLAB_ROOT=/absolute/path/to/IsaacLab
bash scripts/open_room01_sim.sh
```

`ISAACLAB_ROOT` 需要包含 `isaaclab.sh`、`apps/` 和 `source/`，不能只设置到 Python 包目录。Isaac Sim 的许可需按其安装流程处理。

如果沿用原机器的配套 runtime 项目，可改用已有的适配器：

```bash
export ROOM01_ISAAC_RUNTIME=/absolute/path/to/isaac-sim-scenes
bash scripts/open_room01_sim.sh
```

该项目必须已部署运行时，且包含 `scripts/runtime_env.sh`。两种方式任选其一；设置 `ROOM01_ISAAC_VENV` 时优先使用它。

## 验证与使用

```bash
# GUI：机器人保持复位姿态
bash scripts/open_room01_sim.sh

# 无窗口截取头部、左右腕相机
bash scripts/open_room01_sim.sh --headless --capture --output outputs/colleague_preview

# 物体落稳、PhysX 孔洞及相机检查
source scripts/isaac_env.sh
python scripts/validate_room01_battery_task.py --headless --output outputs/colleague_validation
```

GUI 和离屏验证依次运行。也可以在 Isaac Sim 的 File > Open 中直接打开上述 USD 查看模型；物理控制和三相机采集使用本项目启动脚本。

当前包含搬到电脑旁边的机器人和白桌、双 Revo2 手、两块黄海绵和动态电池、固定三盲孔红海绵；左手边的白色条码盒已移除。尺寸与估计项见[场景说明](../assets/room01/battery_task/README.md)。

本版本验证过场景几何、落稳和相机，但机器人抓取插入尚未成功验证。海绵为刚性近似，插入力和形变未标定；电池插入奖励尚未实现，也未训练 RL。旧方块奖励不能作为电池任务成功判据。仓库内历史报告是原机器证据，换机器后请重新运行检查。

原始扫描、照片批量原件、UMI 下载数据、完整 episodes 和本地运行时未上传；它们不影响当前电池场景预览。UMI 回放代码保留为独立历史功能，需要另行获取演示数据。
