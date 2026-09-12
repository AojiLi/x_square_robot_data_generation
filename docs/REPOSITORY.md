# 仓库内容与本地依赖

这是房间重建、固定白桌操作仿真及三路相机数据采集项目的版本化快照。

## 收录范围

- `room01_sim/`、`scripts/`：场景构建、控制、相机、录制与检查代码。
- `assets/`：当前 Gaussian 场景、房间模型、机器人 URDF/网格、固定工位 USD、Blender 审查副本及标定配置。大型二进制通过 Git LFS 保存。
- `reports/`：项目说明、结构化检查记录，以及文档引用的精选截图和当前抓取视频。
- `requirements-*.txt`：已有的重建环境依赖约束。

历史 `gaussians_v2.usdc`、Blender 自动备份、原始 ZIP、`data/`、完整录制 episode、训练 checkpoint、日志、服务器凭据、环境和工具缓存不进入 Git。历史报告中的本地路径是来源记录，完整原始数据需要另行保管。

## 已验证的环境

- Isaac Sim 6.0.1；Isaac Lab 3.0.0-beta2.patch1 对应的本地运行时。
- 固定工位使用 960 Hz 物理步进，26 维关节目标，30 Hz 三路 RGB。
- 头部输出 1600×1200；左右腕部各 640×480。
- 重建环境为 Python 3.11 的 `.venv`；USD 处理环境为 Python 3.12 的 `.venv-usd`，OpenUSD 26.8。

运行时、CUDA、Blender 和 FFmpeg 二进制没有打包进仓库。`scripts/isaac_env.sh` 通过 `ROOM01_ISAAC_RUNTIME` 选择已有运行环境。`tools/modeling_blender_deb/blender.sh` 是原机器的 Blender 启动适配器；换机器时使用自己的 Blender 安装，并按资产说明调用同一个 Python 构建脚本。视频导出脚本中的本地 FFmpeg 路径同样需要对应的工具安装。

这份快照保留当前工程，不宣称在任意机器上无需配置即可一键运行。

## 检查与数据

当前资产和演示结果见 [交付清单](../reports/room01_sim/delivery_manifest.json)。该清单记录的是生成此快照时的实际文件哈希、运行检查和完整本地 episode；完整 episode 没有上传。

重新采集后，运行项目中的物理、相机接口、episode 和视频检查，再生成新的交付清单。不要把历史报告中的通过结果当作另一台机器或另一组参数已通过验证。

腕部起始视角和实际 `reset()` 已统一。相机光心、完整内外参及部分物理参数仍属于估计，详见 [起始视角说明](../reports/room01_wrist_reset_fix/README.md)。

## 第三方来源

机器人来源与原文件完整性见 [机器人资产说明](../assets/robots/quanta_x2/README.md)。Isaac Sim、Isaac Lab、Blender、gsplat 和 Cosmos 需要分别安装并遵循各自的许可。本仓库没有给第三方软件、机器人资产或模型权重重新指定许可。
