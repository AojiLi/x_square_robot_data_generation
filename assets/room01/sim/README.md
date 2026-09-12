# 固定白桌工位：Quanta X2 + 三路相机

本目录是供 Isaac Sim 运行的桌面操作环境。机器人底座、躯干和头部保持固定工作姿态，动作接口控制双臂与双 Revo2 手。房间地板、墙体、玻璃、桌面和主要家具都有碰撞体；白桌上放置了独立动态方块。

## 打开与验证

在项目根目录运行：

```bash
# 图形窗口：控制器运行并保持工作姿态
bash scripts/open_room01_sim.sh

# 三路相机及工位总览的离屏截图
bash scripts/open_room01_sim.sh --headless --capture

# 物理检查：固定底座、落物接触、手指联动与 URDF 运动学
source scripts/isaac_env.sh
python scripts/verify_room01_physics.py --headless --steps 1920

# VLA 接口检查：三路图像、观测空间、动作、时间对齐和可复现重置
python scripts/verify_room01_vla_io.py --headless

# 真实接触抓取测试，录制 PNG 和动作/状态
python scripts/run_room01_grasp_test.py --headless --record

# 在图形窗口观看抓取，完成后保持窗口打开
python scripts/run_room01_grasp_test.py --keep-open
```

上述命令顺序执行。三路高画质渲染在本机 12 GB 显卡上使用约 6–7 GB 显存；启动器会阻止同项目重复开启相机渲染进程，并检查可用显存。物理步频为 960 Hz，相机与动作频率为 30 Hz；仿真时间不等于实际运行耗时。

运行时复用本机 `x_square_copy/isaac-sim-scenes` 中已安装的 Isaac Sim 6.0.1 与 Isaac Lab 3.0 beta2 代码。发行包的 `isaaclab` 元数据版本为 6.1.14。可通过 `ROOM01_ISAAC_RUNTIME` 指定该运行时目录。场景、机器人资产和本项目控制代码均从当前项目读取。

`room01_manipulation.usda` 可直接打开检查场景；位置控制和数据采集请使用启动脚本，让控制器同时运行。

## 文件

| 文件 | 用途 |
|---|---|
| `room01_manipulation.usda` | 完整工位入口：房间、机器人、方块、光照和五个相机机位 |
| `room_environment.usdc` | 完整房间外观与 383 个环境碰撞体 |
| `quanta_x2_physics.usdc` | 本任务的机器人：37 个物理 link，36 个可动关节，含 10 个被动联动关节 |
| `room01_sim_review.blend` | Blender 审查副本，含机器人外观、相机和可切换的碰撞线框 |
| `task_config.json` | 初始姿态、采样区域、动作顺序及三路相机参数 |
| `calibration/user_camera_parameters_20260910.json` | 用户提供的 rgb/ctrl 四组参数，完整保留原始数值与未确认项 |
| `robot_frames.json` / `camera_manifest.json` | 运行时所需的坐标框与相机路径 |
| `textures/` | 从 Blender 保留或烘焙的外观纹理 |
| `reference/` | 用户提供的两张实机摆放照片 |

Blender 审查副本中，`90 Simulation collision proxies` 默认隐藏，打开该集合可查看碰撞体。这个副本用于外观和几何审查；机器人的关节控制、联动与同步相机以 Isaac Sim 运行结果为准。原始完整房间 `.blend`、原 URDF 和之前的 Gaussian 场景均保留。

## 物理与坐标

- 世界为米制 Z-up。原扫描地面位于 Z=-1.466 m，仿真房间统一平移 +1.466 m，地面变为 Z=0，白色桌面为 Z=0.723 m。
- 台面保留实际网格与走线孔；大面与支撑结构使用简化碰撞体。百叶、小字、细线缆等装饰细节不逐片参与碰撞。
- 机器人保留 URDF 质量与惯量，通过平行轴定理合并固定连接。相机和末端坐标框仍保留为 Xform；相邻铰接零件，以及每侧两轴腕部中嵌套壳体的内部凸包重叠，按关节装配关系排除自碰撞；它们与外物的碰撞及其余自碰撞保留。
- 原 URDF 的 10 处手指联动已补入 PhysX。关节控制使用限力 PD 和模型重力补偿；命令、状态均为弧度。PD、转子惯量和联动柔顺参数是初始仿真参数，尚未由实机日志辨识。
- 默认方块尺寸 4×4×4 cm、质量 60 g。摩擦参数是初始值。初始右手随机采样区经过接近和抬升姿态的 IK 检查，不代表整个桌面都在当前固定姿态的可达范围内。

## VLA 接口与数据

`room01_sim.vla_env.Room01VLAEnv` 提供 Gymnasium 的 `reset`、`step` 和 `render`。创建环境前需通过 `room01_sim.bootstrap.launch` 启动应用，示例调用可参考 `scripts/verify_room01_vla_io.py`。

| 字段 | 内容 |
|---|---|
| `images.head` | 头部 E6 右眼 RGB，1200×1600×3，uint8 |
| `images.left_wrist` / `images.right_wrist` | 左右腕部 RGB，各 480×640×3，uint8，同一物理时刻 |
| `state` | 26 维实际关节位置，float32，弧度 |
| `joint_velocity` | 对应的模拟关节速度，弧度/秒 |
| `language_instruction` | 当前任务描述 |
| `timestamp` | 仿真时间，秒 |
| action | 26 维绝对关节位置目标，按 URDF 位置与速度限制处理 |

动作顺序为左臂 7 + 左手 6 + 右臂 7 + 右手 6，完整名称以 `task_config.json` 的 `action_joint_names` 为准。被动指节通过联动跟随，不占独立动作维度。模拟器的物体真值位姿放在 `info` 或辅助数据中，不混入默认 VLA 输入。

录制结果位于 `outputs/room01/vla_episodes/`：

```text
episode_.../
  episode.json                 # 指令、版本、参数、动作顺序与完成状态
  transitions.npz              # 实际状态、执行动作、下一状态、时间及相机位姿
  images/head/000000.png
  images/left_wrist/000000.png
  images/right_wrist/000000.png
```

每行是 t 时刻的观测、随后一个 1/30 秒区间内执行的动作，以及 t+1/30 的状态。PNG 是保留原分辨率的 RGB 图像。仅使用 `episode.json` 中 `status=complete` 的数据。测试控制器使用模拟真值生成抓取轨迹，属于接口和接触测试示例，不是一套经过筛选的专家示教训练集。当前提供通用数据结构；选定具体 VLA 模型后，需按其动作定义和训练格式做适配。

## 视觉与标定状态

外观保留房间照片纹理、烘焙的主要程序材质、桌面标记与细节，并按提供照片调整了机器人手部和外壳材质。三路相机使用原生 OpenCV 镜头模型、DLAA，关闭帧生成与景深模糊。

2026-09-10 已接入用户 `rgb.right` 的头部内参和 1600×1200 分辨率，四个非零畸变系数暂按 OpenCV 鱼眼模型处理，具体公式仍待设备定义确认。左右腕部输出为 640×480，并根据用户指定的真实推理 PNG 拟合圆形成像区域，采用 180° 等距鱼眼假设。腕部内参、光心和三路安装外参仍属于估计。`ctrl` 两组参数尚未证明属于独立 USB 腕部相机，因此完整保存但没有直接套用。详见 [双腕圆形鱼眼调整](../../../reports/room01_wrist_fisheye_alignment/README.md)。

头部临时机位位于 CAD 头壳外；双腕朝向以用户指定的 2026-08-24 request_000001 照片为准，已相对上一版修正 180° 光轴滚转并调整俯仰。左腕手掌在左、右腕手掌在右，两边拇指朝上。修正在三维相机变换中生效，VLA 输入不额外旋转或镜像图片。默认 `reset()`、保存的 USD 和 Blender 审查副本均使用与指定参考一致的侧立手掌起始姿态。普通启动和第 0 帧采集直接使用该姿态，无需 `--joint-pose`。相机固定在腕部，抓取过程中仍随实际手腕转动，见 [默认起始视角修正](../../../reports/room01_wrist_reset_fix/README.md)。当前仍不能视为完成实机标定。

只读检查旧 UMI 数据时，还发现三段头部码流实际为 3840×1200，与 3200×1200 元数据及旧固定右眼裁剪规则冲突。后续混用这些数据前需处理该尺寸差异，不能简单套用当前标定。详细证据与待确认项见 [相机接入报告](../../../reports/room01_camera_calibration/README.md)。

桌面高度、底座到桌面的变换和物件尺寸也需要实物复核；当前照明不代表完成了实景光照标定。

## 重建资产

```bash
bash tools/modeling_blender_deb/blender.sh --background \
  assets/room01/full_room_model/room01_full.blend --python-exit-code 1 \
  --python room01_sim/export_blender.py
.venv-usd/bin/python room01_sim/build_scene.py
.venv-usd/bin/python scripts/validate_room01_assets.py

# 更新 Blender 审查副本
.venv-usd/bin/python room01_sim/export_review_geometry.py
bash tools/modeling_blender_deb/blender.sh --background \
  assets/room01/full_room_model/room01_full.blend --python-exit-code 1 \
  --python room01_sim/build_review_blend.py
```

构建器保留已有 `task_config.json`；缺失时才从 `room01_sim/default_task_config.json` 创建。修改物理或相机参数后，应重跑相应验收。

只修改相机时可执行 `.venv-usd/bin/python scripts/update_room01_cameras.py`，无需重建房间。加载 Isaac 运行环境后，`python scripts/verify_room01_camera_optics.py` 检查数值投影和保存参数，`python scripts/verify_room01_vla_io.py --headless` 检查实际图像与接口。

检查结果见项目中的 `reports/room01_sim/`。物理配置依据 [Isaac Sim 物理基础](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/physics/simulation_fundamentals.html) 和 [PhysX 关节/联动说明](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/articulations.html)。
