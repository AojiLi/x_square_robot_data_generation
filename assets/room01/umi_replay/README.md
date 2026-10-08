# UMI 第 0 条演示回放试验

这套独立 USD 层将 UMI v2 的双手末端轨迹和 Revo2 手指角映射到当前 X2 白桌工位。已跑完三轮物理试验，**尚未实现双块成功入盒**。详细结果、视频和数据检查见 [试验报告](../../../reports/room01_umi_replay/README.md)。

## 场景与坐标

- 三个 5 cm 木立方体：红、绿、蓝，各 72 g；一个 4 cm 红木立方体，42 g。质量均匀，按立方体计算惯量。
- 木盒外尺寸 17.4×14.4×10.2 cm，110 g；四壁约 2 mm，底板暂按 2 mm。五个非重叠碰撞薄板组成可移动空心刚体，重心和惯量按薄板体积计算。
- 两个待抓方块在盒外，另两个在盒内。首帧提供物体身份和内外顺序；物体的米制位置仍通过估计的手内抓取中心和释放位置推定，并非已完成标定的首帧三维重建。
- `registration.json` 记录米制刚体配准、两手固定坐标变换和所有估计项。原始绝对位姿到训练目标点的 −42 mm 局部 X 偏移已做数值核对；该目标点对应实物上的何处尚未确认。
- 手指通道将 `thumb_flex/59°` 对应到 URDF `thumb_proximal`，`thumb_aux/90°` 对应到 `thumb_metacarpal`。其余四指保持顺序。这个映射由名称和限位支持，尚未取得硬件映射标定。

## 准备和运行

第 0 条源数据保存在项目的 `data/umi_replay/episode_000000/`，不进入 Git。`source_manifest.json` 列出原服务器路径和 SHA256。原始数据保持不变。

离线环境使用项目已有的 OpenUSD 环境，另需 `requirements-umi-replay.txt` 中的依赖：

```bash
uv pip install --python .venv-usd/bin/python -r requirements-umi-replay.txt
.venv-usd/bin/python scripts/prepare_room01_umi_replay.py

source scripts/isaac_env.sh
python scripts/run_room01_umi_replay.py --headless --record --review
```

去掉 `--headless` 并加 `--keep-open` 可以在 GUI 中查看，录制结束后继续保持窗口和物理仿真。每次运行创建新的时间戳输出目录；命令返回 0 表示任务成功、2 表示物理任务失败、1 表示执行错误。

准备器以原始绝对位姿恢复两手共同坐标，检查 123 行状态及有效未来动作与 Parquet 一致，再进行连续 IK。默认将初始双手中点放在世界 X=0.68 m，使整条轨迹可达。全局摆放和工具变换属于估计标定，局部轨迹修正单独记录。

已测试的限幅修正可重建、运行：

```bash
.venv-usd/bin/python scripts/correct_room01_umi_replay.py \
  assets/room01/umi_replay/bounded_corrections.json

source scripts/isaac_env.sh
python scripts/run_room01_umi_replay.py --headless --record --review \
  --prepared outputs/room01/umi_corrected --time-scale 2 --servo-integral
```

修正器逐点检查位置≤1 cm、姿态≤5°；手指列保持不变。`--servo-integral` 补偿手臂执行偏差，参考末端轨迹和补偿后的关节命令分别保存。仿真物体始终通过接触搬运。

## 导出和检查

```bash
.venv-usd/bin/python scripts/export_room01_umi_dataset.py \
  /absolute/path/to/replay /absolute/path/to/new_dataset

.venv-usd/bin/python scripts/validate_room01_umi_replay.py \
  /absolute/path/to/replay --dataset /absolute/path/to/new_dataset

.venv-usd/bin/python scripts/export_room01_umi_review.py /absolute/path/to/replay
```

失败回放默认拒绝导出，显式使用 `--allow-failure` 才能生成带 `task_success=false`、`training_eligible=false` 的诊断数据。训练使用者仍须检查这些标记；不能将诊断集当作成功示范。

训练数据保留 LeRobot v2.1、10 Hz、30 维双手增量位姿/手指角、50 步共同锚点动作块和尾部 padding mask。标签从仿真实际运动重算。MDP 的关节控制、原生 RGB、目标与实际末端、物体位姿、相机时间戳分别留存。原生 head 为 1600×1200、双腕各 640×480；训练视频统一为 640×480，缩放记录写入数据说明。

源动作、仿真动作、PID 补偿命令和实际运动是不同记录，不将原 UMI 标签直接粘贴到偏移后的仿真画面。当前没有验证真机训练效果。
