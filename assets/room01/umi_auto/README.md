# 固定 UMI 轨迹，自动求解初始布局

新的批量入口是 `scripts/run_room01_umi_batch.py`。每条演示先完成一次坐标映射和 IK，随后锁定参考轨迹。搜索变量是方块、盒子的初始位置和朝向；不通过改轨迹来适配候选场景。

实现与实测结果见 [自动初始化报告](../../../reports/room01_umi_auto/README.md)。当前没有获得通过全部验收的双块入盒数据。

## 流程

1. 校验源文件和 UMI 状态/动作合同。按源对齐表恢复两手位姿，保留 H50 共同锚点动作语义。
2. 根据四指屈曲信号自动识别抓取与释放区间。当前任务适配器要求左右手各一次抓放；其他结构给出明确拒收原因。
3. 首帧仅用于识别盒外物体的颜色和大小类别，不重建其米制坐标。识别模糊时拒收；单条准备命令可用 `--objects 左侧ID,右侧ID` 提供物体身份，不能用它设置位置。
4. 空载执行固定轨迹，测量实际手部运动和手指角。使用 URDF 碰撞网格、近端/远端接触面、相对朝向及闭合/接近过程，生成多种抓取候选。
5. 物理抓取探测中，将盒子及非目标块暂存在远离当前操作区的位置；待抓方块只在 reset 时摆放。抬升超过 15 mm、在手部附近累计保持至少 0.2 秒只作为抓取初筛，不等同稳定成功。
6. 从探测中实际获得的搬运/释放轨迹，联合求一个盒子初始姿态。检查两次落点的共同容纳区域，以及释放前方块扫过盒壁的情况。释放后的接触交给最终物理回放判断。
7. 完整场景重新回放，通过“双块完全入盒、手离开、稳定 1 秒、盒子未倾覆”后，再做初始位置 ±2 mm 的扰动测试和录制复验。全部通过才可导出成功数据。

几何候选来自采样和碰撞近似，最终判定来自物理仿真。没有找到候选表示本次搜索未找到，不能当作数学上的无解证明。

## 使用

在项目根目录执行。离线依赖见 `requirements-umi-replay.txt`；Isaac 环境由 `scripts/isaac_env.sh` 提供。

```bash
# 只读下载并核对哈希；已有包按固定输入快照复用。
.venv-usd/bin/python scripts/fetch_room01_umi_episodes.py --episodes 0,20,84

# 小批量准备，不启动物理仿真。
.venv-usd/bin/python scripts/run_room01_umi_batch.py \
  --episodes 0,20,84 --prepare-only

# 完整批量搜索。GPU 任务顺序执行，支持中断后继续。
.venv-usd/bin/python scripts/run_room01_umi_batch.py \
  --episodes 0,20,84 --grasp-trials 4 --layout-trials 4 \
  --robustness-trials 2 --wall-seconds 1200 --record-wall-seconds 600 --resume
```

下载 `--episodes all` 会读取服务器的 episode 总数；批量入口的 `--episodes all` 处理全部已缓存的包。没有在本次验证中执行全部 85 条。

`--time-scale` 是每条搜索开始前统一确定的时间缩放，默认 2，所有候选完全相同。没有局部末端修正或手指角搜索。URDF 小数限位与设备整度限位之间最多 0.25° 的舍入会记录在准备信息中，更大的越界直接拒收。

每条至多进行一次空载测量、`grasp-trials` 次抓取候选、`layout-trials` 次完整布局尝试，以及每个成功完整候选的 `robustness-trials` 次扰动复验。搜索总时间由 `wall-seconds` 限制；最终录制另有 `record-wall-seconds` 上限。超时和失败都会写入批量记录，不无限试错。

## 输出与复现

默认输出 `outputs/room01/umi_auto_batch/`：

- `batch.json`：每条输入的状态、拒收原因、轨迹哈希、复验结果和成功数据路径。
- `episode_XXXXXX/preparation.json`、`trajectory.npz`：自动阶段、资产识别、共享标定、锁定的参考轨迹。
- `search_*/grasp_candidates.json`、`box_candidates.json`：程序生成的候选和几何筛选结果。
- `search_*/trial_*/replay.json`：该次初始姿态、控制设置和物理结果；原始状态/命令保存在同目录 NPZ 中。
- `accepted_layout.json`：仅通过完整任务及扰动复验后写入，包含轨迹和物理配置指纹。
- 成功录制会自动导出独立的 LeRobot episode 数据集。抓取探测、失败、超时和复放失败都不会自动进入成功数据集。

每次尝试均重新计算参考轨迹哈希；验证脚本还独立核对每个控制周期的参考关节命令。物体位姿只在初始化时写入，随后由物理接触演化。

相同输入、机器人/场景/控制配置与初始姿态的已完成物理结果可以复用。出现失败复验后，该布局不会继续作为成功种子。`--fresh-reference` 可用于单条搜索强制重测空载轨迹。

```bash
.venv-usd/bin/python scripts/validate_room01_umi_auto.py \
  /path/to/episode_preparation /path/to/search --report /path/to/validation.json

.venv-usd/bin/python scripts/export_room01_umi_review.py \
  /path/to/recorded_trial --source /path/to/source_episode --output /path/to/review
```

`scene.usda` 是可复用的刚体池，其启动默认摆放不是搜索结果。每次真正的初始布局以 `trial_*/replay.json` 的 `initial_object_poses` 为准。旧版手工初始化入口保留用于历史对照；新管线仅复用其中的 USD 建模函数，不调用其手工抓取时刻或固定手内偏移。

`calibration.json` 是按硬件配置共享的现有估计变换。当前光学外参、工具物理原点和摩擦参数仍有估计成分；生成结果属于仿真数据，尚未验证真机训练效果。
