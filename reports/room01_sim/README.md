# 固定白桌工位验收

完整运行说明：[工位与 VLA 接口](../../assets/room01/sim/README.md)。

![Isaac Sim 工位总览](final_preview/ReviewSetup.png)

机器人位于白色桌子前，底座、躯干和头部保持固定工作姿态。环境有 383 个静态碰撞体；机器人经过固定连接合并，保留 37 个物理 link、36 个可动关节，其中 10 个为手指联动关节。双臂和双手共有 26 个独立动作量。

## 三路画面与抓取

双腕视角和默认起始姿态已统一到用户指定的 2026-08-24 request_000001。下面是直接从默认 `reset()` 生成的三路图像，无额外姿态覆盖。见 [起始视角修正](../room01_wrist_reset_fix/README.md)。

![头部、左腕、右腕](final_preview/three_views.jpg)

[三路相机抓取演示（MP4）](three_camera_grasp.mp4)

图像均为 Isaac Sim 实际输出。演示使用限力控制与物体接触，没有将方块绑定到手上。固定初始条件的测试中，方块抬起约 16 cm，并稳定保持 1.87 s。这个结果说明测试流程已跑通，不代表已训练出 VLA 策略，也不是随机场景成功率统计。

## 检查记录

| 检查 | 证据 |
|---|---|
| 米制坐标、资产引用、碰撞和质量结构；碰撞几何不进入 RGB | [asset_verification.json](asset_verification.json) |
| 固定底座、桌面/地面落物、手指联动、URDF 运动学一致性 | [physics_verification.json](physics_verification.json) |
| 三路图像、26 维动作、30 Hz 时间对齐、固定种子重置、投影约定 | [vla_io_verification.json](vla_io_verification.json) |
| 三路各 1000 个点的 OpenCV 数值对照、USD 参数与来源标记 | [镜头参数验证](../room01_camera_calibration/projection_reference_verification.json) |
| 双腕原生圆形鱼眼边界与全量真实图片审查 | [鱼眼边界验证](../room01_wrist_fisheye_alignment/render_boundary_verification.json) / [2223 张图片审查](../room01_wrist_fisheye_alignment/ARCHIVE_REVIEW.md) |
| 抓取轨迹与稳定抬升 | [grasp_final.json](grasp_final.json) |
| 300 个时刻、900 张 PNG 的解码与状态/动作对齐 | [episode_verification.json](episode_verification.json) |
| Blender 审查副本 | [blender_review_verification.json](blender_review_verification.json) |
| 当前交付文件、哈希和示例数据位置 | [delivery_manifest.json](delivery_manifest.json) |

物理步频 960 Hz，动作及三路图像以 30 Hz 仿真时间对齐。2026-09-10 相机更新后的 RGB PNG 保留头部 1600×1200、腕部各 640×480 原始输出；MP4 等比例缩放到共同展示高度，便于同时查看。旧的中间诊断和旧相机配置数据已标为 `superseded` 或 `failed`，最终示例以交付清单指定的 episode 为准。

## 适用边界

当前提供可运行的固定工位、物理控制、VLA 观测接口和同步采集流程。头部已接入用户 `rgb.right` 的内参，但鱼眼公式解释仍待确认；腕部已按真实 PNG 拟合圆形成像区域，采用 180° 等距鱼眼假设，见 [双腕鱼眼对照与检查](../room01_wrist_fisheye_alignment/README.md)。这仍属于图像辅助估计，三路安装外参没有完成实测标定。

`ctrl` 两组完整保存，尚未认定为腕部标定。详见 [相机接入与真实链路核对](../room01_camera_calibration/README.md)，其中也记录了另外一批旧 UMI 头部码流实际 3840×1200、与元数据和固定右眼裁剪规则冲突的问题。

底座到桌面的摆放、光照和物理参数也需要对照实机进一步校准。模型与照片的外观仍有差异，尤其是表面磨损、机器人部分外壳细节和真实光照。数值和运行检查通过不等于完成实机标定。

示例轨迹是程序化接触测试，不能直接当作经过专家筛选的大规模训练数据。接入具体 VLA 模型时，应使用其所需的数据格式及动作映射。
