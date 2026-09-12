# 相机参数接入与实机采集链路核对

2026-09-10：已将用户提供的 `rgb.right` 内参接入头部仿真相机，保留全部四组原始参数。双腕角度以用户明确指定的 2026-08-24 request_000001 为准，见 [最新方向修正](../room01_wrist_view_correction/README.md)。当前属于**部分对齐**：畸变公式的来源定义、`ctrl` 与腕部设备的对应关系、以及实机安装外参尚未确认。

## 当前采用的参数

| 仿真相机 | 输出宽×高 | fx / fy（像素） | cx / cy（像素） | 状态 |
|---|---|---|---|---|
| head | 1600×1200 | 487.884827 / 487.884827 | 802.718140 / 597.726135 | 使用用户 `rgb.right` 的 K 与分辨率 |
| left_wrist | 640×480 | 163.449530 / 163.019046 | 326.993589 / 244.806327 | 用户指定参考会话的边界拟合；180° 等距模型假设 |
| right_wrist | 640×480 | 158.613921 / 158.728938 | 321.645372 / 228.674742 | 用户指定参考会话的边界拟合；180° 等距模型假设 |

头部畸变暂按 OpenCV fisheye / Kannala–Brandt 四系数模型解释：`[0.015975, 0.004984, -0.003770, 0.000312]`。用户数组末四项为零，原始八项完整保存；**不能因为数组长度为 8 就将其当作 OpenCV 针孔模型的 `[k1,k2,p1,p2,k3,k4,k5,k6]`**。这个解释与已查看的强鱼眼画面相符，但仍须设备 SDK 的模型定义确认。

头部相机由旧的内置相机位置移到机器人头壳上方、前方的外置 E6 临时位置，以消除头壳遮挡。双腕已按指定参考会话纠正此前的方向：相对上一版增加 180° 光轴滚转，并增加约 17.5° 向下俯仰估计。这些均为**图像辅助估计**，没有把用户提供的未知坐标系外参直接当成机器人 link 外参。

三路仍在同一物理时刻以 30 Hz 仿真时间输出 RGB。头部保留 1600×1200，腕部各保留 640×480；展示视频会等比例缩放，录制 PNG 不做展示用缩放。

## 已核对的真实来源

只读访问了数据服务器 `training-kai`（192.168.110.11）、下位机 `x2-lower` 和推理机 `x2-upper`。未修改远程采集程序、机器人参数或数据。

- 当前 VLA 链路选择 E6 **物理右眼**。证据是下位机 `umi_live_contract.py` 的 `cam_high_eye=right`、`sol_data_collection.py` 的 E6 右目定义。
- 实际左腕对应 `/camera3/usb_cam3/image_raw/image_compressed`，实际右腕对应 `/camera1/usb_cam1/image_raw/image_compressed`。以 `robot_observation.py` 的物理映射为准；设备别名和旧 SDK 的 Left/Right 与安装侧相反。
- VLA 链路对腕部只做 BGR→RGB，**不旋转**。独立截图工具另有 180° 旋转约定，不能混入 VLA 相机预处理。
- 原 UMI 目录有 301 个条目；本次详细抽查首、中、末三段，不代表完成全部数据的审核。视频位于每段的 `camera/` 和 `extensions/customer_camera/`；`audio/` 中是音频与其时间信息。
- UMI 示例中的两路腕部来自独立 USB 相机，原始 1920×1080、60 Hz；现用机器人腕部流为 640×480、30 Hz。当前没有足够证据说明 `ctrl.left/right` 就是这两只腕部镜头的标定，因此这两组值完整保存但没有覆盖默认腕部内参。

相应代码保存在 [source_evidence](source_evidence/)，是只读审查的文本副本。照片与视频解码帧保存在 [real_reference](real_reference/)，保留各自原来的尺寸和方向。

## 旧 UMI 头部视频的尺寸冲突

以下三段的 H.265 码流都实际解码成 **3840×1200**，而各自 `meta/e6_stream_session.json` 声明 **3200×1200**：

- `20260908_170549_358037100_86862`
- `20260908_182910_773094776_96413`
- `20260908_200428_904117794_132696`

按曝光时间戳测得约 60 Hz；裸码流工具回报的 25 fps 不是可靠采集频率。详见 [raw_stream_audit.json](raw_stream_audit.json)。

已检查的旧转换代码使用固定 `crop=1600:1200:1600:0`。对这些 3840 宽的左右并排画面，这个切法不能代表完整的右半幅。不能直接把用户的 1600×1200 标定套到这些旧原始帧上，也不能把元数据当作解码尺寸。该问题已记录；本任务未重新裁剪或重建服务器上的训练集。

此外，抽查的 UMI 采集记录与下位机已存 E6 截图来自不同设备序列号。用户提供的标定还缺设备身份信息，因此尚未验证其与当前机器人 E6 的序列号匹配。

## 文件与检查

- [完整原始参数](../../assets/room01/sim/calibration/user_camera_parameters_20260910.json)
- [当前运行参数](../../assets/room01/sim/task_config.json)
- [接入状态](calibration_update.json)
- [OpenCV 数值投影与 USD 参数检查](projection_reference_verification.json)
- [当前仿真相机预览](../room01_sim/final_preview/three_views.jpg)
- [原始姿态遮挡诊断图](sim_preview/head.png)
- [VLA 三路同步、尺寸与投影检查](../room01_sim/vla_io_verification.json)

检查用于确认参数写入和仿真执行正确；检查通过不等于实机标定通过。

修改 `task_config.json` 后，执行 `.venv-usd/bin/python scripts/update_room01_cameras.py` 可只更新相机；再使用 Isaac 环境运行 `scripts/verify_room01_camera_optics.py`、`scripts/verify_room01_vla_io.py --headless` 和截图检查。数值验证每路与 OpenCV 比较 1000 个点，并重新打开 USD 检查内参、畸变、尺寸、镜头模型及来源标记。

参数接入采用 [NVIDIA 原生 OpenCV 镜头模型](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html)；四系数鱼眼模型与针孔模型使用不同的参数语义。当前保留关闭景深、DLAA 与同步采集的设置。

## 尚需确认

`ctrl.left/right` 的物理设备归属；`radialDistortion` 的公式和系数顺序；外参的参考坐标系、变换方向和四元数顺序；E6 标定对应的设备与图像模式。这些信息确认后才能完成真实三路相机标定对齐。
