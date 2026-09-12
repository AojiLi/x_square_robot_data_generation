# 双腕圆形鱼眼画面调整

> 安装朝向已进一步按用户指定的 2026-08-24 request_000001 修正，当前结果请看 [指定参考方向对照](../room01_wrist_view_correction/README.md)。本文保留此前的全量边界分析；多数样本的方向不再作为机位基准。

依据用户指定的推理记录，双腕应呈现圆形鱼眼成像区域。随后已完成整个目录 52 个会话、741 组三路图片（2223 张）的检查，详见 [全量审查](ARCHIVE_REVIEW.md) 和 [全部图片索引](CONTACT_INDEX.md)。之前的仿真使用 OpenCV 鱼眼投影，但估计焦距过长，640×480 输出范围内看不到完整的成像圆。现在已从符合现用左右映射的 720 组照片中拟合边界，Isaac Sim 原生输出圆形视野与黑边。

![真实记录与仿真对照](wrist_current_reference_comparison.jpg)

对照图按当前物理左右映射排列，用于检查成像边界。真实记录与仿真的机器人关节姿态不同，不能据此声称安装外参或整个画面已完成标定。

## 参考来源

服务器 `192.168.110.11`（本机可用 SSH 名称 `training-kai`），用户所指路径：

```text
/mnt/data/dzq/inference_records/pi05_inference_records/
20260824T091633.664930Z-pid2632484-1c6fc5df/requests/request_000001/
```

已只读复制 `cam_high.png`、`cam_left_wrist.png`、`cam_right_wrist.png` 和 `request.json`，另读取同一会话的两个相邻请求用于稳健估计边界。三路参考图均为 640×480 RGB；腕部图片解码后的字节哈希与请求记录的 `raw_sha256` 一致。

这份推理记录的头部已经从 E6 右眼 1600×1200 缩放至 640×480。它与上一轮抽查的 2026-09-08 原始 UMI 码流是不同的数据来源，不能将上一轮发现的码流尺寸问题直接归到这份 PNG 记录上。

## 参数变化

| 相机 | 旧估计 fx / fy | 新估计 fx / fy | 新估计 cx / cy | 参考成像圆半径 rx / ry |
|---|---|---|---|---|
| left_wrist | 275 / 366.667 | 165.069 / 164.916 | 313.996 / 237.414 | 259.289 / 259.049 像素 |
| right_wrist | 275 / 366.667 | 161.432 / 160.512 | 314.418 / 252.322 | 253.577 / 252.132 像素 |

圆心与半径由符合现用映射的 720 组真实图像的边界拟合，排除了 21 组旧话题映射记录。**180° 视场角和等距模型是明确的初始假设**，据此使用 `fx=rx/(π/2)`、`fy=ry/(π/2)`，四个 OpenCV 鱼眼畸变系数暂为零。单凭这些工作场景照片无法唯一恢复完整光学标定；这些数值不是厂商内参。

两路继续使用原生 `OmniLensDistortionOpenCvFisheyeAPI`，没有对采集 RGB 追加图像扭曲或黑色遮罩。与拟合边界比较，实际渲染的成像区域 IoU 均约 0.993，边界外区域为黑色。该指标只检查成像边界，不是完整实机画面相似度或标定精度。

头部参数、机器人姿态和三路安装变换在本次调整中保持原值。相比参考图仍会有视角和姿态差异；后续需用对应设备的内参与手眼标定替换估计。

## 左右命名

最初那份历史 `request.json` 记载 `cam_left_wrist` 来自 camera1、`cam_right_wrist` 来自 camera3。全量检查发现 21 组使用此旧映射，其余 720 组为左腕 camera3、右腕 camera1，与现用机器人物理映射一致。因此当前镜头拟合使用这 720 组记录，旧组保留作审查。没有据旧文件名改动真实机器人的左右手或采集路由，机器人左右控制接口仍保持其物理侧含义。

## 检查与文件

- [当前边界拟合与假设](current_lens_boundary_fit.json)
- [左腕当前拟合诊断](cam_left_wrist_current_boundary_fit.png) / [右腕当前拟合诊断](cam_right_wrist_current_boundary_fit.png)
- [Isaac 原生输出的边界检查](render_boundary_verification.json)
- [更新后的三路仿真预览](preview_current/three_views.jpg)
- [VLA 图像、同步和三路目标投影检查](../room01_sim/vla_io_verification.json)
- [当前运行参数](../../assets/room01/sim/task_config.json)
- [当前交付与示例数据](../room01_sim/delivery_manifest.json)

参数处理遵循 [NVIDIA OpenCV 鱼眼镜头说明](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/cameras.html#omnilensdistortionopencvfisheyeapi)。用户之前提供的 `ctrl` 原始参数仍完整保留；其设备归属、畸变公式和外参定义没有因本次图像拟合而被视为已经确认。
