# 腕部图像翻转：实机对照与源码排查

## 开机后的实机复核

`.155` 恢复连接后，已对运行中的腕部相机做只读对照：**两路各 8 帧，SDK `GetRawImage` 返回的 JPEG 与相同时间戳的 ROS 压缩图像逐字节一致**。本次实际读取链路中，SDK 没有额外旋转、镜像、重新编码或改变图像内容。

| SDK 接口 | 实测对应 ROS 话题 | 同戳 JPEG 完全一致 |
|---|---|---|
| `LeftArmCameraStub` | camera1 | 8 / 8 |
| `RightArmCameraStub` | camera3 | 8 / 8 |

按现有物理映射配置，实际左腕使用 camera3 / SDK Right，实际右腕使用 camera1 / SDK Left。SDK 类名与逻辑左右手不是同一层命名。

**当前截图程序的 180°旋转在 `/home/xr/camera_capture/capture_frames.py` 第 108 行**，发生在 SDK 返回、PIL 解码之后：

```python
bgr = np.rot90(rgb, 2)[:, :, ::-1].copy()
```

其中 `np.rot90(rgb, 2)` 是旋转 180°，最后一维反序只是 RGB→BGR。现用推理客户端的 `camera_transport.py` 不做此旋转，并且其源码与此前读取的 `.199` 当前服务端对应文件完全一致。

取本次 SDK 原始 JPEG，PIL 与推理解码器输出像素一致；JPEG 没有要求额外旋转的 EXIF 方向。执行从截图脚本提取的上述实际代码，输出严格等于原始图像旋转 180°。因此，**如果比较的是截图脚本输出和推理输入，两者的方向差异来自截图后处理，而不是 SDK 读取错误**。

![同帧 ROS、SDK 与截图处理对照](live_sdk_ros_screenshot_comparison.jpg)

图中前两列是本次读取的同戳原图，第三列是对同一图像执行实际截图脚本旋转操作得到的结果，没有启动截图程序、头部相机应用或机器人控制。

证据：[逐帧字节与哈希对照](live_sdk_ros_comparison.json)、[实机图像方向验证](live_rotation_verification.json)、[开机后读取的截图源码](source_155_online/capture_frames.py.txt)。探针只订阅已有 ROS 相机话题并调用相机读取 RPC，没有打开 UVC 设备、创建发布者、重启服务或发送机器人动作。记录中的 `age_when_pair_matched_ms` 包含探针缓存等待，不能用作 SDK 调用延迟。

## 先前找到的旧服务端旋转代码

在 `.199` 找到了明确的 **180°旋转代码**，但它位于保留的旧部署副本；当前 `/home/dzq/pi05_inference` 的对应函数已取消旋转。可确认的是“旧服务端后处理能够造成翻转”，**不能直接将历史异常归因于 SDK 解码，也不能仅凭旧副本存在就认定当前推理使用了它**。

## 主机与目录

| 主机 | 本次检查结果 |
|---|---|
| 192.168.110.157，`xr-SER` | 主机及已检查的机器人运行容器未发现 `/home/xr/pi05_umi_inference`；运行的主要是 `cx002_master` 遥操作相关容器 |
| 192.168.110.199，`slzl-System-Product-Name` | 找到 `/home/dzq/pi05_inference`、`/home/dzq/umi-deployment/server` 和 `/home/dzq/openpi`；主机名与历史推理记录一致 |
| 192.168.110.155，`xr-ser7-dvt3-8` | 首次检查未连接；用户开机后已在线读取客户端、截图程序、相机发布器、SDK 桩代码，并完成同戳 SDK/ROS 实机比较 |

本次未启动策略服务、摄像应用或机器人控制，也未修改远程程序。

## 已重现的 180°旋转

旧副本 `/home/dzq/umi-deployment/server/camera_transport.py` 第 50 行：

```python
rotated = cv2.rotate(image_bgr, cv2.ROTATE_180)
```

它在 OpenCV 解码之后主动旋转，再转换为 RGB；不是由 `GetRawImage` 或图像解码自动产生的这个旋转操作。源码还明确将协议标为 `umi_ros_compressed_wrist_v1`、变换标为 `decode_bgr_rotate_180_to_rgb`。见 [旧版源码副本](source_199_old/camera_transport.py.txt)。

当前 `/home/dzq/pi05_inference/camera_transport.py` 第 41–59 行仅解码并执行 BGR→RGB，协议是 `umi_ros_compressed_wrist_v2`，变换标为 `decode_bgr_to_rgb_no_rotation`。见 [当前源码副本](source_199/camera_transport.py.txt)。

使用同一张不对称测试 PNG，实际执行两份源码中的图像函数得到：

| 路径 | 结果 |
|---|---|
| 旧版 + v1 压缩图像包 | 像素结果严格等于原图旋转 180° |
| 当前版 + v2 压缩图像包 | 保留原始像素方向 |
| 两版的旧式 raw RGB 数组兼容路径 | 都不旋转 |
| v1 / v2 协议交叉混用 | 报错拒绝，并非静默翻转 |

因此是否触发旋转还取决于**实际加载的源码和输入格式**。离线对照的 6 项检查通过，见 [新旧版对照测试](old_current_comparison_test.json)。这项重现不证明某一历史请求一定经过旧版压缩包路径。

## 当前服务端其余图像流程

- `e6_tcp_cam_h.py` 第 1214–1218 行分别解码两个已有腕部键，没有左右交换。第 1275 行附近记录图像，第 1282 行附近再调用模型推理。
- `umi_policy_common.py` 第 63–64 行将左右腕键分别传入对应的模型图像键。第 30 行的 `transpose` 是 CHW→HWC 数据布局转换，不是镜像或 180°旋转。
- `inference_recorder.py` 的 `_policy_image` 及 PNG 保存只处理布局、数值类型和 RGB/BGR，不另行翻转。`requests/.../cam_*_wrist.png` 是模型适配和 224×224 缩放之前保存的输入。
- OpenPI 的 `ResizeImages` 等比例缩放并填边，不改变方向。`sample_actions` 使用 `train=False`；源码中的训练旋转增强也排除了 wrist 图像。

已用 PNG、JPEG、不对称左右图、CHW 布局和两组真实记录图片验证上述纯图像函数，24 项检查通过。没有加载大模型或连接机器人，见 [当前图像链测试](image_pipeline_test.json)。

## 不能忽略的启动入口差异

**旧目录的 `start_umi_v2_server.sh` 第 82 行仍硬编码执行 `/home/dzq/pi05_inference/serve_umi_v2_policy.py`。** 因此“执行了旧目录中的 shell 脚本”本身并不能证明加载了旧旋转函数；应看最终 Python 入口及 `camera_transport.__file__`。

检查时未发现以所查 `serve_umi` / `serve_policy` Python 入口运行的对应服务进程，无法取得一次正在执行的请求来核对模块加载位置。当前文件、旧副本和历史记录的关系应按证据区分，不能用目录名称替代实际加载路径。

## 左右交换与旋转是两件事

此前全量 741 组记录中，21 组写的是 left=camera1 / right=camera3，后续 720 组是 left=camera3 / right=camera1。开机后在线读取的机器人客户端源码也明确订阅后者：

```text
left  <- /camera3/usb_cam3/image_raw/image_compressed
right <- /camera1/usb_cam1/image_raw/image_compressed
```

这是 ROS 话题到逻辑左右键的对应关系；与对某张图像旋转 180°不同。该客户端图像链直接订阅 ROS 压缩图像，并非独立截图工具的 SDK `GetRawImage` 调用链。

此外，之前取得的 2026-09-09 独立截图记录明确包含 `rotation_degrees=180`，而 VLA 当前合同为 no_rotation。这也说明不能把“SDK 截图工具保存的图片”直接当作未处理的 SDK 原始输出，见 [已有截图变换记录](../room01_camera_calibration/real_reference/verified_rot180_20260909T130346_534233Z_f49d02_metadata.json)。

## 当前结论

1. 已找到并重现旧服务端的显式 180°旋转，不能简单归咎于 SDK 解码。
2. 当前 `.199` 服务端图像处理函数不旋转或交换左右；现存代码中的动作坐标旋转不作用于图像。
3. 开机后的同戳实机比较已证明当前 SDK 与 ROS 图像字节一致；现用推理保留原方向，截图脚本另加 180°旋转。
4. 早期某次历史推理究竟加载了哪份源码、使用了哪种输入格式，仍缺当时的加载路径与原始帧对应证据。当前实机测试不能倒推所有历史记录的唯一成因。

本报告与图像函数测试只做诊断，没有修改原有相机朝向、软件变换或推理配置。
