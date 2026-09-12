# Isaac Sim 模糊问题检查

用户报告 `assets/room01/room01_scene.usda` 在 Isaac Sim 中比项目说明的对比图更模糊。本次使用本机 Isaac Sim 6.0.1，对同一个模型、同一相机位姿和同一输出尺寸进行了实际渲染。

## 已确认的情况

1. 原项目说明右侧图来自 gsplat，左侧为照片；它并非 Isaac Sim 截图。旧版训练分辨率为 496 × 368，小图展示减弱了模型缺陷的可见程度。
2. 原 USD 没有保存查看器渲染配置，也没有绑定合适的默认预览相机。本机 Isaac 默认采用 DLSS Performance；496 × 368 测试中记录到了 248 × 184 的内部渲染尺寸。
3. 在相同相机下，USD 中的模型可以正常渲染，并非 Gaussian 数据转换后丢失。显示流程会增加噪声和细节损失，模型本身也已有模糊。
4. 改用 DLAA 原生分辨率后，桌面视角与 gsplat 参考图的 SSIM 从 0.729 提升到 0.859；房间视角约为 0.817，与原来的 0.820 接近。因此不能宣称每个视角都因设置变化而同幅度变清楚。

DLSS 会利用较低分辨率图像重建输出，DLAA 使用原生分辨率；相关设置见 [NVIDIA RTX Real-Time 2.0 文档](https://docs.omniverse.nvidia.com/materials-and-rendering/latest/rtx-renderer_rt.html)。实际参数记录在各次渲染的 `manifest.json` 中。

![同模型的渲染流程对比](dlss_dlaa_comparison.jpg)

左列：gsplat 参考；中列：Isaac 默认 DLSS；右列：Isaac 原生分辨率 DLAA。三列使用相同的 v2 模型与相机。

## 已写入场景的预览设置

- RTX Real-Time 2.0 (`RealTimePathTracing`)。
- NVIDIA DLAA，原生分辨率；DLSS 回退模式设为 Quality。
- 关闭帧生成、景深和运动模糊。
- Gaussian 保持训练时的显示空间颜色处理。
- 默认相机 `/World/ReviewRoom`，另有 `ReviewTable`、`ReviewChair`、`ReviewFloor`。
- 预览机位根据真实扫描相机做朝向校正和 4:3 裁剪，不改变 Gaussian 几何。原始 `ScanCamera` 与 `CaptureReplay` 保留。

在一次全新的 Isaac 进程中，以原来的默认 DLSS 启动后重新打开场景，已验证 USD 自动加载 DLAA，且视口相机自动绑定到 `ReviewRoom`。结果在 `fixed_scene/verification.json`，图像在同目录。

打开场景后可从视口相机菜单选择 `ReviewRoom` 或 `ReviewTable`。自由移动到扫描没有覆盖的位置，仍可能出现明显缺失或变形。

## 原分辨率细化

在 v2 的基础上，以裁切后的原图 992 × 736 继续训练到 28,000 步，得到 3,001,853 个 Gaussian。相机位姿保持原标定，未增加新的扫描数据。以下比较使用同一批 49 个保留视角、同一 992 × 736 输出尺寸：

| 模型 | 平均 PSNR | 平均 SSIM |
|---|---:|---:|
| v2，按 992 × 736 重新渲染 | 21.0069 dB | 0.72875 |
| v3，原分辨率细化 | 21.6277 dB | 0.75224 |

38/49 个视角的 PSNR 提高，46/49 个视角的 SSIM 提高。这个提升是有限的；玻璃、椅面和地毯仍有重建误差。旧 README 中 v2 的 21.87 dB / 0.7679 是 496 × 368 评估，不能直接与这里的原分辨率数字比较。

v3 已替换主场景引用的 `gaussians.usdc`，并在一次新的 Isaac Sim 6.0.1 进程中重新打开、渲染确认。验证记录为 `fixed_scene_v3/verification.json`，包含实际加载的 3,001,853 个 Gaussian 和文件 SHA-256。旧模型保存在 `assets/room01/gaussians_v2.usdc`，旧版场景入口为 `room01_scene_v2.usda`。

![相同原分辨率下的细节对比](fullres_detail_comparison.jpg)

下面是在相同预览相机与 DLAA 配置下，Isaac Sim 实际输出的原尺寸局部对比：

![Isaac Sim v2 与 v3 局部对比](isaac_v2_v3_details.jpg)

原生渲染的改进并非每个指标都一致：房间裁剪视角的照片 PSNR 从 17.62 降到 17.27 dB，但 SSIM 从 0.518 升到 0.526；桌面裁剪视角 PSNR 从 21.74 升到 21.99 dB、SSIM 从 0.920 升到 0.923。细节见 `isaac_v2_v3_metrics.json`。玻璃区域仍明显模糊，不能用全局均值掩盖它。

这些仍是同一次扫描内的诊断指标。保留视角未用于 RGB 优化，但 Polycam 位姿和初始化几何来自完整扫描，不代表独立实景泛化成绩。

## 复现与证据

- `default_metrics.json`、`quality_metrics.json`：同机位 gsplat / Isaac 渲染比较。
- `fullres_model_comparison.json`：49 个原分辨率视角的逐帧变化。
- `preview_profile.json`：预览相机和保存的渲染配置。
- `fixed_scene/verification.json`：原模型下，重新打开文件后的配置和实际输出检查。
- 原预览配置备份：`room01_scene_before_preview_fix.usda`，用于还原文件内容。
- `isaac_path_2/manifest.json`：传统 `PathTracing` 在本次配置中输出全黑，未通过场景检查，因此本场景采用已验证的 RTX Real-Time 2.0。

诊断脚本为 `scripts/diagnose_isaac_render.py`；保存预览设置用 `scripts/configure_scene_preview.py`；重新打开场景并验证用 `scripts/verify_isaac_preview.py`。Isaac 脚本须使用安装了 Isaac Sim 6.0.1 的 Python 环境执行。
