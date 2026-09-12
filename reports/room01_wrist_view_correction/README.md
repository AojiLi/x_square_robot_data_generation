# 双腕相机按指定参考修正

当前双腕视角以用户明确指定的 `20260824T091633.664930Z-pid2632484-1c6fc5df/requests/request_000001/` 为准：左腕对应其中的 `cam_left_wrist.png`，右腕对应同一组的 `cam_right_wrist.png`。后续批次的多数画面或话题统计不再决定这两个机位的显示方向。

![指定实机参考与修正后仿真](reference_comparison.jpg)

左列为用户指定的真实 PNG，右列是 Isaac Sim 原生渲染。右列采用接近参考照片的侧立手掌姿态进行对照；完整手臂位姿并未实测配准。材质、手部细节、光照和相机光心仍有差异，不能把这张对照图当成逐像素标定结果。

## 已修正的方向

- 两路相机相对上一版各增加 **180° 光轴滚转**，纠正手掌所在的一侧与上下方向。
- 根据参考中的手部位置，增加约 **17.5° 向下的光轴俯仰估计**。
- 左腕手掌在画面左侧，右腕手掌在右侧；两边拇指都位于其余手指上方。
- 方向修正在相机的三维安装变换中生效，输出仍是原生鱼眼 RGB，没有在保存后旋转或镜像图片。
- 圆形边界参数改回同一参考会话的拟合结果；180° 等距鱼眼视场仍为假设，完整实机内外参未完成标定。

## 实际默认起始姿态

默认 `reset()`、保存的 USD 和 Blender 审查副本已统一为侧立手掌姿态。普通启动与第 0 帧采集会直接使用该姿态，无需指定额外的 `--joint-pose`。

本目录的 `task_pose_preview/` 是旧的掌心向下诊断截图，已被 [当前默认三路图像](../room01_wrist_reset_fix/initial_preview/three_views.jpg)替代。[起始姿态修正记录](../room01_wrist_reset_fix/README.md)包含新旧差异与检查。相机始终固定在腕部，抓取过程中会随实际手腕转动。

## 参数与验收

当前入口仍为 [task_config.json](../../assets/room01/sim/task_config.json) 和 [room01_manipulation.usda](../../assets/room01/sim/room01_manipulation.usda)。配置中将该参考标记为 `explicit_user_selection`，并保存安装变换、源图片与估计依据。

- [参考方向检查](orientation_verification.json)：左右手掌所在侧、拇指方向、正确腕部挂载和指定参考。
- [OpenCV / USD 镜头参数检查](../room01_camera_calibration/projection_reference_verification.json)。
- [实际三路图像、投影和同步检查](../room01_sim/vla_io_verification.json)。
- [当前示例与交付清单](../room01_sim/delivery_manifest.json)。

原始参考照片、服务器记录和机器人控制程序均未修改。全量图片统计仍可在 [之前的目录审查](../room01_wrist_fisheye_alignment/ARCHIVE_REVIEW.md) 中查看；那份统计不替代用户指定的视角基准。
