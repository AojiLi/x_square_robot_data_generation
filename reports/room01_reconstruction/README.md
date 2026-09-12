# 本次处理结果

场景、结果图片和运行入口见项目根目录 [README](../../README.md)。

- 数据集结构检查：`prepared_dataset_checks.json`。
- CUDA 渲染器前向/反向检查：`cuda_smoke.json`。
- OpenUSD 场景和相机投影检查：`usd_scene_checks.json`。
- 机器人原始复制清单：`robot_copy_manifest.json`，记录复制当时的字节，其中包含源项目的 LFS 指针。
- 机器人 LFS 对象清单：`robot_lfs_manifest.json`。
- 机器人实体资源就绪检查：`robot_asset_checks.json`，应在 LFS 实体补齐后使用。
- 原目录已有机器人 USD 与 URDF 零关节位姿的一致性抽查：`robot_usd_urdf_consistency.json`。该检查不是新场景的物理验收。

当前第三版最终 49 帧的逐帧图像指标位于 `../../outputs/room01/gsplat_fullres_v3/eval_028000.json`，对应渲染 PNG 位于同目录的 `eval_renders/`。Isaac Sim 实际渲染和显示配置修复记录位于 `../room01_blur_diagnosis/`。
