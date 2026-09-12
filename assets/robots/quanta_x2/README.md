# Quanta X2 / 自变量量子 2

入口：`quanta_x2_dual_revo2_bridge.urdf`。

从用户指定的本机 `x_square_copy/isaac-sim-scenes/assets/robots/quanta_x2` 复制，保留 URDF、`mesh/`、`revo2_external/` 和 `flange_adapter/` 的相对路径。

- URDF：103 个 link、102 个 joint，包含双轮、双臂和双 Revo2 手。
- 122 个独立网格引用已全部补齐为真实 STL，并通过格式检查。
- 源目录中的 Git LFS 占位文件已在本副本中实体化，124 个 LFS 文件均通过其原始 SHA-256 校验。部分文件通过现有 USD 中保留的网格字节恢复，只有完整哈希匹配时才采纳。
- URDF 的 SHA-256 为 `9f57ade374229fc51f1dc84283679c36f2d45a7bdd88a590fa3d3c9e58548e94`，与原文件一致。

`quanta_x2_robot.usdc` 是原目录已有机器人 USD 转换件的合并副本，包含实例化网格。它保留固定底座关节 `root_joint`，适合先做模型显示；后续轮式移动需要使用相应自由底座导入配置并验证物理行为。

完整资源检查位于项目的 `reports/room01_reconstruction/robot_asset_checks.json` 和 `robot_mesh_format_checks.json`。本次文件准备没有连接实机，也没有完成机器人与扫描场景的动力学验证。
