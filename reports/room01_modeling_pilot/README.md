# 房间局部建模样板

这是独立的 Blender 建模试验，范围为一张桌子的桌板、走线孔、相邻白色背板，以及一组窗框。主场景 `assets/room01/room01_scene.usda` 未替换。

## 文件

- `../../assets/room01/modeling_pilot/room01_modeling_pilot.blend`：可编辑 Blender 工程；带隐藏的扫描参考集合。
- `../../assets/room01/modeling_pilot/room01_modeling_pilot.usda`：独立 Isaac 场景；默认 ReviewTable，相机菜单可选 ReviewRoom。
- `comparison.png`：同一相机、736×552、Isaac 原生 DLAA 的旧版与样板对比。
- `ReviewTable.png`、`ReviewRoom.png`：样板的原尺寸 Isaac 输出。

## 范围和限制

1. 桌面高度、布局和窗平面来自原扫描。桌板约 1.22×0.62 m；厚度、倒角、走线孔轮廓及窗框宽度是近似建模参数，尚未用真实尺寸复核。
2. 窗户目前是深色窗框和淡色不透明占位面，没有重建窗外建筑，也没有完成真实玻璃的透射与反射材质。
3. 材质和照明用于查看几何；原照片中的划痕、污渍、光照外观没有被恢复。边界规整不等于照片还原度提高。
4. 保留的其余房间背景来自裁切后的原始纹理扫描网格，椅子等家具未重新建模。没有添加碰撞、机器人或任务逻辑。
5. 尝试高斯背景后发现局部漂浮斑块及切割接缝影响判断，因此交付样板使用扫描网格背景。对比中的背景表示方法也有变化，不能把整张图的变化全部归因于建模。

## 复现

在项目根目录执行：

```bash
bash tools/modeling_blender_deb/blender.sh --background --factory-startup --python-exit-code 1 --python assets/room01/modeling_pilot/build_blender.py
.venv-usd/bin/python assets/room01/modeling_pilot/export_modeled_usd.py
.venv-usd/bin/python assets/room01/modeling_pilot/assemble_usd.py
.venv-usd/bin/python assets/room01/modeling_pilot/use_mesh_context.py
```

本地 Blender 4.0.2 以发行包解压到项目 tools 目录的方式运行；没有安装或升级系统软件。该构建没有原生 USD 导出算子，因此读取 Blender 求值后的网格，经已安装的 OpenUSD SDK 写入等效 UsdGeom.Mesh。15 个建模网格均保留了几何坐标和材质绑定。

`render_isaac.py` 使用现有 Isaac Sim 6.0.1 Python 运行时作离屏检查。独立配置目录为 `tools/modeling_pilot_runtime`。

验证通过：15 个 Blender 网格的顶点与 USD 导出逐点一致，USD 依赖无缺失；两个同机位的原生 Isaac 输出已生成。主高斯资产 SHA-256 与建模前一致。灯光为样板预览布光，不是从照片反演出的实景光照。
