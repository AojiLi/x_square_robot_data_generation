# 完整房间 Blender 模型

打开 `room01_full.blend`。这是按照提供的 Polycam 照片、相机位姿与扫描网格重新建立的可编辑模型，使用原扫描的米制、Z 轴向上坐标。

已建内容包括：

- 带内凹墙角的墙体、踢脚线、交错铺装地毯、分格吊顶、通风口与烟感。
- 木纹门、把手与锁孔、温控器、玻璃隔断和逐片百叶。
- 窗框、密封条、玻璃、开窗把手及墙面摄像头。
- 白色桌子的桌板、桌腿、走线孔、红色标记和表面轻微划痕。
- 光学工作台的孔阵列材质、钢架、调平脚、显示器、键盘、鼠标、主机与耳机。
- 三把独立办公椅和圆凳，包括网布几何、椅框、扶手、气杆、五星脚和双脚轮。
- 大型航空设备箱的边框、护角、铆钉、提手和脚轮；部分标签直接来自原照片。
- 纸箱堆、工具箱、打开的箱盖与翻折纸板，以及照片中的主要桌面物件和地面线缆。

## 查看方式

默认以便于检查的剖开视图打开，`02 West wall`、`04 Entry door and glazed partition`、`06 Ceiling tiles and services` 三个集合在视口中隐藏。它们仍保存在工程内；在右侧 Outliner 中打开相应集合的显示开关即可查看完整围合房间。外景照片背景也单独放在 `15 Exterior photographic context - unmeasured` 集合中。

`16 Review cameras` 包含十个与原照片匹配的相机，还包括 `Overview_cutaway` 和 `Interior_wide`。`99 Original scan reference - hidden` 是隐藏的原扫描参考，最终模型预览不依赖扫描网格来填充家具。

工程已打包照片和材质图片。可使用常规 Blender 4.x 打开，也可在项目根目录运行：

```bash
bash tools/modeling_blender_deb/blender.sh assets/room01/full_room_model/room01_full.blend
```

## 尺度与适用范围

房间外包络约 4.77 × 4.10 m，层高约 2.66 m，地面在原扫描坐标的 Z=-1.466 m。右侧内凹部分保留。墙面位置由扫描平面拟合；物件位置由照片相机、扫描交点和多视角外观联合确定。

这是一份视觉重建模型。未拍清的厚度、弧度、包装内部结构和部分小物件采用近似建模，尚未用实物测量逐项复核。材质和预览灯光按照片外观调整，不代表完成了光照标定。窗外使用照片背景，其距离与几何不用于测量。碰撞、机器人和仿真任务逻辑没有在这次建模中添加。

## 构建与证据

`build_room.py` 是构建入口，`modeling_lib.py`、`seating.py` 和 `furniture.py` 保留可调整的几何参数。修改后在项目根目录运行：

```bash
bash tools/modeling_blender_deb/blender.sh --background --factory-startup --python-exit-code 1 --python assets/room01/full_room_model/build_room.py
```

检查和预览在 `reports/room01_full_room_model/`。相机投影检查验证的是相机参数转换，不等同于对全部物件几何精度的验证。预览图由 Blender Cycles 直接渲染，和之前的 Isaac 高斯图属于不同模型与渲染流程。
