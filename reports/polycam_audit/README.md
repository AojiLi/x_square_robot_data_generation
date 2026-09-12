# 本次 Polycam 扫描：检查结果与下一步

检查日期：2026-09-09。

**这批导出文件具备开始场景重建的基本输入。下一步是建立保持真实尺度的训练数据集，验证相机与几何对齐，再训练第一版 3D Gaussian 场景。** 本次已完成文件和配对检查，以及少量视角的几何投影抽查；尚未训练 Gaussian，也未完成碰撞模型或 Isaac Sim 导入。

## 实际文件内容

| 文件 | 已核实内容 | 后续用途 |
|---|---|---|
| `Image_room.zip` | 488 个时间戳；各有原始 RGB、校正 RGB、原始相机、校正相机；另有深度、校正深度、置信度、SfM 统计等 | 信息最完整，作为预处理主来源 |
| `Raw_room.zip` | 同一批 488 个视角；包含原始/校正 RGB 与相机、原始深度、置信度，另有 `mesh_info.json` | 提供几何对齐元数据并保留原始备份 |
| `GLTF_room.zip` | 实际是 `9.9.2026.glb`；251,470 个顶点、428,941 个三角形、3 份材质/纹理 | 场景外观参考、几何对齐及碰撞模型制作依据 |
| `Point_cloud_ply.zip` | 2,514,700 个点，只有 XYZ 和 RGB | 几何分析、坐标对齐后用作 Gaussian 初始化候选 |
| `OBJ_room.zip` | OBJ、MTL、3 张纹理 | 通用网格备份 |
| `Weixin Image_20260909070316_4_2.jpg` | 尺寸截图，可读到 3.81 m、2.68 m、1.12 m | 核对对应墙段；不等同于独立的现场实测 |

RGB 的 976 张文件是 **488 张原始图 + 同一批 488 张校正图**，不能当作 976 个独立视角。Images 与 Raw 两个 ZIP 共有的 2,928 个文件已逐一读取比较，内容完全一致。

PLY 文件没有 Gaussian 的尺度、旋转、透明度和球谐系数等属性，所以不是已经训练好的 3DGS。NVIDIA 的 Gaussian 导入器对所需属性有明确说明。[GSplat 格式要求](https://docs.omniverse.nvidia.com/extensions/latest/ext_gsplat-converter/manual.html)

## 已完成的检查

- 5 个 ZIP 的 CRC 检查全部通过，并记录了各 ZIP 的 SHA-256。
- 488 个视角按时间戳核对；原始/校正 RGB、相机、各深度与置信度变体均能找到对应文件。
- 976 张 RGB 和 2,928 张深度/置信度 PNG 全部成功解码。
- 所有 RGB 均为 1024 × 768；原始深度及置信度为 256 × 192；校正深度为 512 × 384。
- 校正相机的旋转矩阵正交性最大误差约为 8.3e-8；这说明矩阵结构正常，不代表姿态绝对准确。
- 均匀抽查了 20 个 RGB 视角，并将 GLB 顶点按校正相机投影到 4 个视角。房门、桌边、椅子和地面的大体布局相符；尚未做密集深度误差测量或精确接触几何验收。

预览：[20 个采样视角](sample_views.jpg) · [RGB 与 GLB 顶点投影对照](camera_geometry_check.jpg)。右侧投影图只显示网格顶点，黑色空隙主要来自稀疏投影，不能当成 Gaussian 训练效果或直接判为模型缺洞。照片在这里保持原始像素方向，后续不可只旋转照片而不更新标定。

## 预处理必须处理的细节

1. **成套使用校正图像和校正相机。** 优先 `corrected_images` + `corrected_cameras`，处理去畸变边缘时同步更新内参。Polycam 官方将它们定义为去畸变图像和全局优化后的位姿。[Polycam 数据格式](https://github.com/PolyCam/polyform#polycams-data-format)

2. **按时间戳选择深度变体。** Images ZIP 的每个深度目录有 488 个普通 `.png` 和 488 个 `.Clean.png`。不要把二者当成不同帧，也不要将两个目录独立排序后按位置配对。`frame_manifest.json` 记录的是来源关系，尚未认证校正深度与置信度的像素配准；使用深度监督前，还要验证内参缩放、去畸变、零深度及置信度过滤。原始深度的毫米单位见 [Polycam 深度说明](https://github.com/PolyCam/polyform#depth)。

3. **记录完整坐标和尺度变换。** GLB 的包围盒轴尺寸为约 6.142 × 3.143 × 5.187，PLY 为约 6.140 × 5.186 × 3.136，轴顺序不同，不能直接放在相同变换下叠加。GLB 到扫描相机世界需使用 `mesh_info.json` 中以列主序存储的 `alignmentTransform` 的逆，再做目标系统坐标变换；本次 4 视角抽查采用了这个关系。[Polycam 对齐说明](https://github.com/PolyCam/polyform#rawglb-thumbnail-video-mesh_info) 对训练器的自动居中和缩放也必须记录或显式关闭，以便导出后恢复米制空间。以上包围盒包含全部扫描内容，不是房间净长宽。

4. **对弱纹理区域先做质量验证。** `sfm_stats.json` 报告 368/488 帧为弱连接、66 帧视觉孤立。这些计数可能重叠，不能相加作为坏帧总数；视觉孤立也不等于没有 ARKit 位姿。样本中可见白墙、玻璃和百叶窗，这些区域值得优先检查。原始与校正相机平移差的中位数约 2.60 cm，最大约 20.92 cm；这是优化改变量，不是实测定位误差。先验证重投影和新视角重建，再决定是否需要补拍。

## 建议执行顺序

1. **生成一个可复现的训练数据集。** 从上述 ZIP 读取完整的校正 RGB/相机配对，建立 `transforms.json` 或后端要求的相机文件；将点云/网格对齐到相机坐标，做适度降采样作为初始化候选；保留各来源和变换。预留约 10% 的评估视角，第一轮目的是诊断重建质量，不将相邻扫描帧的评估分数当成陌生视角泛化能力。

2. **训练一版轻量 Gaussian 基线。** 建议先选 Nerfstudio 的 Splatfacto：它有现成的 Polycam 数据入口，也支持 Gaussian 训练和 PLY 导出。这是工程选择，不是声称 VLK 指定使用该后端。当前 shell 中没有 `ns-process-data` / `ns-train`，当前系统 Python 也没有 PyTorch/Nerfstudio；尚未安装训练环境或测试 CUDA 扩展。本机检测到 RTX 5070 Ti Laptop，约 12 GB 显存，首次可从降分辨率和受控 Gaussian 数量开始，以实际显存占用为准。[Polycam 导入](https://docs.nerf.studio/quickstart/custom_dataset.html#polycam-capture) · [Splatfacto](https://docs.nerf.studio/nerfology/methods/splat.html)

   深度文件存在不代表默认训练器就会把它用作损失。本阶段需明确区分米制位姿/几何初始化与显式深度监督；后者要验证后端实际实现后再接入。

3. **验收场景，再制作碰撞几何。** 从计划中的机器人相机高度、桌前视角和近桌视角检查重影、漂浮点、桌沿及可见面缺失。测量实际桌面长宽高与关键墙段，建立地面、墙体、桌面的简洁碰撞几何。若有要移动的物体，将它们从静态背景中分离并单独建模；扫描里没看到的接触表面需要补充观测或测量。

4. **导出到 Isaac Sim 做一个小闭环。** 将训练好的 Gaussian 转为兼容的 ParticleField/USD，配上已对齐的碰撞几何、机器人和独立物体。先验证一个物体、一个动作和所有需要的相机，再扩增轨迹。NuRec 渲染检查工具可在已知相机位姿下输出图像并与参考图比较，但当前普通点云不能直接视为完成的 Gaussian 场景。[NVIDIA NuRec 渲染工具](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/nurec_utils.html)

VLK 原论文的主线是米制场景重建与标注、运动合成、机器人视角渲染；本次下一步对应第一阶段。桌面双臂与灵巧手可以复用场景准备，但动作生成和接触验证要适配实际平台。[VLK §3.1](https://arxiv.org/html/2606.30645v1#S3.SS1)

检查明细：`audit.json`；逐帧来源清单：`frame_manifest.json`。从项目根目录运行 `python3 scripts/audit_polycam.py` 可重跑文件完整性和结构检查；需要 Pillow。采样图与投影图是本次独立抽查产物，不由该检查脚本生成。
