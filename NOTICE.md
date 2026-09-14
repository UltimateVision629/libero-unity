# 第三方组件与归属声明 (NOTICE)

本仓库包含或依赖以下第三方内容。使用、修改、再分发时请遵守各自的许可条款。

---

## 1. LIBERO（上游基准）

- **来源**：https://github.com/Lifelong-Robot-Learning/LIBERO
- **许可**：MIT License，Copyright (c) 2023 Lifelong Robot Learning
- **本仓库中的体现**：
  - `Assets/LIBERO/Scripts/` 下的 C# 移植代码（LIBERO 仿真核心的 Unity/C# 版本）
  - `Assets/LIBERO/assets/bddl_files/` —— 从上游 Python 工程复制的 BDDL 任务定义
  - 场景构建 / 区域采样 / 位姿初始化等逻辑的移植
- **合规要求**：MIT 要求保留版权声明与许可文本。见本目录 `LICENSE`。

> 本工程是 LIBERO 的一个 **Unity 移植（MVP）**，并非官方版本。README 已如实说明。

---

## 2. 物体资产库（**已从本仓库排除**）

以下目录属于上游 LIBERO 自带的物体库，**未包含在本仓库中**（见 `.gitignore`）：

| 目录 | 出处 |
|---|---|
| `Assets/LIBERO/assets/turbosquid_objects/` | TurboSquid 购买的 3D 模型 |
| `Assets/LIBERO/assets/stable_hope_objects/` | STABLeHOpe 物体库 |
| `Assets/LIBERO/assets/stable_scanned_objects/` | 扫描实物库 |
| `Assets/LIBERO/assets/articulated_objects/` | 铰接物体（橱柜/微波炉等） |
| `Assets/LIBERO/assets/textures/` | 上述物体所用贴图 |

**排除原因**：这些资产的再分发条款不明确或受限（TurboSquid 模型的再分发取决于
原始购买许可；STABLeHOpe 等库亦有各自条款）。它们**不是本移植工程的产物**。

**获取方式**：从上游 LIBERO 仓库取得后，放到 `Assets/LIBERO/assets/<同名目录>/`
即可，无需修改代码。

**为什么可以安全排除**：本工程的 `put_in_box` 场景
（`assets/robots/so100_mjcf/libero_put_block_in_box.xml`）**不依赖**以上任何目录——
它只用到 `assets/robots/so100_mjcf/`（SO100 模型 + STL）以及 XML 中定义的程序化
几何（`<geom type="box">` 的桌面、盒子与三个方块）。实测
`libero_put_block_in_box.scene` 中上述目录名的引用次数均为 0。

---

## 3. MuJoCo Unity 插件

- **来源**：https://github.com/google-deepmind/mujoco （`unity/` 目录，tag 3.2.4）
- **许可**：Apache License 2.0
- **本仓库中的体现**：`Packages/org.mujoco/`（内嵌包，含运行时/编辑器脚本与
  原生库 `mujoco.dll`）
- **合规要求**：Apache-2.0 要求保留许可与声明文件。若再分发，请一并保留
  `Packages/org.mujoco/LICENSE`（如上游包内提供）及本声明。
- **说明**：该内嵌包是为「在本机 Tuanjie/Unity 中导入 MJCF 场景」而放入的；
  官方分发渠道是 Unity Package Manager。

---

## 4. URDF Importer

- **来源**：https://github.com/Unity-Technologies/URDF-Importer
- **许可**：Apache License 2.0
- **本仓库中的体现**：仅在 `Packages/manifest.json` 中作为 git 依赖声明，源码由
  Unity 包管理器在本地解析（`Library/PackageCache/`，不入库）。
- **用途**：LIBERO 的 Franka Panda 相关代码使用 `Unity.Robotics.UrdfImporter`
  与 `ArticulationBody`。本工程的 `put_in_box`（SO100）路径不使用它。

---

## 5. SO-ARM100 / SO100 模型

- **来源**：TheRobotStudio 的 SO-ARM100 开源硬件项目
- **本仓库中的体现**：`Assets/LIBERO/assets/robots/so100_mjcf/`
 （`so_arm100.xml`、`libero_put_block_in_box.xml`、`libero_pick_up_red_block.xml`
 及配套 STL 网格，共 45 个文件 / 约 3 MB）
- **说明**：SO-ARM100 为开源项目；本仓库对其 MJCF 做了场景化改造
 （加入桌面、盒子、方块与相机锚点）。请以原始项目的许可条款为准。

---

## 6. 训练数据集

- **位置**：独立的 Hugging Face 私有仓库（不在本 GitHub 仓库中）
- **来源**：由本工程作者通过 Joy-Con 遥操作在 Unity/MuJoCo 场景中自行采集
  （101 集 `success=True` 的演示数据）
- **说明**：采集所依赖的场景来自上述第三节（SO100 模型）与 LIBERO，但**采集产生的
  数据本身**为本工程作者所有。

---

## 免责声明

本文件仅为归属与来源说明，**不构成法律意见**。若计划商业使用或大规模再分发，
请就各第三方组件的具体许可条款自行核实，必要时咨询法律专业人士。
