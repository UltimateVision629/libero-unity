# UnityRobotEnv

A Unity + MuJoCo environment for **dual-arm SO100 robot teleoperation and data collection**:
Joy-Con teleop → MuJoCo physics in Unity → (observation, action) trajectories → VLA training.

Not a LIBERO product. This project began as a MuJoCo-in-Unity integration and grew into a
standalone teleoperation/collection rig; the LIBERO-derived parts are limited and attributed
in [`NOTICE.md`](NOTICE.md).

| | |
|---|---|
| Simulator | Unity (Tuanjie 2022.3) + MuJoCo Unity plugin 3.2.4 |
| Robot | SO-ARM100 / SO100, single right arm in the `put_in_box` scene |
| Teleop | Joy-Con over a Python client → `JoyConReceiver` (TCP :5555) |
| Data | `TrainingServer` (TCP :5556) serves `get_obs` / `step` / `reset` / `get_task` |
| Recorded demos | 101 successful episodes, 38,892 frames |

## Project Structure

```
Assets/UnityRobotEnv/
├── assets/bddl_files/           BDDL task definitions (from upstream LIBERO)
├── Scripts/
│   ├── BDDL/
│   │   ├── BDDLToken.cs         Token types for BDDL lexer
│   │   ├── Tokenizer.cs         Lisp-style tokenizer
│   │   ├── AST.cs               S-Expression AST (SAtom, SList)
│   │   ├── SExprParser.cs       Token stream to S-Expression parser
│   │   ├── BDDLParser.cs        S-Expression to BDDLProblem mapper
│   │   ├── ProblemModel.cs      Strongly-typed BDDL data structures
│   │   └── BDDLParseException.cs
│   └── Core/
│       ├── AssetDatabase.cs     Asset path resolution
│       ├── SceneBuilder.cs      Scene construction from BDDL (fixtures + objects + placement)
│       ├── LiberoEnvironment.cs Main environment loop (step/reset/check_success)
│       ├── ObservationCollector.cs Camera & proprioception data collection
│       ├── ObjectState.cs       Object position/rotation query wrapper
│       ├── RegionSampler.cs     Random sampling within BDDL-defined regions
│       └── FrankaPandaController.cs Robot IK and action interface
└── Tests/
    ├── BDDLParserTests.cs       BDDL parsing unit tests
    └── LiberoEnvironmentTests.cs Environment integration tests
```

## MVP Scope

- [x] BDDL file parsing (Lisp-like DSL)
- [x] Scene construction from BDDL (fixtures, objects, placement regions)
- [x] Random object placement via region samplers
- [x] Step/reset/check_success environment loop
- [x] Observation collection (camera images + proprioception)
- [x] Goal success checking (On predicate)
- [ ] Full Franka Panda IK solver (currently simplified)
- [ ] MuJoCo Unity plugin integration
- [ ] gRPC/Python communication for training

## Getting Started

1. Open the project in Unity 2022.3 LTS or later
2. Run `Window > General > Test Runner` to execute unit tests
3. Add a `LiberoEnvironment` component to a GameObject
4. Set `BDDLFilePath` to a .bddl file path
5. Press Play to initialize the scene

## License

MIT - See LICENSE file

## Replay & Validation

回放录制的 demo（开环执行）和离线验证：

```bash
conda activate lerobot-kin

# 在线回放（需 Unity Play 模式运行中）
python test/replay_action_via_ik.py --traj ../DatasetsCollector/demos/episode_0001_pick_up_the_red_block_20260728_220541_success.npz

# 离线批量验证
python test/validate_replay_offline.py --dir ../DatasetsCollector/demos

# 验证 MuJoCo FK 与记录 EEF 的物理一致性
python test/verify_eef_match.py
```

### IK 后端配置

`arm_ik.py` 提供统一的 Strategy 模式接口，collect / replay / inference 三端共用 `--ik-backend` 切换：

```bash
# 默认 mujoco（与 Unity 同一物理模型，FK 误差 ~4.5e-6 m）
python test/replay_action_via_ik.py --traj <file.npz>

# 原 lerobot 后端（C 扩展，与 MuJoCo 差 30-50cm——旧行为）
python test/replay_action_via_ik.py --traj <file.npz> --ik-backend lerobot
```

| 后端 | IK 方法 | 模型一致性 | 适用 |
|------|---------|-----------|------|
| `mujoco` (默认) | MuJoCo FK + scipy least_squares + Tikhonov 正则 | ✅ 同一模型 | 回放 / 推理 |
| `lerobot` | C 扩展 fknm.pyd | ❌ 差 30-50cm（回放失败根因） | 采集保留兼容 |
| `placo` | placo + URDF | ⚠️ URDF ≠ XML | stub |

**回放目标模式**（`--ik-target`）：`eef` 直接用 `.npz` 记录的 MuJoCo EEF 观测（误差 2-7mm，仅 mujoco）；`pose` 走链式翻译（所有后端通用，误差 2-4cm rate-limit 滞后）；`auto` 自动检测 obs 有效性切换（默认）。

## 资产获取（重要）

本仓库**不包含** LIBERO 自带的第三方物体资产库。若要跑 `put_in_box` 之外的
LIBERO 任务，请自行从上游获取后放到对应路径：

```bash
# 从上游 LIBERO 取得后，放到：
Assets/UnityRobotEnv/assets/turbosquid_objects/      # TurboSquid 购买的模型
Assets/UnityRobotEnv/assets/stable_hope_objects/     # STABLeHOpe 物体库
Assets/UnityRobotEnv/assets/stable_scanned_objects/  # 扫描实物库
Assets/UnityRobotEnv/assets/articulated_objects/     # 铰接物体（橱柜/微波炉等）
Assets/UnityRobotEnv/assets/textures/                # 上述物体所用贴图
```

上游地址：https://github.com/Lifelong-Robot-Learning/LIBERO

**排除原因与影响范围**：

- 这些资产的再分发条款不明确或受限（TurboSquid 模型的再分发取决于原始购买许可），
  且它们不是本移植工程的产物。
- **本工程的 `put_in_box` 场景不依赖它们**：只用到
  `assets/robots/so100_mjcf/`（SO100 模型 + STL，45 个文件 / 约 3 MB）以及 XML 里
  定义的程序化几何（桌面、盒子、三个方块都是 `<geom type="box">`）。实测
  `libero_put_block_in_box.scene` 对这些目录名的引用次数均为 0。
- 也就是说，**克隆后直接跑 `put_in_box` 无需任何额外下载**。

`.gitignore` 已包含这些目录的排除规则。注意 `.gitignore` 只对未跟踪文件生效，
这些目录此前已被跟踪，因此索引中也已用 `git rm -r --cached` 移除。

## MuJoCo Unity 插件（依赖说明）

本工程用 MuJoCo 的 Unity 插件把 MJCF 导入场景、驱动仿真与观测采集。该插件
（Apache-2.0，来自 https://github.com/google-deepmind/mujoco 的 `unity/` 目录，
版本 **3.2.4**）以**内嵌包**形式随仓库分发：

```
Packages/org.mujoco/           # Runtime 62 个 + Editor 14 个 C# 脚本
Packages/org.mujoco/mujoco.dll # 原生库，与 Assets/mujoco.dll 版本对齐
Packages/manifest.json         # 依赖清单（必须入库，否则包无法解析）
```

插件自带的 `Tests/` 已移除（需要完整 NUnit，内嵌使用不需要）。

**注意 `Packages/` 必须在版本控制中。** 本工程早期因 `.gitignore` 里一条无锚定的
`packages/` 规则（Windows 下大小写不敏感）误把整个 `Packages/` 排除，导致新克隆的
仓库既没有依赖清单也没有插件，编辑器里看不到 MuJoCo 导入菜单。该规则已移除，
`Packages/` 现已入库。

## 第三方归属

第三方组件的来源与许可见 [`NOTICE.md`](NOTICE.md)。

## References

- Original LIBERO: https://github.com/Lifelong-Robot-Learning/LIBERO
- robosuite: https://robosuite.ai
- BDDL: https://github.com/StanfordVL/bddl
- MuJoCo: https://github.com/google-deepmind/mujoco
