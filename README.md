# LIBERO for Unity

Unity implementation of the LIBERO benchmark - benchmarking knowledge transfer for lifelong robot learning.

This is a minimum viable product (MVP) porting the core LIBERO simulation engine from Python/MuJoCo to Unity C#.

## Project Structure

```
Assets/LIBERO/
├── assets/bddl_files/           BDDL task definition files (copied from Python project)
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

## References

- Original LIBERO: https://github.com/Lifelong-Robot-Learning/LIBERO
- robosuite: https://robosuite.ai
- BDDL: https://github.com/StanfordVL/bddl
