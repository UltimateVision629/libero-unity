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

## References

- Original LIBERO: https://github.com/Lifelong-Robot-Learning/LIBERO
- robosuite: https://robosuite.ai
- BDDL: https://github.com/StanfordVL/bddl
