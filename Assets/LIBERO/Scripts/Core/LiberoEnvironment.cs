using System.Collections;
using System.Collections.Generic;
using LIBERO.Networking;
using UnityEngine;
using System.Linq;

namespace LIBERO.Core
{
    public class LiberoEnvironment : MonoBehaviour
    {
        [Header("BDDL Configuration")]
        public string BDDLFilePath;
        public string SuiteName = "libero_spatial";

        [Header("Scene References")]
        public GameObject ArenaRoot;
        public GameObject ObjectParent;
        public RobotArmController Robot;
        public RobotArmController Robot2;
        public ObservationCollector ObsCollector;

        [Header("Joy-Con Control")]
        public bool UseJoyCon;
        public JoyConReceiver JoyConInput;
        public float JoyConPosScale = 1.0f;
        public float JoyConRotScale = 0.5f;

        private Vector3 _jcHomePos0, _jcHomePos1;
        private Vector3 _jcHomeJoyConPos0, _jcHomeJoyConPos1;
        private float _jcHomeBaseYaw0, _jcHomeBaseYaw1;
        private bool _jcCalibrated0, _jcCalibrated1;

        [Header("Robot")]
        public RobotType RobotTypeValue = RobotType.Panda;
        public IKMode IKMode = IKMode.DLS;

        [Header("Settings")]
        public int ControlFrequency = 20;
        public int Horizon = 1000;
        public float RewardScale = 1.0f;
        public bool UseObjectObservations = true;
        public ArenaType ArenaTypeValue = ArenaType.Table;

        private BDDL.BDDLProblem _problem;
        private SceneBuilder _sceneBuilder;
        private bool _isInitialized;
        private int _stepCount;
        // --- 新增：物理引擎热身帧计数器 ---
        private int _warmupFrames = 0;

        public BDDL.BDDLProblem Problem => _problem;
        public SceneBuilder Scene => _sceneBuilder;
        public bool IsDone { get; private set; }
        public float LastReward { get; private set; }

        private static void SafeDestroy(Object obj)
        {
            if (obj == null) return;
            if (Application.isPlaying)
                Object.Destroy(obj);
            else
                Object.DestroyImmediate(obj);
        }

        [ContextMenu("Initialize Scene")]
        public void InitializeScene()
        {
            ClearScene();
            Initialize(BDDLFilePath);
        }

        private void ClearScene()
        {
            if (ArenaRoot != null) SafeDestroy(ArenaRoot);
            if (ObjectParent != null) SafeDestroy(ObjectParent);
            ArenaRoot = null;
            ObjectParent = null;
            _sceneBuilder = null;
            _isInitialized = false;
            _warmupFrames = 0;
        }

        private void Start()
        {
            // Reuse auto-created JoyConReceiver (from Bootstrap) or create one
            if (JoyConInput == null)
            {
                JoyConInput = FindObjectOfType<JoyConReceiver>();
                if (JoyConInput == null)
                {
                    var jcGo = new GameObject("JoyConReceiver");
                    jcGo.transform.SetParent(transform);
                    JoyConInput = jcGo.AddComponent<JoyConReceiver>();
                    Debug.Log("[LiberoEnvironment] Auto-created JoyConReceiver (TCP port 5555)");
                }
                UseJoyCon = true;
            }

            if (!string.IsNullOrEmpty(BDDLFilePath))
                InitializeScene();
        }

        public void Initialize(string bddlFilePath = null)
        {
            if (_isInitialized) return;

            if (bddlFilePath != null)
                BDDLFilePath = bddlFilePath;

            if (string.IsNullOrEmpty(BDDLFilePath))
            {
                Debug.LogError("BDDL file path is empty!");
                return;
            }

            if (!System.IO.File.Exists(BDDLFilePath))
            {
                Debug.LogError($"BDDL file not found: {BDDLFilePath}");
                return;
            }

            _problem = BDDL.BDDLParser.ParseFile(BDDLFilePath);

            Debug.Log("Loaded BDDL: " + _problem.ProblemName);
            Debug.Log("Language: " + _problem.LanguageInstruction);

            _sceneBuilder = new SceneBuilder(_problem);

            ArenaRoot = new GameObject("Arena");
            ArenaRoot.transform.SetParent(transform);

            var ground = GameObject.CreatePrimitive(PrimitiveType.Cube);
            ground.name = "Ground";
            ground.transform.SetParent(ArenaRoot.transform);
            ground.transform.localPosition = new Vector3(0, -0.925f, 0);
            ground.transform.localScale = new Vector3(5f, 0.1f, 5f);

            ObjectParent = new GameObject("Objects");
            ObjectParent.transform.SetParent(transform);

            _sceneBuilder.Build(ArenaRoot, ObjectParent);

            if (ObjectParent != null)
            {
                var sceneRigidbodies = ObjectParent.GetComponentsInChildren<Rigidbody>();
                foreach (var rb in sceneRigidbodies)
                {
                    rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                    rb.solverIterations = 50; 
                    rb.solverVelocityIterations = 20;
                }
            }

            BuildRobot();

            _isInitialized = true;

            Debug.Log("Scene built: " + _sceneBuilder.ObjectStates.Count + " objects, " + _sceneBuilder.FixtureStates.Count + " fixtures");
            foreach (var kv in _sceneBuilder.FixtureStates)
                Debug.Log("  Fixture '" + kv.Key + "' (" + kv.Value.Position.x.ToString("F2") + ", " + kv.Value.Position.y.ToString("F2") + ", " + kv.Value.Position.z.ToString("F2") + ")");
            foreach (var kv in _sceneBuilder.ObjectStates)
                Debug.Log("  Object '" + kv.Key + "' (" + kv.Value.Position.x.ToString("F2") + ", " + kv.Value.Position.y.ToString("F2") + ", " + kv.Value.Position.z.ToString("F2") + ")");
            Debug.Log("Goal: " + string.Join(", ", _problem.ObjOfInterest));

            EnsureCamera();
        }

        private void EnsureCamera()
        {
            if (Camera.main != null) SafeDestroy(Camera.main.gameObject);

            var cameraGo = new GameObject("Main Camera");
            var cam = cameraGo.AddComponent<Camera>();
            cameraGo.tag = "MainCamera";
            cameraGo.AddComponent<CameraSwitcher>();
            Debug.Log("Auto-created Main Camera with CameraSwitcher");
        }

        private void BuildRobot()
        {
            switch (RobotTypeValue)
            {
                case RobotType.Panda:
                    BuildPandaRobot();
                    break;
                case RobotType.SO100:
                    BuildSO100Robot();
                    break;
            }
        }

        private void BuildPandaRobot()
        {
            GameObject panda = null;
            ArticulationBody[] joints = null;
            Transform eef = null;
            ArticulationBody leftFinger = null, rightFinger = null;

            var prefab = Resources.Load<GameObject>("Robots/Panda");
            if (prefab != null)
            {
                panda = Object.Instantiate(prefab);
                panda.name = "Panda";

                SafeDestroy(panda.GetComponent<Unity.Robotics.UrdfImporter.Control.Controller>());
                SafeDestroy(panda.GetComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>());
                foreach (var jc in panda.GetComponentsInChildren<JointControl>())
                    SafeDestroy(jc);

                var allABs = panda.GetComponentsInChildren<ArticulationBody>();
                foreach (var ab in allABs)
                {
                    if (ab.transform.parent == panda.transform ||
                        ab.transform.parent?.GetComponent<ArticulationBody>() == null)
                    {
                        ab.immovable = true;
                        ab.jointType = ArticulationJointType.FixedJoint;
                        break;
                    }
                }

                var jointList = new List<ArticulationBody>();
                foreach (var ab in allABs)
                {
                    if (ab.jointType == ArticulationJointType.RevoluteJoint)
                    {
                        ab.twistLock = ArticulationDofLock.LimitedMotion;
                        var drive = ab.xDrive;
                        drive.stiffness = 10000f;
                        drive.damping = 200f;
                        drive.forceLimit = 200f;
                        ab.xDrive = drive;
                        jointList.Add(ab);
                    }
                    else if (ab.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        ab.linearLockX = ArticulationDofLock.LimitedMotion;
                        var drive = ab.xDrive;
                        drive.stiffness = 1000f;
                        drive.damping = 100f;
                        ab.xDrive = drive;
                        if (ab.name.Contains("left") || ab.name.Contains("Left"))
                            leftFinger = ab;
                        else if (ab.name.Contains("right") || ab.name.Contains("Right"))
                            rightFinger = ab;
                    }
                }
                joints = jointList.ToArray();
                eef = panda.transform.FindRecursive("panda_grasptarget")
                    ?? panda.transform;
            }
            else
            {
                string urdfPath = AssetDatabase.GetAssetPath("robots/panda_urdf/panda.urdf");
                string meshDir = AssetDatabase.GetAssetPath("robots/panda");
                if (!System.IO.File.Exists(urdfPath)) return;

                var result = UrdfArmBuilder.Build(urdfPath, meshDir);
                if (result.Root == null) return;
                panda = result.Root;
                joints = result.Joints;
                eef = result.GripSite ?? result.EEFTransform;
                leftFinger = result.LeftFinger;
                rightFinger = result.RightFinger;
            }

            panda.transform.SetParent(ArenaRoot.transform);
            panda.transform.localPosition = GetRobotBasePosition();

            if (ArenaTypeValue != ArenaType.Floor &&
                ArenaTypeValue != ArenaType.CoffeeTable &&
                ArenaTypeValue != ArenaType.LivingRoom)
            {
                string mountPath = AssetDatabase.GetAssetPath("mounts/rethink_mount.xml");
                if (System.IO.File.Exists(mountPath))
                {
                    Transform mountParent = panda.transform.FindRecursive("panda_link0")
                        ?? panda.transform;
                    RobotBuilder.AttachMount(mountParent.gameObject, mountPath);
                }
            }

            if (Robot == null)
                Robot = panda.AddComponent<FrankaPandaController>();

            Robot.ArmJointCount = 7;
            Robot.RootAB = FindRootArticulationBody(panda);
            Robot.Joints = joints;
            Robot.EEFTransform = eef;
            Robot.LeftFinger = leftFinger;
            Robot.RightFinger = rightFinger;
            Robot.InitializeJoints();
            Robot.ResetToHomePose();

            // Apply IK mode from LiberoEnvironment inspector
            Robot.IkSolverMode = IKMode;

            Physics.SyncTransforms();
            var rootAb2 = FindRootArticulationBody(panda);
            if (rootAb2 != null)
                rootAb2.TeleportRoot(panda.transform.position, panda.transform.rotation);

            if (panda.GetComponent<KeyboardController>() == null)
            {
                var kbd = panda.AddComponent<KeyboardController>();
                kbd.Robot = Robot;
            }

            Debug.Log($"[LiberoEnvironment] Panda built at {panda.transform.localPosition}, "
                + $"joints={joints?.Length}, eef={eef != null}");
        }

        private void BuildSO100Robot()
        {
            Robot = BuildSingleSO100(-0.2f, "SO100_L", false);
            Robot2 = BuildSingleSO100(0.2f, "SO100_R", false);
        }

        private RobotArmController BuildSingleSO100(float xPosition, string robotName, bool addKeyboard)
        {
            GameObject so100 = null;
            ArticulationBody[] joints = null;
            Transform eef = null;
            ArticulationBody gripperJoint = null;

            var prefab = Resources.Load<GameObject>("Robots/SO100");
            if (prefab != null)
            {
                so100 = Object.Instantiate(prefab);
                so100.name = robotName;

                SafeDestroy(so100.GetComponent<Unity.Robotics.UrdfImporter.Control.Controller>());
                SafeDestroy(so100.GetComponent<Unity.Robotics.UrdfImporter.Control.FKRobot>());
                foreach (var jc in so100.GetComponentsInChildren<JointControl>())
                    SafeDestroy(jc);

                var allABs = so100.GetComponentsInChildren<ArticulationBody>();
                foreach (var ab in allABs)
                {
                    // 1. 给【所有】关节强制穿上防穿模护甲
                    ab.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
                    
                    // 2. 增加物理解算迭代次数（至关重要！）
                    // 当夹爪马达发力 (500f) 挤压方块时，引擎需要更多次计算来阻止它们融合
                    ab.solverIterations = 50; 
                    ab.solverVelocityIterations = 20;

                    // 3. 只有底座（根节点）需要被钉死在天上
                    if (ab.transform.parent == so100.transform ||
                        ab.transform.parent?.GetComponent<ArticulationBody>() == null)
                    {
                        ab.immovable = true;
                        ab.jointType = ArticulationJointType.FixedJoint;
                        // 删掉了该死的 break; ！让循环继续处理剩下的手臂和夹爪！
                    }
                }

                var armJointsList = new List<ArticulationBody>();
                var allRevolute = new List<ArticulationBody>();
                foreach (var ab in allABs)
                {
                    if (ab.jointType == ArticulationJointType.RevoluteJoint)
                    {
                        ab.twistLock = ArticulationDofLock.LimitedMotion;
                        var drive = ab.xDrive;
                        drive.stiffness = 10000f;
                        drive.damping = 200f;
                        drive.forceLimit = 200f;
                        ab.xDrive = drive;
                        allRevolute.Add(ab);
                    }
                    else if (ab.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        armJointsList.Add(ab);
                    }
                }

                if (allRevolute.Count >= 6)
                {
                    gripperJoint = allRevolute[allRevolute.Count - 1];
                    for (int i = 0; i < allRevolute.Count - 1; i++)
                        armJointsList.Add(allRevolute[i]);
                }
                else
                {
                    armJointsList.AddRange(allRevolute);
                }

                joints = armJointsList.ToArray();
                eef = so100.transform.FindRecursive("gripper")
                    ?? so100.transform;
            }
            else
            {
                string urdfPath = AssetDatabase.GetAssetPath("robots/so100/so100.urdf");
                string meshDir = AssetDatabase.GetAssetPath("robots/so100");
                if (!System.IO.File.Exists(urdfPath))
                {
                    Debug.LogError($"[LiberoEnvironment] SO100 URDF not found: {urdfPath}");
                    return null;
                }

                var config = new RobotBuildConfig
                {
                    EEFLinkNames = new[] { "gripper" },
                    GripperJointLinkName = "jaw"
                };

                var result = UrdfArmBuilder.Build(urdfPath, meshDir, config);
                if (result.Root == null)
                {
                    Debug.LogError("[LiberoEnvironment] SO100 URDF build failed");
                    return null;
                }

                so100 = result.Root;
                so100.name = robotName;

                var armJointsList = new List<ArticulationBody>();
                gripperJoint = result.GripperJoint;

                foreach (var ab in result.Joints)
                {
                    if (gripperJoint != null && ab == gripperJoint)
                        continue;
                    if (ab.jointType == ArticulationJointType.RevoluteJoint)
                    {
                        armJointsList.Add(ab);
                    }
                    else if (ab.jointType == ArticulationJointType.PrismaticJoint)
                    {
                        armJointsList.Add(ab);
                    }
                }

                joints = armJointsList.ToArray();
                eef = result.EEFTransform;
            }

            if (gripperJoint != null)
            {
                gripperJoint.linearLockX = ArticulationDofLock.LockedMotion;
                gripperJoint.linearLockY = ArticulationDofLock.LockedMotion;
                gripperJoint.linearLockZ = ArticulationDofLock.LockedMotion;
                var gDrive = gripperJoint.xDrive;
                gDrive.stiffness = 10000f;
                gDrive.damping = 100f;
                gDrive.forceLimit = 15f;
                gripperJoint.xDrive = gDrive;
            }

            so100.transform.rotation = Quaternion.Euler(0, -90, 0);
            so100.transform.SetParent(ArenaRoot.transform);
            so100.transform.localPosition = new Vector3(xPosition, -0.075f, GetRobotBasePosition().z);
            string basePath = AssetDatabase.GetAssetPath("robots/so100/base.obj");

            string SO100_basePath = AssetDatabase.GetAssetPath("robots/so100/base.obj");
            if (System.IO.File.Exists(SO100_basePath))
            {
                Mesh baseMesh = MeshFileParser.Load(SO100_basePath, Vector3.one);
                var baseGo = new GameObject("base_pedestal");
                baseGo.transform.SetParent(so100.transform, false);
                // 调整位置使底座贴地、支撑机械臂
                baseGo.transform.localPosition = new Vector3(0, -0.875f, 0);
                baseGo.transform.localScale = Vector3.one;
                baseGo.AddComponent<MeshFilter>().sharedMesh = baseMesh;
                baseGo.AddComponent<MeshRenderer>().material = 
                    new Material(Shader.Find("Standard")) { color = Color.gray };
            }

            var controller = so100.AddComponent<SO100Controller>();
            controller.ArmJointCount = 5;
            controller.RootAB = FindRootArticulationBody(so100);
            controller.Joints = joints;
            controller.EEFTransform = eef;
            controller.GripperJoint = gripperJoint;
            controller.HomePoseDegrees = new float[] { 0f, 30f, -60f, 0f, 0f };
            controller.InitializeJoints();
            controller.ResetToHomePose();

            // Apply IK mode from LiberoEnvironment inspector
            controller.IkSolverMode = IKMode;

            Physics.SyncTransforms();
            var rootAb = FindRootArticulationBody(so100);
            if (rootAb != null)
                rootAb.TeleportRoot(so100.transform.position, so100.transform.rotation);

            if (addKeyboard && so100.GetComponent<KeyboardController>() == null)
            {
                var kbd = so100.AddComponent<KeyboardController>();
                kbd.Robot = controller;
            }

            Debug.Log($"[LiberoEnvironment] SO100 '{robotName}' built at {so100.transform.localPosition}, "
                + $"joints={joints?.Length}, eef={eef != null}, gripper={gripperJoint != null}");

            return controller;
        }

        private static ArticulationBody FindRootArticulationBody(GameObject go)
        {
            var abs = go.GetComponentsInChildren<ArticulationBody>();
            foreach (var ab in abs)
            {
                if (ab.transform.parent == go.transform ||
                    ab.transform.parent?.GetComponent<ArticulationBody>() == null)
                    return ab;
            }
            return null;
        }

        private Vector3 GetRobotBasePosition()
        {
            switch (ArenaTypeValue)
            {
                case ArenaType.Floor:
                case ArenaType.CoffeeTable:
                case ArenaType.LivingRoom:
                    return new Vector3(0, 0, -0.8f);
                case ArenaType.Study:
                    return new Vector3(0, 0, -0.55f);
                case ArenaType.Kitchen:
                    return new Vector3(0, 0, -0.55f);
                default:
                    return new Vector3(0, 0, -0.6f);
            }
        }

        public Observation ResetEnvironment()
        {
            if (!_isInitialized)
            {
                Debug.LogError("Environment not initialized!");
                return new Observation();
            }

            _stepCount = 0;
            IsDone = false;
            LastReward = 0f;

            if (_sceneBuilder != null && ObjectParent != null)
                _sceneBuilder.RandomizeObjectPlacements(ObjectParent.transform);

            if (Robot != null)
                Robot.ResetToHomePose();
            if (Robot2 != null)
                Robot2.ResetToHomePose();

            StartCoroutine(WaitForPhysicsSettle());

            return GatherObservation();
        }

        private IEnumerator WaitForPhysicsSettle()
        {
            for (int i = 0; i < 10; i++)
            {
                Physics.Simulate(Time.fixedDeltaTime);
                yield return new WaitForFixedUpdate();
            }
        }

        public (Observation obs, float reward, bool done, Dictionary<string, object> info) Step(float[] action)
        {
            if (!_isInitialized)
            {
                Debug.LogError("Environment not initialized!");
                return (new Observation(), 0f, true, null);
            }

            if (Robot != null)
                Robot.ApplyAction(action);

            _stepCount++;

            var obs = GatherObservation();
            IsDone = CheckSuccess();
            LastReward = IsDone ? 1.0f * RewardScale : 0f;

            var info = new Dictionary<string, object>
            {
                ["step"] = _stepCount,
                ["success"] = IsDone
            };

            if (_stepCount >= Horizon)
                IsDone = true;

            return (obs, LastReward, IsDone, info);
        }

        private Observation GatherObservation()
        {
            if (ObsCollector != null)
            {
                var allStates = new Dictionary<string, ObjectState>();
                if (_sceneBuilder?.ObjectStates != null)
                    foreach (var kv in _sceneBuilder.ObjectStates) allStates[kv.Key] = kv.Value;
                if (_sceneBuilder?.FixtureStates != null)
                    foreach (var kv in _sceneBuilder.FixtureStates) allStates[kv.Key] = kv.Value;

                return ObsCollector.Collect(allStates, Robot);
            }

            return new Observation
            {
                JointPositions = new float[7],
                EEFPosition = new float[3],
                EEFQuaternion = new float[4],
                GripperQPos = new float[2],
                ObjectPositions = new Dictionary<string, float[]>(),
                ObjectQuaternions = new Dictionary<string, float[]>()
            };
        }

        public bool CheckSuccess()
        {
            if (_problem == null || _problem.Goal.Conditions == null)
                return false;

            foreach (var condition in _problem.Goal.Conditions)
            {
                if (condition.Type != BDDL.GoalType.On)
                    continue;

                var objState = _sceneBuilder?.GetObjectState(condition.ObjectName);
                var targetState = _sceneBuilder?.GetObjectState(condition.TargetName);

                if (objState == null || targetState == null)
                    return false;

                if (!objState.IsOnTopOf(targetState))
                    return false;
            }

            return true;
        }

        public bool IsFixture(string objectName)
        {
            return _sceneBuilder?.FixtureStates?.ContainsKey(objectName) ?? false;
        }

        public ObjectState GetObjectState(string objectName)
        {
            return _sceneBuilder?.GetObjectState(objectName);
        }

        public string GetLanguageInstruction()
        {
            return _problem?.LanguageInstruction ?? "";
        }

        private void Update()
        {
            if (UseJoyCon && JoyConInput != null && JoyConInput.HasData)
            {
                // 热身延迟：等待 60 帧确保机械臂完全抬升到 HomePose
                if (_warmupFrames < 5)
                {
                    _warmupFrames++;
                    return;
                }

                // Disable keyboard control to avoid conflicting with JoyCon
                DisableKeyboardControllers();
                ProcessJoyConInput();
            }
        }

        private void ProcessJoyConInput()
        {
            if (Robot != null)
                ApplyJoyConToRobot(0, Robot);
            if (Robot2 != null)
                ApplyJoyConToRobot(1, Robot2);
        }

        private void DisableKeyboardControllers()
        {
            var kbds = FindObjectsOfType<KeyboardController>();
            foreach (var kbd in kbds)
            {
                if (kbd.enabled)
                {
                    kbd.enabled = false;
                    Debug.Log("[LiberoEnvironment] KeyboardController disabled (JoyCon active)");
                }
            }
        }

        private Quaternion _jcHomeRot0;
        private Quaternion _jcHomeRot1;
        private Quaternion _jcInitialJoyConRot0;
        private Quaternion _jcInitialJoyConRot1;
        public float JoyConRotSlerpSpeed = 15f;

        private void ApplyJoyConToRobot(int robotIndex, RobotArmController controller)
        {
            JoyConPose jc = JoyConInput.GetRobotPose(robotIndex);

            // ── Joint-angle direct-drive mode (from joycon_sender.py) ──
            if (jc.joints != null && jc.joints.Length >= 5)
            {
                // joints = [base_yaw, J2, J3, J4, J5, gripper] in radians
                Debug.Log($"[JoyCon] Robot{robotIndex} joints received (rad): " +
                    $"[{jc.joints[0]:F3}, {jc.joints[1]:F3}, {jc.joints[2]:F3}, {jc.joints[3]:F3}, {jc.joints[4]:F3}], gripper={jc.gripper:F2}");
                float[] positionsDeg = new float[Mathf.Min(jc.joints.Length, controller.ArmJointCount)];
                for (int i = 0; i < positionsDeg.Length; i++)
                    positionsDeg[i] = jc.joints[i] * Mathf.Rad2Deg;
                Debug.Log($"[JoyCon] Robot{robotIndex} positionsDeg: [{string.Join(", ", positionsDeg.Select(v => $"{v:F1}°"))}]");
                controller.SetJointPositions(positionsDeg);
                Debug.Log($"[JoyCon] {jc.joints.Length} {controller.ArmJointCount}");
                if (jc.joints.Length > controller.ArmJointCount)
                    controller.SetGripper(jc.joints[controller.ArmJointCount]);
                // Send current joint angles back to Python for closed-loop IK seeding
                controller.GetJointPositions(out float[] curJointsRad);
                JoyConInput?.SetJointFeedback(robotIndex, curJointsRad);
                return;
            }

            // ── EEF pose mode (from joycon_bridge.py) ──
            if (jc.pos == null || jc.pos.Length < 3)
                return;
            if (controller.EEFTransform == null)
                return;

            // Joy-Con frame:  X=forward, Y=left, Z=up  (units: metres)
            // Unity frame:    X=right,   Y=up,   Z=forward
            float jcX = jc.pos[0];  // forward
            float jcY = jc.pos[1];  // left
            float jcZ = jc.pos[2];  // up

            // Convert Joy-Con pos → Unity pos
            //  Joy(X前,Y左,Z上) → Unity(X右,Y上,Z前) = Joy(Y, Z, X)
            Vector3 joyConPosUnity = new Vector3(jcY, jcZ, jcX);

            // ── EEF orientation from Joy-Con tilt ────────────────
            // jc.rot = [roll, pitch, 0.0] in radians (Joy-Con frame, already transformed by Python)
            // Joy-Con:  roll=Z, pitch=Y, yaw=X  (forward=Z)
            // Unity:    roll=Z, pitch=X, yaw=Y  (forward=Z)
            float jcRoll = jc.rot != null && jc.rot.Length >= 3 ? jc.rot[0] : 0f;
            float jcPitch = jc.rot != null && jc.rot.Length >= 3 ? jc.rot[1] : 0f;
            // jc.rot[2] is always 0.0 — yaw handled by base rotation J0

            // Build Unity rotation: Joy-Con pitch(Y) → Unity pitch(X), Joy-Con roll(Z) → Unity roll(Z)
            Quaternion rawJoyConRot = Quaternion.Euler(
                jcPitch * Mathf.Rad2Deg,  // Unity X-pitch ← Joy-Con Y-pitch
                0f,                        // Unity Y-yaw = 0 (base yaw drives J0)
                -jcRoll * Mathf.Rad2Deg    // Unity Z-roll ← Joy-Con Z-roll
            );

            // Base yaw: absolute Madgwick yaw (rad → deg), driven directly to J0
            float baseYawDeg = jc.baseYaw * Mathf.Rad2Deg;

            // ── Calibration: capture initial EEF pose on first frame ──
            if (robotIndex == 0)
            {
                if (!_jcCalibrated0)
                {
                    _jcHomePos0 = controller.EEFTransform.position;
                    _jcHomeRot0 = controller.EEFTransform.rotation;
                    _jcHomeJoyConPos0 = joyConPosUnity;
                    _jcHomeBaseYaw0 = baseYawDeg;
                    _jcInitialJoyConRot0 = rawJoyConRot;
                    _jcCalibrated0 = true;
                    Debug.Log($"[JoyCon] Robot0 calibrated. EEF home = {_jcHomePos0}, rot={_jcHomeRot0.eulerAngles}, JoyConRot={_jcInitialJoyConRot0.eulerAngles}, baseYaw={baseYawDeg:F1}°");
                    return; // skip first frame — wait for valid delta
                }

                // Position: delta from calibration home, rotated into base-local frame
                Vector3 deltaPos = joyConPosUnity - _jcHomeJoyConPos0;
                float currentBaseDeg = baseYawDeg - _jcHomeBaseYaw0;
                Vector3 worldDelta = Quaternion.Euler(0, -currentBaseDeg, 0) * deltaPos;
                Vector3 targetPos = _jcHomePos0 + worldDelta * JoyConPosScale;

                // Rotation: Joy-Con tilt delta applied to home EEF rotation
                Quaternion joyConDelta = rawJoyConRot * Quaternion.Inverse(_jcInitialJoyConRot0);
                Quaternion targetRot = joyConDelta * _jcHomeRot0;

                // Base yaw delta: drive J0 directly
                float targetBaseDeg = currentBaseDeg;

                controller.ServoTowardPoseWithBase(targetPos, targetRot, targetBaseDeg, 0, 0.35f, 0.2f, 0.25f);
                controller.SetGripper(jc.gripper);
            }
            else
            {
                if (!_jcCalibrated1)
                {
                    _jcHomePos1 = controller.EEFTransform.position;
                    _jcHomeRot1 = controller.EEFTransform.rotation;
                    _jcHomeJoyConPos1 = joyConPosUnity;
                    _jcHomeBaseYaw1 = baseYawDeg;
                    _jcInitialJoyConRot1 = rawJoyConRot;
                    _jcCalibrated1 = true;
                    Debug.Log($"[JoyCon] Robot1 calibrated. EEF home = {_jcHomePos1}, rot={_jcHomeRot1.eulerAngles}, JoyConRot={_jcInitialJoyConRot1.eulerAngles}, baseYaw={baseYawDeg:F1}°");
                    return;
                }

                Vector3 deltaPos = joyConPosUnity - _jcHomeJoyConPos1;
                float currentBaseDeg = baseYawDeg - _jcHomeBaseYaw1;
                Vector3 worldDelta = Quaternion.Euler(0, -currentBaseDeg, 0) * deltaPos;
                Vector3 targetPos = _jcHomePos1 + worldDelta * JoyConPosScale;

                Quaternion joyConDelta = rawJoyConRot * Quaternion.Inverse(_jcInitialJoyConRot1);
                Quaternion targetRot = joyConDelta * _jcHomeRot1;

                float targetBaseDeg = currentBaseDeg;

                controller.ServoTowardPoseWithBase(targetPos, targetRot, targetBaseDeg, 0, 0.35f, 0.2f, 0.25f);
                controller.SetGripper(jc.gripper);
            }
        }

        private void OnDestroy()
        {
            _isInitialized = false;
        }

        private void OnDrawGizmos()
        {
            if (_sceneBuilder?.ObjectStates == null) return;

            Gizmos.color = Color.green;
            foreach (var kv in _sceneBuilder.ObjectStates)
            {
                Gizmos.DrawWireSphere(kv.Value.Position, 0.05f);
            }

            Gizmos.color = Color.blue;
            foreach (var kv in _sceneBuilder.FixtureStates)
            {
                Gizmos.DrawWireCube(kv.Value.Position, Vector3.one * 0.1f);
            }

            Gizmos.color = Color.red;
            foreach (var objName in _problem?.ObjOfInterest ?? new List<string>())
            {
                var state = GetObjectState(objName);
                if (state != null)
                    Gizmos.DrawWireSphere(state.Position, 0.08f);
            }
        }
    }

    public static class SceneBuilderExtensions
    {
        public static ObjectState GetObjectState(this SceneBuilder builder, string objectName)
        {
            if (builder.ObjectStates.TryGetValue(objectName, out var objState))
                return objState;
            if (builder.FixtureStates.TryGetValue(objectName, out var fixState))
                return fixState;
            return null;
        }
    }

    internal static class TransformExtensions
    {
        public static Transform FindRecursive(this Transform parent, string name)
        {
            if (parent.name == name) return parent;
            for (int i = 0; i < parent.childCount; i++)
            {
                var found = parent.GetChild(i).FindRecursive(name);
                if (found != null) return found;
            }
            return null;
        }
    }
}
