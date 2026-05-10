using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        public FrankaPandaController Robot;
        public ObservationCollector ObsCollector;

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

        public BDDL.BDDLProblem Problem => _problem;
        public SceneBuilder Scene => _sceneBuilder;
        public bool IsDone { get; private set; }
        public float LastReward { get; private set; }

        [ContextMenu("Initialize Scene")]
        public void InitializeScene()
        {
            ClearScene();
            Initialize(BDDLFilePath);
        }

        private void ClearScene()
        {
            if (ArenaRoot != null) DestroyImmediate(ArenaRoot);
            if (ObjectParent != null) DestroyImmediate(ObjectParent);
            ArenaRoot = null;
            ObjectParent = null;
            _sceneBuilder = null;
            _isInitialized = false;
        }

        private void Start()
        {
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
            if (Camera.main != null) DestroyImmediate(Camera.main.gameObject);

            var cameraGo = new GameObject("Main Camera");
            var cam = cameraGo.AddComponent<Camera>();
            cameraGo.tag = "MainCamera";
            cameraGo.AddComponent<CameraSwitcher>();
            Debug.Log("Auto-created Main Camera with CameraSwitcher");
        }

        private void BuildRobot()
        {
            GameObject panda = null;
            ArticulationBody[] joints = null;
            Transform eef = null;
            ArticulationBody leftFinger = null, rightFinger = null;

            // Try URDF prefab first
            var prefab = Resources.Load<GameObject>("Robots/Panda");
            if (prefab != null)
            {
                panda = Object.Instantiate(prefab);
                panda.name = "Panda";

                // Configure all ArticulationBodies
                var allABs = panda.GetComponentsInChildren<ArticulationBody>();
                // Find true root AB (URDF Importer puts it on a child, not root GO)
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
                // Fallback: build from URDF manually
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

            // Mount (for tabletop scenes)
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

            Robot.Joints = joints;
            Robot.EEFTransform = eef;
            Robot.LeftFinger = leftFinger;
            Robot.RightFinger = rightFinger;
            Robot.InitializeJoints();

            Physics.SyncTransforms();
            var rootAb2 = FindRootArticulationBody(panda);
            if (rootAb2 != null)
                rootAb2.TeleportRoot(panda.transform.position, panda.transform.rotation);

            if (panda.GetComponent<KeyboardController>() == null)
            {
                var kbd = panda.AddComponent<KeyboardController>();
                kbd.Robot = Robot;
            }

            Debug.Log($"[LiberoEnvironment] Robot built at {panda.transform.localPosition}, "
                + $"joints={joints?.Length}, eef={eef != null}");
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

        public Observation Reset()
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

            // Let physics settle
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
