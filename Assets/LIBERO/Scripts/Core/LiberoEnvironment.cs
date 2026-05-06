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
            if (Camera.main != null) return;

            var cameraGo = new GameObject("Main Camera");
            var cam = cameraGo.AddComponent<Camera>();
            cameraGo.tag = "MainCamera";
            cameraGo.transform.position = new Vector3(1.5f, 1.5f, 1.5f);
            cameraGo.transform.LookAt(Vector3.zero);
            Debug.Log("Auto-created Main Camera");
        }

        private void BuildRobot()
        {
            string robotXmlPath = AssetDatabase.GetAssetPath("robots/panda/robot.xml");
            if (!System.IO.File.Exists(robotXmlPath))
            {
                Debug.LogWarning("Robot XML not found: " + robotXmlPath);
                return;
            }

            var result = RobotBuilder.BuildArm(robotXmlPath);
            if (result.Root == null) return;

            result.Root.name = "Panda";
            result.Root.transform.SetParent(ArenaRoot.transform);
            result.Root.transform.localPosition = GetRobotBasePosition();

            // Mount (for tabletop scenes)
            if (ArenaTypeValue != ArenaType.Floor &&
                ArenaTypeValue != ArenaType.CoffeeTable &&
                ArenaTypeValue != ArenaType.LivingRoom)
            {
                string mountPath = AssetDatabase.GetAssetPath("mounts/rethink_mount.xml");
                if (System.IO.File.Exists(mountPath))
                {
                    Transform mountParent = result.Root.transform.Find("base") ?? result.Root.transform;
                    RobotBuilder.AttachMount(mountParent.gameObject, mountPath);
                }
            }

            // Gripper
            if (result.EEFTransform != null)
            {
                string gripperPath = AssetDatabase.GetAssetPath("grippers/panda_gripper.xml");
                if (System.IO.File.Exists(gripperPath))
                    RobotBuilder.AttachGripper(result.EEFTransform, gripperPath, ref result);
            }

            if (Robot == null)
                Robot = result.Root.AddComponent<FrankaPandaController>();

            Robot.Joints = result.Joints;
            Robot.EEFTransform = result.GripSite ?? result.EEFTransform;
            Robot.LeftFinger = result.LeftFinger;
            Robot.RightFinger = result.RightFinger;
            Robot.InitializeJoints();

            Physics.SyncTransforms();
            var rootAb = result.Root.GetComponent<ArticulationBody>();
            if (rootAb != null)
                rootAb.TeleportRoot(result.Root.transform.position, result.Root.transform.rotation);

            Debug.Log($"[LiberoEnvironment] Robot built at {result.Root.transform.localPosition}");
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
}
