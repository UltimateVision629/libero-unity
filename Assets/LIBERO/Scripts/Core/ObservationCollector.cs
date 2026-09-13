using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Mujoco;

namespace LIBERO.Core
{
    public struct Observation
    {
        public byte[] AgentviewImage;
        public byte[] EyeInHandImage;
        public int ImageWidth;
        public int ImageHeight;

        public float[] JointPositions;   // robot_0 (right arm)
        public float[] EEFPosition;
        public float[] EEFQuaternion;
        public float[] GripperQPos;

        public float[] JointPositions1;  // robot_1 (left arm)
        public float[] EEFPosition1;
        public float[] EEFQuaternion1;
        public float[] GripperQPos1;

        public Dictionary<string, float[]> ObjectPositions;
        public Dictionary<string, float[]> ObjectQuaternions;

        public Dictionary<string, object> ToPythonDict()
        {
            var dict = new Dictionary<string, object>
            {
                ["agentview_image"] = AgentviewImage,
                ["eye_in_hand_image"] = EyeInHandImage,
                ["robot0_joint_pos"] = JointPositions,
                ["robot0_eef_pos"] = EEFPosition,
                ["robot0_eef_quat"] = EEFQuaternion,
                ["robot0_gripper_qpos"] = GripperQPos,
                ["robot1_joint_pos"] = JointPositions1,
                ["robot1_eef_pos"] = EEFPosition1,
                ["robot1_eef_quat"] = EEFQuaternion1,
                ["robot1_gripper_qpos"] = GripperQPos1
            };

            if (ObjectPositions != null)
            {
                foreach (var kv in ObjectPositions)
                    dict[$"{kv.Key}_pos"] = kv.Value;
            }
            if (ObjectQuaternions != null)
            {
                foreach (var kv in ObjectQuaternions)
                    dict[$"{kv.Key}_quat"] = kv.Value;
            }

            return dict;
        }
    }

    public class ObservationCollector : MonoBehaviour
    {
        /// <summary>
        /// 播放器必须持续跑主循环，否则失焦时 Unity 暂停、Update() 不再被调用 →
        /// TCP 命令（reset/get_obs/step）全部 30s 超时（2026-09-09 诊断：主线程
        /// wchan=hrtimer_nanosleep、~10% CPU，xdotool 聚焦后命令 0.00–0.53s 秒回）。
        /// 无头服务器/批处理没有窗口可聚焦，必须靠这个开关；此前靠 xdotool
        /// keep-focus 循环掩盖，属于外部依赖。
        /// </summary>
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void ConfigurePlayer()
        {
            Application.runInBackground = true;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<ObservationCollector>() != null) return;
            var go = new GameObject("ObservationCollector");
            DontDestroyOnLoad(go);
            go.AddComponent<ObservationCollector>();
        }

        [Header("Camera Settings")]
        public Camera AgentviewCamera;
        public Camera EyeInHandCamera;
        public int ImageWidth = 224;
        public int ImageHeight = 224;

        [Header("Robot References")]
        public GameObject RobotRoot;
        public ArticulationBody[] JointBodies;
        public RobotArmController Robot0;
        public RobotArmController Robot1;

        // Two-stage rendering: render at camera's native aspect ratio (16:9),
        // then letterbox into square output to preserve full horizontal FOV.
        private RenderTexture _agentviewRenderRT;
        private RenderTexture _eyeInHandRenderRT;
        private Texture2D _agentviewRenderTex;
        private Texture2D _eyeInHandRenderTex;
        private Texture2D _agentviewTex;
        private Texture2D _eyeInHandTex;
        private int _renderHeight;

        private void Awake()
        {
            if (AgentviewCamera == null)
                CreateAgentviewCamera();

            // 固定 16:9，不取屏幕宽高比：训练数据与 v2 均为 224×126。4:3 屏幕会让
            // _renderHeight 变成 168、水平 FOV 从 91.5° 缩到 75.2°，推理画面被放大 1.22×
            //（2026-09-09 诊断的 v3 FOV bug）。
            const float kAgentviewAspect = 16f / 9f;
            if (AgentviewCamera != null)
                AgentviewCamera.aspect = kAgentviewAspect;
            _renderHeight = Mathf.RoundToInt(ImageWidth / kAgentviewAspect);

            // 16:9 render targets
            _agentviewRenderRT = new RenderTexture(ImageWidth, _renderHeight, 24, RenderTextureFormat.ARGB32);
            _agentviewRenderTex = new Texture2D(ImageWidth, _renderHeight, TextureFormat.RGB24, false);
            _eyeInHandRenderRT = new RenderTexture(ImageWidth, _renderHeight, 24, RenderTextureFormat.ARGB32);
            _eyeInHandRenderTex = new Texture2D(ImageWidth, _renderHeight, TextureFormat.RGB24, false);

            // Square output textures (letterboxed)
            _agentviewTex = new Texture2D(ImageWidth, ImageWidth, TextureFormat.RGB24, false);
            _eyeInHandTex = new Texture2D(ImageWidth, ImageWidth, TextureFormat.RGB24, false);

            // 构建版本不会自动更新天空盒环境光探针（SkyManager 只处理编辑器打开过的场景），
            // 不调用会让画面比采集数据暗 25–30 灰度级 → 训练/推理输入分布不一致
            //（2026-09-09 诊断：编辑器采集 mean≈129 vs 构建渲染 mean≈100）。
            DynamicGI.UpdateEnvironment();
        }

        private GameObject _siteGo;
        private bool _siteLogged;
        private Vector3 _sitePosLogged;

        private void CreateAgentviewCamera()
        {
            var camGo = new GameObject("Agentview Camera");
            camGo.transform.SetParent(transform);
            var cam = camGo.AddComponent<Camera>();
            cam.nearClipPlane = 0.1f;
            cam.enabled = true;
            camGo.transform.position = new Vector3(0f, 0.50f, -0.70f);   // site 就绪前的兜底
            camGo.transform.LookAt(Vector3.zero);   // 目视目标固定为原点（前方），不随桌移动

            // 主视角：agentview 作为 MainCamera + 最高 depth（最后渲染 → Game 视图显示）。
            // 场景 Main Camera（CameraSwitcher 8/9/0 视角）让出 tag，仍可切换查看。
            foreach (var oldCam in Camera.allCameras)
                if (oldCam != cam && oldCam.tag == "MainCamera")
                    oldCam.tag = "Untagged";
            cam.tag = "MainCamera";
            cam.depth = 2;

            AgentviewCamera = cam;
            _siteGo = null;
            _siteLogged = false;
        }

        /// <summary>
        /// 每帧把相机同步到 agentview_site（XML 定义的相机位）。site 在模型加载后
        /// 才创建、transform 在物理步进后才同步——一次性放置容易踩时序（postInit
        /// 已触发 / 变换未同步），导致相机停在兜底位置；逐帧同步保证 XML 里的相机
        /// 改动一定生效。
        /// 日志：位置首次同步或变化 &gt;10cm 时打印一次，Unity 坐标 (x, y, z)。
        /// 若打印值不是最新 XML（(0, 0.4, -1.1)），说明 Unity 没重新导入 XML。
        /// </summary>
        private void Update()
        {
            if (AgentviewCamera == null) return;
            if (_siteGo == null)
                _siteGo = FindAgentviewSite();
            if (_siteGo == null || _siteGo.transform.position == Vector3.zero)
                return;   // 模型未加载或变换未同步，等下一帧

            AgentviewCamera.transform.position = _siteGo.transform.position;
            AgentviewCamera.transform.rotation = _siteGo.transform.rotation;
            AgentviewCamera.transform.LookAt(Vector3.zero);

            if (!_siteLogged || Vector3.Distance(_siteGo.transform.position, _sitePosLogged) > 0.1f)
            {
                _siteLogged = true;
                _sitePosLogged = _siteGo.transform.position;
                Debug.Log($"[ObsCollector] Camera synced to agentview_site: {_siteGo.transform.position}");
            }

            UpdateEyeInHandCamera();
        }

        private GameObject FindAgentviewSite()
        {
            // 前缀匹配：MuJoCo 导入的 GameObject 名可能带 _NNN 后缀，精确名
            // GameObject.Find 会失败 → 相机落到兜底位置，XML 相机改动不生效。
            foreach (var site in FindObjectsOfType<MjSite>())
                if (site.MujocoName != null && site.MujocoName.StartsWith("agentview_site"))
                    return site.gameObject;
            return GameObject.Find("agentview_site");
        }

        // ── Eye-in-hand camera（2026-08-17，新场景 libero_put_block_in_box）───────────
        // 懒创建：找到 XML 的 wrist_cam_site 才创建相机。每帧把相机放在
        // R_eef_site 的【世界坐标系正上方 6cm】（不依赖 XML 局部系换算——局部偏移
        // 经基座/Fixed_Jaw 两级 euler 旋转后方向不可靠，实测出现在爪侧方），
        // 朝向 = wrist_cam_site 的 rotation（euler 已把 +Z 指向爪延伸方向，画面 =
        // 爪下部 + 前方目标）。爪子动 → eef 位置实时更新 → 相机贴爪跟随。
        // 旧场景无该 site → 永不创建，行为与之前完全一致。
        private GameObject _wristSiteGo;
        private GameObject _wristEefGo;
        private bool _wristCamLogged;
        private Vector3 _wristPosLogged;

        private void UpdateEyeInHandCamera()
        {
            if (_wristSiteGo == null)
                _wristSiteGo = FindSiteGo("wrist_cam_site");
            if (_wristSiteGo == null || _wristSiteGo.transform.position == Vector3.zero)
                return;   // 旧场景/模型未就绪 → 不创建

            if (EyeInHandCamera == null)
                CreateEyeInHandCamera();
            if (_wristEefGo == null)
                _wristEefGo = FindSiteGo("R_eef_site");
            if (_wristEefGo == null)
                return;

            EyeInHandCamera.transform.position = _wristEefGo.transform.position
                                                 + Vector3.up * 0.06f;   // 世界系正上方 6cm
            // 朝向 = site（+Z 朝爪前方）+ 固定俯角 25°（先按 site 朝向再下倾，
            // 任何爪姿态都成立；实测 -25° 为仰视、+25° 为俯视，想调俯视程度改 25f）
            EyeInHandCamera.transform.rotation = _wristSiteGo.transform.rotation
                                                 * Quaternion.Euler(25f, 0, 0);

            if (!_wristCamLogged || Vector3.Distance(_wristSiteGo.transform.position, _wristPosLogged) > 0.1f)
            {
                _wristCamLogged = true;
                _wristPosLogged = _wristSiteGo.transform.position;
                Debug.Log($"[ObsCollector] Wrist camera synced to wrist_cam_site: {_wristSiteGo.transform.position}");
            }
        }

        private GameObject FindSiteGo(string prefix)
        {
            foreach (var site in FindObjectsOfType<MjSite>())
                if (site.MujocoName != null && site.MujocoName.StartsWith(prefix))
                    return site.gameObject;
            return GameObject.Find(prefix);
        }

        private void CreateEyeInHandCamera()
        {
            var camGo = new GameObject("Wrist Camera");
            camGo.transform.SetParent(transform);
            var cam = camGo.AddComponent<Camera>();
            cam.nearClipPlane = 0.01f;   // 爪尖很近，小近平面
            cam.enabled = true;

            // 右上角小窗（2026-08-17）：wrist 相机渲染进小 RenderTexture →
            // Game 视图右上角 RawImage 实时显示；相机不进主画面（主画面 = agentview）。
            // obs 采集时 CaptureAndLetterbox 临时切换 targetTexture 再恢复，互不冲突。
            _wristPreviewRT = new RenderTexture(ImageWidth, _renderHeight, 24, RenderTextureFormat.ARGB32);
            _wristPreviewRT.Create();
            cam.targetTexture = _wristPreviewRT;
            cam.depth = 0;

            // Canvas（ScreenSpaceOverlay，参照 ResetButton 的运行时创建模式）
            var canvasGo = new GameObject("WristPreviewCanvas",
                typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            canvasGo.transform.SetParent(transform, false);
            var canvas = canvasGo.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = canvasGo.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);

            // RawImage：右上角小窗
            var imgGo = new GameObject("WristPreview", typeof(RectTransform), typeof(RawImage));
            imgGo.transform.SetParent(canvasGo.transform, false);
            var rt = imgGo.GetComponent<RectTransform>();
            rt.anchorMin = new Vector2(1f, 1f);   // top-right
            rt.anchorMax = new Vector2(1f, 1f);
            rt.pivot = new Vector2(1f, 1f);
            rt.anchoredPosition = new Vector2(-20f, -20f);
            rt.sizeDelta = new Vector2(224f, 126f);
            _wristPreviewRawImage = imgGo.GetComponent<RawImage>();
            _wristPreviewRawImage.texture = _wristPreviewRT;

            EyeInHandCamera = cam;
        }

        private RenderTexture _wristPreviewRT;
        private RawImage _wristPreviewRawImage;

        /// <summary>显示/隐藏右上角腕部小窗（相机与 RT 恒在，obs 采集不受影响）。</summary>
        public void SetWristPreview(bool show)
        {
            if (_wristPreviewRawImage != null)
                _wristPreviewRawImage.enabled = show;
        }

        public Observation Collect(Dictionary<string, ObjectState> objectStates, RobotArmController robot)
        {
            var obs = new Observation
            {
                // Output is square (letterboxed)
                ImageWidth = ImageWidth,
                ImageHeight = ImageWidth,
                ObjectPositions = new Dictionary<string, float[]>(),
                ObjectQuaternions = new Dictionary<string, float[]>()
            };

            // Capture camera images: render at 16:9, letterbox to square
            if (AgentviewCamera != null)
                obs.AgentviewImage = CaptureAndLetterbox(AgentviewCamera, _agentviewRenderRT,
                    _agentviewRenderTex, _agentviewTex);

            if (EyeInHandCamera != null)
                obs.EyeInHandImage = CaptureAndLetterbox(EyeInHandCamera, _eyeInHandRenderRT,
                    _eyeInHandRenderTex, _eyeInHandTex);

            // Robot proprioception — use passed robot or public fields, fall back to MuJoCo
            var r0 = robot ?? Robot0;
            var r1 = Robot1;

            if (r0 != null)
            {
                r0.GetJointPositions(out float[] jointPositions);
                r0.GetEEFPose(out Vector3 eefPos, out Quaternion eefQuat);
                r0.GetGripperState(out float[] gripperQPos);

                obs.JointPositions = jointPositions;
                obs.EEFPosition = new float[] { eefPos.x, eefPos.y, eefPos.z };
                obs.EEFQuaternion = new float[] { eefQuat.x, eefQuat.y, eefQuat.z, eefQuat.w };
                obs.GripperQPos = gripperQPos;
            }

            if (r1 != null)
            {
                r1.GetJointPositions(out float[] jointPositions1);
                r1.GetEEFPose(out Vector3 eefPos1, out Quaternion eefQuat1);
                r1.GetGripperState(out float[] gripperQPos1);

                obs.JointPositions1 = jointPositions1;
                obs.EEFPosition1 = new float[] { eefPos1.x, eefPos1.y, eefPos1.z };
                obs.EEFQuaternion1 = new float[] { eefQuat1.x, eefQuat1.y, eefQuat1.z, eefQuat1.w };
                obs.GripperQPos1 = gripperQPos1;
            }

            if (r0 == null && r1 == null)
            {
                var mjJoy = FindObjectOfType<MjJoyConController>();
                if (mjJoy != null)
                {
                    if (mjJoy.TryGetRobotState(0, out float[] j0, out Vector3 p0, out Quaternion q0))
                    {
                        obs.JointPositions = j0;
                        obs.EEFPosition = new float[] { p0.x, p0.y, p0.z };
                        obs.EEFQuaternion = new float[] { q0.x, q0.y, q0.z, q0.w };
                        obs.GripperQPos = new float[] { j0.Length > 0 ? j0[j0.Length - 1] : 0f, 0f };
                        Debug.Log($"[ObsCollector] Robot0 OK: joints={j0.Length}, eef=({p0.x:F3},{p0.y:F3},{p0.z:F3})");
                    }
                    if (mjJoy.TryGetRobotState(1, out float[] j1, out Vector3 p1, out Quaternion q1))
                    {
                        obs.JointPositions1 = j1;
                        obs.EEFPosition1 = new float[] { p1.x, p1.y, p1.z };
                        obs.EEFQuaternion1 = new float[] { q1.x, q1.y, q1.z, q1.w };
                        obs.GripperQPos1 = new float[] { j1.Length > 0 ? j1[j1.Length - 1] : 0f, 0f };
                        Debug.Log($"[ObsCollector] Robot1 OK: joints={j1.Length}, eef=({p1.x:F3},{p1.y:F3},{p1.z:F3})");
                    }
                }
            }

            // Object states
            if (objectStates != null)
            {
                foreach (var kv in objectStates)
                {
                    var pos = kv.Value.Position;
                    var rot = kv.Value.Rotation;
                    obs.ObjectPositions[kv.Key] = new float[] { pos.x, pos.y, pos.z };
                    obs.ObjectQuaternions[kv.Key] = new float[] { rot.x, rot.y, rot.z, rot.w };
                }
            }

            return obs;
        }

        /// <summary>
        /// Render a camera at its native aspect ratio (e.g. 16:9) into a render
        /// texture, then letterbox the result into a square output texture with
        /// black bars on top and bottom. Preserves the full horizontal FOV.
        /// </summary>
        private byte[] CaptureAndLetterbox(Camera cam, RenderTexture renderRT,
            Texture2D renderTex, Texture2D outputTex)
        {
            // Step 1: Render camera at native aspect ratio
            var prevTarget = cam.targetTexture;
            cam.targetTexture = renderRT;
            cam.Render();

            // Step 2: Read 16:9 pixels from render texture
            RenderTexture.active = renderRT;
            renderTex.ReadPixels(new Rect(0, 0, renderRT.width, renderRT.height), 0, 0);
            renderTex.Apply();
            RenderTexture.active = null;
            cam.targetTexture = prevTarget;

            // Step 3: Letterbox into square output (black bars top & bottom)
            int outSize = outputTex.width;
            int inWidth = renderRT.width;
            int inHeight = renderRT.height;
            int yOffset = (outSize - inHeight) / 2;

            // Fill with black
            var black = new Color[outSize * outSize];
            for (int i = 0; i < black.Length; i++)
                black[i] = Color.black;
            outputTex.SetPixels(black);

            // Copy rendered content into the center band
            outputTex.SetPixels(0, yOffset, inWidth, inHeight, renderTex.GetPixels());
            outputTex.Apply();

            return outputTex.GetRawTextureData();
        }

        private void OnDestroy()
        {
            if (_agentviewRenderRT != null) _agentviewRenderRT.Release();
            if (_eyeInHandRenderRT != null) _eyeInHandRenderRT.Release();
            if (_wristPreviewRT != null) _wristPreviewRT.Release();
            if (_agentviewRenderTex != null) Destroy(_agentviewRenderTex);
            if (_eyeInHandRenderTex != null) Destroy(_eyeInHandRenderTex);
            if (_agentviewTex != null) Destroy(_agentviewTex);
            if (_eyeInHandTex != null) Destroy(_eyeInHandTex);
        }
    }
}
