using UnityEngine;

namespace UnityRobotEnv.Core
{
    public class CameraSwitcher : MonoBehaviour
    {
        [System.Serializable]
        public struct Preset
        {
            public string name;
            public Vector3 position;
            public Quaternion rotation;
        }

        public Preset[] presets = new Preset[]
        {
            new Preset { name = "俯瞰", position = new Vector3(0.5f, 1.8f, -1.5f), rotation = Quaternion.LookRotation(new Vector3(0, 0.4f, -0.3f) - new Vector3(0.5f, 1.8f, -1.5f)) },
            new Preset { name = "侧面", position = new Vector3(1.5f, 0.6f, 0.2f),  rotation = Quaternion.LookRotation(new Vector3(0, 0.4f, -0.3f) - new Vector3(1.5f, 0.6f, 0.2f)) },
            new Preset { name = "顶部", position = new Vector3(0, 3f, -0.3f),    rotation = Quaternion.LookRotation(new Vector3(0, 0.4f, -0.3f) - new Vector3(0, 3f, -0.3f)) },
        };

        private int _index;

        void Start()
        {
            if (presets.Length > 0)
                ApplyPreset(0);
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.Alpha8)) ApplyPreset(0);
            if (Input.GetKeyDown(KeyCode.Alpha9)) ApplyPreset(1);
            if (Input.GetKeyDown(KeyCode.Alpha0)) ApplyPreset(2);
        }

        void ApplyPreset(int i)
        {
            if (i >= presets.Length) return;
            _index = i;
            var cam = GetComponent<Camera>();
            cam.transform.position = presets[i].position;
            cam.transform.rotation = presets[i].rotation;
            Debug.Log($"[Camera] {presets[i].name} (key {i + 1})");
        }
    }
}
