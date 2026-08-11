using LIBERO.Networking;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace LIBERO.UI
{
    /// <summary>
    /// Runtime-created "重置" (Reset) button pinned to the top-left of the Game view.
    /// Clicking it triggers the same MuJoCo scene reset as the TCP reset command
    /// (TrainingServer.ResetScene: mj_resetData + both arms back to home).
    /// Auto-created via [RuntimeInitializeOnLoadMethod] like the other bootstrap
    /// components (TrainingServer / JoyConReceiver / MjJoyConController) — no
    /// scene assets required.
    /// </summary>
    public class ResetButton : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<ResetButton>() != null)
                return;
            var go = new GameObject("ResetButton");
            go.AddComponent<ResetButton>();
            DontDestroyOnLoad(go);
        }

        private void Awake()
        {
            // ── Canvas (Screen Space Overlay) ────────────────────────────
            var canvasGo = new GameObject(
                "ResetCanvas", typeof(Canvas), typeof(CanvasScaler), typeof(GraphicRaycaster));
            canvasGo.transform.SetParent(transform, false);
            var canvas = canvasGo.GetComponent<Canvas>();
            canvas.renderMode = RenderMode.ScreenSpaceOverlay;
            var scaler = canvasGo.GetComponent<CanvasScaler>();
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ScaleWithScreenSize;
            scaler.referenceResolution = new Vector2(1920f, 1080f);

            // ── EventSystem (uGUI clicks require it) ─────────────────────
            if (FindObjectOfType<EventSystem>() == null)
            {
                var esGo = new GameObject(
                    "EventSystem", typeof(EventSystem), typeof(StandaloneInputModule));
                esGo.transform.SetParent(transform, false);
            }

            // ── Button (top-left corner) ─────────────────────────────────
            var btnGo = new GameObject(
                "ResetButton", typeof(RectTransform), typeof(Image), typeof(Button));
            btnGo.transform.SetParent(canvasGo.transform, false);
            var rt = btnGo.GetComponent<RectTransform>();
            rt.anchorMin = new Vector2(0f, 1f);   // top-left
            rt.anchorMax = new Vector2(0f, 1f);
            rt.pivot = new Vector2(0f, 1f);
            rt.anchoredPosition = new Vector2(20f, -20f);
            rt.sizeDelta = new Vector2(120f, 40f);

            var img = btnGo.GetComponent<Image>();
            img.color = new Color(0f, 0f, 0f, 0.6f);

            var btn = btnGo.GetComponent<Button>();
            btn.onClick.AddListener(OnResetClicked);

            // ── Label ────────────────────────────────────────────────────
            var textGo = new GameObject("Text", typeof(RectTransform), typeof(Text));
            textGo.transform.SetParent(btnGo.transform, false);
            var trt = textGo.GetComponent<RectTransform>();
            trt.anchorMin = Vector2.zero;
            trt.anchorMax = Vector2.one;
            trt.offsetMin = Vector2.zero;
            trt.offsetMax = Vector2.zero;
            var text = textGo.GetComponent<Text>();
            text.text = "重置";
            text.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            text.fontSize = 20;
            text.alignment = TextAnchor.MiddleCenter;
            text.color = Color.white;
        }

        private void OnResetClicked()
        {
            var ts = FindObjectOfType<TrainingServer>();
            if (ts != null)
            {
                ts.ResetScene();
                Debug.Log("[ResetButton] Scene reset requested.");
            }
            else
            {
                Debug.LogWarning("[ResetButton] TrainingServer not found — scene reset skipped.");
            }
        }
    }
}
