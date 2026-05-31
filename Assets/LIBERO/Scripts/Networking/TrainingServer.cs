using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using LIBERO.Core;
using UnityEngine;

namespace LIBERO.Networking
{
    /// <summary>
    /// Lightweight TCP server that exposes LiberoEnvironment for RL/BC training.
    /// Runs on port 5556, separate from JoyConReceiver (5555).
    /// JSON-line protocol: {"cmd":"reset"} / {"cmd":"step","action":[...]} / {"cmd":"get_obs"}
    /// </summary>
    public class TrainingServer : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<TrainingServer>() != null) return;
            var go = new GameObject("TrainingServer");
            DontDestroyOnLoad(go);
            go.AddComponent<TrainingServer>();
        }

        [Header("TCP Settings")]
        public int ListenPort = 5556;
        public bool AutoStart = true;

        [Header("Task (used without LiberoEnvironment)")]
        public string LanguageInstruction = "pick up the red block";

        [Header("References")]
        public LiberoEnvironment Env;

        private TcpListener _listener;
        private TcpClient _client;
        private Thread _recvThread;
        private readonly ConcurrentQueue<string> _msgQueue = new ConcurrentQueue<string>();
        private readonly ConcurrentQueue<string> _replyQueue = new ConcurrentQueue<string>();
        private volatile bool _running;
        private readonly object _sendLock = new object();

        public bool IsConnected { get; private set; }

        void Start()
        {
            if (Env == null)
                Env = FindObjectOfType<LiberoEnvironment>();
            if (AutoStart)
                StartServer();
        }

        public void StartServer()
        {
            if (_running) return;
            _running = true;
            _recvThread = new Thread(ServerLoop)
            {
                IsBackground = true,
                Name = "TrainingServer"
            };
            _recvThread.Start();
        }

        private void ServerLoop()
        {
            try
            {
                _listener = new TcpListener(IPAddress.Loopback, ListenPort);
                _listener.Start();
                Debug.Log($"[TrainingServer] Listening on 127.0.0.1:{ListenPort}");

                while (_running)
                {
                    if (_client == null || !_client.Connected)
                    {
                        try
                        {
                            _client = _listener.AcceptTcpClient();
                            _client.NoDelay = true;
                            IsConnected = true;
                            Debug.Log("[TrainingServer] Python client connected.");
                        }
                        catch (SocketException)
                        {
                            if (!_running) break;
                            Thread.Sleep(500);
                            continue;
                        }
                    }

                    try
                    {
                        using (var stream = _client.GetStream())
                        using (var reader = new StreamReader(stream, Encoding.UTF8))
                        {
                            while (_running && _client.Connected)
                            {
                                string line = reader.ReadLine();
                                if (line == null) break;
                                _msgQueue.Enqueue(line);

                                // Wait for reply from main thread, then send
                                string reply = SpinWaitForReply(5000);
                                if (reply != null)
                                {
                                    lock (_sendLock)
                                    {
                                        byte[] payload = Encoding.UTF8.GetBytes(reply + "\n");
                                        stream.Write(payload, 0, payload.Length);
                                        stream.Flush();
                                    }
                                }
                            }
                        }
                    }
                    catch (IOException)
                    {
                        Debug.LogWarning("[TrainingServer] Python client disconnected.");
                    }

                    CleanupClient();
                    IsConnected = false;
                }
            }
            catch (SocketException ex)
            {
                Debug.LogError($"[TrainingServer] Socket error: {ex.Message}");
            }
            finally
            {
                CleanupClient();
                _listener?.Stop();
            }
        }

        private string SpinWaitForReply(int timeoutMs)
        {
            int waited = 0;
            while (waited < timeoutMs)
            {
                if (_replyQueue.TryDequeue(out string reply))
                    return reply;
                Thread.Sleep(10);
                waited += 10;
            }
            return "{\"error\":\"timeout\"}";
        }

        private void CleanupClient()
        {
            try { _client?.Close(); } catch { }
            _client = null;
        }

        void Update()
        {
            while (_msgQueue.TryDequeue(out string msg))
            {
                string reply = ProcessMessage(msg);
                _replyQueue.Enqueue(reply);
            }
        }

        private string ProcessMessage(string json)
        {
            try
            {
                string cmd = ExtractString(json, "cmd");

                switch (cmd)
                {
                    case "reset":
                        return HandleReset();
                    case "step":
                        float[] action = ExtractFloatArray(json, "action");
                        return HandleStep(action);
                    case "get_obs":
                        return HandleGetObs();
                    case "get_task":
                        return HandleGetTask();
                    default:
                        return $"{{\"error\":\"unknown cmd: {cmd}\"}}";
                }
            }
            catch (Exception ex)
            {
                return $"{{\"error\":\"{ex.Message}\"}}";
            }
        }

        private string HandleReset()
        {
            if (Env != null)
                return ObsToJson(Env.ResetEnvironment());

            // Fallback: collect observation directly (MuJoCo XML import mode)
            return ObsToJson(CollectObsFallback());
        }

        private string HandleStep(float[] action)
        {
            if (Env == null)
            {
                // Stub: no LiberoEnvironment, return current obs with zero reward
                var fallbackObs = CollectObsFallback();
                return "{\"reward\":0.0000,\"done\":false,\"step\":0,\"success\":false,\"obs\":" + ObsToJson(fallbackObs) + "}";
            }

            var (obs, reward, done, info) = Env.Step(action);

            var sb = new StringBuilder();
            sb.Append("{");
            sb.Append($"\"reward\":{reward:F4},");
            sb.Append($"\"done\":{(done ? "true" : "false")},");
            sb.Append($"\"step\":{info.GetValueOrDefault("step", 0)},");
            sb.Append($"\"success\":{((bool)info.GetValueOrDefault("success", false) ? "true" : "false")},");
            sb.Append("\"obs\":");
            sb.Append(ObsToJson(obs));
            sb.Append("}");
            return sb.ToString();
        }

        private string HandleGetObs()
        {
            if (Env != null)
            {
                var method = typeof(LiberoEnvironment).GetMethod("GatherObservation",
                    System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                return ObsToJson((Observation)method.Invoke(Env, null));
            }

            // Fallback: collect observation directly (MuJoCo XML import mode)
            return ObsToJson(CollectObsFallback());
        }

        private Observation CollectObsFallback()
        {
            var collector = FindObjectOfType<ObservationCollector>();
            if (collector == null)
                return new Observation { JointPositions = new float[7], EEFPosition = new float[3], EEFQuaternion = new float[4], GripperQPos = new float[2] };
            return collector.Collect(null, null);
        }

        private string HandleGetTask()
        {
            if (Env != null)
            {
                string lang = Env.GetLanguageInstruction();
                return $"{{\"language_instruction\":\"{EscapeJson(lang)}\"}}";
            }
            return $"{{\"language_instruction\":\"{EscapeJson(LanguageInstruction)}\"}}";
        }

        private string ObsToJson(Observation obs)
        {
            var dict = obs.ToPythonDict();
            var sb = new StringBuilder();
            sb.Append("{");

            // agentview image → base64 PNG
            if (dict.TryGetValue("agentview_image", out object imgObj) && imgObj is byte[] imgBytes && imgBytes.Length > 0)
            {
                string b64 = Convert.ToBase64String(EncodePng(imgBytes, obs.ImageWidth, obs.ImageHeight));
                sb.Append($"\"agentview_image\":\"{b64}\",");
                sb.Append($"\"image_width\":{obs.ImageWidth},");
                sb.Append($"\"image_height\":{obs.ImageHeight},");
            }

            // eye_in_hand image (optional)
            if (dict.TryGetValue("eye_in_hand_image", out object wristObj) && wristObj is byte[] wristBytes && wristBytes.Length > 0)
            {
                string b64 = Convert.ToBase64String(EncodePng(wristBytes, obs.ImageWidth, obs.ImageHeight));
                sb.Append($"\"eye_in_hand_image\":\"{b64}\",");
            }

            // proprioception
            sb.Append("\"robot0_joint_pos\":");
            sb.Append(FloatArrayToJson(obs.JointPositions));
            sb.Append(",\"robot0_eef_pos\":");
            sb.Append(FloatArrayToJson(obs.EEFPosition));
            sb.Append(",\"robot0_eef_quat\":");
            sb.Append(FloatArrayToJson(obs.EEFQuaternion));
            sb.Append(",\"robot0_gripper_qpos\":");
            sb.Append(FloatArrayToJson(obs.GripperQPos));

            // object states
            if (obs.ObjectPositions != null && obs.ObjectPositions.Count > 0)
            {
                sb.Append(",\"object_positions\":{");
                bool first = true;
                foreach (var kv in obs.ObjectPositions)
                {
                    if (!first) sb.Append(",");
                    sb.Append($"\"{EscapeJson(kv.Key)}\":{FloatArrayToJson(kv.Value)}");
                    first = false;
                }
                sb.Append("}");
            }

            sb.Append("}");
            return sb.ToString();
        }

        private static byte[] EncodePng(byte[] rawRgb, int width, int height)
        {
            // rawRgb is RGB24 from Texture2D.GetRawTextureData()
            // Flip vertically then encode as PNG via Texture2D
            var tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            tex.LoadRawTextureData(rawRgb);
            tex.Apply();
            byte[] png = tex.EncodeToPNG();
            Destroy(tex);
            return png;
        }

        private static string FloatArrayToJson(float[] arr)
        {
            if (arr == null) return "[]";
            var sb = new StringBuilder();
            sb.Append("[");
            for (int i = 0; i < arr.Length; i++)
            {
                if (i > 0) sb.Append(",");
                sb.Append(arr[i].ToString("F6"));
            }
            sb.Append("]");
            return sb.ToString();
        }

        private static string ExtractString(string json, string key)
        {
            string search = $"\"{key}\"";
            int idx = json.IndexOf(search);
            if (idx < 0) return null;

            int colon = json.IndexOf(':', idx + search.Length);
            if (colon < 0) return null;

            // skip whitespace
            int start = colon + 1;
            while (start < json.Length && (json[start] == ' ' || json[start] == '\"'))
                start++;

            // if quoted, find closing quote
            if (start > colon + 1 && json[colon + 1] == '\"')
            {
                int end = json.IndexOf('\"', start);
                if (end < 0) end = json.Length;
                return json.Substring(start, end - start);
            }

            // unquoted → read until comma or }
            int end2 = json.IndexOfAny(new[] { ',', '}' }, start);
            if (end2 < 0) end2 = json.Length;
            return json.Substring(start, end2 - start).Trim();
        }

        private static float[] ExtractFloatArray(string json, string key)
        {
            string search = $"\"{key}\"";
            int idx = json.IndexOf(search);
            if (idx < 0) return new float[0];

            int bracket = json.IndexOf('[', idx + search.Length);
            int end = json.IndexOf(']', bracket);
            if (bracket < 0 || end < 0) return new float[0];

            string inner = json.Substring(bracket + 1, end - bracket - 1);
            string[] parts = inner.Split(',');
            float[] result = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
                float.TryParse(parts[i].Trim(), out result[i]);
            return result;
        }

        private static string EscapeJson(string s)
        {
            return s?.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n") ?? "";
        }

        void OnDestroy()
        {
            _running = false;
            CleanupClient();
            _listener?.Stop();
            _recvThread?.Join(2000);
        }
    }
}
