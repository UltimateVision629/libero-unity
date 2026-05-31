using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

namespace LIBERO.Networking
{
    [Serializable]
    public struct JoyConPose
    {
        public float[] pos;      // [x, y, z] metres in Joy-Con frame (used in Unity position IK)
        public float[] rot;      // [roll, pitch, yaw] radians (legacy, not used)
        public float[] joints;   // [yaw, J2, J3, J4, J5] radians (legacy, not used)
        public float baseYaw;    // base rotation J0 target (rad, ±90°), from stick H
        public float gripper;
        public int button;
    }

    public class JoyConReceiver : MonoBehaviour
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        private static void AutoCreate()
        {
            if (FindObjectOfType<JoyConReceiver>() != null) return;
            var go = new GameObject("JoyConReceiver");
            DontDestroyOnLoad(go);
            go.AddComponent<JoyConReceiver>();
        }

        [Header("TCP Settings")]
        public int ListenPort = 5555;
        public bool AutoStart = true;

        [Header("Status")]
        public bool IsConnected;

        private TcpListener _listener;
        private TcpClient _client;
        private Thread _recvThread;
        private readonly ConcurrentQueue<byte[]> _msgQueue = new ConcurrentQueue<byte[]>();
        private volatile bool _running;

        private JoyConPose _robot0Pose;
        private JoyConPose _robot1Pose;
        private readonly object _poseLock = new object();

        public bool HasData { get; private set; }

        // ── Joint feedback to Python (closed-loop IK seed) ─────────
        // Written by main thread (LiberoEnvironment), read by recv thread.
        // Index 0 = robot_0, 1 = robot_1; each float[5] = [J0..J4] rad
        private float[][] _jointFeedback = new float[2][] { new float[5], new float[5] };
        private bool _jointFeedbackValid;
        private readonly object _feedbackLock = new object();

        /// <summary>Called from main thread every frame to supply current joint angles.</summary>
        public void SetJointFeedback(int robotIndex, float[] jointsRad)
        {
            if (jointsRad == null) return;
            lock (_feedbackLock)
            {
                var dest = _jointFeedback[robotIndex];
                int n = Math.Min(dest.Length, jointsRad.Length);
                for (int i = 0; i < n; i++)
                    dest[i] = jointsRad[i];
                _jointFeedbackValid = true;
            }
        }

        void Start()
        {
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
                Name = "JoyConReceiver"
            };
            _recvThread.Start();
        }

        private void ServerLoop()
        {
            try
            {
                _listener = new TcpListener(IPAddress.Loopback, ListenPort);
                _listener.Start();
                Debug.Log($"[JoyConReceiver] Listening on 127.0.0.1:{ListenPort}");

                while (_running)
                {
                    if (_client == null || !_client.Connected)
                    {
                        try
                        {
                            _client = _listener.AcceptTcpClient();
                            _client.NoDelay = true; // disable Nagle for low latency
                            IsConnected = true;
                            Debug.Log("[JoyConReceiver] Python bridge connected.");
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
                                if (line == null) break; // connection closed

                                // ── Write joint-feedback reply for closed-loop IK seed ──
                                WriteJointFeedback(stream);

                                _msgQueue.Enqueue(Encoding.UTF8.GetBytes(line));
                            }
                        }
                    }
                    catch (IOException)
                    {
                        Debug.LogWarning("[JoyConReceiver] Python bridge disconnected. Waiting for reconnect...");
                    }

                    CleanupClient();
                    IsConnected = false;
                    HasData = false;
                }
            }
            catch (SocketException ex)
            {
                Debug.LogError($"[JoyConReceiver] Socket error: {ex.Message}");
            }
            finally
            {
                CleanupClient();
                _listener?.Stop();
            }
        }

        private void CleanupClient()
        {
            try { _client?.Close(); } catch { }
            _client = null;
        }

        void Update()
        {
            // Process incoming messages on main thread
            while (_msgQueue.TryDequeue(out byte[] raw))
            {
                try
                {
                    string json = Encoding.UTF8.GetString(raw);
                    ProcessMessage(json);
                    HasData = true;
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"[JoyConReceiver] Failed to parse message: {ex.Message}");
                }
            }
        }

        private void ProcessMessage(string json)
        {
            // Simple manual JSON parse to avoid Newtonsoft dependency
            lock (_poseLock)
            {
                int idx = json.IndexOf("\"robot_0\"");
                if (idx >= 0)
                    _robot0Pose = ParseRobotData(json, idx);

                idx = json.IndexOf("\"robot_1\"");
                if (idx >= 0)
                    _robot1Pose = ParseRobotData(json, idx);
            }
        }

        private JoyConPose ParseRobotData(string json, int startIdx)
        {
            var pose = new JoyConPose();
            int p;

            p = json.IndexOf("\"pos\"", startIdx);
            if (p >= 0) pose.pos = ParseFloatArray(json, p);

            p = json.IndexOf("\"rot\"", startIdx);
            if (p >= 0) pose.rot = ParseFloatArray(json, p);

            p = json.IndexOf("\"base_yaw\"", startIdx);
            if (p >= 0) pose.baseYaw = ParseFloatValue(json, p);

            p = json.IndexOf("\"gripper\"", startIdx);
            if (p >= 0) pose.gripper = ParseFloatValue(json, p);

            p = json.IndexOf("\"joints\"", startIdx);
            if (p >= 0) pose.joints = ParseFloatArray(json, p);

            p = json.IndexOf("\"button\"", startIdx);
            if (p >= 0) pose.button = (int)ParseFloatValue(json, p);

            // Debug.Log($"[pose] {pose.joints:F1}");
            // if (pose.joints != null && pose.joints.Length >= 5)
            //     Debug.Log($"[JoyConReceiver] parsed joints (rad): [{pose.joints[0]:F3}, {pose.joints[1]:F3}, {pose.joints[2]:F3}, {pose.joints[3]:F3}, {pose.joints[4]:F3}]");

            // Debug.Log($"[json] {json}");

            return pose;
        }

        private float[] ParseFloatArray(string json, int startIdx)
        {
            int bracket = json.IndexOf('[', startIdx);
            int end = json.IndexOf(']', bracket);
            if (bracket < 0 || end < 0) return new float[3];

            string inner = json.Substring(bracket + 1, end - bracket - 1);
            string[] parts = inner.Split(',');
            float[] result = new float[parts.Length];
            for (int i = 0; i < parts.Length; i++)
                float.TryParse(parts[i].Trim(), out result[i]);
            return result;
        }

        private float ParseFloatValue(string json, int startIdx)
        {
            int colon = json.IndexOf(':', startIdx);
            if (colon < 0) return 0f;

            int end = json.IndexOfAny(new[] { ',', '}' }, colon);
            if (end < 0) end = json.Length;

            string val = json.Substring(colon + 1, end - colon - 1).Trim();
            float.TryParse(val, out float result);
            return result;
        }

        public JoyConPose GetRobotPose(int robotIndex)
        {
            lock (_poseLock)
            {
                return robotIndex == 0 ? _robot0Pose : _robot1Pose;
            }
        }

        /// <summary>
        /// Serialise current joint angles into a single-line JSON and write back to Python.
        /// Called from the recv thread immediately after each received message.
        /// </summary>
        private void WriteJointFeedback(NetworkStream stream)
        {
            bool valid;
            float[] fb0, fb1;
            lock (_feedbackLock)
            {
                valid = _jointFeedbackValid;
                if (!valid) return;
                fb0 = (float[])_jointFeedback[0].Clone();
                fb1 = (float[])_jointFeedback[1].Clone();
            }

            var sb = new StringBuilder();
            sb.Append("{\"fb\":[");

            sb.Append("[");
            for (int i = 0; i < fb0.Length; i++)
            {
                if (i > 0) sb.Append(",");
                sb.Append(fb0[i].ToString("F6"));
            }
            sb.Append("],");

            sb.Append("[");
            for (int i = 0; i < fb1.Length; i++)
            {
                if (i > 0) sb.Append(",");
                sb.Append(fb1[i].ToString("F6"));
            }
            sb.Append("]}");

            byte[] payload = Encoding.UTF8.GetBytes(sb.ToString() + "\n");
            try
            {
                stream.Write(payload, 0, payload.Length);
                stream.Flush();
            }
            catch (IOException)
            {
                // Python side may close the socket — ignore
            }
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
