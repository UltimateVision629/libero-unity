using System;
using UnityEngine;

namespace LIBERO.Core
{
    public class ObjectState
    {
        private GameObject _gameObject;
        private Transform _transform;
        private readonly bool _isFixture;

        public string ObjectName { get; private set; }
        public GameObject GameObject => _gameObject;

        public float XmlBottomOffset { get; set; }
        public float XmlTopOffset { get; set; }
        public float XmlHorizontalRadius { get; set; }
        public bool HasXmlPlacementParams { get; set; }

        public Vector3 Position
        {
            get => _transform?.position ?? Vector3.zero;
            set { if (_transform != null) _transform.position = value; }
        }

        public Quaternion Rotation
        {
            get => _transform?.rotation ?? Quaternion.identity;
            set { if (_transform != null) _transform.rotation = value; }
        }

        public ObjectState(string objectName, GameObject gameObject, bool isFixture = false)
        {
            ObjectName = objectName;
            _gameObject = gameObject;
            _transform = gameObject?.transform;
            _isFixture = isFixture;
        }

        public void SetGameObject(GameObject go)
        {
            _gameObject = go;
            _transform = go?.transform;
        }

        public bool IsAbove(ObjectState other, float threshold = 0.02f)
        {
            if (_transform == null || other._transform == null) return false;
            return Position.y > other.Position.y + threshold;
        }

        public bool IsOnTopOf(ObjectState other, float contactThreshold = 0.05f)
        {
            if (_transform == null || other._transform == null) return false;

            Vector3 diff = Position - other.Position;
            float verticalDist = Mathf.Abs(diff.x) + Mathf.Abs(diff.z);
            float heightDiff = Position.y - other.Position.y;

            return verticalDist < 0.15f && heightDiff > -0.01f && heightDiff < 0.3f;
        }
    }
}
