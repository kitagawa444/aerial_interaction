using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces; // 追加: TimeMsg はここ
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit; // ← 追加

public class HandMocapPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/wrist/mocap/pose";
    public string frameId = "world";
    public float publishRateHz = 30.0f;

    [Header("Controller Settings")]
    public XRNode controllerNode = XRNode.RightHand; // RightHand または LeftHand
    public Transform controllerTransform; // OVRCameraRigの RightHandAnchor または LeftHandAnchor をアタッチ

    ROSConnection ros;
    float _publishInterval;
    float _lastPublishTime;

    void Start()
    {
        Debug.Log("[HandMocapPublisher] Start");

        // controllerTransform が未設定なら XRI のコントローラから自動取得を試みる
        if (controllerTransform == null)
        {
            var controllers = FindObjectsOfType<XRController>(true);
            Debug.Log($"[HandMocapPublisher] Found {controllers.Length} XRController(s)");
            foreach (var c in controllers)
            {
                Debug.Log($"[HandMocapPublisher] XRController object: {c.gameObject.name}");
                var nameLower = c.gameObject.name.ToLower();
                if ((controllerNode == XRNode.RightHand && nameLower.Contains("right")) ||
                    (controllerNode == XRNode.LeftHand  && nameLower.Contains("left")))
                {
                    controllerTransform = c.transform;
                    Debug.Log($"[HandMocapPublisher] Auto-assigned controllerTransform: {controllerTransform.name}");
                    break;
                }
            }

            if (controllerTransform == null)
            {
                // fallback: common names
                string[] candidates = {
                    "right controller","left controller",
                    "right_controller","left_controller",
                    "right hand","left hand",
                    "right","left"
                };
                foreach (var cand in candidates)
                {
                    var go = GameObject.Find(cand);
                    if (go != null)
                    {
                        controllerTransform = go.transform;
                        Debug.Log($"[HandMocapPublisher] Assigned by GameObject.Find: {controllerTransform.name}");
                        break;
                    }
                }
            }
        }

        if (controllerTransform == null)
        {
            Debug.LogWarning("[HandMocapPublisher] Controller Transform not set. Please assign Right/Left Controller under XR Origin.");
            enabled = false;
            return;
        }

        Debug.Log($"[HandMocapPublisher] Using controllerTransform: {controllerTransform.name}");
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);

        _publishInterval = 1.0f / publishRateHz;
        _lastPublishTime = Time.time;
    }

    void Update()
    {
        if (Time.time - _lastPublishTime < _publishInterval) return;

        PublishPose();
        _lastPublishTime = Time.time;
    }

    void PublishPose()
    {
        // Unity (Left-handed) -> ROS (Right-handed, FLU) 変換
        Vector3 unityPos = controllerTransform.position;
        Quaternion unityRot = controllerTransform.rotation;

        // Unity: X=Right, Y=Up, Z=Forward
        // ROS: X=Forward, Y=Left, Z=Up
        
        // 変換ルール (Unity -> ROS):
        // ROS.x = Unity.z
        // ROS.y = -Unity.x
        // ROS.z = Unity.y
        
        PointMsg position = new PointMsg(
            unityPos.z, 
            -unityPos.x, 
            unityPos.y
        );

        // Rotation変換 (Unity -> ROS)
        QuaternionMsg orientation = new QuaternionMsg(
            unityRot.z,
            -unityRot.x,
            unityRot.y,
            -unityRot.w
        );

        float t = Time.time;
        int sec_i = Mathf.FloorToInt(t);
        uint nsec_u = (uint)((t - sec_i) * 1e9f);

        PoseStampedMsg msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg
                {
                    sec = (uint)sec_i,
                    nanosec = nsec_u
                }
            },
            pose = new PoseMsg
            {
                position = position,
                orientation = orientation
            }
        };

        ros.Publish(topicName, msg);
    }
}