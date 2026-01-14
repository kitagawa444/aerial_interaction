using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces; // 追加: TimeMsg はここ
using UnityEngine.XR;

public class HeadMocapPublisher : MonoBehaviour
{
    [Header("ROS Settings")]
    public string topicName = "/eye/mocap/pose";
    public string frameId = "world";
    public float publishRateHz = 30.0f;

    [Header("Controller Settings")]
    public XRNode controllerNode = XRNode.RightHand; // RightHand または LeftHand
    public Transform controllerTransform; // XRIの Right/Left Controller Transform をアタッチ

    ROSConnection ros;
    float _publishInterval;
    float _lastPublishTime;

    void Start()
    {
        // コントローラーのTransformが設定されていない場合の警告
        if (controllerTransform == null)
        {
            Debug.LogWarning("[HeadMocapPublisher] Controller Transform not set. Please assign Right/Left Controller Transform under XR Origin.");
            enabled = false;
            return;
        }

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

        PointMsg position = new PointMsg(
            unityPos.z,
            -unityPos.x,
            unityPos.y
        );

        QuaternionMsg orientation = new QuaternionMsg(
            unityRot.z,
            -unityRot.x,
            unityRot.y,
            -unityRot.w
        );

        // ROS stamp（TimeMsgがuint想定のためuintで作る）
        float t = Time.time;
        uint sec = (uint)Mathf.FloorToInt(t);
        uint nanosec = (uint)((t - (float)sec) * 1e9f);

        PoseStampedMsg msg = new PoseStampedMsg
        {
            header = new HeaderMsg
            {
                frame_id = frameId,
                stamp = new TimeMsg
                {
                    sec = sec,
                    nanosec = nanosec
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