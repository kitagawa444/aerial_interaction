using System.Collections;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using UnityEngine.XR;

public class XRButtonServiceCaller : MonoBehaviour
{
    // (Inspector保持可) 未使用でも残す
    public InputActionReference buttonAction;
    public InputActionReference buttonAction2;
    public InputActionReference buttonAction3;
    public InputActionReference buttonAction4;

    public string handgraspTopic = "/handgrasp";
    public string snakebiteTopic = "/snakebite";
    public string robotHeadPoseTopic = "/robot_head_pose";
    public string triggerTopic = "/dragon/trigger_handover";
    // new: finger-specific topics (defaults to existing)
    public string indexFingerTopic = "/dragon/trigger_handover";
    public string middleFingerTopic = "/snakebite";
    public Transform robotHeadTransform;

    ROSConnection ros;
    UnityEngine.XR.InputDevice rightDevice;
    bool lastPrimary = false;
    bool lastSecondary = false;
    bool lastTrigger = false;
    bool lastGrip = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        Debug.Log("[XRButtonServiceCaller] Using device-based input (RightHand).");

        // register publishers required by ROSConnection
        ros.RegisterPublisher<RosMessageTypes.Std.EmptyMsg>(handgraspTopic);
        ros.RegisterPublisher<RosMessageTypes.Std.EmptyMsg>(snakebiteTopic);
        ros.RegisterPublisher<RosMessageTypes.Std.EmptyMsg>(triggerTopic);
        ros.RegisterPublisher<RosMessageTypes.Std.EmptyMsg>("/unity_heartbeat");
        ros.RegisterPublisher<RosMessageTypes.Geometry.PoseStampedMsg>(robotHeadPoseTopic);

        // quick test publish on start
        PublishEmpty(handgraspTopic);
        // start periodic heartbeat to verify runtime publishes
        StartCoroutine(Heartbeat());
    }

    void Update()
    {
        if (!rightDevice.isValid)
        {
            rightDevice = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
            return;
        }

        // primaryButton (A)
        if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.primaryButton, out bool primary))
        {
            if (primary && !lastPrimary)
            {
                // rising edge
                PublishRobotHeadPose();
            }
            lastPrimary = primary;
        }

        // index finger: prefer triggerButton (digital), fallback to trigger (analog)
        if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.triggerButton, out bool trigBtn))
        {
            if (trigBtn && !lastTrigger)
            {
                PublishEmpty(handgraspTopic);
            }
            lastTrigger = trigBtn;
        }
        else if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.trigger, out float trigAnalog))
        {
            bool trigPressed = trigAnalog > 0.1f;
            if (trigPressed && !lastTrigger)
            {
                PublishEmpty(handgraspTopic);
            }
            lastTrigger = trigPressed;
        }

        // secondaryButton (B)
        if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.secondaryButton, out bool secondary))
        {
            if (secondary && !lastSecondary)
            {
                PublishEmpty(triggerTopic);
            }
            lastSecondary = secondary;
        }

        // middle finger (grip)
        if (rightDevice.TryGetFeatureValue(UnityEngine.XR.CommonUsages.gripButton, out bool grip))
        {
            if (grip && !lastGrip)
            {
                PublishEmpty(snakebiteTopic);
            }
            lastGrip = grip;
        }
    }

    // publish PoseStamped (synchronous, device-based)
    void PublishRobotHeadPose()
    {
        if (robotHeadTransform == null)
        {
            Debug.LogWarning("[XRButtonServiceCaller] robotHeadTransform not set.");
            return;
        }

        Vector3 u = robotHeadTransform.position;
        Quaternion uq = robotHeadTransform.rotation;

        // Unity -> ROS conversion (keep your existing convention)
        float rx = u.z;
        float ry = -u.x;
        float rz = u.y;

        float rqx = uq.z;
        float rqy = -uq.x;
        float rqz = uq.y;
        float rqw = -uq.w;

        var ps = new PoseStampedMsg();
        ps.header = new RosMessageTypes.Std.HeaderMsg();
        ps.header.frame_id = "world";
        ps.pose = new PoseMsg();
        ps.pose.position = new PointMsg((double)rx, (double)ry, (double)rz);
        ps.pose.orientation = new QuaternionMsg((double)rqx, (double)rqy, (double)rqz, (double)rqw);

        ros.Publish(robotHeadPoseTopic, ps);
        Debug.Log($"[XRButtonServiceCaller] Published PoseStamped to {robotHeadPoseTopic}");
    }

    IEnumerator Heartbeat()
    {
        var msg = new EmptyMsg();
        while (true)
        {
            PublishEmpty("/unity_heartbeat");
            yield return new WaitForSeconds(2f);
        }
    }

    void PublishEmpty(string topic)
    {
        try
        {
            if (ros == null)
            {
                Debug.LogWarning($"[XRButtonServiceCaller] ROSConnection is null, cannot publish {topic}");
                return;
            }
            var msg = new EmptyMsg();
            ros.Publish(topic, msg);
            Debug.Log($"[XRButtonServiceCaller] Published Empty to {topic}");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[XRButtonServiceCaller] Publish to {topic} failed: {e}");
        }
    }
}