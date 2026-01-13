// using UnityEngine;
// using Unity.Robotics.ROSTCPConnector;
// using RosMessageTypes.Geometry;
// using RosMessageTypes.Std;

// public class HeadMocapPublisher : MonoBehaviour
// {
//     [Header("ROS Settings")]
//     public string topicName = "head/mocap";
//     public string frameId = "world";
//     public float publishRateHz = 30.0f;

//     [Header("Target")]
//     public Transform targetTransform; // Main Cameraをここにアタッチ

//     ROSConnection ros;
//     float _publishInterval;
//     float _lastPublishTime;

//     void Start()
//     {
//         // ターゲットが設定されていない場合、MainCameraを自動取得
//         if (targetTransform == null)
//         {
//             if (Camera.main != null)
//             {
//                 targetTransform = Camera.main.transform;
//             }
//             else
//             {
//                 Debug.LogWarning("[HeadMocapPublisher] Target not set and MainCamera not found.");
//                 enabled = false;
//                 return;
//             }
//         }

//         ros = ROSConnection.GetOrCreateInstance();
//         ros.RegisterPublisher<PoseStampedMsg>(topicName);

//         _publishInterval = 1.0f / publishRateHz;
//         _lastPublishTime = Time.time;
//     }

//     void Update()
//     {
//         if (Time.time - _lastPublishTime < _publishInterval) return;

//         PublishPose();
//         _lastPublishTime = Time.time;
//     }

//     void PublishPose()
//     {
//         // Unity (Left-handed) -> ROS (Right-handed, FLU) 変換
//         // Position: Unity(x, y, z) -> ROS(z, -x, y) 
//         // ※この変換は一般的なTFに従いますが、プロジェクトの座標系によって調整が必要です。
//         //   RootPoseSubscriberの逆を行うのが基本です。
        
//         Vector3 unityPos = targetTransform.position;
//         Quaternion unityRot = targetTransform.rotation;

//         // Unity: X=Right, Y=Up, Z=Forward
//         // ROS: X=Forward, Y=Left, Z=Up
        
//         // 変換ルール (Unity -> ROS):
//         // ROS.x = Unity.z
//         // ROS.y = -Unity.x
//         // ROS.z = Unity.y
        
//         PointMsg position = new PointMsg(
//             unityPos.z, 
//             -unityPos.x, 
//             unityPos.y
//         );

//         // Rotation変換 (Unity -> ROS)
//         // ROS.x = Unity.z
//         // ROS.y = -Unity.x
//         // ROS.z = Unity.y
//         // ROS.w = -Unity.w
        
//         QuaternionMsg orientation = new QuaternionMsg(
//             unityRot.z,
//             -unityRot.x,
//             unityRot.y,
//             -unityRot.w
//         );

//         PoseStampedMsg msg = new PoseStampedMsg
//         {
//             header = new HeaderMsg
//             {
//                 frame_id = frameId,
//                 stamp = new TimeMsg
//                 {
//                     sec = (uint)Time.time,
//                     nanosec = (uint)((Time.time - (uint)Time.time) * 1e9)
//                 }
//             },
//             pose = new PoseMsg
//             {
//                 position = position,
//                 orientation = orientation
//             }
//         };

//         ros.Publish(topicName, msg);
//     }
// }