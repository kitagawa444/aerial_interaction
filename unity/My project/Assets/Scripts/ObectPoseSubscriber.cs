using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
 
public class ObjectPoseSubscriber : MonoBehaviour
{
    // 追加: PoseStamped を受け取るトピック
    public string objectPoseTopic = "/object_pose";
    
    ROSConnection ros;
 
    [Header("Adjustments")]
    // 軸の向き補正 (必要に応じてInspectorで調整してください)
    public Vector3 rotationOffset = new Vector3(0, 0, 0);
    public Vector3 positionOffset = Vector3.zero;

    [Header("Debug")]
    public bool showDebugGizmos = true;

    Vector3 _targetPos;
    Quaternion _targetRot;
    bool _hasData = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();

        // PoseStamped を購読（TF関連の購読は削除）
        ros.Subscribe<PoseStampedMsg>(objectPoseTopic, OnPoseStampedMsg);
 
        // 物理演算コンポーネントの削除
        var allAbs = GetComponentsInChildren<ArticulationBody>();
        foreach (var ab in allAbs) Destroy(ab);

        var allColliders = GetComponentsInChildren<Collider>();
        foreach (var c in allColliders) Destroy(c);
        
        var allRbs = GetComponentsInChildren<Rigidbody>();
        foreach (var rb in allRbs) Destroy(rb);

        Debug.Log($"[ObjectPoseSubscriber] Listening to Pose topic: {objectPoseTopic}");
    }

    void Update()
    {
        if (!_hasData) return;

        transform.position = _targetPos + positionOffset;
        transform.rotation = _targetRot * Quaternion.Euler(rotationOffset);
    }

    // 追加: PoseStamped のコールバック
    void OnPoseStampedMsg(PoseStampedMsg msg)
    {
        var p = msg.pose.position;
        var q = msg.pose.orientation;

        // ROS -> Unity 座標変換（既存の Transform 変換ルールに合わせる）
        Vector3 pos = new Vector3(-(float)p.y, (float)p.z, (float)p.x);
        Quaternion rot = new Quaternion(-(float)q.y, (float)q.z, (float)q.x, -(float)q.w);

        Debug.Log($"[PoseStamped] ROS Pos({p.x:F3},{p.y:F3},{p.z:F3}) Q({q.x:F3},{q.y:F3},{q.z:F3},{q.w:F3}) -> Unity Pos({pos.x:F3},{pos.y:F3},{pos.z:F3})");

        _targetPos = pos;
        _targetRot = rot;
        _hasData = true;
    }

    void OnDrawGizmos()
    {
        if (!showDebugGizmos || !_hasData) return;
        Gizmos.color = new Color(1, 0, 0, 0.5f);
        Vector3 drawPos = _targetPos + positionOffset;
        Gizmos.DrawSphere(drawPos, 0.3f);
        Gizmos.DrawLine(Vector3.zero, drawPos);
    }
}