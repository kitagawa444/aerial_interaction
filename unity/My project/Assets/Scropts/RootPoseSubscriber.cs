using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class RootPoseSubscriber : MonoBehaviour
{
    public string topic = "/dragon/ground_truth";
    ROSConnection ros;

    [Header("Adjustments")]
    // ★軸の向きが違う場合、ここで補正する (例: (0, -90, 0) や (90, 0, 0))
    public Vector3 rotationOffset = new Vector3(0, 0, 0); 
    // 位置がズレている場合の補正
    public Vector3 positionOffset = Vector3.zero;

    [Header("Debug")]
    public bool showDebugGizmos = true;

    Vector3 _targetPos;
    Quaternion _targetRot;
    bool _hasData = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<OdometryMsg>(topic, OnMsg);

        // ★物理演算を完全に切る（破壊する）
        // 可視化だけなら ArticulationBody も Collider も不要です。
        // これらが残っていると、Transform移動を邪魔したり発散したりします。
        
        var allAbs = GetComponentsInChildren<ArticulationBody>();
        foreach (var ab in allAbs)
        {
            Destroy(ab); // 物理コンポーネントを削除
        }

        var allColliders = GetComponentsInChildren<Collider>();
        foreach (var c in allColliders)
        {
            Destroy(c); // 衝突判定を削除
        }
        
        var allRbs = GetComponentsInChildren<Rigidbody>();
        foreach (var rb in allRbs)
        {
            Destroy(rb); // Rigidbodyがあればそれも削除
        }

        Debug.Log("[RootPoseSubscriber] Physics components destroyed. Running in visualization-only mode.");
    }

    void Update()
    {
        if (!_hasData) return;

        // 補正値を適用して移動・回転
        transform.position = _targetPos + positionOffset;
        // 回転オフセットを適用（元の回転 * オフセット）
        transform.rotation = _targetRot * Quaternion.Euler(rotationOffset);
    }

    void OnMsg(OdometryMsg msg)
    {
        var p = msg.pose.pose.position;
        var o = msg.pose.pose.orientation;

        if (double.IsNaN(p.x)) return;

        // ROS (FLU) -> Unity (RUF) 変換
        Vector3 pos = new Vector3((float)-p.y, (float)p.z, (float)p.x);
        
        var C = new Matrix4x4(
            new Vector4(0, 0, 1, 0),
            new Vector4(-1, 0, 0, 0),
            new Vector4(0, 1, 0, 0),
            new Vector4(0, 0, 0, 1));
        Quaternion qRos = new Quaternion((float)o.x, (float)o.y, (float)o.z, (float)o.w);
        Quaternion rot = (C * Matrix4x4.Rotate(qRos) * C.inverse).rotation;

        _targetPos = pos;
        _targetRot = rot;
        _hasData = true;
    }

    void OnDrawGizmos()
    {
        if (!showDebugGizmos || !_hasData) return;
        Gizmos.color = new Color(1, 0, 0, 0.5f);
        // Gizmosも補正後の位置に出す
        Vector3 drawPos = _targetPos + positionOffset;
        Gizmos.DrawSphere(drawPos, 0.3f);
        Gizmos.DrawLine(Vector3.zero, drawPos);
    }
}