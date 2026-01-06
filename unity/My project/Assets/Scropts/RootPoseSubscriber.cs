using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Tf2;
using RosMessageTypes.Geometry;

public class RootPoseSubscriber : MonoBehaviour
{
    // TFのトピック名 (通常は "/tf")
    public string tfTopic = "/tf";
    // 監視対象のフレーム名
    public string targetFrame = "dragon/root";
    
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
        // TFMessageを受信するように変更
        ros.Subscribe<TFMessageMsg>(tfTopic, OnTFMsg);

        // 物理演算コンポーネントの削除
        var allAbs = GetComponentsInChildren<ArticulationBody>();
        foreach (var ab in allAbs) Destroy(ab);

        var allColliders = GetComponentsInChildren<Collider>();
        foreach (var c in allColliders) Destroy(c);
        
        var allRbs = GetComponentsInChildren<Rigidbody>();
        foreach (var rb in allRbs) Destroy(rb);

        Debug.Log($"[RootPoseSubscriber] Listening to TF topic: {tfTopic} for frame: {targetFrame}");
    }

    void Update()
    {
        if (!_hasData) return;

        transform.position = _targetPos + positionOffset;
        transform.rotation = _targetRot * Quaternion.Euler(rotationOffset);
    }

    void OnTFMsg(TFMessageMsg msg)
    {
        foreach (var transformStamped in msg.transforms)
        {
            // 指定したチャイルドフレームIDを持つ変換を探す
            if (transformStamped.child_frame_id == targetFrame)
            {
                UpdateTargetPose(transformStamped.transform);
                return;
            }
        }
    }

    void UpdateTargetPose(TransformMsg rosTransform)
    {
        var t = rosTransform.translation;
        var r = rosTransform.rotation;

        // ROS -> Unity 座標変換
        // 並進の設定 (y, z, x) に合わせて、回転の成分も (y, z, x) の順で取得
        // Standard ROS to Unity
        Vector3 pos = new Vector3(-(float)t.y, (float)t.z, (float)t.x);
        Quaternion rot = new Quaternion(-(float)r.y, (float)r.z, (float)r.x, -(float)r.w);

        // クォータニオンとオイラー角の両方を計算して比較しやすくする
        Vector3 rosEuler = new Quaternion((float)r.x, (float)r.y, (float)r.z, (float)r.w).eulerAngles;
        Vector3 unityEuler = rot.eulerAngles;

        // ログを統合して出力
        Debug.Log($"[RootPose]\n" +
                  $"ROS Raw   : Pos({t.x:F3}, {t.y:F3}, {t.z:F3}) Q({r.x:F3}, {r.y:F3}, {r.z:F3}, {r.w:F3}) E({rosEuler.x:F1}, {rosEuler.y:F1}, {rosEuler.z:F1})\n" +
                  $"Unity     : Pos({pos.x:F3}, {pos.y:F3}, {pos.z:F3}) Q({rot.x:F3}, {rot.y:F3}, {rot.z:F3}, {rot.w:F3}) E({unityEuler.x:F1}, {unityEuler.y:F1}, {unityEuler.z:F1})");

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