using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System.Collections.Generic;

public class JointStatesSubscriber : MonoBehaviour
{
    ROSConnection ros;
    Dictionary<string, Transform> joints;

    [Header("Axis Settings")]
    // ★デフォルトをY軸(0, 1, 0)に変更しました
    public Vector3 defaultAxis = new Vector3(0, 1, 0);

    // ★追加: 名前(yaw/pitch/roll)から軸を自動判定する
    public bool autoAxisFromName = true;

    [Header("Name mapping")]
    public bool enableNameNormalization = true;
    public bool stripNamespace = true;
    public bool stripUnityCloneSuffix = true;
    public string stripSuffix = "";

    [Header("Debug")]
    public bool logMissingJoints = true;
    public bool logArticulationScan = true;
    public bool throttleMissingLogs = true;
    public float missingLogIntervalSec = 1.0f;
    public int missingLogSampleCount = 5;

    [Header("URDF joint->childLink mapping")]
    public bool mapRosJointNameToChildLinkName = true;

    [Header("Mode")]
    public bool useRosInput = true;

    int _missingCount;
    readonly HashSet<string> _missingSamples = new HashSet<string>();
    float _nextMissingLogTime;
    readonly List<string> _unityJointNames = new List<string>();

    void Start()
    {
        Debug.Log("[JointStatesSubscriber] Start() subscribing /dragon/joint_states (Transform Mode)");
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<JointStateMsg>("/dragon/joint_states", OnJointStates);

        // 物理コンポーネント破壊
        var allAbs = GetComponentsInChildren<ArticulationBody>();
        foreach (var ab in allAbs) Destroy(ab);
        var allColliders = GetComponentsInChildren<Collider>();
        foreach (var c in allColliders) Destroy(c);
        var allRbs = GetComponentsInChildren<Rigidbody>();
        foreach (var rb in allRbs) Destroy(rb);

        joints = new Dictionary<string, Transform>();
        _unityJointNames.Clear();

        var allTransforms = GetComponentsInChildren<Transform>();
        int added = 0;

        foreach (var t in allTransforms)
        {
            if (string.IsNullOrEmpty(t.name)) continue;

            RegisterJointKey(t.name, t);
            if (enableNameNormalization)
                RegisterJointKey(NormalizeKey(t.name), t);

            _unityJointNames.Add(t.name);
            added++;
        }

        if (logArticulationScan)
        {
            Debug.Log($"[JointStatesSubscriber] Transform scan: total={allTransforms.Length}, jointsAdded={added}, dictKeys={joints.Count}");
        }

        _nextMissingLogTime = Time.time + missingLogIntervalSec;
    }

    void RegisterJointKey(string key, Transform t)
    {
        if (string.IsNullOrEmpty(key)) return;
        if (!joints.ContainsKey(key)) joints.Add(key, t);
    }

    void OnJointStates(JointStateMsg msg)
    {
        if (!useRosInput) return;
        if (msg?.name == null || msg?.position == null) return;

        int n = Mathf.Min(msg.name.Length, msg.position.Length);

        for (int i = 0; i < n; i++)
        {
            string rawName = msg.name[i];

            if (!TryResolveJoint(rawName, out var t))
            {
                if (logMissingJoints) LogMissing(rawName);
                continue;
            }

            double posRad = msg.position[i];
            float angleDeg = (float)posRad * Mathf.Rad2Deg;

            // ★軸の決定ロジック
            Vector3 axis = defaultAxis;
            
            if (autoAxisFromName)
            {
                // ROSの軸(右手法) -> Unityの軸(左手法) への一般的なマッピング
                // joint名に "yaw", "pitch", "roll" が含まれている場合、軸を切り替える
                
                if (rawName.Contains("yaw")) 
                {
                    // ROS Z-axis (Yaw) -> Unity Y-axis (Up)
                    axis = -Vector3.up; 
                }
                else if (rawName.Contains("pitch")) 
                {
                    // ROS Y-axis (Pitch) -> Unity X-axis (Right)
                    // ※モデルによっては -X (Vector3.left) の場合もあるので、逆回転するならここを変える
                    axis = Vector3.right; 
                }
                else if (rawName.Contains("roll")) 
                {
                    // ROS X-axis (Roll) -> Unity Z-axis (Forward)
                    axis = -Vector3.forward; 
                }
            }

            t.localRotation = Quaternion.AngleAxis(angleDeg, axis);
        }

        if (logMissingJoints && throttleMissingLogs && Time.time >= _nextMissingLogTime)
        {
            if (_missingCount > 0)
            {
                Debug.LogWarning($"[JointStatesSubscriber] missing joints: {_missingCount} samples=[{string.Join(", ", _missingSamples)}]");
                _missingCount = 0;
                _missingSamples.Clear();
            }
            _nextMissingLogTime = Time.time + missingLogIntervalSec;
        }
    }

    void LogMissing(string rawName)
    {
        if (throttleMissingLogs)
        {
            _missingCount++;
            if (_missingSamples.Count < missingLogSampleCount)
            {
                string hint = FindHint(rawName);
                _missingSamples.Add(string.IsNullOrEmpty(hint) ? rawName : $"{rawName}->{hint}");
            }
        }
        else
        {
            Debug.LogWarning($"[JointStatesSubscriber] joint not found: '{rawName}'");
        }
    }

    bool TryResolveJoint(string rosJointName, out Transform t)
    {
        if (joints.TryGetValue(rosJointName, out t)) return true;
        if (enableNameNormalization)
        {
            string k = NormalizeKey(rosJointName);
            if (joints.TryGetValue(k, out t)) return true;
        }
        if (mapRosJointNameToChildLinkName)
        {
            string child = GuessChildLinkNameFromJointName(rosJointName);
            if (!string.IsNullOrEmpty(child))
            {
                if (joints.TryGetValue(child, out t)) return true;
                if (enableNameNormalization)
                {
                    string ck = NormalizeKey(child);
                    if (joints.TryGetValue(ck, out t)) return true;
                }
            }
        }
        t = null;
        return false;
    }

    static string GuessChildLinkNameFromJointName(string jointName)
    {
        if (string.IsNullOrEmpty(jointName)) return null;
        if (jointName.StartsWith("gimbal") && (jointName.EndsWith("_roll") || jointName.EndsWith("_pitch")))
            return jointName + "_module";
        if (jointName.StartsWith("rotor"))
            return "thrust" + jointName.Substring("rotor".Length);
        if (jointName.StartsWith("joint") && jointName.EndsWith("_pitch"))
        {
            string mid = jointName.Substring("joint".Length);
            mid = mid.Substring(0, mid.Length - "_pitch".Length);
            return "inter_joint" + mid;
        }
        if (jointName.StartsWith("joint") && jointName.EndsWith("_yaw"))
        {
            string mid = jointName.Substring("joint".Length);
            mid = mid.Substring(0, mid.Length - "_yaw".Length);
            if (int.TryParse(mid, out int k)) return "link" + (k + 1);
        }
        return null;
    }

    string NormalizeKey(string name)
    {
        if (string.IsNullOrEmpty(name)) return name;
        string s = name;
        if (stripNamespace)
        {
            int slash = s.LastIndexOf('/');
            if (slash >= 0 && slash + 1 < s.Length) s = s.Substring(slash + 1);
        }
        if (stripUnityCloneSuffix)
        {
            int idx = s.LastIndexOf(" (");
            if (idx > 0 && s.EndsWith(")")) s = s.Substring(0, idx);
        }
        if (!string.IsNullOrEmpty(stripSuffix) && s.EndsWith(stripSuffix))
            s = s.Substring(0, s.Length - stripSuffix.Length);
        return s;
    }

    string FindHint(string rosName)
    {
        if (_unityJointNames == null || _unityJointNames.Count == 0) return null;
        string prefix = rosName.Split('_')[0];
        foreach (var u in _unityJointNames)
            if (u.StartsWith(prefix)) return u;
        return null;
    }
}