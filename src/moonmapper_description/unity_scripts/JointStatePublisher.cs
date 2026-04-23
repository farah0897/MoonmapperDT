// JointStatePublisher.cs
//
// Publiserer sensor_msgs/JointState paa /joint_states ved aa lese
// ArticulationBody-jointene i MoonMapper-roveren.
//
// Plassering i Unity:
//   1. Legg dette scriptet paa MoonMapper-roten (moonmapper-objektet i Hierarchy).
//   2. I Inspector: sett JointStatePublisher -> JointRoot = samme GameObject.
//   3. Klikk "Auto Populate From Children" i Inspector (hoeyreklikk script-header).
//   4. Verifiser at alle jointene dukker opp i listen.
//
// Alternativt kan du dra ArticulationBody-ene manuelt inn i Joints-listen.

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

public class JointStatePublisher : MonoBehaviour
{
    [Header("ROS 2")]
    [Tooltip("ROS-topic aa publisere JointState-meldinger paa.")]
    public string topicName = "joint_states";

    [Tooltip("Antall publiseringer per sekund (Hz).")]
    [Range(1f, 200f)]
    public float publishRateHz = 50f;

    [Header("Joints")]
    [Tooltip(
        "ArticulationBody-er som skal publiseres. Kan fylles automatisk " +
        "fra barn via context-menyen paa dette komponenten.")]
    public List<ArticulationBody> joints = new List<ArticulationBody>();

    [Tooltip(
        "Valgfritt: overskriver navnet fra GameObject-en ved samme indeks. " +
        "La listen vaere tom for aa bruke GameObject-navnet direkte.")]
    public List<string> jointNameOverrides = new List<string>();

    [Tooltip(
        "Sett frame_id paa header (ikke kritisk for JointState).")]
    public string frameId = "";

    private ROSConnection _ros;
    private float _publishPeriod;
    private float _lastPublishTime;

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<JointStateMsg>(topicName);
        _publishPeriod = 1f / Mathf.Max(publishRateHz, 0.1f);

        if (joints.Count == 0)
        {
            Debug.LogWarning(
                $"[JointStatePublisher] Ingen joints satt opp paa {name}. " +
                "Bruk 'Auto Populate From Children' fra context-menyen, eller " +
                "dra dem inn manuelt.");
        }
    }

    private void Update()
    {
        if (joints.Count == 0) return;
        if (Time.time - _lastPublishTime < _publishPeriod) return;
        _lastPublishTime = Time.time;

        var msg = new JointStateMsg
        {
            header = new HeaderMsg
            {
                stamp = ToRosTime(Time.time),
                frame_id = frameId,
            },
            name = new string[joints.Count],
            position = new double[joints.Count],
            velocity = new double[joints.Count],
            effort = new double[joints.Count],
        };

        for (int i = 0; i < joints.Count; i++)
        {
            var body = joints[i];
            if (body == null) continue;

            msg.name[i] = (i < jointNameOverrides.Count &&
                           !string.IsNullOrEmpty(jointNameOverrides[i]))
                ? jointNameOverrides[i]
                : body.name;

            // ArticulationBody.jointPosition er i radianer for revolute
            // joints og meter for prismatic. Roveren er kun revolute.
            if (body.jointPosition.dofCount > 0)
                msg.position[i] = body.jointPosition[0];

            if (body.jointVelocity.dofCount > 0)
                msg.velocity[i] = body.jointVelocity[0];

            // jointForce er Newton-meter (revolute).
            if (body.jointForce.dofCount > 0)
                msg.effort[i] = body.jointForce[0];
        }

        _ros.Publish(topicName, msg);
    }

    private static TimeMsg ToRosTime(float unitySeconds)
    {
        // builtin_interfaces/Time i ROS 2:
        //   int32  sec
        //   uint32 nanosec
        int sec = Mathf.FloorToInt(unitySeconds);
        uint nsec = (uint)((unitySeconds - sec) * 1e9f);
        return new TimeMsg { sec = sec, nanosec = nsec };
    }

    // Hoeyreklikk paa komponentens navn i Inspector -> Auto Populate From Children
    [ContextMenu("Auto Populate From Children")]
    private void AutoPopulate()
    {
        joints.Clear();
        jointNameOverrides.Clear();
        var bodies = GetComponentsInChildren<ArticulationBody>(true);
        foreach (var b in bodies)
        {
            // Hopp over roten (base_link) som ikke er en joint
            if (b.isRoot) continue;
            joints.Add(b);
            jointNameOverrides.Add(b.name);
        }
        Debug.Log(
            $"[JointStatePublisher] Auto-fylte {joints.Count} joints " +
            $"fra barna til {name}.");
    }
}
