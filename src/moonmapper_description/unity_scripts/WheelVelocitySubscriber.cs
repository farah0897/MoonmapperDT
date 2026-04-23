// WheelVelocitySubscriber.cs
//
// Abonnerer paa std_msgs/Float64MultiArray paa /unity/wheel_velocities
// og setter hjulets xDrive-targetVelocity paa hver av de seks hjul-
// ArticulationBody-ene.
//
// Meldings-rekkefoelge (satt av unity_cmd_vel_node.py paa ROS-siden):
//     [wheel_l1, wheel_l2, wheel_l3, wheel_r1, wheel_r2, wheel_r3]  i rad/s
//
// Plassering i Unity:
//   1. Legg scriptet paa MoonMapper-roten.
//   2. Dra de seks hjul-ArticulationBody-ene inn i Wheel Joints-listen
//      i rekkefoelgen [l1, l2, l3, r1, r2, r3].
//   3. Valgfritt: juster Drive Damping/ForceLimit hvis hjulene stryker
//      eller gir etter.

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class WheelVelocitySubscriber : MonoBehaviour
{
    [Header("ROS 2")]
    [Tooltip("Topic som publiseres av unity_cmd_vel_node.py paa ROS-siden.")]
    public string topicName = "unity/wheel_velocities";

    [Header("Wheels")]
    [Tooltip(
        "Rekkefoelge maa matche ROS-siden: " +
        "[wheel_l1, wheel_l2, wheel_l3, wheel_r1, wheel_r2, wheel_r3].")]
    public ArticulationBody[] wheelJoints = new ArticulationBody[6];

    [Header("Drive")]
    [Tooltip("xDrive.damping. Hoeyere = haardere holde hastighet.")]
    public float damping = 50f;

    [Tooltip(
        "xDrive.forceLimit. Maks moment kontrolleren kan bruke (Nm). " +
        "Sett hoeyt nok til at hjulene faktisk klarer aa stoppe/akselerere " +
        "raskt. 100 Nm er greit for en 5 cm rover.")]
    public float forceLimit = 100f;

    [Tooltip("Sekunder uten melding foer hjulene stopper (sikkerhet).")]
    public float commandTimeout = 0.5f;

    [Tooltip(
        "Logg hver melding i Unity Console. Nyttig for debugging, " +
        "men spammer konsollen.")]
    public bool logIncoming = false;

    private ROSConnection _ros;
    private double[] _targetVelRadPerSec = new double[6];
    private float _lastMessageTime = -1f;

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.Subscribe<Float64MultiArrayMsg>(topicName, OnWheelCmd);

        if (wheelJoints.Length != 6)
        {
            Debug.LogError(
                "[WheelVelocitySubscriber] wheelJoints-array maa ha " +
                "noeyaktig 6 elementer i rekkefoelgen [l1,l2,l3,r1,r2,r3].");
            enabled = false;
            return;
        }

        for (int i = 0; i < wheelJoints.Length; i++)
        {
            if (wheelJoints[i] == null)
            {
                Debug.LogWarning(
                    $"[WheelVelocitySubscriber] wheelJoints[{i}] er null. " +
                    "Husk aa dra hjulet inn i Inspector.");
                continue;
            }
            ConfigureDrive(wheelJoints[i]);
        }
    }

    private void ConfigureDrive(ArticulationBody body)
    {
        // Rene hastighetsdrives: stiffness=0 saa kontrolleren ikke
        // forsoeker aa holde en posisjon, bare en hastighet.
        var drive = body.xDrive;
        drive.stiffness = 0f;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        drive.targetVelocity = 0f;
        body.xDrive = drive;
    }

    private void OnWheelCmd(Float64MultiArrayMsg msg)
    {
        if (msg.data == null || msg.data.Length < 6) return;
        for (int i = 0; i < 6; i++) _targetVelRadPerSec[i] = msg.data[i];
        _lastMessageTime = Time.time;

        if (logIncoming)
        {
            Debug.Log(
                $"[WheelVelocitySubscriber] rad/s: " +
                $"L=[{msg.data[0]:F2},{msg.data[1]:F2},{msg.data[2]:F2}] " +
                $"R=[{msg.data[3]:F2},{msg.data[4]:F2},{msg.data[5]:F2}]");
        }
    }

    private void FixedUpdate()
    {
        // Watchdog: hvis ROS-siden slutter aa publisere, stopp hjulene.
        bool stale = (_lastMessageTime < 0f) ||
                     (Time.time - _lastMessageTime > commandTimeout);

        for (int i = 0; i < wheelJoints.Length; i++)
        {
            var body = wheelJoints[i];
            if (body == null) continue;

            float targetDegPerSec = stale
                ? 0f
                : (float)(_targetVelRadPerSec[i] * Mathf.Rad2Deg);

            var drive = body.xDrive;
            drive.targetVelocity = targetDegPerSec;
            body.xDrive = drive;
        }
    }
}
