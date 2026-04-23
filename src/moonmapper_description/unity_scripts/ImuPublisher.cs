// ImuPublisher.cs
//
// Publiserer sensor_msgs/Imu paa /imu ved aa lese ArticulationBody-
// tilstanden til imu_link (orientering, vinkel-hastighet, lineaer
// akselerasjon i ROS/REP-103-frame).
//
// Plassering i Unity:
//   1. Finn imu_link-GameObject-en under moonmapper-roveren.
//   2. Legg scriptet paa imu_link.
//   3. Imu-link SKAL ha en ArticulationBody-komponent
//      (kommer automatisk fra URDF-Importer).
//
// NB om koordinater:
//   Unity:     +X hoeyre, +Y opp,     +Z framover
//   ROS REP-103: +X framover, +Y venstre, +Z opp
//   Konverteringen er:  (x_ros, y_ros, z_ros) = (z_unity, -x_unity, y_unity)

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

[RequireComponent(typeof(ArticulationBody))]
public class ImuPublisher : MonoBehaviour
{
    [Header("ROS 2")]
    public string topicName = "imu";
    public string frameId = "imu_link";

    [Tooltip("Publisering per sekund. 100 Hz er vanlig for IMU.")]
    [Range(1f, 500f)]
    public float publishRateHz = 100f;

    [Header("Noise (valgfri)")]
    [Tooltip("Gaussisk stoeys standardavvik paa vinkel-hastighet (rad/s).")]
    public float angularNoiseStd = 0.0f;
    [Tooltip("Gaussisk stoeys standardavvik paa lineaer akselerasjon (m/s^2).")]
    public float linearNoiseStd = 0.0f;

    private ROSConnection _ros;
    private ArticulationBody _body;
    private float _period;
    private float _lastPublish;
    private Vector3 _prevVelocityWorld;

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<ImuMsg>(topicName);
        _body = GetComponent<ArticulationBody>();
        _period = 1f / Mathf.Max(publishRateHz, 0.1f);
        _prevVelocityWorld = _body.velocity;
    }

    private void FixedUpdate()
    {
        if (Time.time - _lastPublish < _period) return;
        float dt = Time.time - _lastPublish;
        _lastPublish = Time.time;
        if (dt < 1e-5f) return;

        // --- Orientering ------------------------------------------------
        Quaternion qUnity = _body.transform.rotation;
        // Unity-quaternion -> ROS-quaternion (REP-103 basis)
        // (x_ros, y_ros, z_ros, w_ros) = (z_u, -x_u, y_u, w_u)  med
        // justert tegn (wiki.ros.org/geometry2/CoordinateFrameConventions).
        var orientation = new QuaternionMsg
        {
            x = qUnity.z,
            y = -qUnity.x,
            z = qUnity.y,
            w = -qUnity.w,
        };

        // --- Vinkel-hastighet ------------------------------------------
        Vector3 wLocalUnity = _body.angularVelocity;
        Vector3 wRos = UnityWorldVecToRos(wLocalUnity);
        wRos += GaussianVec(angularNoiseStd);

        // --- Lineaer akselerasjon (inkl. gravitasjon) -----------------
        Vector3 vWorldUnity = _body.velocity;
        Vector3 aWorldUnity = (vWorldUnity - _prevVelocityWorld) / dt;
        _prevVelocityWorld = vWorldUnity;
        // IMU-konvensjon: rapporter akselerasjon + (-gravity)-vektor
        // saa en stillestaaende IMU viser +9.81 paa z-aksen.
        aWorldUnity -= Physics.gravity;  // Physics.gravity er (0, -9.81, 0)
        Vector3 aLocalUnity =
            _body.transform.InverseTransformDirection(aWorldUnity);
        Vector3 aRos = UnityBodyVecToRos(aLocalUnity);
        aRos += GaussianVec(linearNoiseStd);

        var msg = new ImuMsg
        {
            header = new HeaderMsg
            {
                stamp = ToRosTime(Time.time),
                frame_id = frameId,
            },
            orientation = orientation,
            orientation_covariance = new double[9],   // 0 = ukjent
            angular_velocity = new Vector3Msg
            {
                x = wRos.x, y = wRos.y, z = wRos.z,
            },
            angular_velocity_covariance = new double[9],
            linear_acceleration = new Vector3Msg
            {
                x = aRos.x, y = aRos.y, z = aRos.z,
            },
            linear_acceleration_covariance = new double[9],
        };

        _ros.Publish(topicName, msg);
    }

    // Unity world-vector -> ROS world-vector (REP-103):
    //   (x_ros, y_ros, z_ros) = (z_unity, -x_unity, y_unity)
    private static Vector3 UnityWorldVecToRos(Vector3 v) =>
        new Vector3(v.z, -v.x, v.y);

    // Unity body-vector (allerede i link-frame) -> ROS body-vector
    private static Vector3 UnityBodyVecToRos(Vector3 v) =>
        new Vector3(v.z, -v.x, v.y);

    private static Vector3 GaussianVec(float std)
    {
        if (std <= 0f) return Vector3.zero;
        return new Vector3(
            NextGaussian() * std,
            NextGaussian() * std,
            NextGaussian() * std);
    }

    private static float NextGaussian()
    {
        // Box-Muller
        float u1 = 1f - Random.value;
        float u2 = 1f - Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) *
               Mathf.Cos(2f * Mathf.PI * u2);
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
}
