// CameraImagePublisher.cs
//
// Publiserer sensor_msgs/Image fra en Unity-Camera, pluss tilhoerende
// sensor_msgs/CameraInfo (med intrinsics utledet fra kamera-FOV).
//
// Plassering i Unity:
//   1. Lag et nytt GameObject som barn av stereo_left_camera_link /
//      stereo_right_camera_link / depth_camera_link.
//   2. Legg til en Camera-komponent paa det. Roter -90 rundt X for aa
//      matche ROS-kamera-konvensjon (Z framover).
//   3. Legg dette scriptet paa samme GameObject.
//   4. I Inspector: sett ImageTopic, CameraInfoTopic og FrameId slik
//      det matcher ros_gz_bridge.yaml / Unity_setup.md §3.5.
//
// Ytelse:
//   Hver kamera allokerer sin egen RenderTexture + Texture2D. 320x240
//   @ 15 Hz er typisk 2-3% CPU paa en moderat GPU. 3 kameraer samtidig
//   gaar fint.

using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Std;

[RequireComponent(typeof(Camera))]
public class CameraImagePublisher : MonoBehaviour
{
    [Header("ROS 2")]
    public string imageTopic = "camera/image_raw";
    public string cameraInfoTopic = "camera/camera_info";
    public string frameId = "camera_optical_frame";

    [Header("Bildeformat")]
    public int width = 320;
    public int height = 240;
    [Range(1f, 60f)] public float publishRateHz = 15f;

    private ROSConnection _ros;
    private Camera _camera;
    private RenderTexture _rt;
    private Texture2D _tex;
    private float _period;
    private float _lastPublish;

    private void Start()
    {
        _ros = ROSConnection.GetOrCreateInstance();
        _ros.RegisterPublisher<ImageMsg>(imageTopic);
        _ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);

        _camera = GetComponent<Camera>();
        _rt = new RenderTexture(width, height, 24, RenderTextureFormat.ARGB32);
        _rt.name = $"CamRT_{name}";
        _tex = new Texture2D(width, height, TextureFormat.RGB24, false);
        _period = 1f / Mathf.Max(publishRateHz, 0.1f);

        _camera.targetTexture = _rt;
    }

    private void OnDestroy()
    {
        if (_camera != null) _camera.targetTexture = null;
        if (_rt != null) _rt.Release();
    }

    private void LateUpdate()
    {
        if (Time.time - _lastPublish < _period) return;
        _lastPublish = Time.time;
        StartCoroutine(CaptureAndPublish());
    }

    private IEnumerator CaptureAndPublish()
    {
        // Vent til slutten av framen saa rendering er ferdig.
        yield return new WaitForEndOfFrame();

        RenderTexture prev = RenderTexture.active;
        RenderTexture.active = _rt;
        _tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        _tex.Apply();
        RenderTexture.active = prev;

        var pixels = _tex.GetPixels32();
        byte[] data = new byte[width * height * 3];
        for (int y = 0; y < height; y++)
        {
            // Unity-Y starter nederst, ROS-Y starter oeverst. Flip.
            int srcY = height - 1 - y;
            for (int x = 0; x < width; x++)
            {
                var c = pixels[srcY * width + x];
                int idx = (y * width + x) * 3;
                data[idx + 0] = c.r;
                data[idx + 1] = c.g;
                data[idx + 2] = c.b;
            }
        }

        var stamp = ToRosTime(Time.time);
        var header = new HeaderMsg { stamp = stamp, frame_id = frameId };

        var img = new ImageMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = data,
        };
        _ros.Publish(imageTopic, img);

        // --- CameraInfo -------------------------------------------------
        // Intrinsics fra Unity-kameraets vertikale FOV (deg):
        //   fy = h / (2 * tan(fov_v / 2))
        //   fx = fy * (w/h) / (aspect)  (her aspect = w/h, saa fx = fy)
        float fovVRad = _camera.fieldOfView * Mathf.Deg2Rad;
        double fy = 0.5 * height / Mathf.Tan(fovVRad * 0.5f);
        double fx = fy;  // kvadratiske piksler
        double cx = width * 0.5;
        double cy = height * 0.5;

        var info = new CameraInfoMsg
        {
            header = header,
            height = (uint)height,
            width = (uint)width,
            distortion_model = "plumb_bob",
            d = new double[] { 0, 0, 0, 0, 0 },
            k = new double[]
            {
                fx, 0,  cx,
                0,  fy, cy,
                0,  0,  1,
            },
            r = new double[]
            {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1,
            },
            p = new double[]
            {
                fx, 0,  cx, 0,
                0,  fy, cy, 0,
                0,  0,  1,  0,
            },
        };
        _ros.Publish(cameraInfoTopic, info);
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
