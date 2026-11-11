using System.Collections;
using System.Reflection;
using UnityEngine;
using UnityEngine.Rendering;
using ROS2;
using Awsim.Common;
using Awsim.Entity;

namespace Beytepe.Runtime
{
    /// <summary>
    /// Stereolabs ZED X benzetimi: iki AWSIM CameraSensor'ı (sol/sağ, taban çizgisi B) AYNI karede render eder ve
    /// aynı zaman damgasıyla yayınlar (stereo eşleme için şart; AWSIM'in CameraSensorScheduler'ı kameraları ayrı
    /// karelerde çizer). Görüntüler ZED SDK'nın verdiği gibi düzeltilmiş (rectified, bozulmasız) kabul edilir.
    ///
    /// Yayınlar (BEST_EFFORT, AWSIM kamerası gibi):
    ///   /sensing/camera/zedx/left/image_rect_color   + /sensing/camera/zedx/left/camera_info   (zedx_left_camera_optical_frame)
    ///   /sensing/camera/zedx/right/image_rect_color  + /sensing/camera/zedx/right/camera_info  (zedx_right_camera_optical_frame)
    /// Sağ camera_info'da P[3] = Tx = -fx*B (ROS stereo kuralı; stereo_image_proc doğrudan kullanabilir).
    ///
    /// AWSIM'in CameraRos2Publisher'ından farkı: görüntü verisi GPU'dan GERİ OKUNDUKTAN SONRA, çekim anının zaman
    /// damgasıyla yayınlanır (özgün yayıncı bir önceki çekimin verisini yeni damgayla yayınlıyordu).
    /// </summary>
    public class BeytepeStereoCamera : MonoBehaviour
    {
        [SerializeField] CameraSensor _leftSensor;
        [SerializeField] CameraSensor _rightSensor;
        [SerializeField] UnityEngine.Camera _leftCamera;
        [SerializeField] UnityEngine.Camera _rightCamera;
        [SerializeField] ComputeShader _rosImageShader;

        [SerializeField] int _width = 1920;
        [SerializeField] int _height = 1200;
        [SerializeField] float _hz = 15.0f;
        [SerializeField] float _baseline = 0.12f;
        [SerializeField] float _focalLengthMm = 2.2f;
        [SerializeField] float _pixelSizeUm = 3.0f;

        [SerializeField] string _leftImageTopic = "/sensing/camera/zedx/left/image_rect_color";
        [SerializeField] string _leftInfoTopic = "/sensing/camera/zedx/left/camera_info";
        [SerializeField] string _rightImageTopic = "/sensing/camera/zedx/right/image_rect_color";
        [SerializeField] string _rightInfoTopic = "/sensing/camera/zedx/right/camera_info";
        [SerializeField] string _leftFrame = "zedx_left_camera_optical_frame";
        [SerializeField] string _rightFrame = "zedx_right_camera_optical_frame";

        class Eye
        {
            public string name;
            public CameraSensor sensor;
            public UnityEngine.Camera camera;
            public float tx;
            public IPublisher<sensor_msgs.msg.Image> imagePub;
            public IPublisher<sensor_msgs.msg.CameraInfo> infoPub;
            public sensor_msgs.msg.Image image;
            public sensor_msgs.msg.CameraInfo info;
            public ComputeBuffer[] buffers;   // GPU okuma halkası: okuma gecikmesi bir çekim aralığını aşsa da çekim atlanmaz
            public bool[] busy;
            public int slot;
            public byte[] data;
            public int sec;
            public uint nsec;
            public int published;
        }

        Eye _left, _right;
        const int ReadbackSlots = 3;
        int _kernel = -1;
        int _captures, _skipped;
        float _lastStatTime;

        /// <summary>Sensör seti (BeytepeSensorRig) tarafından Awake'te, Start'tan önce çağrılır.</summary>
        public void Configure(int width, int height, float hz, float baseline, float focalLengthMm, float pixelSizeUm)
        {
            _width = width; _height = height; _hz = hz; _baseline = baseline;
            _focalLengthMm = focalLengthMm; _pixelSizeUm = pixelSizeUm;
        }

        /// <summary>Kamera adı: konular /sensing/camera/{name}/{left,right}/..., çerçeveler {name}_{left,right}_camera_optical_frame.</summary>
        public void SetName(string name)
        {
            _leftImageTopic = $"/sensing/camera/{name}/left/image_rect_color";
            _leftInfoTopic = $"/sensing/camera/{name}/left/camera_info";
            _rightImageTopic = $"/sensing/camera/{name}/right/image_rect_color";
            _rightInfoTopic = $"/sensing/camera/{name}/right/camera_info";
            _leftFrame = $"{name}_left_camera_optical_frame";
            _rightFrame = $"{name}_right_camera_optical_frame";
        }

        IEnumerator Start()
        {
            yield return null;   // AwsimRos2Node.Initialize (AutowareSimulationDemo.Start) tamamlansın

            if (_leftSensor == null || _rightSensor == null || _rosImageShader == null)
            {
                Debug.LogError("[Beytepe] ZED X: kamera ya da shader atanmamış; stereo kamera kapalı");
                yield break;
            }

            // Sol göz (ROS +y) / sağ göz (ROS -y): Unity x = -ROS y
            SetEyeOffset(_leftSensor.transform, +_baseline / 2f);
            SetEyeOffset(_rightSensor.transform, -_baseline / 2f);

            _left = MakeEye("sol", _leftSensor, _leftCamera, 0f, _leftImageTopic, _leftInfoTopic, _leftFrame);
            _right = MakeEye("sağ", _rightSensor, _rightCamera, -1f, _rightImageTopic, _rightInfoTopic, _rightFrame);
            _kernel = _rosImageShader.FindKernel("RosImageShaderKernel");
            _lastStatTime = Time.time;
            Debug.Log($"[Beytepe] ZED X hazır: {_width}x{_height} @ {_hz} Hz, fx {_left.info.K[0]:F1} px, Tx(sağ) {_right.info.P[3]:F2}");

            float period = 1.0f / Mathf.Max(_hz, 0.1f);
            float next = Time.time + period;
            while (true)
            {
                if (Time.time >= next)
                {
                    next += period;
                    if (next < Time.time) next = Time.time + period;   // uzun bir duraklamadan sonra yetişmeye çalışma
                    int ls = FreeSlot(_left), rs = FreeSlot(_right);
                    if (ls < 0 || rs < 0)
                        _skipped++;                                    // tüm okuma yuvaları dolu
                    else
                    {
                        _left.slot = ls; _right.slot = rs;
                        CapturePair();
                    }
                }
                if (Time.time - _lastStatTime >= 30f)
                {
                    Debug.Log($"[Beytepe] ZED X: {_captures} çift çekildi, sol {_left.published} / sağ {_right.published} yayınlandı, {_skipped} atlandı");
                    _lastStatTime = Time.time;
                }
                yield return null;
            }
        }

        void SetEyeOffset(Transform eye, float rosY)
        {
            eye.localPosition = Ros2Utility.Ros2ToUnityPosition(new Vector3(0f, rosY, 0f));
            eye.localRotation = Quaternion.identity;
        }

        Eye MakeEye(string name, CameraSensor sensor, UnityEngine.Camera cam, float txSign, string imageTopic, string infoTopic, string frame)
        {
            // Çözünürlük ve lens: CameraSensor fx = genişlik / sensör genişliği * odak uzaklığı
            SetPrivate(sensor, "_width", _width);
            SetPrivate(sensor, "_height", _height);
            if (cam != null)
            {
                cam.usePhysicalProperties = true;
                cam.sensorSize = new Vector2(_width * _pixelSizeUm * 1e-3f, _height * _pixelSizeUm * 1e-3f);
                cam.focalLength = _focalLengthMm;
                cam.gateFit = UnityEngine.Camera.GateFitMode.Horizontal;
                cam.nearClipPlane = 0.1f;
                int ground = LayerMask.NameToLayer("BeytepeGround");
                if (ground >= 0 && (cam.cullingMask & (1 << ground)) == 0)
                {
                    cam.cullingMask |= 1 << ground;   // güvenlik: Beytepe modeli render edilmezse görüntü boş kalır
                    Debug.LogWarning($"[Beytepe] ZED X {name}: katman maskesine BeytepeGround eklendi");
                }
            }
            sensor.Initialize();
            if (cam != null) cam.enabled = false;   // her karede kendiliğinden render etmesin; yalnız Render() çağrısıyla

            float fx = _focalLengthMm / (_pixelSizeUm * 1e-3f);
            float cx = (_width - 1) * 0.5f, cy = (_height - 1) * 0.5f;
            var e = new Eye { name = name, sensor = sensor, camera = cam, tx = txSign * fx * _baseline };
            e.image = new sensor_msgs.msg.Image
            {
                Header = new std_msgs.msg.Header { Frame_id = frame },
                Width = (uint)_width,
                Height = (uint)_height,
                Encoding = "bgr8",
                Is_bigendian = 0,
                Step = (uint)(_width * 3),
            };
            e.data = new byte[_width * _height * 3];
            e.image.Data = e.data;
            e.info = new sensor_msgs.msg.CameraInfo
            {
                Header = new std_msgs.msg.Header { Frame_id = frame },
                Width = (uint)_width,
                Height = (uint)_height,
                Distortion_model = "plumb_bob",
                D = new double[] { 0, 0, 0, 0, 0 },
                Binning_x = 0,
                Binning_y = 0,
                Roi = new sensor_msgs.msg.RegionOfInterest { X_offset = 0, Y_offset = 0, Height = 0, Width = 0, Do_rectify = false },
            };
            double[] K = { fx, 0, cx, 0, fx, cy, 0, 0, 1 };
            double[] R = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
            double[] P = { fx, 0, cx, e.tx, 0, fx, cy, 0, 0, 0, 1, 0 };
            for (int i = 0; i < 9; i++) { e.info.K[i] = K[i]; e.info.R[i] = R[i]; }
            for (int i = 0; i < 12; i++) e.info.P[i] = P[i];
            e.buffers = new ComputeBuffer[ReadbackSlots];
            e.busy = new bool[ReadbackSlots];
            for (int i = 0; i < ReadbackSlots; i++)
                e.buffers[i] = new ComputeBuffer(_width * _height * 3 / sizeof(uint), sizeof(uint));

            var qos = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                      DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                      HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1).GetQosProfile();
            e.imagePub = AwsimRos2Node.CreatePublisher<sensor_msgs.msg.Image>(imageTopic, qos);
            e.infoPub = AwsimRos2Node.CreatePublisher<sensor_msgs.msg.CameraInfo>(infoTopic, qos);
            sensor.OnOutput += d => OnEyeOutput(e, d);
            return e;
        }

        void CapturePair()
        {
            AwsimRos2Node.GetTime(out var sec, out var nsec);   // iki göz için aynı çekim anı
            _left.sec = sec; _left.nsec = nsec;
            _right.sec = sec; _right.nsec = nsec;
            _left.sensor.DoRender();
            _right.sensor.DoRender();
            _captures++;
        }

        void OnEyeOutput(Eye e, CameraSensor.IReadOnlyOutputData d)
        {
            var rt = d.OutputRenderTexture;
            if (rt == null || rt.width != _width || rt.height != _height) return;
            _rosImageShader.SetInt("_width", _width);
            _rosImageShader.SetInt("_height", _height);
            _rosImageShader.SetTexture(_kernel, "_InputTexture", rt);
            int slot = e.slot;
            var buf = e.buffers[slot];
            _rosImageShader.SetBuffer(_kernel, "_RosImageBuffer", buf);
            _rosImageShader.Dispatch(_kernel, (buf.count + 127) / 128, 1, 1);   // numthreads(128,1,1)
            e.busy[slot] = true;
            int sec = e.sec; uint nsec = e.nsec;
            AsyncGPUReadback.Request(buf, req => OnReadback(e, slot, req, sec, nsec));
        }

        static int FreeSlot(Eye e)
        {
            for (int i = 0; i < e.busy.Length; i++)
                if (!e.busy[i]) return i;
            return -1;
        }

        void OnReadback(Eye e, int slot, AsyncGPUReadbackRequest req, int sec, uint nsec)
        {
            e.busy[slot] = false;
            if (req.hasError || e.imagePub == null)
                return;
            req.GetData<byte>().CopyTo(e.data);
            e.image.Data = e.data;
            e.image.Header.Stamp.Sec = sec; e.image.Header.Stamp.Nanosec = nsec;
            e.info.Header.Stamp.Sec = sec; e.info.Header.Stamp.Nanosec = nsec;
            e.imagePub.Publish(e.image);
            e.infoPub.Publish(e.info);
            e.published++;
        }

        static void SetPrivate(object target, string field, object value)
        {
            var f = target.GetType().GetField(field, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public);
            if (f != null) f.SetValue(target, value);
            else Debug.LogWarning($"[Beytepe] ZED X: {target.GetType().Name}.{field} alanı bulunamadı");
        }

        void OnDestroy()
        {
            foreach (var e in new[] { _left, _right })
            {
                if (e == null) continue;
                if (e.buffers != null)
                    foreach (var b in e.buffers) b?.Release();
                if (e.imagePub != null) AwsimRos2Node.RemovePublisher<sensor_msgs.msg.Image>(e.imagePub);
                if (e.infoPub != null) AwsimRos2Node.RemovePublisher<sensor_msgs.msg.CameraInfo>(e.infoPub);
            }
        }
    }
}
