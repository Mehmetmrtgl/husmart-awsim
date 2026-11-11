using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using UnityEngine;
using RGLUnityPlugin;
using Awsim.Common;
using Awsim.Entity;

namespace Beytepe.Runtime
{
    /// <summary>
    /// Beytepe: ego aracının sensör setini çalışma anında seçer (yeniden derleme gerekmez).
    ///
    /// AWSIM'e verilen --json_path dosyasındaki "SensorRig" bölümünü okur:
    ///   * bölüm yok ya da "Name": "lexus"  -> AWSIM'in özgün Lexus sensörleri; HİÇBİR ŞEY değişmez.
    ///   * "Name": "husmart"                -> Hüsmart sensör seti:
    ///        - Hesai JT128 lidar (çatı; /sensing/lidar/top/pointcloud_raw(_ex), çerçeve hesai_top). VLP-16 kapatılır.
    ///        - Stereolabs ZED X stereo kamera (sol/sağ, aynı anda çekim, /sensing/camera/zedx/...).
    ///        - Xsens MTi-7 GNSS/INS (IMU 100 Hz + yön, GNSS NavSatFix, INS konumu).
    ///        - Lexus'un XX1 çatı sensör kiti görseli gizlenir (gerçek araçta yok; lidar/kamera onu görmesin).
    ///        - AWSIM'in GNSS'i (/sensing/gnss/pose_with_covariance, Autoware ilk konum için kullanır) MTi-7 anteninin
    ///          yerine taşınır.
    /// Tüm konumlar ROS base_link çerçevesindedir (x ileri, y sol, z yukarı; metre) ve açılar roll/pitch/yaw (derece).
    /// Autoware tarafındaki ~/autoware/husmart_sensor_kit kalibrasyonu bu değerlerle AYNI olmalıdır.
    /// </summary>
    public class BeytepeSensorRig : MonoBehaviour
    {
        [Serializable]
        public class LidarConfig
        {
            public Vector3 Position = new Vector3(0.90f, 0.0f, 1.84f);     // JT128 ölçüm merkezi (tahmini: çatı ortası)
            public Vector3 RollPitchYawDeg = Vector3.zero;                // (0,0,0) = dik, kubbe yukarı; roll 180 = ters
            public float MaxRange = 60.0f;
            public int Hz = 10;
        }

        [Serializable]
        public class StereoConfig
        {
            public bool Enabled = true;
            public int Width = 1920;                                       // ZED X: 1920x1200 (HD1200) ya da 960x600 (SVGA)
            public int Height = 1200;
            public float Hz = 15.0f;
            public float BaselineM = 0.12f;
            public float FocalLengthMm = 2.2f;                             // 2.2 mm lens (110°x80° üst sınır)
            public float PixelSizeUm = 3.0f;                               // 3 µm piksel
            public Vector3 Position = new Vector3(1.65f, 0.0f, 1.80f);     // iki kameranın ortası (tahmini: çatı ön kenarı)
            public Vector3 RollPitchYawDeg = new Vector3(0.0f, 5.0f, 0.0f);// 5° aşağı bakış
        }

        [Serializable]
        public class LidarEntry
        {
            public string Name = "top";                 // konu: /sensing/lidar/{Name}/pointcloud_raw_ex
            public string Model = "HesaiJT128";         // RGL LidarModel adı: HesaiJT128, HesaiQT128C2X, VelodyneVLP16, ...
            public string FrameId = "hesai_top";
            public Vector3 Position = Vector3.zero;      // ROS base_link
            public Vector3 RollPitchYawDeg = Vector3.zero;
            public float MaxRange = 0f;                 // 0 = modelin varsayılanı
            public float HorizontalResolutionDeg = 0f;  // 0 = modelin varsayılanı
            public int Hz = 10;
            public bool PublishPcl24 = false;           // ayrıca /pointcloud_raw (XYZI) yayınla
        }

        [Serializable]
        public class CameraEntry : StereoConfig
        {
            public string Name = "zedx";                // konular /sensing/camera/{Name}/{left,right}/...
        }

        [Serializable]
        public class InsConfig
        {
            public bool Enabled = true;
            public int ImuHz = 100;
            public float GnssHz = 4.0f;
            public Vector3 Position = new Vector3(1.20f, 0.0f, 0.45f);     // MTi-7 (tahmini: araç ortası, orta konsol)
            public Vector3 RollPitchYawDeg = Vector3.zero;
            public Vector3 AntennaPosition = new Vector3(0.40f, 0.0f, 1.82f);  // GNSS anteni (tahmini: çatı arkası)
            public bool Noise = true;
        }

        [Serializable]
        public class RigConfig
        {
            public string Name = "lexus";
            public bool HideLexusSensorKit = true;
            public bool DisableLexusCamera = true;      // Lexus trafik ışığı kamerası (Hüsmart'ta yok): kare başı render + 10 Hz yayın durur
            public LidarConfig Lidar = new LidarConfig();
            public StereoConfig Stereo = new StereoConfig();
            public InsConfig Ins = new InsConfig();
            public List<LidarEntry> Lidars = new List<LidarEntry>();     // boşsa: tek JT128 ("Lidar" alanı)
            public List<CameraEntry> Cameras = new List<CameraEntry>();  // boşsa: tek ZED X ("Stereo" alanı)
        }

        [Serializable]
        class JsonRoot
        {
            public RigConfig SensorRig = new RigConfig();
        }

        [Header("Ego (sahne kurucusu doldurur)")]
        [SerializeField] Transform _baseLink;
        [SerializeField] GameObject _lexusVlp16;             // özgün çatı lidarı (velodyne_top altındaki VLP-16)
        [SerializeField] GameObject _lexusSensorKitVisual;   // XX1 Sensor Kit görseli
        [SerializeField] Transform _lexusGnss;               // AWSIM GnssSensor

        [Header("Hüsmart sensörleri (sahne kurucusu oluşturur, varsayılan kapalı)")]
        [SerializeField] GameObject _jt128;
        [SerializeField] BeytepeStereoCamera _zedx;
        [SerializeField] BeytepeMti7 _mti7;

        public RigConfig Config { get; private set; }

        void Awake()
        {
            Config = LoadConfig(out var source);
            bool husmart = string.Equals(Config.Name, "husmart", StringComparison.OrdinalIgnoreCase);
            if (!husmart)
            {
                if (_jt128 != null) _jt128.SetActive(false);
                if (_zedx != null) _zedx.gameObject.SetActive(false);
                if (_mti7 != null) _mti7.gameObject.SetActive(false);
                Debug.Log($"[Beytepe] sensör seti: lexus (AWSIM özgün sensörleri) [{source}]");
                return;
            }

            var log = new StringBuilder($"[Beytepe] sensör seti: HÜSMART [{source}]\n");

            // Lexus'a özgü çatı donanımı: gerçek araçta yok.
            if (_lexusVlp16 != null) _lexusVlp16.SetActive(false);
            if (Config.HideLexusSensorKit && _lexusSensorKitVisual != null) _lexusSensorKitVisual.SetActive(false);
            log.AppendLine($"  VLP-16 kapalı; XX1 çatı kiti görseli {(Config.HideLexusSensorKit ? "gizli" : "görünür")}");
            if (Config.DisableLexusCamera)
            {
                // Unity kamerası etkin kaldıkça HDRP onu HER karede render eder (AWSIM'in özgün durumu). Kapatılır;
                // zamanlayıcının 10 Hz'lik elle render'ı da Start'ta (EgoVehicle başlatıldıktan sonra) durdurulur.
                foreach (var cs in GetComponentsInChildren<CameraSensor>(true))
                {
                    if (_zedx != null && cs.transform.IsChildOf(_zedx.transform)) continue;
                    foreach (var cam in cs.GetComponentsInChildren<UnityEngine.Camera>(true)) cam.enabled = false;
                }
                log.AppendLine("  Lexus trafik ışığı kamerası kapatıldı (/sensing/camera/traffic_light/* yayınlanmaz)");
            }

            // LiDAR'lar: sahne kurucusunun eklediği (kapalı) JT128 nesnesi şablondur; her girdi için bir kopyası açılır.
            var lidars = Config.Lidars != null && Config.Lidars.Count > 0 ? Config.Lidars
                : new List<LidarEntry> { new LidarEntry { Name = "top", Model = "HesaiJT128", FrameId = "hesai_top",
                                                          Position = Config.Lidar.Position, RollPitchYawDeg = Config.Lidar.RollPitchYawDeg,
                                                          MaxRange = Config.Lidar.MaxRange, Hz = Config.Lidar.Hz, PublishPcl24 = true } };
            if (_jt128 != null)
            {
                _jt128.SetActive(false);
                foreach (var e in lidars)
                {
                    try { log.AppendLine("  " + MakeLidar(e)); }
                    catch (Exception ex) { log.AppendLine($"  LiDAR {e.Name}: HATA {ex.Message}"); }
                }
            }

            // Stereo kameralar: sahne kurucusunun eklediği (kapalı) ZED X nesnesi şablondur.
            var cams = Config.Cameras != null && Config.Cameras.Count > 0 ? Config.Cameras : null;
            if (_zedx != null)
            {
                _zedx.gameObject.SetActive(false);
                if (cams == null)
                {
                    var s0 = Config.Stereo;
                    cams = new List<CameraEntry> { new CameraEntry { Name = "zedx", Enabled = s0.Enabled, Width = s0.Width, Height = s0.Height, Hz = s0.Hz,
                        BaselineM = s0.BaselineM, FocalLengthMm = s0.FocalLengthMm, PixelSizeUm = s0.PixelSizeUm,
                        Position = s0.Position, RollPitchYawDeg = s0.RollPitchYawDeg } };
                }
                foreach (var c in cams)
                {
                    if (!c.Enabled) { log.AppendLine($"  ZED X {c.Name}: kapalı"); continue; }
                    var go = Instantiate(_zedx.gameObject, _baseLink != null ? _baseLink : _zedx.transform.parent);
                    go.name = c.Name + "_camera_center";
                    var st = go.GetComponent<BeytepeStereoCamera>();
                    SetPoseInBaseLink(go.transform, c.Position, c.RollPitchYawDeg);
                    st.SetName(c.Name);
                    st.Configure(c.Width, c.Height, c.Hz, c.BaselineM, c.FocalLengthMm, c.PixelSizeUm);
                    go.SetActive(true);
                    float fx = c.FocalLengthMm / (c.PixelSizeUm * 1e-3f);
                    log.AppendLine($"  ZED X {c.Name}: {c.Width}x{c.Height} @ {c.Hz} Hz, taban {c.BaselineM} m, fx {fx:F1} px " +
                                   $"(YGA {2f * Mathf.Atan(0.5f * c.Width / fx) * Mathf.Rad2Deg:F1}°), merkez base_link {Fmt(c.Position)} rpy {Fmt(c.RollPitchYawDeg)}");
                }
            }

            // Xsens MTi-7
            if (_mti7 != null)
            {
                var n = Config.Ins;
                if (n.Enabled)
                {
                    SetPoseInBaseLink(_mti7.transform, n.Position, n.RollPitchYawDeg);
                    _mti7.Configure(n.ImuHz, n.GnssHz, n.Noise);
                    if (_mti7.Antenna != null)
                        SetPoseInBaseLink(_mti7.Antenna, n.AntennaPosition, Vector3.zero);
                    _mti7.gameObject.SetActive(true);
                    log.AppendLine($"  Xsens MTi-7: IMU {n.ImuHz} Hz, GNSS {n.GnssHz} Hz, gürültü {(n.Noise ? "açık" : "kapalı")}, " +
                                   $"base_link {Fmt(n.Position)} rpy {Fmt(n.RollPitchYawDeg)}, anten {Fmt(n.AntennaPosition)}");
                }
                else
                {
                    _mti7.gameObject.SetActive(false);
                    log.AppendLine("  Xsens MTi-7: kapalı");
                }
            }

            // AWSIM GNSS (Autoware ilk konum) -> MTi-7 anteni
            if (_lexusGnss != null)
            {
                SetPoseInBaseLink(_lexusGnss, Config.Ins.AntennaPosition, Vector3.zero);
                log.AppendLine($"  AWSIM GNSS (/sensing/gnss/pose_with_covariance) anten konumuna taşındı: {Fmt(Config.Ins.AntennaPosition)}");
            }
            Debug.Log(log.ToString());
        }

        string MakeLidar(LidarEntry e)
        {
            var model = (LidarModel)Enum.Parse(typeof(LidarModel), e.Model);
            var go = Instantiate(_jt128, _baseLink != null ? _baseLink : _jt128.transform.parent);   // şablon kapalı -> kopya kapalı
            go.name = e.FrameId;
            SetPoseInBaseLink(go.transform, e.Position, e.RollPitchYawDeg);

            var cfg = LidarConfigurationLibrary.ByModel[model]();
            var la = cfg.laserArray;
            int minRing = la.lasers.Min(l => l.ringId);   // Autoware ring_outlier_filter halka 0..127 bekler
            for (int i = 0; i < la.lasers.Length; i++)
            {
                la.lasers[i].ringId -= minRing;
                if (e.MaxRange > 0f) la.lasers[i].maxRange = e.MaxRange;
            }
            cfg.laserArray = la;
            if (e.MaxRange > 0f && cfg is UniformRangeLidarConfiguration u) u.maxRange = e.MaxRange;
            if (e.HorizontalResolutionDeg > 0f) cfg.horizontalResolution = e.HorizontalResolutionDeg;

            var lidar = go.GetComponent<LidarSensor>();
            lidar.modelPreset = model;
            lidar.configuration = cfg;
            lidar.AutomaticCaptureHz = e.Hz;
            lidar.doValidateConfigurationOnStartup = false;

            var model3d = go.transform.Find("sensor_model");   // görsel model boyutu (yalnız ekranda; RGL lidar altını dışlar)
            if (model3d != null)
                model3d.localScale = model == LidarModel.HesaiQT128C2X ? new Vector3(0.11f, 0.06f, 0.11f)   // QT128 ~Φ110x120 mm
                                   : model == LidarModel.HesaiJT128 ? new Vector3(0.06f, 0.034f, 0.06f) : new Vector3(0.10f, 0.04f, 0.10f);

            var pub = go.GetComponent<Awsim.Entity.RglLidarPublisher>();
            pub.frameId = e.FrameId;
            var list = new List<Awsim.Entity.PointCloud2Publisher>();
            foreach (var p in pub.pointCloud2Publishers)
            {
                bool ex = p.topic.EndsWith("_ex");
                if (!ex && !e.PublishPcl24) continue;
                list.Add(new Awsim.Entity.PointCloud2Publisher { topic = $"/sensing/lidar/{e.Name}/" + (ex ? "pointcloud_raw_ex" : "pointcloud_raw"),
                                                                 publish = true, fieldsPreset = p.fieldsPreset, fields = p.fields });
            }
            pub.pointCloud2Publishers = list;
            go.SetActive(true);
            var rng = cfg.GetRayRanges();
            return $"LiDAR {e.Name} ({e.Model}, çerçeve {e.FrameId}): {la.lasers.Length} kanal x {cfg.HorizontalSteps} = {cfg.PointCloudSize} ışın, " +
                   $"{e.Hz} Hz, menzil en fazla {rng.Max(r => r.y):F0} m, base_link {Fmt(e.Position)} rpy {Fmt(e.RollPitchYawDeg)}";
        }

        IEnumerator Start()
        {
            if (Config == null || !string.Equals(Config.Name, "husmart", StringComparison.OrdinalIgnoreCase) || !Config.DisableLexusCamera)
                yield break;
            yield return null;   // EgoVehicle.Initialize zamanlayıcıyı başlatmış olsun
            int n = 0;
            foreach (var sch in GetComponentsInChildren<CameraSensorScheduler>(true)) { sch.StopAllCoroutines(); n++; }
            Debug.Log($"[Beytepe] Lexus kamera zamanlayıcısı durduruldu ({n})");
        }

        RigConfig LoadConfig(out string source)
        {
            source = "json yok";
            var path = CommandLineUtility.GetCommandLineArg("--json_path");
            if (string.IsNullOrEmpty(path))
                path = Environment.GetEnvironmentVariable("BEYTEPE_SENSOR_JSON");   // düzenleyicide deneme için
            if (string.IsNullOrEmpty(path) || !File.Exists(path))
                return new RigConfig();
            try
            {
                var root = JsonUtility.FromJson<JsonRoot>(File.ReadAllText(path));
                source = path;
                return root?.SensorRig ?? new RigConfig();
            }
            catch (Exception e)
            {
                Debug.LogError($"[Beytepe] sensör seti okunamadı ({path}): {e.Message}; özgün sensörler kullanılıyor");
                source = "okuma hatası";
                return new RigConfig();
            }
        }

        /// <summary>ROS base_link çerçevesindeki konum/roll-pitch-yaw'ı Unity yerel pozuna çevirip uygular.</summary>
        void SetPoseInBaseLink(Transform t, Vector3 rosPos, Vector3 rosRpyDeg)
        {
            if (t == null) return;
            if (_baseLink != null && t.parent != _baseLink)
                t.SetParent(_baseLink, false);
            t.localPosition = Ros2Utility.Ros2ToUnityPosition(rosPos);
            t.localRotation = RosRpyToUnity(rosRpyDeg);
        }

        /// <summary>ROS (sağ el, x ileri z yukarı) roll/pitch/yaw -> Unity (sol el) dönüşü.</summary>
        public static Quaternion RosRpyToUnity(Vector3 rpyDeg)
        {
            double r = rpyDeg.x * Math.PI / 180.0, p = rpyDeg.y * Math.PI / 180.0, y = rpyDeg.z * Math.PI / 180.0;
            double cr = Math.Cos(r / 2), sr = Math.Sin(r / 2), cp = Math.Cos(p / 2), sp = Math.Sin(p / 2), cy = Math.Cos(y / 2), sy = Math.Sin(y / 2);
            // q = Rz(yaw) * Ry(pitch) * Rx(roll)  (ROS / URDF kuralı)
            var qRos = new Quaternion(
                (float)(sr * cp * cy - cr * sp * sy),
                (float)(cr * sp * cy + sr * cp * sy),
                (float)(cr * cp * sy - sr * sp * cy),
                (float)(cr * cp * cy + sr * sp * sy));
            return Ros2Utility.Ros2ToUnityQuaternion(qRos);
        }

        static string Fmt(Vector3 v) => $"({v.x:F2}, {v.y:F2}, {v.z:F2})";
    }
}
