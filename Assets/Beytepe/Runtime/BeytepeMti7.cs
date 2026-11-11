using System;
using System.Collections;
using UnityEngine;
using ROS2;
using Awsim.Common;

namespace Beytepe.Runtime
{
    /// <summary>
    /// Xsens MTi-7 GNSS/INS benzetimi (MTi-7 + harici u-blox GNSS alıcısı).
    ///
    /// Yayınlar:
    ///   /sensing/imu/mti7/imu_raw               sensor_msgs/Imu, ImuHz (100 Hz)   çerçeve mti7_imu_link
    ///        açısal hız + özgül kuvvet (durağanda +9.81 yukarı, REP-145) + INS yönü (ENU = harita çerçevesi)
    ///   /sensing/gnss/mti7/nav_sat_fix          sensor_msgs/NavSatFix, GnssHz      çerçeve gnss_link (anten)
    ///        enlem/boylam: harita çerçevesinin gerçek coğrafi referansı (map_projector_info.yaml: tmerc,
    ///        orijin 39.87116440956932 N, 32.7356890084472 E, k=1, WGS84) ile ters Transverse Mercator.
    ///        Yükseklik = harita "ele" değeri + _altitudeOffsetM (haritanın yükseklik datumu bilinmiyor).
    ///   /sensing/ins/mti7/pose_with_covariance  geometry_msgs/PoseWithCovarianceStamped, 50 Hz, çerçeve map
    ///        INS çözümü: MTi-7'nin harita çerçevesindeki konumu ve yönü.
    ///
    /// Gürültü (MTi-7 yeni revizyon datasheet değerleri; ivmeölçer için kaynaklar 70-120 µg/√Hz arasında):
    ///   jiroskop beyaz gürültü 0.003 °/s/√Hz, sapma kararsızlığı 6 °/h (Gauss-Markov, τ 100 s)
    ///   ivmeölçer beyaz gürültü 70 µg/√Hz, sapma kararsızlığı 40 µg (Gauss-Markov, τ 100 s)
    ///   INS yön hatası roll/pitch 0.2°, yaw 0.8° (yavaş değişen, τ 30 s) — VARSAYIM
    ///   GNSS yatay 0.8 m / düşey 1.5 m (yavaş değişen, τ 60 s) + 0.15 m beyaz — VARSAYIM (u-blox M8 sınıfı, SBAS yok)
    /// Fizik adımı 100 Hz (0.01 s); ölçümler fizik adımında, o adımın benzetim zamanıyla damgalanır.
    /// </summary>
    public class BeytepeMti7 : MonoBehaviour
    {
        [SerializeField] Transform _antenna;
        public Transform Antenna => _antenna;

        [Header("Hızlar")]
        [SerializeField] int _imuHz = 100;
        [SerializeField] float _gnssHz = 4.0f;
        [SerializeField] float _insPoseHz = 50.0f;
        [SerializeField] bool _noise = true;

        [Header("Konular / çerçeveler")]
        [SerializeField] string _imuTopic = "/sensing/imu/mti7/imu_raw";
        [SerializeField] string _imuFrame = "mti7_imu_link";
        [SerializeField] string _navSatFixTopic = "/sensing/gnss/mti7/nav_sat_fix";
        [SerializeField] string _gnssFrame = "gnss_link";
        [SerializeField] string _insPoseTopic = "/sensing/ins/mti7/pose_with_covariance";

        [Header("IMU gürültüsü (MTi-7)")]
        [SerializeField] float _gyroNoiseDensityDegPerSqrtHz = 0.003f;
        [SerializeField] float _gyroBiasInstabilityDegPerHour = 6.0f;
        [SerializeField] float _accelNoiseDensityUgPerSqrtHz = 70.0f;
        [SerializeField] float _accelBiasInstabilityUg = 40.0f;
        [SerializeField] float _biasTauSec = 100.0f;

        [Header("INS / GNSS hatası (varsayım)")]
        [SerializeField] float _rollPitchErrorDeg = 0.2f;
        [SerializeField] float _yawErrorDeg = 0.8f;
        [SerializeField] float _attitudeTauSec = 30.0f;
        [SerializeField] float _gnssHorizontalStdM = 0.8f;
        [SerializeField] float _gnssVerticalStdM = 1.5f;
        [SerializeField] float _gnssWhiteStdM = 0.15f;
        [SerializeField] float _gnssTauSec = 60.0f;

        [Header("Coğrafi referans (map_projector_info.yaml)")]
        [SerializeField] double _originLatDeg = 39.87116440956932;
        [SerializeField] double _originLonDeg = 32.7356890084472;
        [SerializeField] double _altitudeOffsetM = 0.0;

        Rigidbody _rb;
        bool _ready, _hasLast;
        Vector3 _lastVel, _lastPos;
        Quaternion _lastRot;
        double _imuAcc, _gnssAcc, _insAcc;
        System.Random _rng;
        Vector3 _gyroBias, _accelBias, _attErr, _gnssErr;   // ROS gövde / ENU eksenlerinde
        IPublisher<sensor_msgs.msg.Imu> _imuPub;
        IPublisher<sensor_msgs.msg.NavSatFix> _fixPub;
        IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> _insPub;
        sensor_msgs.msg.Imu _imuMsg;
        sensor_msgs.msg.NavSatFix _fixMsg;
        geometry_msgs.msg.PoseWithCovarianceStamped _insMsg;
        int _nImu, _nFix, _nIns;
        float _lastStat;

        /// <summary>Sensör seti (BeytepeSensorRig) tarafından Awake'te çağrılır.</summary>
        public void Configure(int imuHz, float gnssHz, bool noise)
        {
            _imuHz = Mathf.Max(1, imuHz);
            _gnssHz = Mathf.Max(0.1f, gnssHz);
            _noise = noise;
        }

        IEnumerator Start()
        {
            yield return null;   // AwsimRos2Node.Initialize tamamlansın
            _rb = GetComponentInParent<Rigidbody>();
            _rng = new System.Random(Environment.TickCount ^ 0x5eed);
            if (_noise)
            {
                // Açılışta sapmalar kararlı durum dağılımından çekilir.
                _gyroBias = RandVec(Deg2Rad(_gyroBiasInstabilityDegPerHour / 3600.0));
                _accelBias = RandVec(_accelBiasInstabilityUg * 1e-6 * 9.80665);
                _attErr = new Vector3((float)(Gauss() * Deg2Rad(_rollPitchErrorDeg)), (float)(Gauss() * Deg2Rad(_rollPitchErrorDeg)), (float)(Gauss() * Deg2Rad(_yawErrorDeg)));
                _gnssErr = new Vector3((float)(Gauss() * _gnssHorizontalStdM), (float)(Gauss() * _gnssHorizontalStdM), (float)(Gauss() * _gnssVerticalStdM));
            }

            var reliable = new QosSettings(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE,
                                           DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE,
                                           HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1000).GetQosProfile();
            _imuPub = AwsimRos2Node.CreatePublisher<sensor_msgs.msg.Imu>(_imuTopic, reliable);
            _fixPub = AwsimRos2Node.CreatePublisher<sensor_msgs.msg.NavSatFix>(_navSatFixTopic, reliable);
            _insPub = AwsimRos2Node.CreatePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_insPoseTopic, reliable);

            double gyroStd = _noise ? Deg2Rad(_gyroNoiseDensityDegPerSqrtHz) * Math.Sqrt(_imuHz) : 0.0;
            double accStd = _noise ? _accelNoiseDensityUgPerSqrtHz * 1e-6 * 9.80665 * Math.Sqrt(_imuHz) : 0.0;
            _imuMsg = new sensor_msgs.msg.Imu
            {
                Header = new std_msgs.msg.Header { Frame_id = _imuFrame },
                Orientation = new geometry_msgs.msg.Quaternion { W = 1 },
                Angular_velocity = new geometry_msgs.msg.Vector3(),
                Linear_acceleration = new geometry_msgs.msg.Vector3(),
            };
            double rp2 = Math.Pow(Deg2Rad(_rollPitchErrorDeg), 2), y2 = Math.Pow(Deg2Rad(_yawErrorDeg), 2);
            double g2 = Math.Max(gyroStd * gyroStd, 1e-8), a2 = Math.Max(accStd * accStd, 1e-6);
            for (int i = 0; i < 9; i++)
            {
                _imuMsg.Orientation_covariance[i] = 0;
                _imuMsg.Angular_velocity_covariance[i] = 0;
                _imuMsg.Linear_acceleration_covariance[i] = 0;
            }
            _imuMsg.Orientation_covariance[0] = rp2; _imuMsg.Orientation_covariance[4] = rp2; _imuMsg.Orientation_covariance[8] = y2;
            _imuMsg.Angular_velocity_covariance[0] = g2; _imuMsg.Angular_velocity_covariance[4] = g2; _imuMsg.Angular_velocity_covariance[8] = g2;
            _imuMsg.Linear_acceleration_covariance[0] = a2; _imuMsg.Linear_acceleration_covariance[4] = a2; _imuMsg.Linear_acceleration_covariance[8] = a2;

            _fixMsg = new sensor_msgs.msg.NavSatFix
            {
                Header = new std_msgs.msg.Header { Frame_id = _gnssFrame },
                Status = new sensor_msgs.msg.NavSatStatus
                {
                    Status = sensor_msgs.msg.NavSatStatus.STATUS_FIX,
                    Service = (ushort)(sensor_msgs.msg.NavSatStatus.SERVICE_GPS | sensor_msgs.msg.NavSatStatus.SERVICE_GLONASS | sensor_msgs.msg.NavSatStatus.SERVICE_GALILEO),
                },
                Position_covariance_type = sensor_msgs.msg.NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            };
            double h2 = _gnssHorizontalStdM * _gnssHorizontalStdM + _gnssWhiteStdM * _gnssWhiteStdM;
            double v2 = _gnssVerticalStdM * _gnssVerticalStdM + _gnssWhiteStdM * _gnssWhiteStdM;
            for (int i = 0; i < 9; i++) _fixMsg.Position_covariance[i] = 0;
            _fixMsg.Position_covariance[0] = h2; _fixMsg.Position_covariance[4] = h2; _fixMsg.Position_covariance[8] = v2;

            _insMsg = new geometry_msgs.msg.PoseWithCovarianceStamped
            {
                Header = new std_msgs.msg.Header { Frame_id = "map" },
                Pose = new geometry_msgs.msg.PoseWithCovariance(),
            };
            for (int i = 0; i < 36; i++) _insMsg.Pose.Covariance[i] = 0;
            _insMsg.Pose.Covariance[0] = h2; _insMsg.Pose.Covariance[7] = h2; _insMsg.Pose.Covariance[14] = v2;
            _insMsg.Pose.Covariance[21] = rp2; _insMsg.Pose.Covariance[28] = rp2; _insMsg.Pose.Covariance[35] = y2;

            _lastStat = Time.time;
            _ready = true;
            Debug.Log($"[Beytepe] Xsens MTi-7 hazır: IMU {_imuHz} Hz (jiroskop σ {gyroStd:E2} rad/s, ivme σ {accStd:E2} m/s²), " +
                      $"GNSS {_gnssHz} Hz, INS {_insPoseHz} Hz, gürültü {(_noise ? "açık" : "kapalı")}");
        }

        void FixedUpdate()
        {
            if (!_ready) return;
            float dt = Time.fixedDeltaTime;
            var pos = transform.position;
            var rot = transform.rotation;
            Vector3 vel = _rb != null ? _rb.GetPointVelocity(pos) : (_hasLast ? (pos - _lastPos) / dt : Vector3.zero);
            Vector3 wWorld;
            if (_rb != null)
                wWorld = _rb.angularVelocity;
            else if (_hasLast)
            {
                (rot * Quaternion.Inverse(_lastRot)).ToAngleAxis(out var ang, out var axis);
                if (ang > 180f) ang -= 360f;
                wWorld = float.IsNaN(axis.x) ? Vector3.zero : axis * (ang * Mathf.Deg2Rad / dt);
            }
            else
                wWorld = Vector3.zero;

            if (!_hasLast)
            {
                _lastVel = vel; _lastPos = pos; _lastRot = rot; _hasLast = true;
                return;
            }
            Vector3 accWorld = (vel - _lastVel) / dt;
            _lastVel = vel; _lastPos = pos; _lastRot = rot;

            if (_noise) StepErrors(dt);

            AwsimRos2Node.GetTime(out var sec, out var nsec);   // FixedUpdate içinde: bu fizik adımının zamanı

            _imuAcc += dt;
            double imuPeriod = 1.0 / _imuHz;
            if (_imuAcc + 1e-6 >= imuPeriod)
            {
                _imuAcc -= imuPeriod;
                // Özgül kuvvet (ivmeölçerin ölçtüğü): a - g  (Unity: g = (0, -9.81, 0)), gövde eksenlerinde
                var fRos = Ros2Utility.UnityToRos2Position(transform.InverseTransformDirection(accWorld - Physics.gravity));
                var wRos = Ros2Utility.UnityToRos2AngularVelocity(transform.InverseTransformDirection(wWorld));
                if (_noise)
                {
                    double gStd = Deg2Rad(_gyroNoiseDensityDegPerSqrtHz) * Math.Sqrt(_imuHz);
                    double aStd = _accelNoiseDensityUgPerSqrtHz * 1e-6 * 9.80665 * Math.Sqrt(_imuHz);
                    wRos += _gyroBias + RandVec(gStd);
                    fRos += _accelBias + RandVec(aStd);
                }
                var q = InsOrientationRos(rot);
                _imuMsg.Orientation.X = q.x; _imuMsg.Orientation.Y = q.y; _imuMsg.Orientation.Z = q.z; _imuMsg.Orientation.W = q.w;
                _imuMsg.Angular_velocity.X = wRos.x; _imuMsg.Angular_velocity.Y = wRos.y; _imuMsg.Angular_velocity.Z = wRos.z;
                _imuMsg.Linear_acceleration.X = fRos.x; _imuMsg.Linear_acceleration.Y = fRos.y; _imuMsg.Linear_acceleration.Z = fRos.z;
                _imuMsg.Header.Stamp.Sec = sec; _imuMsg.Header.Stamp.Nanosec = nsec;
                _imuPub.Publish(_imuMsg);
                _nImu++;
            }

            _insAcc += dt;
            if (_insAcc + 1e-6 >= 1.0 / _insPoseHz)
            {
                _insAcc -= 1.0 / _insPoseHz;
                var p = MapPosition(pos) + (_noise ? _gnssErr : Vector3.zero);
                var q = InsOrientationRos(rot);
                _insMsg.Pose.Pose.Position.X = p.x; _insMsg.Pose.Pose.Position.Y = p.y; _insMsg.Pose.Pose.Position.Z = p.z;
                _insMsg.Pose.Pose.Orientation.X = q.x; _insMsg.Pose.Pose.Orientation.Y = q.y; _insMsg.Pose.Pose.Orientation.Z = q.z; _insMsg.Pose.Pose.Orientation.W = q.w;
                _insMsg.Header.Stamp.Sec = sec; _insMsg.Header.Stamp.Nanosec = nsec;
                _insPub.Publish(_insMsg);
                _nIns++;
            }

            _gnssAcc += dt;
            if (_gnssAcc + 1e-6 >= 1.0 / _gnssHz)
            {
                _gnssAcc -= 1.0 / _gnssHz;
                var a = MapPosition(_antenna != null ? _antenna.position : pos);
                if (_noise)
                    a += _gnssErr + RandVec(_gnssWhiteStdM);
                TmercInverse(a.x, a.y, out var lat, out var lon);
                _fixMsg.Latitude = lat;
                _fixMsg.Longitude = lon;
                _fixMsg.Altitude = a.z + _altitudeOffsetM;
                _fixMsg.Header.Stamp.Sec = sec; _fixMsg.Header.Stamp.Nanosec = nsec;
                _fixPub.Publish(_fixMsg);
                _nFix++;
            }

            if (Time.time - _lastStat >= 30f)
            {
                Debug.Log($"[Beytepe] MTi-7: {_nImu} IMU, {_nIns} INS, {_nFix} GNSS mesajı yayınlandı");
                _lastStat = Time.time;
            }
        }

        // --- yardımcılar

        Vector3 MapPosition(Vector3 unityPos)
        {
            var p = Ros2Utility.UnityToRos2Position(unityPos);
            if (MgrsPosition.Instance != null) p += MgrsPosition.Instance.Mgrs.Position;   // Beytepe: 0
            return p;
        }

        Quaternion InsOrientationRos(Quaternion unityRot)
        {
            var q = Ros2Utility.UnityToRosRotation(unityRot);    // harita (ENU) <- mti7_imu_link
            if (_noise)
            {
                // yavaş değişen INS yön hatası: önce gövde roll/pitch, sonra ENU yaw
                var errRp = AxisAngleRos(new Vector3(1, 0, 0), _attErr.x) * AxisAngleRos(new Vector3(0, 1, 0), _attErr.y);
                var errYaw = AxisAngleRos(new Vector3(0, 0, 1), _attErr.z);
                q = errYaw * q * errRp;
            }
            return q;
        }

        // ROS (sağ el) eksen-açı -> quaternion (Unity Quaternion yapısı yalnız saklama için kullanılıyor)
        static Quaternion AxisAngleRos(Vector3 axis, float angleRad)
        {
            float s = Mathf.Sin(angleRad / 2f);
            return new Quaternion(axis.x * s, axis.y * s, axis.z * s, Mathf.Cos(angleRad / 2f));
        }

        void StepErrors(float dt)
        {
            _gyroBias = GaussMarkov(_gyroBias, Deg2Rad(_gyroBiasInstabilityDegPerHour / 3600.0), _biasTauSec, dt);
            _accelBias = GaussMarkov(_accelBias, _accelBiasInstabilityUg * 1e-6 * 9.80665, _biasTauSec, dt);
            var rp = GaussMarkov(new Vector3(_attErr.x, _attErr.y, 0f), Deg2Rad(_rollPitchErrorDeg), _attitudeTauSec, dt);
            var yw = GaussMarkov(new Vector3(_attErr.z, 0f, 0f), Deg2Rad(_yawErrorDeg), _attitudeTauSec, dt);
            _attErr = new Vector3(rp.x, rp.y, yw.x);
            var h = GaussMarkov(new Vector3(_gnssErr.x, _gnssErr.y, 0f), _gnssHorizontalStdM, _gnssTauSec, dt);
            var v = GaussMarkov(new Vector3(_gnssErr.z, 0f, 0f), _gnssVerticalStdM, _gnssTauSec, dt);
            _gnssErr = new Vector3(h.x, h.y, v.x);
        }

        Vector3 GaussMarkov(Vector3 x, double sigma, double tau, double dt)
        {
            double phi = Math.Exp(-dt / tau);
            double q = sigma * Math.Sqrt(1.0 - phi * phi);
            return new Vector3((float)(x.x * phi + q * Gauss()), (float)(x.y * phi + q * Gauss()), (float)(x.z * phi + q * Gauss()));
        }

        Vector3 RandVec(double sigma) => new Vector3((float)(sigma * Gauss()), (float)(sigma * Gauss()), (float)(sigma * Gauss()));

        double Gauss()
        {
            double u1 = 1.0 - _rng.NextDouble(), u2 = _rng.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        static double Deg2Rad(double d) => d * Math.PI / 180.0;

        /// <summary>
        /// Ters Transverse Mercator (Snyder 1987 serileri), WGS84, k0 = 1, orijin (_originLat, _originLon), yanlış
        /// doğu/kuzey 0. Haritanın düğümlerindeki lat/lon ile karşılaştırıldı: hata ~0.1 mm (kampüs ölçeğinde).
        /// x = doğu, y = kuzey (harita çerçevesi).
        /// </summary>
        void TmercInverse(double x, double y, out double latDeg, out double lonDeg)
        {
            const double a = 6378137.0, f = 1.0 / 298.257223563, k0 = 1.0;
            double e2 = f * (2 - f), ep2 = e2 / (1 - e2);
            double lat0 = Deg2Rad(_originLatDeg), lon0 = Deg2Rad(_originLonDeg);
            double m = Meridian(lat0, a, e2) + y / k0;
            double mu = m / (a * (1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256));
            double e1 = (1 - Math.Sqrt(1 - e2)) / (1 + Math.Sqrt(1 - e2));
            double p1 = mu + (3 * e1 / 2 - 27 * Math.Pow(e1, 3) / 32) * Math.Sin(2 * mu)
                           + (21 * e1 * e1 / 16 - 55 * Math.Pow(e1, 4) / 32) * Math.Sin(4 * mu)
                           + (151 * Math.Pow(e1, 3) / 96) * Math.Sin(6 * mu)
                           + (1097 * Math.Pow(e1, 4) / 512) * Math.Sin(8 * mu);
            double s1 = Math.Sin(p1), c1 = Math.Cos(p1), t1 = Math.Tan(p1);
            double C1 = ep2 * c1 * c1, T1 = t1 * t1;
            double N1 = a / Math.Sqrt(1 - e2 * s1 * s1);
            double R1 = a * (1 - e2) / Math.Pow(1 - e2 * s1 * s1, 1.5);
            double D = x / (N1 * k0);
            double lat = p1 - (N1 * t1 / R1) * (D * D / 2
                         - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * ep2) * Math.Pow(D, 4) / 24
                         + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * ep2 - 3 * C1 * C1) * Math.Pow(D, 6) / 720);
            double lon = lon0 + (D - (1 + 2 * T1 + C1) * Math.Pow(D, 3) / 6
                         + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * ep2 + 24 * T1 * T1) * Math.Pow(D, 5) / 120) / c1;
            latDeg = lat * 180.0 / Math.PI;
            lonDeg = lon * 180.0 / Math.PI;
        }

        static double Meridian(double phi, double a, double e2)
        {
            return a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * phi
                        - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * Math.Sin(2 * phi)
                        + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * Math.Sin(4 * phi)
                        - (35 * e2 * e2 * e2 / 3072) * Math.Sin(6 * phi));
        }

        void OnDestroy()
        {
            if (_imuPub != null) AwsimRos2Node.RemovePublisher<sensor_msgs.msg.Imu>(_imuPub);
            if (_fixPub != null) AwsimRos2Node.RemovePublisher<sensor_msgs.msg.NavSatFix>(_fixPub);
            if (_insPub != null) AwsimRos2Node.RemovePublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(_insPub);
        }
    }
}
