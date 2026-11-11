using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.CompilerServices;
using UnityEngine;
using Awsim.Common;
using Awsim.Usecase.TrafficSimulation;

namespace Beytepe.Runtime
{
    /// <summary>
    /// NPC araçların egoyu (Autoware'in sürdüğü aracı) da bir araç gibi hesaba katması. 2026-11-09, 11. tur olayları üzerine.
    ///
    /// AWSIM'in kendi NPC mantığı egoyu yalnızca iki yolla görür:
    ///  1) öndeki engel taraması: NPC'nin şerit ortası boyunca 1 m genişliğinde bir kutu,
    ///  2) kavşakta öncelik: ego, NPC'nin yol vermesi gereken şeridin (RightOfWayLanes) tam üstündeyse.
    /// Beytepe haritasında öncelik bilgisi çok az (170 şeridin 14'ü kavşak şeridi) ve kutu dar. Sonuç (11. tur):
    ///  - (-205, 232): sol dönüş yapan NPC, kavşağa egodan yaklaşık 1 s önce girdi; Autoware sert frenledi (3.7 m/s²).
    ///  - (179, -235): ego hedef şeride geçerken arkadan 30 km/sa gelen NPC egoyu ancak yanına gelince gördü; çarpıştılar.
    ///
    /// Bu sınıf iki kural ekler. NPC'lerin birbirine karşı davranışı değişmez.
    ///  A) Kesme algısı: egonun gövdesi ya da CutInPredictSec sonraki tahmini konumu NPC'nin yol koridoruna
    ///     (NPC yarı genişliği + CutInMargin) girerse, ego NPC'nin "öndeki aracı" sayılır. NPC, AWSIM'in olağan
    ///     takip ve fren kurallarıyla yavaşlar (DistanceToFrontVehicle).
    ///  B) Kavşak ve birleşmede yol verme: NPC'nin önündeki yol, egonun gidebileceği şeritlerle (haritadan, tüm kollar)
    ///     kesişiyor ya da birleşiyorsa ve ego çakışma bölgesine YieldHorizonSec içinde varacaksa, NPC bölgeyi egodan
    ///     en az YieldMarginSec önce boşaltamayacaksa çakışmadan önce durur; ego geçince devam eder.
    ///     NPC en fazla YieldMaxDecel ile duramayacak kadar yakınsa (geçmeye kararlıysa) durmaz.
    ///  C) Ego beklerken bırakma (ReleaseWhenEgoWaits, 2026-11-09, 12. tur üzerine): ego bu NPC'yle çakışma noktasından
    ///     en az 1 m önce durmuşsa ya da orada durmak için frendeyse (Autoware zaten yol veriyor), NPC ego için beklemez:
    ///     B'deki yol vermesini hemen bırakır ve AWSIM'in kendi "öncelikli şeritte ego var" beklemesi de o NPC için kalkar
    ///     (IsEgoWaitingFor). 12. turda ikisi birbirini bekliyordu; ego 3 kavşakta 6-8 s durdu.
    ///     Bıraktıktan sonra NPC 2 s yeniden yol vermeye başlamaz (karar gidip gelmesin); sonra B'nin koşulları yine geçerli.
    ///     Ego çakışmanın içindeyse ya da önünü kapatıyorsa C uygulanmaz; ego NPC'nin önündeyse A ve engel taraması yine durdurur.
    ///
    /// Ayarlar: AWSIM'e verilen --json_path dosyasındaki "NpcEgo" bölümü (yoksa aşağıdaki varsayılanlar).
    /// "Enabled": false yazılırsa AWSIM'in özgün NPC davranışına dönülür (yeniden derleme gerekmez).
    /// Player.log'da "[Beytepe] NPC" ile başlayan satırlar her yol vermeyi, kesmeyi ve bırakmayı yazar.
    /// Bağlantılar (AWSIM dosyalarında, "Beytepe:" yorumlu satırlar): NpcVehicleCognitionStep.Execute bu sınıfın
    /// Apply'ını çağırır ve öncelik denetiminde (ShouldYieldDueToLanes) IsEgoWaitingFor'u sorar; NpcVehicleDecisionStep
    /// durma noktasına DistanceToEgoYieldPoint'i katar; NpcVehicleInternalState o alanı taşır.
    /// </summary>
    public static class BeytepeNpcEgoAwareness
    {
        [Serializable]
        public class Settings
        {
            public bool Enabled = true;           // false: AWSIM'in özgün NPC davranışı
            public bool CutInEnabled = true;      // A) kesme algısı
            public float CutInMargin = 0.5f;      // m: NPC yarı genişliğine eklenen pay
            public float CutInPredictSec = 1.0f;  // s: egonun bu kadar sonraki tahmini konumu da denetlenir (0 = yalnız şimdiki)
            public bool YieldEnabled = true;      // B) kavşak ve birleşmede yol verme
            public float YieldHorizonSec = 7f;    // s: ego çakışma bölgesine bundan geç varacaksa dikkate alınmaz
            public float YieldMarginSec = 1.5f;   // s: NPC bölgeyi egonun varışından en az bu kadar önce boşaltmalı (12. turdan sonra 3'ten 1.5'e)
            public float YieldMaxDecel = 3f;      // m/s²: NPC ancak bu yavaşlamayla durabiliyorsa yol vermeye başlar
            public bool ReleaseWhenEgoWaits = true; // C) ego çakışmadan önce bekliyorsa NPC beklemez (B ve AWSIM'in ego önceliği)
            public bool Log = true;               // Player.log'a "[Beytepe] NPC ..." satırları
        }

        [Serializable]
        class JsonRoot
        {
            public Settings NpcEgo = new Settings();
        }

        class NpcData
        {
            public bool Yielding;
            public float YieldSince;
            public bool CutIn;
            public float CutInSince, CutInLastSeen, CutInMinDist;
            public int CutInMaxMode;
            public bool EgoWaits;                 // C) ego bu NPC'yle çakışmadan önce bekliyor (son Apply adımında)
            public float EgoWaitsAt = -1f;
            public float NoYieldUntil;            // C) bırakmadan sonra bu ana kadar yeniden yol verme yok
            public bool RowReleased;              // C) AWSIM'in ego önceliği bu bekleyişte kaldırıldı (bir kez yazmak için)
        }

        struct Seg
        {
            public Vector2 A, B;
            public float Sa, Sb;              // egonun yolu boyunca uzaklık (m, base_link izdüşümünden; arkası eksi)
            public float X0, X1, Y0, Y1;      // sınır kutusu
        }

        const float LaneMatchDist = 2.5f;     // m: ego bu kadar yakın şeritlerin üzerinde sayılır
        const float LaneMatchDeg = 45f;       // °: şerit yönü ile ego yönü arasındaki en büyük fark
        const float NpcLookahead = 60f;       // m: NPC'nin önündeki yolun incelenen uzunluğu
        const float CutInRange = 90f;         // m: ego bundan uzaksa A denetlenmez
        const float YieldRange = 220f;        // m: ego bundan uzaksa B denetlenmez
        const float ConflictPad = 0.3f;       // m: iki aracın yarı genişlikleri toplamına eklenen pay
        const float StopGap = 1.5f;           // m: NPC çakışma noktasından bu kadar önce durmayı hedefler
        const float LaneEntryMaxBack = 25f;   // m: çakışma şeridin girişine bundan yakınsa NPC şerit girişinde bekler
        const float MaxYieldSec = 40f;        // s: bir NPC en fazla bu kadar bekler (kilitlenmeye karşı)
        const float CoDirDeg = 35f;           // °: bundan küçük açı = aynı yöne birleşme
        const float HeadOnDeg = 150f;         // °: bundan büyük açı = karşı yönden paralel (çakışma sayılmaz)
        const float EgoWaitMinGap = 1f;       // m: C) ego çakışmadan en az bu kadar önce duruyorsa "bekliyor"
        const float EgoWaitZone = 20f;        // m: C) egonun tahmini duruş yeri çakışmadan en çok bu kadar önce olmalı
        const float EgoWaitMinDecel = 0.6f;   // m/s²: C) bundan hafif yavaşlama fren sayılmaz
        const float EgoStoppedSpeed = 0.3f;   // m/s: C) bunun altı "durmuş"
        const float ReleaseCooldownSec = 2f;  // s: C) bırakmadan sonra yeniden yol vermeye başlamadan önce
        const float DecelSmoothSec = 0.3f;    // s: egonun yavaşlaması bu zaman sabitiyle yumuşatılır

        static Settings S = new Settings();
        static bool _init;
        static string _source = "varsayılan";
        static TrafficLane[] _lanes = Array.Empty<TrafficLane>();
        static float _egoFront = 3.79f, _egoRear = 1.10f, _egoHalfW = 0.95f;   // Autoware sample_vehicle (Lexus) ölçüleri
        static string _egoDimSource = "varsayılan";
        static Rigidbody _egoRb;

        static Vector2 _egoPos, _egoDir;
        static float _egoYaw, _egoSpeed, _egoYawRate;
        static float _egoDecel;               // m/s², yumuşatılmış; + = fren
        static Vector2 _lastEgoPos;
        static float _lastEgoYaw, _lastEgoSpeed, _lastTime = -1f;

        static readonly List<Vector2> _egoPts = new List<Vector2>(32);
        static readonly List<Seg> _tube = new List<Seg>(256);
        static bool _tubeOk;
        static readonly List<Vector2> _np = new List<Vector2>(64);
        static readonly List<float> _ns = new List<float>(64);
        static readonly List<TrafficLane> _nl = new List<TrafficLane>(64);         // _nl[j]: _np[j] -> _np[j+1] parçasının şeridi
        static readonly List<TrafficLane> _laneStartLane = new List<TrafficLane>(8);
        static readonly List<float> _laneStartS = new List<float>(8);
        static readonly Dictionary<TrafficLane, float> _visited = new Dictionary<TrafficLane, float>();
        static readonly Stack<(TrafficLane lane, float s, int depth)> _stack = new Stack<(TrafficLane lane, float s, int depth)>();
        static readonly ConditionalWeakTable<NpcVehicleInternalState, NpcData> _data = new ConditionalWeakTable<NpcVehicleInternalState, NpcData>();
        static readonly ConditionalWeakTable<NpcVehicleInternalState, NpcData>.CreateValueCallback _create = _ => new NpcData();

        /// <summary>Oturum boyunca başlayan yol verme, kesme ve (C) bırakma olayı sayıları.</summary>
        public static int YieldCount { get; private set; }
        public static int CutInCount { get; private set; }
        public static int ReleaseCount { get; private set; }

        /// <summary>
        /// NpcVehicleCognitionStep.Execute sonunda, her fizik adımında bir kez çağrılır.
        /// A için DistanceToFrontVehicle'ı kısaltabilir, B için DistanceToEgoYieldPoint'i yazar.
        /// </summary>
        public static void Apply(IReadOnlyList<NpcVehicleInternalState> states, Transform ego)
        {
            if (states == null) return;
            try
            {
                if (!_init) Init(ego);
                if (!S.Enabled || ego == null)
                {
                    ClearYield(states);
                    return;
                }
                Step(states, ego);
            }
            catch (Exception e)
            {
                // Beklenmedik hata: NPC benzetimi durmasın, AWSIM'in özgün davranışına dönülsün.
                S.Enabled = false;
                ClearYield(states);
                Debug.LogError($"[Beytepe] NPC ego farkındalığı hata verdi ve KAPATILDI (NPC'ler özgün AWSIM davranışında): {e}");
            }
        }

        static void ClearYield(IReadOnlyList<NpcVehicleInternalState> states)
        {
            foreach (var st in states) if (st != null) st.DistanceToEgoYieldPoint = float.MaxValue;
        }

        /// <summary>
        /// C) AWSIM'in öncelik denetimi (NpcVehicleCognitionStep, ShouldYieldDueToLanes içindeki "öncelikli şeritte ego var"
        /// dalı) bunu sorar. true: ego bu NPC'yle çakışma noktasından önce bekliyor (durmuş ya da durmak için frende),
        /// yani NPC ego için beklemesin. Değer bir önceki adımdaki Apply'dan gelir (en çok 0.25 s eski). Hata olursa false
        /// (AWSIM'in özgün davranışı).
        /// </summary>
        public static bool IsEgoWaitingFor(NpcVehicleInternalState st)
        {
            try
            {
                if (!_init || !S.Enabled || !S.ReleaseWhenEgoWaits || st == null) return false;
                if (!_data.TryGetValue(st, out var d) || !d.EgoWaits || Time.fixedTime - d.EgoWaitsAt > 0.25f) return false;
                if (!d.RowReleased)
                {
                    d.RowReleased = true;
                    ReleaseCount++;
                    if (S.Log)
                        Debug.Log($"[Beytepe] NPC {Name(st)}: ego öncelikli şeritte bekliyor; NPC ego için durmuyor " +
                                  $"(ego {_egoSpeed * 3.6f:F0} km/sa), NPC {Ros(V2(st.FrontCenterPosition))}");
                }
                return true;
            }
            catch (Exception)
            {
                return false;
            }
        }

        static void Step(IReadOnlyList<NpcVehicleInternalState> states, Transform ego)
        {
            float now = Time.fixedTime;
            UpdateEgo(ego, now);
            _tubeOk = S.YieldEnabled && BuildEgoTube();

            foreach (var st in states)
            {
                if (st == null) continue;
                st.DistanceToEgoYieldPoint = float.MaxValue;
                if (st.ShouldDespawn || st.Vehicle == null || st.CurrentFollowingLane == null || st.FollowingLanes.Count == 0)
                    continue;

                var d = _data.GetValue(st, _create);
                if (!d.EgoWaits) d.RowReleased = false;   // C) önceki adımda beklemiyordu: yeni bekleyiş yine yazılsın
                d.EgoWaits = false;                       // C) aşağıda Yield yeniden hesaplar
                var front = V2(st.FrontCenterPosition);
                float egoDist = Vector2.Distance(front, _egoPos);
                bool needA = S.CutInEnabled && egoDist < CutInRange;
                bool needB = _tubeOk && egoDist < YieldRange;
                if (!needA && !needB)
                {
                    if (d.CutIn) EndCutIn(st, d, now);
                    if (d.Yielding) EndYield(st, d, now, "ego uzaklaştı");
                    continue;
                }

                BuildNpcPath(st);
                if (_np.Count < 3) continue;

                if (needA) CutIn(st, d, now);
                else if (d.CutIn) EndCutIn(st, d, now);

                if (needB) st.DistanceToEgoYieldPoint = Yield(st, d, now);
                else if (d.Yielding) EndYield(st, d, now, "ego durdu ya da uzaklaştı");
            }
        }

        // ---------------------------------------------------------------- başlatma

        static void Init(Transform ego)
        {
            _init = true;
            S = LoadSettings(out _source);
            try
            {
                _lanes = UnityEngine.Object.FindObjectsByType<TrafficLane>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            }
            catch (Exception e)
            {
                Debug.LogError($"[Beytepe] NPC ego farkındalığı: şeritler bulunamadı ({e.Message}); yol verme kapalı");
                _lanes = Array.Empty<TrafficLane>();
            }
            if (ego != null)
            {
                _egoRb = ego.GetComponent<Rigidbody>();
                MeasureEgo(ego);
            }
            if (S.Log)
                Debug.Log($"[Beytepe] NPC ego farkındalığı: {(S.Enabled ? "açık" : "KAPALI")} " +
                          $"(kesme algısı {(S.CutInEnabled ? "açık" : "kapalı")}: pay {S.CutInMargin:F2} m, tahmin {S.CutInPredictSec:F1} s; " +
                          $"yol verme {(S.YieldEnabled ? "açık" : "kapalı")}: ufuk {S.YieldHorizonSec:F1} s, pay {S.YieldMarginSec:F1} s, " +
                          $"en fazla {S.YieldMaxDecel:F1} m/s²; ego beklerken bırakma {(S.ReleaseWhenEgoWaits ? "açık" : "kapalı")}); " +
                          $"ego ön {_egoFront:F2} m, arka {_egoRear:F2} m, yarı genişlik {_egoHalfW:F2} m " +
                          $"({_egoDimSource}); {_lanes.Length} şerit; hız {(_egoRb != null ? "Rigidbody'den" : "konum farkından")}; ayar: {_source}");
        }

        static Settings LoadSettings(out string source)
        {
            source = "varsayılan (json yok)";
            var path = CommandLineUtility.GetCommandLineArg("--json_path");
            if (string.IsNullOrEmpty(path))
                path = Environment.GetEnvironmentVariable("BEYTEPE_SENSOR_JSON");   // düzenleyicide deneme için
            if (string.IsNullOrEmpty(path) || !File.Exists(path))
                return new Settings();
            try
            {
                var text = File.ReadAllText(path);
                var root = JsonUtility.FromJson<JsonRoot>(text);
                source = text.Contains("\"NpcEgo\"") ? path + " (NpcEgo bölümü)" : path + " (NpcEgo bölümü yok, varsayılanlar)";
                return root?.NpcEgo ?? new Settings();
            }
            catch (Exception e)
            {
                Debug.LogError($"[Beytepe] NPC ego ayarları okunamadı ({path}): {e.Message}; varsayılanlar kullanılıyor");
                source = "okuma hatası, varsayılan";
                return new Settings();
            }
        }

        /// <summary>Egonun gövde ölçülerini "Colliders" altındaki çarpıştırıcılardan çıkarır (base_link'e göre).</summary>
        static void MeasureEgo(Transform ego)
        {
            try
            {
                var root = ego.Find("Colliders") ?? ego;
                float minX = float.MaxValue, maxX = float.MinValue, minZ = float.MaxValue, maxZ = float.MinValue;
                int n = 0;
                foreach (var c in root.GetComponentsInChildren<Collider>(true))
                {
                    if (c.isTrigger || c is WheelCollider) continue;
                    Bounds b;
                    if (c is MeshCollider mc && mc.sharedMesh != null) b = mc.sharedMesh.bounds;
                    else if (c is BoxCollider bc) b = new Bounds(bc.center, bc.size);
                    else continue;
                    for (int i = 0; i < 8; i++)
                    {
                        var corner = new Vector3((i & 1) == 0 ? b.min.x : b.max.x,
                                                 (i & 2) == 0 ? b.min.y : b.max.y,
                                                 (i & 4) == 0 ? b.min.z : b.max.z);
                        var l = ego.InverseTransformPoint(c.transform.TransformPoint(corner));
                        minX = Mathf.Min(minX, l.x); maxX = Mathf.Max(maxX, l.x);
                        minZ = Mathf.Min(minZ, l.z); maxZ = Mathf.Max(maxZ, l.z);
                    }
                    n++;
                }
                if (n == 0) { _egoDimSource = "çarpıştırıcı yok, varsayılan"; return; }
                float hw = 0.5f * (maxX - minX), front = maxZ, rear = -minZ;
                if (hw < 0.5f || hw > 1.6f || front < 1.5f || front > 7f || rear < 0f || rear > 4f)
                {
                    _egoDimSource = $"çarpıştırıcı ölçüsü kuşkulu (ön {front:F2}, arka {rear:F2}, yarı genişlik {hw:F2}), varsayılan";
                    return;
                }
                _egoFront = front; _egoRear = rear; _egoHalfW = hw;
                _egoDimSource = $"{n} çarpıştırıcıdan";
            }
            catch (Exception e)
            {
                _egoDimSource = $"ölçülemedi ({e.Message}), varsayılan";
            }
        }

        // ---------------------------------------------------------------- ego

        static void UpdateEgo(Transform ego, float now)
        {
            var p = ego.position;
            _egoPos = new Vector2(p.x, p.z);
            var f = ego.forward;
            var f2 = new Vector2(f.x, f.z);
            _egoDir = f2.sqrMagnitude > 1e-6f ? f2.normalized : Vector2.up;
            _egoYaw = Mathf.Atan2(_egoDir.x, _egoDir.y);   // rad; Unity yaw (+z'den +x'e)
            if (_egoRb != null)
            {
                var v = _egoRb.linearVelocity;
                _egoSpeed = Vector2.Dot(new Vector2(v.x, v.z), _egoDir);
                _egoYawRate = _egoRb.angularVelocity.y;
            }
            else if (_lastTime >= 0f && now > _lastTime)
            {
                float dt = now - _lastTime;
                _egoSpeed = Vector2.Dot(_egoPos - _lastEgoPos, _egoDir) / dt;
                _egoYawRate = Mathf.DeltaAngle(_lastEgoYaw * Mathf.Rad2Deg, _egoYaw * Mathf.Rad2Deg) * Mathf.Deg2Rad / dt;
            }
            // Yavaşlama (C için): hız farkından, DecelSmoothSec ile yumuşatılmış; + = fren
            if (_lastTime >= 0f && now > _lastTime)
            {
                float dt = now - _lastTime;
                float aRaw = -(_egoSpeed - _lastEgoSpeed) / dt;
                _egoDecel += (aRaw - _egoDecel) * Mathf.Min(1f, dt / DecelSmoothSec);
            }
            _lastEgoPos = _egoPos; _lastEgoYaw = _egoYaw; _lastEgoSpeed = _egoSpeed; _lastTime = now;

            // Gövde örnek noktaları: şimdiki konum ve (ego ilerliyorsa) sabit hız ve dönüş hızıyla tahmini konum
            _egoPts.Clear();
            AddFootprint(_egoPos, _egoYaw);
            if (S.CutInPredictSec > 0f && _egoSpeed > 0.5f)
            {
                var q = _egoPos;
                float yaw = _egoYaw;
                const int steps = 10;
                float dt = S.CutInPredictSec / steps;
                for (int i = 0; i < steps; i++)
                {
                    yaw += _egoYawRate * dt;
                    q += _egoSpeed * dt * new Vector2(Mathf.Sin(yaw), Mathf.Cos(yaw));
                }
                AddFootprint(q, yaw);
            }
        }

        static void AddFootprint(Vector2 c, float yaw)
        {
            var fwd = new Vector2(Mathf.Sin(yaw), Mathf.Cos(yaw));
            var right = new Vector2(fwd.y, -fwd.x);
            for (int i = 0; i <= 4; i++)
            {
                float z = Mathf.Lerp(-_egoRear, _egoFront, i / 4f);
                _egoPts.Add(c + fwd * z + right * _egoHalfW);
                _egoPts.Add(c + fwd * z - right * _egoHalfW);
            }
            _egoPts.Add(c + fwd * _egoFront);
            _egoPts.Add(c - fwd * _egoRear);
        }

        /// <summary>
        /// Egonun önündeki olası yollar ("tüp"): egonun üzerinde olduğu şeritler (2.5 m, 45° içinde; şerit değiştirirken
        /// ikisi de) ve onların ardılları, tüm kollar. Uzunluk hıza göre 30-150 m. Ego dururken de kurulur (C, egonun
        /// çakışmadan önce beklediğini görmek için); geri giderken kurulmaz.
        /// </summary>
        static bool BuildEgoTube()
        {
            _tube.Clear();
            if (_egoSpeed < -0.1f || _lanes.Length == 0) return false;
            float maxLen = Mathf.Clamp(_egoSpeed * (S.YieldHorizonSec + 2f) + _egoFront + 10f, 30f, 150f);
            _visited.Clear();
            _stack.Clear();
            float cosLim = Mathf.Cos(LaneMatchDeg * Mathf.Deg2Rad);

            foreach (var lane in _lanes)
            {
                if (lane == null) continue;
                var w = lane.Waypoints;
                if (w == null || w.Length < 2) continue;
                int bestK = -1;
                float bestD = LaneMatchDist, bestT = 0f;
                for (int k = 0; k < w.Length - 1; k++)
                {
                    var a = V2(w[k]);
                    var ab = V2(w[k + 1]) - a;
                    float l2 = ab.sqrMagnitude;
                    if (l2 < 1e-4f) continue;
                    float t = Mathf.Clamp01(Vector2.Dot(_egoPos - a, ab) / l2);
                    float dist = Vector2.Distance(_egoPos, a + t * ab);
                    if (dist > bestD) continue;
                    if (Vector2.Dot(ab, _egoDir) < cosLim * Mathf.Sqrt(l2)) continue;
                    bestD = dist; bestK = k; bestT = t;
                }
                if (bestK >= 0) AddMatchedLane(lane, bestK, bestT, maxLen);
            }

            while (_stack.Count > 0 && _tube.Count < 600)
            {
                var (lane, s0, depth) = _stack.Pop();
                if (lane == null || s0 >= maxLen || depth > 6) continue;
                if (_visited.TryGetValue(lane, out var prevS) && prevS <= s0 + 0.1f) continue;
                _visited[lane] = s0;
                var w = lane.Waypoints;
                if (w == null || w.Length < 2) continue;
                float s = s0;
                for (int k = 0; k < w.Length - 1 && s < maxLen; k++)
                {
                    var a = V2(w[k]);
                    var b = V2(w[k + 1]);
                    float l = Vector2.Distance(a, b);
                    if (l < 0.01f) continue;
                    AddSeg(a, b, s, s + l);
                    s += l;
                }
                if (s < maxLen)
                    foreach (var nl in lane.NextLanes) _stack.Push((nl, s, depth + 1));
            }
            return _tube.Count > 0;
        }

        static void AddMatchedLane(TrafficLane lane, int k, float t, float maxLen)
        {
            var w = lane.Waypoints;
            var q = Vector2.Lerp(V2(w[k]), V2(w[k + 1]), t);
            _visited[lane] = -1e9f;   // bu şerit artık baştan eklenmez (egonun arkası ileride sayılmasın)

            // geriye: egonun arkasına (-_egoRear) kadar
            float sb = 0f;
            var hi = q;
            for (int idx = k; idx >= 0 && sb > -_egoRear; idx--)
            {
                var lo = V2(w[idx]);
                float l = Vector2.Distance(lo, hi);
                if (l <= 0.01f) continue;
                float sLo = sb - l;
                if (sLo < -_egoRear)
                {
                    lo = Vector2.Lerp(hi, lo, (sb + _egoRear) / l);
                    sLo = -_egoRear;
                }
                AddSeg(lo, hi, sLo, sb);
                sb = sLo;
                hi = lo;
            }

            // ileriye
            float s = 0f;
            var cur = q;
            for (int j = k + 1; j < w.Length && s < maxLen; j++)
            {
                var b = V2(w[j]);
                float l = Vector2.Distance(cur, b);
                if (l > 0.01f) { AddSeg(cur, b, s, s + l); s += l; }
                cur = b;
            }
            if (s < maxLen)
                foreach (var nl in lane.NextLanes) _stack.Push((nl, s, 1));
        }

        static void AddSeg(Vector2 a, Vector2 b, float sa, float sb)
        {
            _tube.Add(new Seg
            {
                A = a, B = b, Sa = sa, Sb = sb,
                X0 = Mathf.Min(a.x, b.x), X1 = Mathf.Max(a.x, b.x),
                Y0 = Mathf.Min(a.y, b.y), Y1 = Mathf.Max(a.y, b.y)
            });
        }

        // ---------------------------------------------------------------- NPC yolu

        /// <summary>NPC'nin yolu: arka orta, ön orta, sonra izleyeceği şerit noktaları (NpcLookahead m). _ns ön ortadan uzaklık.</summary>
        static void BuildNpcPath(NpcVehicleInternalState st)
        {
            _np.Clear(); _ns.Clear(); _nl.Clear(); _laneStartLane.Clear(); _laneStartS.Clear();
            var back = V2(st.BackCenterPosition);
            var front = V2(st.FrontCenterPosition);
            float len = Mathf.Max(Vector2.Distance(back, front), 0.5f);
            _np.Add(back); _ns.Add(-len);
            _np.Add(front); _ns.Add(0f); _nl.Add(st.CurrentFollowingLane);
            float s = 0f;
            var prev = front;
            for (int li = 0; li < st.FollowingLanes.Count && s < NpcLookahead; li++)
            {
                var lane = st.FollowingLanes[li];
                var w = lane?.Waypoints;
                if (w == null || w.Length == 0) break;
                int start = li == 0 ? Mathf.Clamp(st.WaypointIndex, 0, w.Length - 1) : 1;
                if (li > 0) { _laneStartLane.Add(lane); _laneStartS.Add(s); }
                for (int k = start; k < w.Length && s < NpcLookahead; k++)
                {
                    var p = V2(w[k]);
                    float l = Vector2.Distance(prev, p);
                    if (l < 0.05f) continue;
                    _nl.Add(lane);
                    s += l;
                    _np.Add(p); _ns.Add(s);
                    prev = p;
                }
            }
        }

        /// <summary>Noktanın NPC yoluna izdüşümü: s (ön ortadan, m) ve yanal uzaklık (m).</summary>
        static bool ProjectOnNpcPath(Vector2 p, out float s, out float lat)
        {
            s = 0f; lat = float.MaxValue;
            bool ok = false;
            for (int j = 0; j < _np.Count - 1; j++)
            {
                var a = _np[j];
                var ab = _np[j + 1] - a;
                float l2 = ab.sqrMagnitude;
                if (l2 < 1e-6f) continue;
                float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / l2);
                float d = Vector2.Distance(p, a + t * ab);
                if (d < lat)
                {
                    lat = d;
                    s = Mathf.Lerp(_ns[j], _ns[j + 1], t);
                    ok = true;
                }
            }
            return ok;
        }

        // ---------------------------------------------------------------- A) kesme algısı

        static void CutIn(NpcVehicleInternalState st, NpcData d, float now)
        {
            float hw = 0.5f * st.Width + S.CutInMargin;
            float len = -_ns[0];
            float best = float.MaxValue;
            foreach (var q in _egoPts)
            {
                if (!ProjectOnNpcPath(q, out float sq, out float lat)) continue;
                if (lat > hw || sq < -0.5f * len) continue;
                if (sq < best) best = sq;
            }

            float boxcast = st.DistanceToFrontVehicle;   // AWSIM engel taraması (bu adım)
            bool extra = false;
            if (best < float.MaxValue)
            {
                float dist = Mathf.Max(best, 0f);
                extra = dist < 40f && boxcast > dist + 2f;
                if (dist < st.DistanceToFrontVehicle) st.DistanceToFrontVehicle = dist;
                if (extra)
                {
                    if (!d.CutIn)
                    {
                        d.CutIn = true; d.CutInSince = now; d.CutInMinDist = dist; d.CutInMaxMode = (int)st.SpeedMode;
                        CutInCount++;
                        if (S.Log)
                            Debug.Log($"[Beytepe] NPC {Name(st)}: ego koridoruna giriyor, {dist:F1} m önde " +
                                      $"(engel taraması {(boxcast < 1e6f ? boxcast.ToString("F1") + " m" : "görmüyor")}), " +
                                      $"NPC {st.Speed * 3.6f:F0} km/sa, ego {_egoSpeed * 3.6f:F0} km/sa, NPC {Ros(V2(st.FrontCenterPosition))}");
                    }
                    d.CutInLastSeen = now;
                    d.CutInMinDist = Mathf.Min(d.CutInMinDist, dist);
                }
            }
            if (d.CutIn)
            {
                d.CutInMaxMode = Mathf.Max(d.CutInMaxMode, (int)st.SpeedMode);
                if (!extra && now - d.CutInLastSeen > 1f) EndCutIn(st, d, now);
            }
        }

        static void EndCutIn(NpcVehicleInternalState st, NpcData d, float now)
        {
            if (!d.CutIn) return;
            d.CutIn = false;
            if (S.Log)
                Debug.Log($"[Beytepe] NPC {Name(st)}: kesme bitti, {d.CutInLastSeen - d.CutInSince:F1} s, en yakın {d.CutInMinDist:F1} m, " +
                          $"NPC'nin en sert modu {(NpcVehicleSpeedMode)d.CutInMaxMode}");
        }

        // ---------------------------------------------------------------- B) kavşak ve birleşmede yol verme

        static float Yield(NpcVehicleInternalState st, NpcData d, float now)
        {
            float dConf = 0.5f * st.Width + _egoHalfW + ConflictPad;
            int r = FindConflict(dConf, out float sN, out float sE, out float ang, out int seg);
            if (r == 2) return EndYield(st, d, now, "NPC egonun yolunda");
            if (r == 0) return EndYield(st, d, now, "çakışma kalmadı");
            if (ang < CoDirDeg && sE <= _egoFront) return EndYield(st, d, now, "egonun arkasına katılıyor");
            if (ang > HeadOnDeg) return EndYield(st, d, now, "karşı yönden paralel");

            // C) Ego çakışmadan önce bekliyorsa (durmuş ya da orada durmak için frende) Autoware zaten yol veriyor: NPC beklemez.
            float dE = sE - _egoFront;
            if (S.ReleaseWhenEgoWaits && EgoWaitsBefore(dE))
            {
                d.EgoWaits = true;
                d.EgoWaitsAt = now;
                if (d.Yielding)
                {
                    ReleaseCount++;
                    d.NoYieldUntil = now + ReleaseCooldownSec;
                    EndYield(st, d, now, $"ego bekliyor: çakışmaya {dE:F1} m, ego {_egoSpeed * 3.6f:F0} km/sa, fren {_egoDecel:F1} m/s²");
                }
                return float.MaxValue;
            }

            float tE = sE <= _egoFront ? 0f : dE / Mathf.Max(_egoSpeed, 0.1f);
            float len = -_ns[0];
            float vEff = Mathf.Max(st.Speed, 3f);
            float tNout = (sN + len + 2f * dConf) / vEff;

            float sStop = sN - StopGap;
            var laneC = seg >= 0 && seg < _nl.Count ? _nl[seg] : null;
            int li = laneC != null ? _laneStartLane.IndexOf(laneC) : -1;
            if (li >= 0)
            {
                float sStart = _laneStartS[li];
                if (sStart > 0f && sN - sStart <= LaneEntryMaxBack) sStop = Mathf.Min(sStop, sStart - 0.3f);
            }

            if (!d.Yielding)
            {
                if (now < d.NoYieldUntil) return float.MaxValue;   // C) az önce bırakıldı
                bool want = tE <= S.YieldHorizonSec && tE < tNout + S.YieldMarginSec && _egoSpeed >= 0.3f;
                if (!want) return float.MaxValue;
                float need = st.Speed * st.Speed / (2f * Mathf.Max(S.YieldMaxDecel, 0.5f));
                if (sStop < need || sStop < 0.5f) return float.MaxValue;   // rahat duramaz: geçmeye kararlı
                d.Yielding = true;
                d.YieldSince = now;
                YieldCount++;
                if (S.Log)
                    Debug.Log($"[Beytepe] NPC {Name(st)}: egoya yol veriyor; ego çakışmaya {tE:F1} s sonra varacak " +
                              $"({_egoSpeed * 3.6f:F0} km/sa, {Mathf.Max(sE - _egoFront, 0f):F1} m), NPC {sN:F1} m uzakta " +
                              $"({st.Speed * 3.6f:F0} km/sa, şerit {LaneName(laneC)}, açı {ang:F0}°), {sStop:F1} m ileride duracak; " +
                              $"NPC {Ros(V2(st.FrontCenterPosition))}");
                return sStop;
            }

            bool keep = tE <= S.YieldHorizonSec + 2f && tE < tNout + S.YieldMarginSec + 1f && sStop > -1f;
            if (!keep) return EndYield(st, d, now, $"ego {tE:F1} s uzakta");
            if (now - d.YieldSince > MaxYieldSec) return EndYield(st, d, now, $"{MaxYieldSec:F0} s bekleme sınırı");
            return sStop;
        }

        /// <summary>
        /// C) Ego, önündeki çakışma noktasına (önünden dE m) varmadan duracak mı? Durmuşsa ya da belirgin fren yapıyor ve
        /// bu frenle çakışmadan en az EgoWaitMinGap önce, en çok EgoWaitZone önce duracaksa evet. Virajda hafif yavaşlama
        /// (EgoWaitMinDecel altı) ya da çakışmadan çok uzakta durma sayılmaz.
        /// </summary>
        static bool EgoWaitsBefore(float dE)
        {
            if (dE < EgoWaitMinGap) return false;
            if (_egoSpeed < EgoStoppedSpeed) return true;
            if (_egoDecel < EgoWaitMinDecel) return false;
            float dStop = _egoSpeed * _egoSpeed / (2f * _egoDecel);
            return dStop <= dE - EgoWaitMinGap && dStop >= dE - EgoWaitZone;
        }

        static float EndYield(NpcVehicleInternalState st, NpcData d, float now, string reason)
        {
            if (d.Yielding)
            {
                d.Yielding = false;
                if (S.Log)
                    Debug.Log($"[Beytepe] NPC {Name(st)}: yol verme bitti ({reason}), {now - d.YieldSince:F1} s sürdü");
            }
            return float.MaxValue;
        }

        /// <summary>
        /// NPC yolunun egonun tüpüne ilk yaklaştığı yer. 0: yok; 1: var (sN NPC'nin, sE egonun yolu boyunca uzaklık,
        /// ang yönler arası açı, seg NPC yolundaki parça); 2: NPC'nin önü zaten tüpün içinde (egonun yolunda).
        /// </summary>
        static int FindConflict(float dConf, out float sN, out float sE, out float ang, out int seg)
        {
            sN = sE = ang = 0f; seg = -1;
            if (DistToTube(_np[1], dConf, out _, out _) < dConf) return 2;
            for (int j = 1; j < _np.Count - 1; j++)
            {
                var a = _np[j];
                var b = _np[j + 1];
                if (!SegNearTube(a, b, dConf)) continue;
                float l = Vector2.Distance(a, b);
                var dir = (b - a) / Mathf.Max(l, 1e-4f);
                int n = Mathf.Max(1, Mathf.CeilToInt(l / 0.5f));
                for (int i = 0; i <= n; i++)
                {
                    var p = Vector2.Lerp(a, b, (float)i / n);
                    if (DistToTube(p, dConf, out float se, out Vector2 tdir) < dConf)
                    {
                        sN = _ns[j] + l * i / n;
                        sE = se;
                        ang = Vector2.Angle(dir, tdir);
                        seg = j;
                        return 1;
                    }
                }
            }
            return 0;
        }

        static float DistToTube(Vector2 p, float maxD, out float s, out Vector2 dir)
        {
            float best = float.MaxValue;
            s = 0f; dir = Vector2.up;
            for (int i = 0; i < _tube.Count; i++)
            {
                var g = _tube[i];
                if (p.x < g.X0 - maxD || p.x > g.X1 + maxD || p.y < g.Y0 - maxD || p.y > g.Y1 + maxD) continue;
                var ab = g.B - g.A;
                float l2 = ab.sqrMagnitude;
                if (l2 < 1e-6f) continue;
                float t = Mathf.Clamp01(Vector2.Dot(p - g.A, ab) / l2);
                float d = Vector2.Distance(p, g.A + t * ab);
                float sAt = Mathf.Lerp(g.Sa, g.Sb, t);
                if (d < best - 1e-3f || (Mathf.Abs(d - best) <= 1e-3f && sAt < s))
                {
                    best = d;
                    s = sAt;
                    dir = ab / Mathf.Sqrt(l2);
                }
            }
            return best;
        }

        static bool SegNearTube(Vector2 a, Vector2 b, float maxD)
        {
            float x0 = Mathf.Min(a.x, b.x) - maxD, x1 = Mathf.Max(a.x, b.x) + maxD;
            float y0 = Mathf.Min(a.y, b.y) - maxD, y1 = Mathf.Max(a.y, b.y) + maxD;
            for (int i = 0; i < _tube.Count; i++)
            {
                var g = _tube[i];
                if (g.X1 < x0 || g.X0 > x1 || g.Y1 < y0 || g.Y0 > y1) continue;
                if (SegSegDist(a, b, g.A, g.B) < maxD) return true;
            }
            return false;
        }

        static float SegSegDist(Vector2 p1, Vector2 p2, Vector2 q1, Vector2 q2)
        {
            if (SegmentsIntersect(p1, p2, q1, q2)) return 0f;
            return Mathf.Min(Mathf.Min(PointSegDist(p1, q1, q2), PointSegDist(p2, q1, q2)),
                             Mathf.Min(PointSegDist(q1, p1, p2), PointSegDist(q2, p1, p2)));
        }

        static float PointSegDist(Vector2 p, Vector2 a, Vector2 b)
        {
            var ab = b - a;
            float l2 = ab.sqrMagnitude;
            if (l2 < 1e-6f) return Vector2.Distance(p, a);
            float t = Mathf.Clamp01(Vector2.Dot(p - a, ab) / l2);
            return Vector2.Distance(p, a + t * ab);
        }

        static bool SegmentsIntersect(Vector2 p1, Vector2 p2, Vector2 q1, Vector2 q2)
        {
            float d1 = Cross(q2 - q1, p1 - q1), d2 = Cross(q2 - q1, p2 - q1);
            float d3 = Cross(p2 - p1, q1 - p1), d4 = Cross(p2 - p1, q2 - p1);
            return ((d1 > 0f && d2 < 0f) || (d1 < 0f && d2 > 0f)) && ((d3 > 0f && d4 < 0f) || (d3 < 0f && d4 > 0f));
        }

        static float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;

        // ---------------------------------------------------------------- yardımcılar

        static Vector2 V2(Vector3 v) => new Vector2(v.x, v.z);

        /// <summary>Unity (x, z) -> ROS harita (x, y): x = z, y = -x.</summary>
        static string Ros(Vector2 u) => $"({u.y:F1}, {-u.x:F1})";

        static string Name(NpcVehicleInternalState st) => st.Vehicle != null ? $"{st.Vehicle.Id}" : "?";

        static string LaneName(TrafficLane l) => l == null ? "?" : l.name.Replace("TrafficLane.", "");
    }
}
