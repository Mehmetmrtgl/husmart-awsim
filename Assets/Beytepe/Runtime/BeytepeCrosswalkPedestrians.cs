using System.Collections.Generic;
using UnityEngine;
using Awsim.Entity;

namespace Beytepe.Runtime
{
    /// <summary>
    /// Beytepe: yaya geçitlerinde RASTGELE yaya geçişleri.
    ///  * Her geçit bağımsızdır; geçişler rastgele aralıklarla (min..max interval) başlar.
    ///  * Grup büyüklüğü rastgele: %40 tek, %25 iki, %15 üç, %20 "kalabalık" (4-8 kişi, yarısında karışık yönler).
    ///  * Her yayanın hızı, başlama gecikmesi ve geçit içindeki hattı rastgeledir.
    ///  * Bir geçişin son yayası geçitten çıktıktan sonra en az quietGap saniye yeni geçiş başlamaz:
    ///    araç bekledikten sonra mutlaka geçebilir (sürekli geçiş yok).
    ///  * Yayalar kaldırımda, geçidin curbOffset metre gerisinde belirir, karşı kaldırımda kaybolur
    ///    (kaldırımda sürekli bekleyen kalabalık Autoware'i boşuna bekletmesin diye).
    ///  * Yol verme (gerçekçi yaya): geçide yaklaşan bir araç (ego ya da NPC) geçidin önünde en fazla
    ///    yieldDecel m/s² ile duramayacak kadar yakın/hızlıysa, henüz kaldırımda olan yayalar yola adım atmaz,
    ///    oldukları yerde bekler; araç durunca ya da geçidi geçince yürümeye devam eder. Yoldaki yayalar yürür.
    ///    (Önceden yayalar araçtan bağımsız çıkıyordu: araç geçide 2-3 m kala önüne çıkan yaya için
    ///    Autoware -3.4 m/s² ile frenleyip durma noktasını aşıyordu.)
    ///    Ayrıca araç geçide minTimeGap saniyeden kısa sürede varacaksa da çıkmaz (boşluk kabulü) ve yaya
    ///    belirdikten sonra en az minCurbWaitSec kaldırımda görünür durur. Yaya önceliği sürer: araç uzaksa ya da
    ///    durmuşsa yaya geçer ve araç yol vermek zorundadır.
    ///  * Belirme denetimi: geçide spawnMinTta saniyeden kısa sürede varacak bir araç (ego ya da NPC) varken o geçitte
    ///    yeni yaya belirmez (virajdan gelen araç da sayılır); araç geçince ya da yeterince uzaktayken belirir. Böylece Autoware yayayı kaldırımda
    ///    beklerken uzaktan görür. (Tur 10'da yaya, 21 km/sa giden araç geçide ~3 s kala kaldırımda birden belirdi;
    ///    algı onu ancak 12 m kala kararlı izledi ve araç 3.8 m/s² ile frenledi.)
    /// Yaya modeli AWSIM'in Pedestrian bileşenini kullanır (yürüme animasyonu hız ile, zemin takibi raycast ile).
    /// </summary>
    public class BeytepeCrosswalkPedestrians : MonoBehaviour
    {
        [System.Serializable]
        public class Crosswalk
        {
            public Vector3 start;   // geçidin bir ucu (Unity dünya koordinatı)
            public Vector3 end;     // karşı ucu
            public float width;     // geçidin eni (yayalar bu bant içinde farklı hatlardan geçer)
        }

        [SerializeField] GameObject[] _pedestrianPrefabs;
        [SerializeField] List<Crosswalk> _crosswalks = new List<Crosswalk>();
        [SerializeField] int _poolPerCrosswalk = 8;
        [SerializeField] float _minIntervalSec = 15f;
        [SerializeField] float _maxIntervalSec = 75f;
        [SerializeField] float _quietGapSec = 12f;
        [SerializeField] float _minSpeed = 0.9f;
        [SerializeField] float _maxSpeed = 1.6f;
        [SerializeField] float _maxStartDelaySec = 3f;
        [SerializeField] float _curbOffset = 3f;

        [Header("Yol verme (yaklaşan araç)")]
        [SerializeField] bool _yieldToVehicles = true;
        [SerializeField] float _yieldDecel = 2.0f;        // m/s²: araç bundan sert frenlemek zorunda kalacaksa yaya çıkmaz
        [SerializeField] float _yieldLookahead = 60f;     // m: bu uzaklıktan ötedeki araçlara bakılmaz
        [SerializeField] float _stopMargin = 2.0f;        // m: aracın geçidin bu kadar önünde durabilmesi istenir
        [SerializeField] float _vehicleFront = 4.0f;      // m: araç kökünden ön tampona (ego: arka aks -> ön ~3.9 m)
        [SerializeField] float _vehicleRear = 2.5f;       // m: araç kökünden arka tampona (geçti mi kontrolü)
        [SerializeField] float _stoppedSpeed = 0.5f;      // m/s: bundan yavaş araç durmuş sayılır (yaya geçebilir)
        [SerializeField] float _holdMinSec = 1.0f;        // s: bekleme kararı en az bu kadar sürer (titreşmesin)
        [SerializeField] float _minTimeGap = 2.5f;        // s: araç geçide bundan kısa sürede varacaksa yaya çıkmaz (gerçek yaya
                                                          //    boşluk kabulü; tur 06'da yaya 2.3 s'de varacak aracın önüne çıktı)
        [SerializeField] float _minCurbWaitSec = 1.5f;    // s: yaya belirdikten sonra en az bu kadar kaldırımda durur, algı
                                                          //    onu yola çıkmadan görsün (birden belirip hemen çıkmasın)

        [Header("Belirme denetimi (yeni yaya ne zaman ortaya çıkar)")]
        [SerializeField] float _spawnMinTta = 6f;         // s: bir araç geçide bundan kısa sürede varacaksa o geçitte yeni yaya
                                                          //    belirmez; araç geçince ya da uzaktayken belirir (0 = denetim kapalı)
        [SerializeField] float _spawnLookahead = 120f;    // m: belirme denetiminde bu uzaklıktan ötedeki araçlara bakılmaz
        [SerializeField] float _spawnConeDeg = 50f;       // °: virajdan gelen araç, yönü geçidin merkezine bu açıdan yakınsa
                                                          //    sayılır (tur 10, geçit 5: düz uzatılan yön geçidi ancak 30 m kala
                                                          //    kesiyordu, merkeze açı 8-25° idi)

        class Vehicle
        {
            public Transform tf;
            public AccelVehicle ego;    // ego: hız doğrudan rigidbody'den; NPC: konum farkından
            public Vector3 pos, vel;
            public bool init;
        }

        readonly Dictionary<Transform, Vehicle> _vehicles = new Dictionary<Transform, Vehicle>();
        float _nextVehicleScan;

        class Walker
        {
            public GameObject go;
            public Pedestrian ped;
            public bool active;
            public Vector3 from, to;
            public float speed, startTime, progress, length;
        }

        class CrosswalkState
        {
            public int index;
            public Crosswalk cw;
            public readonly List<Walker> pool = new List<Walker>();
            public float nextEvent;
            public float lastBusy = -1e9f;
            public System.Random rng;
            public float holdUntil = -1e9f;   // bu zamana kadar kaldırımdaki yayalar bekler
            public bool holding;
            public int holds;                 // kaç kez araca yol verildi (günlük için)
            public float deferSince = -1f;    // yeni yaya belirmesi bu zamandan beri araç yüzünden erteleniyor (-1: ertelenmiyor)
            public int defers;                // kaç kez ertelendi (günlük için)
        }

        readonly List<CrosswalkState> _states = new List<CrosswalkState>();

        void Start()
        {
            if (_pedestrianPrefabs == null || _pedestrianPrefabs.Length == 0)
            {
                Debug.LogWarning("[Beytepe] yaya prefabı yok; yaya geçişleri kapalı");
                return;
            }
            for (int i = 0; i < _crosswalks.Count; i++)
            {
                var st = new CrosswalkState
                {
                    index = i,
                    cw = _crosswalks[i],
                    rng = new System.Random(System.Environment.TickCount ^ (i * 7919 + 13)),
                };
                for (int k = 0; k < _poolPerCrosswalk; k++)
                {
                    var prefab = _pedestrianPrefabs[st.rng.Next(_pedestrianPrefabs.Length)];
                    if (prefab == null) continue;
                    var go = Instantiate(prefab, st.cw.start, Quaternion.identity, transform);
                    go.name = $"CrosswalkPedestrian_{i}_{k}";
                    var ped = go.GetComponent<Pedestrian>();
                    go.SetActive(false);
                    if (ped != null) st.pool.Add(new Walker { go = go, ped = ped });
                }
                st.nextEvent = Time.time + Rand(st, 3f, _maxIntervalSec);   // ilk geçiş de rastgele zamanda
                _states.Add(st);
            }
            Debug.Log($"[Beytepe] yaya geçitleri: {_states.Count} geçit, geçit başına {_poolPerCrosswalk} yaya");
        }

        void Update()
        {
            foreach (var st in _states)
                foreach (var w in st.pool)
                    if (w.active) w.ped.OnUpdate();
        }

        void FixedUpdate()
        {
            float t = Time.time;   // FixedUpdate içinde fizik adımının zamanı
            if (_yieldToVehicles || _spawnMinTta > 0f)
                UpdateVehicles(t);
            foreach (var st in _states)
            {
                // Yaklaşan araç konforlu duramayacaksa kaldırımdaki yayalar beklesin (kısa süreli kararlar titreşmesin)
                bool anyOnSidewalk = false;
                foreach (var w in st.pool)
                    if (w.active && w.progress < _curbOffset) { anyOnSidewalk = true; break; }
                if (_yieldToVehicles && anyOnSidewalk)
                {
                    if (VehicleTooClose(st.cw, out float vKmh, out float gapM))
                    {
                        if (!st.holding)
                        {
                            st.holds++;
                            Debug.Log($"[Beytepe] yaya geçidi {st.index}: araç yaklaşıyor ({vKmh:F0} km/sa, geçide {gapM:F1} m), " +
                                      $"kaldırımdaki yayalar bekliyor (yol verme {st.holds})");
                        }
                        st.holdUntil = t + _holdMinSec;
                    }
                }
                st.holding = t < st.holdUntil;
                bool busy = false;
                foreach (var w in st.pool)
                {
                    if (!w.active) continue;
                    busy = true;
                    var flat = w.to - w.from; flat.y = 0f;
                    var rot = Quaternion.LookRotation(flat.sqrMagnitude > 1e-6f ? flat.normalized : Vector3.forward);
                    bool onSidewalk = w.progress < _curbOffset;   // henüz yola adım atmamış
                    if (t >= w.startTime && !(st.holding && onSidewalk))
                        w.progress += w.speed * Time.fixedDeltaTime;
                    if (w.progress >= w.length)
                    {
                        w.active = false;
                        w.go.SetActive(false);
                        continue;
                    }
                    var pos = Vector3.Lerp(w.from, w.to, w.progress / w.length);
                    w.ped.PoseInput = new Pose(pos, rot);
                    w.ped.OnFixedUpdate();
                }
                if (busy)
                    st.lastBusy = t;
                else if (t >= st.nextEvent && t - st.lastBusy >= _quietGapSec)
                {
                    // Belirme denetimi: geçide yakında varacak bir araç varsa yeni yaya henüz belirmez, araç geçince belirir
                    if (_spawnMinTta > 0f && VehicleArrivingSoon(st.cw, out float sKmh, out float sGap, out float sTta))
                    {
                        if (st.deferSince < 0f)
                        {
                            st.deferSince = t;
                            st.defers++;
                            Debug.Log($"[Beytepe] yaya geçidi {st.index}: araç geçide {sTta:F1} s sonra varacak ({sKmh:F0} km/sa, " +
                                      $"geçide {sGap:F1} m), yeni yaya belirmesi erteleniyor (erteleme {st.defers})");
                        }
                    }
                    else
                    {
                        float waited = st.deferSince >= 0f ? t - st.deferSince : 0f;
                        st.deferSince = -1f;
                        StartEvent(st, t, waited);
                    }
                }
            }
        }

        void StartEvent(CrosswalkState st, float t, float waitedSec = 0f)
        {
            double r = st.rng.NextDouble();
            int n = r < 0.40 ? 1 : r < 0.65 ? 2 : r < 0.80 ? 3 : st.rng.Next(4, 9);   // %20 kalabalık (4-8)
            bool crowd = n >= 4;
            bool mixed = crowd && st.rng.NextDouble() < 0.5;
            bool dirAB = st.rng.NextDouble() < 0.5;

            var free = st.pool.FindAll(w => !w.active);
            n = Mathf.Min(n, free.Count);
            Vector3 a = st.cw.start, b = st.cw.end;
            var along = b - a; along.y = 0f; along.Normalize();
            var side = Vector3.Cross(Vector3.up, along);
            float band = Mathf.Max(st.cw.width - 0.8f, 0.4f);

            for (int k = 0; k < n; k++)
            {
                var w = free[k];
                bool ab = mixed ? st.rng.NextDouble() < 0.5 : dirAB;
                float lat = ((float)st.rng.NextDouble() - 0.5f) * band;
                var s = (ab ? a - along * _curbOffset : b + along * _curbOffset) + side * lat;
                var e = (ab ? b + along * _curbOffset : a - along * _curbOffset) + side * lat;
                w.from = s;
                w.to = e;
                w.length = Vector3.Distance(s, e);
                w.progress = 0f;
                w.speed = Rand(st, _minSpeed, _maxSpeed);
                w.startTime = t + Rand(st, _minCurbWaitSec, _minCurbWaitSec + _maxStartDelaySec);
                var flat = e - s; flat.y = 0f;
                w.go.transform.SetPositionAndRotation(s, Quaternion.LookRotation(flat.normalized));
                w.go.SetActive(true);
                w.ped.Initialize();
                w.ped.PoseInput = new Pose(s, Quaternion.LookRotation(flat.normalized));
                w.active = true;
            }
            st.nextEvent = t + Rand(st, _minIntervalSec, _maxIntervalSec);
            Debug.Log($"[Beytepe] yaya geçidi {st.index}: {n} yaya geçiyor" +
                      (crowd ? (mixed ? " (kalabalık, karışık yön)" : " (kalabalık)") : "") +
                      (waitedSec > 0f ? $" (araç yüzünden {waitedSec:F1} s ertelenmişti)" : "") +
                      $"; sonraki en erken {st.nextEvent - t:F0} s sonra");
        }

        static float Rand(CrosswalkState st, float a, float b) => a + (float)st.rng.NextDouble() * (b - a);

        static float Cross(Vector2 a, Vector2 b) => a.x * b.y - a.y * b.x;

        /// <summary>Ego (AccelVehicle) ve NPC (PoseVehicle) araçlarını izler; hızları konum farkından (yumuşatılmış).</summary>
        void UpdateVehicles(float t)
        {
            if (t >= _nextVehicleScan)
            {
                // NPC'ler havuzdan etkinleşip kaybolur: saniyede bir yeniden tara
                _nextVehicleScan = t + 1f;
                var seen = new HashSet<Transform>();
                foreach (var v in FindObjectsByType<AccelVehicle>(FindObjectsSortMode.None))
                {
                    seen.Add(v.transform);
                    if (!_vehicles.ContainsKey(v.transform))
                        _vehicles[v.transform] = new Vehicle { tf = v.transform, ego = v };
                }
                foreach (var v in FindObjectsByType<PoseVehicle>(FindObjectsSortMode.None))
                    if (v.isActiveAndEnabled && v.VisualObjectRootActiveSelf)
                        seen.Add(v.RigidBodyTransform != null ? v.RigidBodyTransform : v.transform);
                foreach (var tf in seen)
                    if (!_vehicles.ContainsKey(tf))
                        _vehicles[tf] = new Vehicle { tf = tf };
                var gone = new List<Transform>();
                foreach (var kv in _vehicles)
                    if (kv.Key == null || !seen.Contains(kv.Key)) gone.Add(kv.Key);
                foreach (var tf in gone) _vehicles.Remove(tf);
            }
            float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);
            foreach (var v in _vehicles.Values)
            {
                if (v.tf == null) continue;
                var p = v.tf.position;
                if (v.ego != null)
                    v.vel = v.ego.Velocity;
                else if (v.init)
                {
                    var inst = (p - v.pos) / dt;
                    v.vel = (inst - v.vel).sqrMagnitude > 400f ? inst : Vector3.Lerp(v.vel, inst, 0.3f);   // ışınlanmada sıfırla
                }
                v.pos = p; v.init = true;
            }
        }

        /// <summary>Geçidin yatay düzlemdeki geometrisi: bir uç (a), eksen yönü (u), uzunluk ve yarı genişlik.</summary>
        struct CrosswalkGeom
        {
            public Vector2 a, u;
            public float len, half;
        }

        static bool TryGeom(Crosswalk cw, out CrosswalkGeom g)
        {
            g = default;
            g.a = new Vector2(cw.start.x, cw.start.z);
            var ab = new Vector2(cw.end.x, cw.end.z) - g.a;
            g.len = ab.magnitude;
            if (g.len < 0.1f) return false;
            g.u = ab / g.len;
            g.half = Mathf.Max(cw.width, 0.5f) * 0.5f;
            return true;
        }

        /// <summary>
        /// Araç bu geçide doğru geliyor mu? Geliyorsa ön tamponundan geçidin yakın kenarına boşluğu (m) ve hızını
        /// (m/s) verir. Araç hareket yönünde ilerlerse geçit eksenini kesiyorsa sayılır. Durmuş araç (yaya geçebilir),
        /// geçide paralel giden, geçidin yanından geçen ve geçidi tamamen geçmiş araç sayılmaz.
        /// Boşluk eksi olabilir: araç şu an geçidin üstünde.
        /// </summary>
        bool Approaching(Vehicle v, CrosswalkGeom g, out float gap, out float sp)
        {
            gap = 0f; sp = 0f;
            if (v.tf == null || !v.init) return false;
            var vel = new Vector2(v.vel.x, v.vel.z);
            sp = vel.magnitude;
            if (sp < _stoppedSpeed) return false;
            var d = vel / sp;
            float den = Cross(d, g.u);
            if (Mathf.Abs(den) < 0.2f) return false;              // geçide paralel giden araç geçidi kesmez
            var ap = g.a - new Vector2(v.pos.x, v.pos.z);
            float s = Cross(ap, g.u) / den;                   // araç kökünden geçit eksenine (hareket yönünde)
            float along = Cross(ap, d) / den;                 // kesişimin geçit üzerindeki yeri (0..len)
            if (along < -1.5f || along > g.len + 1.5f) return false;   // araç geçidin yanından geçiyor
            float halfAlong = g.half / Mathf.Abs(den);        // geçit bandının araç yönündeki yarı genişliği
            if (s + halfAlong + _vehicleRear < 0f) return false;  // araç geçidi tamamen geçti
            gap = s - halfAlong - _vehicleFront;              // ön tampondan geçidin yakın kenarına
            return true;
        }

        /// <summary>
        /// Bu geçide doğru gelen ve geçidin önünde en fazla _yieldDecel ile duramayacak bir araç var mı?
        /// Gereken yavaşlama v² / (2 (boşluk - durma payı)); araç geçide _minTimeGap saniyeden kısa sürede varacaksa da evet.
        /// </summary>
        bool VehicleTooClose(Crosswalk cw, out float vKmh, out float gapM)
        {
            vKmh = 0f; gapM = 0f;
            if (!TryGeom(cw, out var g)) return false;
            foreach (var v in _vehicles.Values)
            {
                if (!Approaching(v, g, out float gap, out float sp)) continue;
                if (gap > _yieldLookahead) continue;
                float need = gap <= _stopMargin + 0.1f ? float.PositiveInfinity : sp * sp / (2f * (gap - _stopMargin));
                float tta = gap / sp;                             // araç geçidin yakın kenarına kaç saniyede varır
                if (need > _yieldDecel || tta < _minTimeGap)
                {
                    vKmh = sp * 3.6f; gapM = gap;
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// Belirme denetimi: bu geçide _spawnMinTta saniyeden kısa sürede varacak (ya da şu an geçidin üstünde olan)
        /// hareketli bir araç var mı? Varsa yeni yaya henüz belirmemeli. İki yol:
        ///  1) aracın önü dümdüz uzatılınca geçide değiyor: varış = boşluk / hız;
        ///  2) virajdan gelen araç: önü geçide değmiyor ama yönü geçidin merkezine _spawnConeDeg'den yakın:
        ///     varış = (merkeze kuş uçuşu uzaklık - geçidin yarı boyu - ön tampon) / hız. Kuş uçuşu uzaklık yoldan
        ///     kısa olduğu için bu tahmin erken davranır (temkinli).
        /// </summary>
        bool VehicleArrivingSoon(Crosswalk cw, out float vKmh, out float gapM, out float ttaS)
        {
            vKmh = 0f; gapM = 0f; ttaS = 0f;
            if (!TryGeom(cw, out var g)) return false;
            var center = g.a + g.u * (g.len * 0.5f);
            float reach = Mathf.Max(g.len * 0.5f, g.half);           // merkezden geçidin kenarına (kabaca)
            float cosCone = Mathf.Cos(_spawnConeDeg * Mathf.Deg2Rad);
            foreach (var v in _vehicles.Values)
            {
                float gap, sp, tta;
                if (Approaching(v, g, out gap, out sp))
                {
                    if (gap > _spawnLookahead) continue;
                    tta = gap / sp;                                  // geçidin üstündeki araç için eksi: o da sayılır
                }
                else
                {
                    if (v.tf == null || !v.init) continue;
                    var vel = new Vector2(v.vel.x, v.vel.z);
                    sp = vel.magnitude;
                    if (sp < _stoppedSpeed) continue;
                    var rel = center - new Vector2(v.pos.x, v.pos.z);
                    float dist = rel.magnitude;
                    if (dist > _spawnLookahead || dist < 1e-3f) continue;
                    if (Vector2.Dot(vel, rel) < cosCone * sp * dist) continue;   // geçide doğru gitmiyor
                    gap = dist - reach - _vehicleFront;
                    tta = gap / sp;
                }
                if (tta < _spawnMinTta)
                {
                    vKmh = sp * 3.6f; gapM = gap; ttaS = tta;
                    return true;
                }
            }
            return false;
        }
    }
}
