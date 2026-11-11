// Beytepe binası (2026-11-10): tek başına düzenlenebilir bina (taban çokgeni + yükseklik).
// - Taban: OpenStreetMap bina sınırı, nesneye göre yerel x/z (m). Yükseklik: lidar ölçümü, OSM kat sayısı ya da varsayılan.
// - Basit hal: duvarlar (malzeme 0) + düz çatı (malzeme 1); tabanı zeminin "gomme" m altına iner (eğimli arazide boşluk kalmasın).
// - Cephe detayı ("detayli" açıkken, 2026-11-10): gerçek geometri olarak içeri çekik pencereler (cam = malzeme 2),
//   pencere ortasında dikme, denizlik, kat bantları, çatı parapeti ve çatı ekipmanı (baca / klima kutusu; malzeme 3).
//   Amaç: kameranın (görsel-ataletsel konumlama) ve lidarın gerçek binadaki gibi köşe/kenar bulabilmesi. Fotoğraf basılmaz.
//   Zemin altında kalan pencereler (eğimli arazi) çizilmez; köşe zeminleri "zemin" listesinde (Yere oturt / sahne kurucu ölçer).
// - Fizik: MeshCollider (simülasyondaki lidar ve kamera binayı görür).
// Inspector'da bir değer değişince şekil hemen yenilenir. Nesne taşınabilir, döndürülebilir.
// Sağ tık menüsü: "Yere oturt" (tabanın en alçak zemin noktasına iner, köşe zeminlerini yeniden ölçer).
// Binaların hepsi Assets/Beytepe/Prefabs/Binalar.prefab içinde; sahne kurucu her derlemede bu prefab'ı kullanır.
using System.Collections.Generic;
using UnityEngine;

namespace Beytepe.Runtime
{
    [ExecuteAlways]
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
    public class BeytepeBina : MonoBehaviour
    {
        [Tooltip("Tabanın en alçak zemin noktasından çatıya yükseklik (m)")] public float yukseklik = 10f;
        [Tooltip("Tabanın zemin altına inme derinliği (m)")] public float gomme = 0.5f;
        [Tooltip("Taban çokgeni, nesneye göre yerel (x, z) m")] public List<Vector2> taban = new List<Vector2>();
        public Material duvar;
        public Material cati;
        [Tooltip("Pencere camı malzemesi (boşsa duvar)")] public Material cam;
        [Tooltip("Kat bandı, denizlik, parapet kapağı, çatı ekipmanı malzemesi (boşsa duvar)")] public Material detay;
        [Tooltip("Duvar rengi (duvar dokusuyla çarpılır; 1'den büyük değer dokuyu açar)")] [ColorUsage(false, true)]
        public Color duvarRengi = new Color(0.97f, 0.93f, 0.94f, 1f);
        [Tooltip("OpenStreetMap bina kimliği (bilgi amaçlı)")] public string osmKimlik = "";
        [Tooltip("Yükseklik kaynağı: lidar / osm_kat / varsayilan ...")] public string kaynak = "";

        [Header("Cephe detayı")]
        [Tooltip("Pencere, kat bandı, parapet ve çatı ekipmanını üret")] public bool detayli = false;
        [Tooltip("Pencere sıraları (kapalıysa yalnız parapet ve çatı ekipmanı; küçük ara yapılar için)")] public bool pencereler = true;
        [Tooltip("Kat yüksekliği (m); kat sayısı yükseklikten hesaplanır")] public float katYuksekligi = 3.6f;
        [Tooltip("Pencere genişliği (m)")] public float pencereGenislik = 1.5f;
        [Tooltip("Pencere yüksekliği (m)")] public float pencereYukseklik = 1.6f;
        [Tooltip("Pencere eksenleri arası mesafe (m)")] public float pencereAraligi = 3.0f;
        [Tooltip("Pencerenin duvardan içeri çekilme derinliği (m)")] public float pencereDerinlik = 0.22f;
        [Tooltip("Pencere alt kenarının kat tabanından yüksekliği (m)")] public float denizlik = 0.9f;
        [Tooltip("Bina köşesi ile ilk pencere kenarı arasındaki en az mesafe (m)")] public float kenarPayi = 1.2f;
        [Tooltip("Kat hizalarında dışa çıkık yatay bant")] public bool katBandi = true;
        public float bandYukseklik = 0.3f;
        public float bandCikinti = 0.08f;
        [Tooltip("Çatı kenarındaki duvarın çatıdan yüksekliği (m); 0 = parapet yok")] public float parapet = 0.9f;
        public float parapetKalinlik = 0.25f;
        [Tooltip("Çatıdaki baca sayısı")] public int catiBaca = 0;
        [Tooltip("Çatıdaki klima/makine kutusu sayısı")] public int catiKutu = 0;
        [Header("Saçak ve eğimli çatı (2026-11-11)")]
        [Tooltip("Çatının duvardan dışarı taşma mesafesi (m); 0 = saçak yok. Saçak varsa parapet çizilmez")] public float sacak = 0f;
        [Tooltip("Saçak alnının (dış kenarının) kalınlığı (m)")] public float sacakKalinlik = 0.4f;
        [Tooltip("Saçak alnındaki dikey nervürlerin aralığı (m); 0 = nervür yok")] public float sacakNervur = 0f;
        [Tooltip("Eğimli (kırma) çatının yüksekliği (m); 0 = düz çatı. Eğimli çatıda parapet ve makine kutusu çizilmez")] public float egimliCati = 0f;
        [Tooltip("Çatı eğiminin kenardan içeri genişliği (m); 0 = otomatik (binanın dar yanının yaklaşık yarısı)")] public float egimGenislik = 0f;
        [Tooltip("Köşe noktalarında zemin yüksekliği (nesneye göre, m); Yere oturt ve sahne kurucu doldurur")]
        public List<float> zemin = new List<float>();

        Mesh _mesh;
        List<Vector3> _v; List<Vector2> _uv; List<int>[] _s;

        void OnEnable() { Olustur(); }

#if UNITY_EDITOR
        void OnValidate()
        {
            UnityEditor.EditorApplication.delayCall += () => { if (this != null) Olustur(); };
        }
#endif

        void OnDestroy()
        {
            if (_mesh != null) { if (Application.isPlaying) Destroy(_mesh); else DestroyImmediate(_mesh); }
        }

        static float Alan(List<Vector2> p)
        {
            float a = 0f;
            for (int i = 0; i < p.Count; i++) { var u = p[i]; var v = p[(i + 1) % p.Count]; a += u.x * v.y - v.x * u.y; }
            return a / 2f;   // > 0: (x, z) düzleminde saat yönü tersi
        }

        // Çokgeni (saat yönü tersi) t kadar içeri kaydırır (t < 0: dışarı); köşeler açıortay boyunca
        static List<Vector2> Kaydir(List<Vector2> p, float t)
        {
            var q = new List<Vector2>();
            for (int i = 0; i < p.Count; i++)
            {
                var pr = p[(i + p.Count - 1) % p.Count]; var c = p[i]; var nx = p[(i + 1) % p.Count];
                var e1 = (c - pr).normalized; var e2 = (nx - c).normalized;
                var n1 = new Vector2(-e1.y, e1.x); var n2 = new Vector2(-e2.y, e2.x);   // iç normaller
                var m = n1 + n2; float den = 1f + Vector2.Dot(n1, n2);
                q.Add(den > 0.2f ? c + m * (t / den) : c + n2 * t);
            }
            return q;
        }

        // İçeri kaydırılmış çokgen bozulmamış mı (kenar yönü dönmemiş, alan pozitif, köşeler dışta değil)
        static bool GecerliIc(List<Vector2> p, List<Vector2> q)
        {
            if (Alan(q) <= 0.5f) return false;
            for (int i = 0; i < p.Count; i++)
            {
                int j = (i + 1) % p.Count;
                var a = p[j] - p[i]; var b = q[j] - q[i];
                if (a.sqrMagnitude > 1e-6f && Vector2.Dot(a, b) <= 0f) return false;
                if (!Icinde(q[i], p)) return false;
            }
            return true;
        }

        // En uzun kenara hizalı kutuya göre binanın dar yanı (m)
        static float DarGenislik(List<Vector2> p)
        {
            Vector2 ax = Vector2.right; float best = 0f;
            for (int i = 0; i < p.Count; i++) { var e = p[(i + 1) % p.Count] - p[i]; if (e.magnitude > best) { best = e.magnitude; ax = e.normalized; } }
            var ay = new Vector2(-ax.y, ax.x); float mn = float.MaxValue, mx = float.MinValue;
            foreach (var q in p) { float d = Vector2.Dot(q, ay); mn = Mathf.Min(mn, d); mx = Mathf.Max(mx, d); }
            return mx - mn;
        }

        static float Cross(Vector2 a, Vector2 b, Vector2 c) => (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

        static bool InTri(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
        {
            float d1 = Cross(a, b, p), d2 = Cross(b, c, p), d3 = Cross(c, a, p);
            return d1 >= 0f && d2 >= 0f && d3 >= 0f;
        }

        static bool Icinde(Vector2 pt, List<Vector2> poly)
        {
            bool inside = false;
            for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
            {
                var a = poly[i]; var b = poly[j];
                if ((a.y > pt.y) != (b.y > pt.y) && pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
            }
            return inside;
        }

        // Kulak kırpma ile çatı üçgenleri (saat yönü tersi çokgen); her zaman çokgen-2 üçgen döner, takılırsa yelpazeye düşer.
        static List<int> Ucgenle(List<Vector2> p)
        {
            var res = new List<int>();
            var idx = new List<int>(); for (int i = 0; i < p.Count; i++) idx.Add(i);
            int guard = 0;
            while (idx.Count > 3 && guard++ < 10000)
            {
                bool cut = false;
                for (int i = 0; i < idx.Count; i++)
                {
                    int ia = idx[(i + idx.Count - 1) % idx.Count], ib = idx[i], ic = idx[(i + 1) % idx.Count];
                    if (Cross(p[ia], p[ib], p[ic]) <= 1e-6f) continue;   // içbükey köşe
                    bool any = false;
                    foreach (var j in idx)
                        if (j != ia && j != ib && j != ic && InTri(p[j], p[ia], p[ib], p[ic])) { any = true; break; }
                    if (any) continue;
                    res.Add(ia); res.Add(ib); res.Add(ic); idx.RemoveAt(i); cut = true; break;
                }
                if (!cut) break;
            }
            for (int i = 1; i + 1 < idx.Count; i++) { res.Add(idx[0]); res.Add(idx[i]); res.Add(idx[i + 1]); }
            return res;
        }

        // Unity'de üçgen (a,b,c)'nin ön yüzü Cross(b-a, c-a) yönüne bakar; istenen yöne göre sıralanır.
        static void Tri(List<int> t, List<Vector3> v, int a, int b, int c, Vector3 want)
        {
            if (Vector3.Dot(Vector3.Cross(v[b] - v[a], v[c] - v[a]), want) >= 0f) { t.Add(a); t.Add(b); t.Add(c); }
            else { t.Add(a); t.Add(c); t.Add(b); }
        }

        // Doku koordinatı metre cinsinden: dik yüzlerde (yatay eksen, yükseklik), yatay yüzlerde (x, z)
        static Vector2 Uv(Vector3 p, Vector3 n)
        {
            if (Mathf.Abs(n.y) > 0.5f) return new Vector2(p.x, p.z);
            var t = new Vector3(-n.z, 0f, n.x);
            return new Vector2(Vector3.Dot(p, t), p.y);
        }

        // Dört köşesi sırayla (a-b-c-d çevresi) verilen dörtgen; ön yüzü n yönüne bakar
        void Quad(int s, Vector3 a, Vector3 b, Vector3 c, Vector3 d, Vector3 n)
        {
            int k = _v.Count;
            _v.Add(a); _v.Add(b); _v.Add(c); _v.Add(d);
            _uv.Add(Uv(a, n)); _uv.Add(Uv(b, n)); _uv.Add(Uv(c, n)); _uv.Add(Uv(d, n));
            Tri(_s[s], _v, k, k + 1, k + 2, n); Tri(_s[s], _v, k, k + 2, k + 3, n);
        }

        // Merkez c, eksenler A, B (birim), yarı boyutlar ha, hb; ön yüz n
        void Face(int s, Vector3 c, Vector3 A, Vector3 B, float ha, float hb, Vector3 n)
        {
            A *= ha; B *= hb;
            Quad(s, c - A - B, c + A - B, c + A + B, c - A + B, n);
        }

        // Yönlü kutu: merkez c, yatay eksenler u, w (birim, dik), yarı boyutlar; alt yüz çizilmez
        void Box(int s, Vector3 c, Vector3 u, Vector3 w, float hu, float hv, float hw)
        {
            var up = Vector3.up;
            Face(s, c + u * hu, w, up, hw, hv, u); Face(s, c - u * hu, w, up, hw, hv, -u);
            Face(s, c + w * hw, u, up, hu, hv, w); Face(s, c - w * hw, u, up, hu, hv, -w);
            Face(s, c + up * hv, u, w, hu, hw, up);
        }

        public void Olustur()
        {
            if (taban == null || taban.Count < 3) return;
            var p = new List<Vector2>(taban);
            var gi = new List<int>(); for (int i = 0; i < taban.Count; i++) gi.Add(i);
            if ((p[0] - p[p.Count - 1]).sqrMagnitude < 1e-6f) { p.RemoveAt(p.Count - 1); gi.RemoveAt(gi.Count - 1); }
            if (p.Count < 3) return;
            if (Alan(p) < 0f) { p.Reverse(); gi.Reverse(); }
            bool zOk = zemin != null && zemin.Count == taban.Count;
            var zg = new List<float>(); foreach (var i in gi) zg.Add(zOk ? zemin[i] : 0f);

            float h = Mathf.Max(0.5f, yukseklik), g = Mathf.Max(0f, gomme);
            if (_mesh == null) _mesh = new Mesh { name = "bina", hideFlags = HideFlags.DontSave };
            _mesh.Clear();
            _v = new List<Vector3>(); _uv = new List<Vector2>(); _s = new[] { new List<int>(), new List<int>(), new List<int>(), new List<int>() };

            bool det = detayli;
            float sc = det ? Mathf.Max(0f, sacak) : 0f, sk = Mathf.Clamp(sacakKalinlik, 0.05f, 3f), hr = det ? Mathf.Max(0f, egimliCati) : 0f;
            float par = det && sc <= 0f && hr <= 0f ? Mathf.Max(0f, parapet) : 0f;
            float H = h + par;
            float kh = Mathf.Max(2.4f, katYuksekligi), pw = Mathf.Max(0.3f, pencereGenislik), ph = Mathf.Max(0.3f, pencereYukseklik);
            float pa = Mathf.Max(pw + 0.4f, pencereAraligi), pd = Mathf.Clamp(pencereDerinlik, 0.02f, 1f), dz = Mathf.Max(0f, denizlik);
            // kat sayısı: pencerenin üstü çatıdan en az 0.3 m aşağıda kalan katlar
            // (saçak varsa pencerenin üstü saçak alnının altında kalır)
            int nKat = det && pencereler ? Mathf.Max(0, Mathf.FloorToInt((h - 0.3f - (sc > 0f ? sk : 0f) - dz - ph) / kh) + 1) : 0;

            for (int i = 0; i < p.Count; i++)
            {
                int j = (i + 1) % p.Count;
                var a = p[i]; var b = p[j];
                var d2 = b - a; float L = d2.magnitude; if (L < 1e-3f) continue;
                var A = new Vector3(a.x, 0f, a.y);
                var dir = new Vector3(d2.x, 0f, d2.y) / L;
                var n = new Vector3(d2.y, 0f, -d2.x) / L;   // saat yönü tersi çokgende dış normal
                Vector3 P(float u, float v, float dep) => A + dir * u + Vector3.up * v - n * dep;
                float Zem(float u) => Mathf.Lerp(zg[i], zg[j], Mathf.Clamp01(u / L));

                // pencere sütunları (kenarlardan kenarPayi uzakta, eşit aralıklı)
                // kısa duvar parçalarında köşe payı kısalır (7 m'lik parçada tek pencere ve geniş boş duvar kalmasın)
                float kp = Mathf.Min(Mathf.Max(0f, kenarPayi), Mathf.Max(0.5f, L * 0.12f));
                int nb = nKat > 0 ? Mathf.FloorToInt((L - 2f * kp - pw) / pa) + 1 : 0;
                if (nb < 1 || L < pw + 2f * kp) nb = 0;
                float u0 = (L - (nb - 1) * pa) / 2f;
                var uB = new List<float> { 0f }; var vB = new List<float> { -g };
                for (int k = 0; k < nb; k++) { float uc = u0 + k * pa; uB.Add(uc - pw / 2f); uB.Add(uc + pw / 2f); }
                uB.Add(L);
                for (int f = 0; f < nKat; f++) { float wb = f * kh + dz; vB.Add(wb); vB.Add(wb + ph); }
                vB.Add(H);

                for (int vi = 0; vi + 1 < vB.Count; vi++)
                {
                    float v0 = vB[vi], v1 = vB[vi + 1];
                    if (v1 - v0 < 1e-4f) continue;
                    bool winRow = (vi % 2 == 1);   // pencere bandı
                    if (!winRow) { Quad(0, P(0, v0, 0), P(L, v0, 0), P(L, v1, 0), P(0, v1, 0), n); continue; }
                    for (int ui = 0; ui + 1 < uB.Count; ui++)
                    {
                        float ua = uB[ui], ub = uB[ui + 1];
                        if (ub - ua < 1e-4f) continue;
                        bool hole = (ui % 2 == 1) && v0 >= Mathf.Max(Zem(ua), Zem(ub)) + 0.3f;
                        if (!hole) { Quad(0, P(ua, v0, 0), P(ub, v0, 0), P(ub, v1, 0), P(ua, v1, 0), n); continue; }
                        // içeri çekik pencere: yan, alt, üst iç yüzler + cam + dikme + denizlik
                        Quad(0, P(ua, v0, 0), P(ua, v1, 0), P(ua, v1, pd), P(ua, v0, pd), dir);
                        Quad(0, P(ub, v0, 0), P(ub, v1, 0), P(ub, v1, pd), P(ub, v0, pd), -dir);
                        Quad(0, P(ua, v0, 0), P(ub, v0, 0), P(ub, v0, pd), P(ua, v0, pd), Vector3.up);
                        Quad(0, P(ua, v1, 0), P(ub, v1, 0), P(ub, v1, pd), P(ua, v1, pd), Vector3.down);
                        Quad(2, P(ua, v0, pd), P(ub, v0, pd), P(ub, v1, pd), P(ua, v1, pd), n);
                        float um = (ua + ub) / 2f, vm = (v0 + v1) / 2f;
                        Box(3, P(um, vm, pd - 0.03f), dir, n, 0.035f, (v1 - v0) / 2f, 0.03f);        // dikme
                        Box(3, P(um, vm + (v1 - v0) * 0.18f, pd - 0.03f), dir, n, (ub - ua) / 2f, 0.03f, 0.03f);   // kayıt
                        Box(3, P(um, v0 - 0.04f, -0.03f), dir, n, (ub - ua) / 2f + 0.06f, 0.04f, 0.06f);         // denizlik
                    }
                }

                if (det && katBandi && bandYukseklik > 0f && bandCikinti > 0f)
                    for (int f = 1; f <= nKat; f++)
                    {
                        float y = f * kh; if (y > h - bandYukseklik) break;
                        float hc = bandCikinti / 2f + 0.01f;
                        Box(3, P(L / 2f, y, -bandCikinti / 2f + 0.01f), dir, n, L / 2f + bandCikinti, bandYukseklik / 2f, hc);
                    }
            }

            // çatı (parapet varsa içe kaydırılmış çokgen) + parapet iç yüzü ve kapağı
            var roofPoly = p;
            Vector3 Pi(Vector2 v2, float y) => new Vector3(v2.x, y, v2.y);
            if (par > 0f)
            {
                float t = Mathf.Clamp(parapetKalinlik, 0.05f, 1f);
                var q = Kaydir(p, t);
                roofPoly = q;
                for (int i = 0; i < p.Count; i++)
                {
                    int j = (i + 1) % p.Count;
                    var e = q[j] - q[i]; if (e.sqrMagnitude < 1e-6f) continue;
                    var inN = new Vector3(-e.y, 0f, e.x).normalized;   // parapetin iç yüzü çatıya (içeri) bakar
                    Quad(0, Pi(q[i], h), Pi(q[j], h), Pi(q[j], H), Pi(q[i], H), inN);
                    Quad(3, Pi(p[i], H), Pi(p[j], H), Pi(q[j], H), Pi(q[i], H), Vector3.up);
                }
            }
            // saçak: dışa kaydırılmış çokgen; alın (dış kenar), alt yüz, üst yüz; isteğe bağlı dikey nervürler
            List<Vector2> o = sc > 0f ? Kaydir(p, -sc) : null;
            // eğimli (kırma) çatı: kenardan içeri ve yukarı eğimli yüzler + üstte dar düz sırt
            var bp = o ?? roofPoly;
            List<Vector2> ins = null;
            if (hr > 0f)
            {
                float d = egimGenislik > 0f ? egimGenislik : 0.45f * DarGenislik(bp);
                for (int tr = 0; tr < 10 && d > 0.3f; tr++, d *= 0.7f)
                {
                    var c = Kaydir(bp, d);
                    if (GecerliIc(bp, c)) { ins = c; break; }
                }
            }
            if (o != null)
            {
                float yb = h - sk;
                for (int i = 0; i < p.Count; i++)
                {
                    int j = (i + 1) % p.Count;
                    var e = o[j] - o[i]; float Le = e.magnitude; if (Le < 1e-3f) continue;
                    var outN = new Vector3(e.y, 0f, -e.x) / Le;
                    Quad(3, Pi(o[i], yb), Pi(o[j], yb), Pi(o[j], h), Pi(o[i], h), outN);
                    Quad(3, Pi(p[i], yb), Pi(p[j], yb), Pi(o[j], yb), Pi(o[i], yb), Vector3.down);
                    if (ins == null) Quad(1, Pi(p[i], h), Pi(p[j], h), Pi(o[j], h), Pi(o[i], h), Vector3.up);
                    if (sacakNervur > 0.2f)
                    {
                        var du = new Vector3(e.x, 0f, e.y) / Le;
                        int nn = Mathf.FloorToInt(Le / sacakNervur);
                        for (int k = 1; k < nn; k++)
                            Box(3, Pi(o[i] + e * (k * sacakNervur / Le), yb + sk / 2f) + outN * 0.03f, du, outN, 0.04f, sk / 2f, 0.03f);
                    }
                }
            }
            if (ins != null)
            {
                for (int i = 0; i < bp.Count; i++)
                {
                    int j = (i + 1) % bp.Count;
                    var e = bp[j] - bp[i]; float Le = e.magnitude; if (Le < 1e-3f) continue;
                    var outN = new Vector3(e.y, 0f, -e.x) / Le;
                    float dd = Vector2.Distance((bp[i] + bp[j]) / 2f, (ins[i] + ins[j]) / 2f);
                    Quad(1, Pi(bp[i], h), Pi(bp[j], h), Pi(ins[j], h + hr), Pi(ins[i], h + hr), (outN * hr + Vector3.up * Mathf.Max(dd, 0.1f)).normalized);
                }
                int s0 = _v.Count;
                foreach (var qv in ins) { var w = new Vector3(qv.x, h + hr, qv.y); _v.Add(w); _uv.Add(new Vector2(w.x, w.z)); }
                var st = Ucgenle(ins);
                for (int i = 0; i < st.Count; i += 3) Tri(_s[1], _v, s0 + st[i], s0 + st[i + 1], s0 + st[i + 2], Vector3.up);
            }
            else
            {
                int r0 = _v.Count;
                foreach (var qv in roofPoly) { var w = new Vector3(qv.x, h, qv.y); _v.Add(w); _uv.Add(new Vector2(w.x, w.z)); }
                var rt = Ucgenle(roofPoly);
                for (int i = 0; i < rt.Count; i += 3) Tri(_s[1], _v, r0 + rt[i], r0 + rt[i + 1], r0 + rt[i + 2], Vector3.up);
            }

            if (det && (catiBaca > 0 || catiKutu > 0)) CatiEkipmani(ins != null ? bp : roofPoly, h, ins != null ? hr : 0f);

            _mesh.indexFormat = _v.Count > 65000 ? UnityEngine.Rendering.IndexFormat.UInt32 : UnityEngine.Rendering.IndexFormat.UInt16;
            _mesh.SetVertices(_v);
            _mesh.SetUVs(0, _uv);
            _mesh.subMeshCount = 4;
            for (int s = 0; s < 4; s++) _mesh.SetTriangles(_s[s], s);
            _mesh.RecalculateNormals();
            _mesh.RecalculateTangents();
            _mesh.RecalculateBounds();
            _v = null; _uv = null; _s = null;
            GetComponent<MeshFilter>().sharedMesh = _mesh;
            var mr = GetComponent<MeshRenderer>();
            if (duvar != null && cati != null)
                mr.sharedMaterials = new[] { duvar, cati, cam != null ? cam : duvar, detay != null ? detay : duvar };
            var mpb = new MaterialPropertyBlock();
            mpb.SetColor("_BaseColor", duvarRengi);
            mr.SetPropertyBlock(mpb, 0);
            var mc = GetComponent<MeshCollider>();
            mc.sharedMesh = null; mc.sharedMesh = _mesh;
        }

        // Çatıya baca ve makine kutuları (yerleşim osmKimlik'ten türeyen sabit tohumla; her derlemede aynı)
        // ek > 0: eğimli çatı (makine kutusu yok, bacalar çatı yüksekliği kadar uzar)
        void CatiEkipmani(List<Vector2> poly, float h, float ek = 0f)
        {
            int seed = 17; foreach (var ch in osmKimlik ?? "") seed = seed * 31 + ch;
            var rng = new System.Random(seed);
            float R(float a, float b) => a + (float)rng.NextDouble() * (b - a);
            // eksenler en uzun kenara hizalı
            Vector2 ax = Vector2.right; float best = 0f;
            for (int i = 0; i < poly.Count; i++) { var e = poly[(i + 1) % poly.Count] - poly[i]; if (e.magnitude > best) { best = e.magnitude; ax = e.normalized; } }
            var ay = new Vector2(-ax.y, ax.x);
            var u = new Vector3(ax.x, 0f, ax.y); var w = new Vector3(ay.x, 0f, ay.y);
            Vector2 mn = new Vector2(float.MaxValue, float.MaxValue), mx = -mn;
            foreach (var q in poly) { mn = Vector2.Min(mn, q); mx = Vector2.Max(mx, q); }
            var placed = new List<(Vector2 c, float r)>();
            bool Fits(Vector2 c, float hu, float hw)
            {
                for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2)
                        if (!Icinde(c + ax * (hu + 0.8f) * sx + ay * (hw + 0.8f) * sy, poly)) return false;
                float r = Mathf.Sqrt(hu * hu + hw * hw);
                foreach (var o in placed) if ((o.c - c).magnitude < o.r + r + 0.6f) return false;
                placed.Add((c, r)); return true;
            }
            for (int k = 0; k < (ek > 0f ? 0 : catiKutu); k++)
            {
                float hu = R(0.9f, 1.8f), hw = R(0.6f, 1.2f), hv = R(0.6f, 1.0f);
                for (int tr = 0; tr < 60; tr++)
                {
                    var c = new Vector2(R(mn.x, mx.x), R(mn.y, mx.y));
                    if (!Fits(c, hu, hw)) continue;
                    var c3 = new Vector3(c.x, h + hv, c.y);
                    Box(3, c3, u, w, hu, hv, hw);
                    Box(3, c3 + Vector3.up * (hv + 0.05f), u, w, hu * 0.4f, 0.05f, hw * 0.4f);   // üst ızgara
                    break;
                }
            }
            for (int k = 0; k < catiBaca; k++)
            {
                float s = R(0.18f, 0.3f), hv = R(1.0f, 2.0f) + ek / 2f;
                for (int tr = 0; tr < 60; tr++)
                {
                    var c = new Vector2(R(mn.x, mx.x), R(mn.y, mx.y));
                    if (!Fits(c, s, s)) continue;
                    var c3 = new Vector3(c.x, h + hv, c.y);
                    Box(3, c3, u, w, s, hv, s);
                    Box(3, c3 + Vector3.up * (hv + 0.08f), u, w, s * 1.5f, 0.08f, s * 1.5f);   // baca şapkası
                    break;
                }
            }
        }

        // Taban köşelerinde zemin yüksekliğini ölçer (nesneye göre); diğer binaların çatısı sayılmaz
        public bool ZeminiOlc()
        {
            if (taban == null || taban.Count < 3) return false;
            var col = GetComponent<MeshCollider>();
            bool was = col.enabled; col.enabled = false;
            Physics.SyncTransforms();
            var z = new List<float>(); bool all = true;
            foreach (var q in taban)
            {
                var wp = transform.TransformPoint(new Vector3(q.x, 0f, q.y));
                float best = float.MinValue; bool ok = false;
                foreach (var hit in Physics.RaycastAll(new Vector3(wp.x, wp.y + 60f, wp.z), Vector3.down, 150f))
                    if (hit.collider.GetComponent<BeytepeBina>() == null && hit.point.y > best) { best = hit.point.y; ok = true; }
                z.Add(ok ? best - transform.position.y : 0f); all &= ok;
            }
            col.enabled = was;
            zemin = z;
            return all;
        }

        // Tabanın köşelerinde ve ortasında zemini ölçer; nesneyi en alçak zemin noktasına indirir (yükseklik bu noktadan ölçülür).
        [ContextMenu("Yere oturt (en alçak zemin noktası)")]
        public void YereOturt()
        {
            if (taban == null || taban.Count < 3) return;
            var col = GetComponent<MeshCollider>();
            bool was = col.enabled; col.enabled = false;
            Physics.SyncTransforms();
            var ys = new List<float>();
            var pts = new List<Vector2>(taban); var c = Vector2.zero; foreach (var q in taban) c += q; pts.Add(c / taban.Count);
            foreach (var q in pts)
            {
                var w = transform.TransformPoint(new Vector3(q.x, 0f, q.y));
                // diğer binaların çatısına değil zemine bak (bitişik binalar köşe paylaşır)
                float best = float.MinValue; bool ok = false;
                foreach (var hit in Physics.RaycastAll(new Vector3(w.x, w.y + 60f, w.z), Vector3.down, 150f))
                    if (hit.collider.GetComponent<BeytepeBina>() == null && hit.point.y > best) { best = hit.point.y; ok = true; }
                if (ok) ys.Add(best);
            }
            if (ys.Count > 0)
            {
#if UNITY_EDITOR
                UnityEditor.Undo.RecordObject(transform, "Yere oturt");
                UnityEditor.Undo.RecordObject(this, "Yere oturt");
#endif
                float mn = float.MaxValue; foreach (var y in ys) mn = Mathf.Min(mn, y);
                var pos = transform.position; pos.y = mn; transform.position = pos;
            }
            col.enabled = was;
            ZeminiOlc();
            Olustur();
        }
    }
}
