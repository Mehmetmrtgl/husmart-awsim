// Beytepe nizamiye (ana giriş) yolu (2026-11-12).
// RoadRunner modelinde (Beytepe.fbx) kuzey uçtaki dönüş kulübenin çevresinden dolanan bir halkaydı. Gerçekte araç kulübenin
// önünden (güneyinden) dönüyor, kulübenin doğusundan 3 şerit dışarı çıkıyor, batısından 2 şerit giriyor, batı kenarında bir
// otobüs durağı cebi var (kullanıcının Google Earth çizimi). Lanelet haritası buna göre düzenlendi; bu dosya AWSIM dünyasını
// aynı hale getirir. Veri: Assets/Beytepe/Data/nizamiye.json (ROS harita çerçevesi; Python ile lanelet haritasından üretildi):
//   zemin_hedef / zemin_agirlik : 0.5 m ızgara; yol ve çime dönen alanda zemin yol seviyesine (agirlik 1), çevrede 8 m'lik şevle
//                                 özgün yüksekliğe bağlanır (batıda yamaç kazılır, doğuda dolgu yapılır).
//   cim_maske                   : eski halkanın yeni yolla örtüşmeyen kısmı -> yol/bordür/kaldırım üçgenleri çim alt-meshine taşınır.
//   cizgi_sil_maske             : eski RoadRunner şerit çizgileri silinir (yeni çizgiler cizgi_v/cizgi_t).
//   yol_maske                   : yeni yol alanı (ağaç gizleme için).
//   asfalt_v/asfalt_t           : yeni şeritlerin ve durak cebinin asfalt yüzeyi (yol + 2 cm; MeshCollider'lı).
//   durak                       : otobüs durağı kulübesinin yeri ve yönü.
//   bulut_kutu / bulut_yogunluk : NizamiyeBulutYaz() bu kutudaki tüm sahne yüzeylerini örnekleyip nokta bulutu yazar
//                                 (Autoware nokta bulutu haritasının bu bölgesini güncellemek için).
// FBX'e dokunulmaz; değişen meshler sahneye gömülü kopyadır (kasis düzeltmesindeki gibi). Veri dosyası yoksa hiçbir şey yapılmaz.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

namespace Beytepe.EditorTools
{
    public static partial class BeytepeSceneBuilder
    {
        const string GateDataPath = "Assets/Beytepe/Data/nizamiye.json";
        static string GateCloudPath => Environment.GetEnvironmentVariable("BEYTEPE_NIZAMIYE_BULUT")
                                       ?? Path.Combine(Home, "awsim/beytepe_nizamiye_bulut.bin");

        [Serializable] class NizamiyeDurak { public float x, y, z, yaw_deg; }
        [Serializable] class NizamiyeVeri
        {
            public float x0, y0, cozunurluk; public int nx, ny;
            public string zemin_hedef, zemin_agirlik, cim_maske, cizgi_sil_maske, yol_maske;
            public float[] asfalt_v; public int[] asfalt_t; public float[] cizgi_v; public int[] cizgi_t;
            public NizamiyeDurak durak; public float[] bulut_kutu; public float bulut_yogunluk;
        }

        class NizamiyeIzgara
        {
            public float x0, y0, r; public int nx, ny; public float[] hedef, agirlik; public byte[] cim, sil, yol;
            static float[] F32(string b) { var by = Convert.FromBase64String(b); var f = new float[by.Length / 4]; Buffer.BlockCopy(by, 0, f, 0, by.Length); return f; }
            public NizamiyeIzgara(NizamiyeVeri d)
            {
                x0 = d.x0; y0 = d.y0; r = d.cozunurluk; nx = d.nx; ny = d.ny;
                hedef = F32(d.zemin_hedef); agirlik = F32(d.zemin_agirlik);
                cim = Convert.FromBase64String(d.cim_maske); sil = Convert.FromBase64String(d.cizgi_sil_maske); yol = Convert.FromBase64String(d.yol_maske);
                int n = nx * ny;
                if (hedef.Length != n || agirlik.Length != n || cim.Length != n || sil.Length != n || yol.Length != n)
                    throw new Exception($"nizamiye.json ızgara boyu tutmuyor ({nx}x{ny})");
            }
            // ROS (x, y) -> en yakın hücre; dışarıdaysa -1
            public int Hucre(float x, float y)
            {
                int ix = Mathf.FloorToInt((x - x0) / r), iy = Mathf.FloorToInt((y - y0) / r);
                return (ix < 0 || iy < 0 || ix >= nx || iy >= ny) ? -1 : iy * nx + ix;
            }
            public bool Maske(byte[] m, float x, float y) { int c = Hucre(x, y); return c >= 0 && m[c] != 0; }
            // Hücre merkezleri arasında çift doğrusal; hedefi olmayan (-999) hücreler ağırlık 0 sayılır.
            public bool Zemin(float x, float y, out float hedefZ, out float w)
            {
                hedefZ = 0f; w = 0f;
                float fx = (x - x0) / r - 0.5f, fy = (y - y0) / r - 0.5f;
                int i0 = Mathf.FloorToInt(fx), j0 = Mathf.FloorToInt(fy);
                if (i0 < 0 || j0 < 0 || i0 + 1 >= nx || j0 + 1 >= ny) return false;
                float tx = fx - i0, ty = fy - j0, sw = 0f, sz = 0f, wsum = 0f;
                for (int dj = 0; dj < 2; dj++)
                    for (int di = 0; di < 2; di++)
                    {
                        int c = (j0 + dj) * nx + (i0 + di);
                        float k = (di == 0 ? 1 - tx : tx) * (dj == 0 ? 1 - ty : ty);
                        if (hedef[c] < -900f) continue;
                        sw += k * agirlik[c]; sz += k * hedef[c]; wsum += k;
                    }
                if (wsum < 1e-4f) return false;
                hedefZ = sz / wsum; w = sw;   // eksik köşe ağırlığı 0'a çeker (bölge sınırında yumuşak geçiş)
                return w > 1e-4f;
            }
        }

        static Vector3 UnityToRos(Vector3 u) => new Vector3(u.z, -u.x, u.y);

        static bool IsRoadLikeMat(string n) => new[] { "Asphalt", "Concrete1_Road", "Gutter", "Curb", "Sidewalk" }
            .Any(k => n.IndexOf(k, StringComparison.OrdinalIgnoreCase) >= 0);
        static bool IsMarkingMat(string n) => n.IndexOf("LaneMarking", StringComparison.OrdinalIgnoreCase) >= 0;

        static Material FindModelMaterial(GameObject model, Func<string, bool> pred)
        {
            foreach (var mr in model.GetComponentsInChildren<MeshRenderer>(true))
                foreach (var m in mr.sharedMaterials)
                    if (m != null && pred(m.name)) return m;
            return null;
        }

        // Sahne kurulumunda (AddBumps'tan önce) çağrılır.
        static void ApplyNizamiye(GameObject model, Transform parent)
        {
            if (model == null) return;
            if (!File.Exists(GateDataPath)) { L("Nizamiye: veri yok (" + GateDataPath + "), RoadRunner halkası olduğu gibi kaldı"); return; }
            var d = JsonUtility.FromJson<NizamiyeVeri>(File.ReadAllText(GateDataPath));
            var g = new NizamiyeIzgara(d);
            var matAsphalt = FindModelMaterial(model, n => n.IndexOf("Asphalt", StringComparison.OrdinalIgnoreCase) >= 0);
            var matGrass = FindModelMaterial(model, n => n.IndexOf("Grass", StringComparison.OrdinalIgnoreCase) >= 0);
            var matMark = FindModelMaterial(model, n => IsMarkingMat(n) && n.IndexOf("Yellow", StringComparison.OrdinalIgnoreCase) < 0);
            if (matAsphalt == null || matGrass == null) { L("UYARI: Nizamiye: asfalt/çim malzemesi bulunamadı, atlandı"); return; }

            // Bölge kutusu (Unity dünya x/z): ROS x -> Unity z, ROS y -> Unity -x
            float gx0 = g.x0, gx1 = g.x0 + g.nx * g.r, gy0 = g.y0, gy1 = g.y0 + g.ny * g.r;
            var box = new Bounds(); box.SetMinMax(new Vector3(-gy1, -1000f, gx0), new Vector3(-gy0, 1000f, gx1));

            int nMesh = 0, nVert = 0, nGrass = 0, nMarkDel = 0, nCurb = 0, nSplit = 0, nSkirt = 0;
            var targets = new List<MeshFilter>();
            foreach (var mf in model.GetComponentsInChildren<MeshFilter>(true))
            {
                var src = mf.sharedMesh; var mr = mf.GetComponent<MeshRenderer>();
                if (src == null || mr == null || !src.isReadable) continue;
                if (NoColliderKeywords.Any(k => FullName(mf.transform).IndexOf(k, StringComparison.OrdinalIgnoreCase) >= 0)) continue;
                if (!mr.bounds.Intersects(box)) continue;
                targets.Add(mf);
            }
            // Özgün üst yüzey yüksekliği (değiştirilmemiş çarpıştırıcılara yukarıdan ışın). Yüzeyin 5 cm'den fazla altında kalan
            // köşeler (yol/arazi arasındaki etekler, bordür altı) yüzeyle birlikte kaydırılır, yassılaştırılmaz: aralarda çatlak açılmaz.
            Physics.SyncTransforms();
            var topH = new Dictionary<MeshFilter, float[]>();
            foreach (var mf in targets)
            {
                var vv = mf.sharedMesh.vertices; var l2wp = mf.transform.localToWorldMatrix; var h = new float[vv.Length];
                for (int v = 0; v < vv.Length; v++)
                {
                    var w = l2wp.MultiplyPoint3x4(vv[v]); h[v] = w.y;
                    if (!box.Contains(w)) continue;
                    var ros = UnityToRos(w);
                    if (!g.Zemin(ros.x, ros.y, out _, out _)) continue;
                    if (Physics.Raycast(new Vector3(w.x, w.y + 30f, w.z), Vector3.down, out var hit, 60f, ~0, QueryTriggerInteraction.Ignore)) h[v] = hit.point.y;
                }
                topH[mf] = h;
            }
            foreach (var mf in targets)
            {
                var src = mf.sharedMesh; var mr = mf.GetComponent<MeshRenderer>();
                var l2w = mf.transform.localToWorldMatrix; var w2l = mf.transform.worldToLocalMatrix;

                // --- köşe öznitelikleri (bölme sırasında ara değerlenir)
                var pos = new List<Vector3>(src.vertices);
                var top = new List<float>(topH[mf]);
                var nrm = new List<Vector3>(src.normals); bool hasN = nrm.Count == pos.Count;
                var tan = new List<Vector4>(src.tangents); bool hasT = tan.Count == pos.Count;
                var col = new List<Color>(src.colors); bool hasC = col.Count == pos.Count;
                var uvs = new List<List<Vector4>>(); var uvCh = new List<int>();
                for (int ch = 0; ch < 8; ch++)
                {
                    if (!src.HasVertexAttribute((UnityEngine.Rendering.VertexAttribute)((int)UnityEngine.Rendering.VertexAttribute.TexCoord0 + ch))) continue;
                    var l = new List<Vector4>(); src.GetUVs(ch, l);
                    if (l.Count == pos.Count) { uvs.Add(l); uvCh.Add(ch); }
                }
                int subCount = src.subMeshCount;
                var tris = new List<int>[subCount];
                for (int s = 0; s < subCount; s++) tris[s] = new List<int>(src.GetTriangles(s));

                // --- 1) bölge içindeki büyük üçgenleri kenarı <= 1 m olana dek dörde böl (kaba arazide zemin düzeltmesi
                //        asfaltın altından taşmasın). Ortak kenar orta noktaları paylaşılır; komşu bölünmezse o kenarda
                //        ağırlık 0'dır (köşe kıpırdamaz), çatlak oluşmaz.
                var mid = new Dictionary<long, int>();
                int Mid(int i, int j)
                {
                    long key = i < j ? ((long)i << 32) | (uint)j : ((long)j << 32) | (uint)i;
                    if (mid.TryGetValue(key, out var m)) return m;
                    m = pos.Count;
                    pos.Add((pos[i] + pos[j]) * 0.5f);
                    top.Add((top[i] + top[j]) * 0.5f);
                    if (hasN) nrm.Add((nrm[i] + nrm[j]).normalized);
                    if (hasT) { var tt = (tan[i] + tan[j]) * 0.5f; tan.Add(new Vector4(tt.x, tt.y, tt.z, tan[i].w)); }
                    if (hasC) col.Add((col[i] + col[j]) * 0.5f);
                    foreach (var l in uvs) l.Add((l[i] + l[j]) * 0.5f);
                    mid[key] = m; return m;
                }
                bool Relevant(Vector3 wa, Vector3 wb)   // kenar boyunca 0.5 m'de bir: düzeltme ağırlığı ya da maske var mı
                {
                    int n = Mathf.Max(1, Mathf.CeilToInt(Vector3.Distance(wa, wb) / 0.5f));
                    for (int k = 0; k <= n; k++)
                    {
                        var q = Vector3.Lerp(wa, wb, k / (float)n);
                        if (!box.Contains(q)) continue;
                        var r = UnityToRos(q);
                        if (g.Zemin(r.x, r.y, out _, out _) || g.Maske(g.cim, r.x, r.y) || g.Maske(g.sil, r.x, r.y)) return true;
                    }
                    return false;
                }
                bool splitAny = false;
                for (int s = 0; s < subCount; s++)
                {
                    var outT = new List<int>(tris[s].Count);
                    var stack = new Stack<(int a, int b, int c, int depth)>();
                    var t0 = tris[s];
                    for (int k = 0; k + 2 < t0.Count; k += 3)
                    {
                        stack.Push((t0[k], t0[k + 1], t0[k + 2], 0));
                        while (stack.Count > 0)
                        {
                            var (ia, ib, ic, dep) = stack.Pop();
                            var wa = l2w.MultiplyPoint3x4(pos[ia]); var wb = l2w.MultiplyPoint3x4(pos[ib]); var wc = l2w.MultiplyPoint3x4(pos[ic]);
                            float maxE = Mathf.Max(Vector3.Distance(wa, wb), Mathf.Max(Vector3.Distance(wb, wc), Vector3.Distance(wc, wa)));
                            bool inBox = box.Contains(wa) || box.Contains(wb) || box.Contains(wc) || box.Contains((wa + wb + wc) / 3f);
                            if (dep < 10 && maxE > 1.0f && inBox &&
                                (Relevant(wa, wb) || Relevant(wb, wc) || Relevant(wc, wa) || Relevant((wa + wb + wc) / 3f, (wa + wb + wc) / 3f)))
                            {
                                int mab = Mid(ia, ib), mbc = Mid(ib, ic), mca = Mid(ic, ia);
                                stack.Push((ia, mab, mca, dep + 1)); stack.Push((mab, ib, mbc, dep + 1));
                                stack.Push((mca, mbc, ic, dep + 1)); stack.Push((mab, mbc, mca, dep + 1));
                                splitAny = true; nSplit++;
                                continue;
                            }
                            outT.Add(ia); outT.Add(ib); outT.Add(ic);
                        }
                    }
                    tris[s] = outT;
                }

                // --- 2) zemin düzeltmesi
                var changed = new bool[pos.Count]; int nch = 0;
                for (int v = 0; v < pos.Count; v++)
                {
                    var w = l2w.MultiplyPoint3x4(pos[v]);
                    if (!box.Contains(w)) continue;
                    var ros = UnityToRos(w);
                    if (!g.Zemin(ros.x, ros.y, out var hz, out var wt)) continue;
                    float wc = Mathf.Clamp01(wt);
                    float ny = w.y < top[v] - 0.05f ? w.y + wc * (hz - top[v])       // yüzeyin altı: yüzeyle birlikte kaydır
                                                     : Mathf.Lerp(w.y, hz, wc);       // yüzey (ve bordür üstü): hedefe çek
                    if (w.y < top[v] - 0.05f) nSkirt++;
                    if (Mathf.Abs(ny - w.y) < 1e-4f) continue;
                    w.y = ny; pos[v] = w2l.MultiplyPoint3x4(w); changed[v] = true; nch++;
                }

                // --- 3) alt-meshler: halka -> çim; eski çizgiler silinir; yeni yolun içindeki bordür/oluk/kaldırım -> asfalt
                var mats = mr.sharedMaterials.ToList();
                int grassIdx = mats.FindIndex(m => m != null && m.name.IndexOf("Grass", StringComparison.OrdinalIgnoreCase) >= 0);
                int asphIdx = mats.FindIndex(m => m != null && m.name.IndexOf("Asphalt", StringComparison.OrdinalIgnoreCase) >= 0);
                var toGrass = new List<int>(); var toAsph = new List<int>(); int movedHere = 0, delHere = 0, curbHere = 0;
                for (int s = 0; s < subCount && s < mats.Count; s++)
                {
                    var nm = mats[s] != null ? mats[s].name : "";
                    bool road = IsRoadLikeMat(nm), mark = IsMarkingMat(nm);
                    bool edge = road && nm.IndexOf("Asphalt", StringComparison.OrdinalIgnoreCase) < 0 && nm.IndexOf("Concrete1_Road", StringComparison.OrdinalIgnoreCase) < 0;
                    if (!road && !mark) continue;
                    var keep = new List<int>(tris[s].Count); var t = tris[s];
                    for (int k = 0; k + 2 < t.Count; k += 3)
                    {
                        var c = l2w.MultiplyPoint3x4((pos[t[k]] + pos[t[k + 1]] + pos[t[k + 2]]) / 3f);
                        var ros = UnityToRos(c);
                        bool inCim = g.Maske(g.cim, ros.x, ros.y), inSil = g.Maske(g.sil, ros.x, ros.y);
                        // RoadRunner'da çizgiler asfaltta açılmış boşluklara oturur: silinirse delik kalır (gökyüzü görünür).
                        // Bu yüzden eski çizgi üçgeni silinmez, çime ya da asfalta çevrilir.
                        if (mark && inCim) { toGrass.Add(t[k]); toGrass.Add(t[k + 1]); toGrass.Add(t[k + 2]); delHere++; continue; }
                        if (mark && inSil) { toAsph.Add(t[k]); toAsph.Add(t[k + 1]); toAsph.Add(t[k + 2]); delHere++; continue; }
                        if (road && inCim) { toGrass.Add(t[k]); toGrass.Add(t[k + 1]); toGrass.Add(t[k + 2]); movedHere++; continue; }
                        if (edge && inSil) { toAsph.Add(t[k]); toAsph.Add(t[k + 1]); toAsph.Add(t[k + 2]); curbHere++; continue; }
                        keep.Add(t[k]); keep.Add(t[k + 1]); keep.Add(t[k + 2]);
                    }
                    tris[s] = keep;
                }
                if (nch == 0 && movedHere == 0 && delHere == 0 && curbHere == 0 && !splitAny) continue;
                void Into(ref int idx, Material m, List<int> add)
                {
                    if (add.Count == 0) return;
                    if (idx < 0) { idx = mats.Count; mats.Add(m); }
                    if (idx >= tris.Length) { int old = tris.Length; Array.Resize(ref tris, idx + 1); for (int z = old; z < tris.Length; z++) tris[z] = new List<int>(); }
                    tris[idx].AddRange(add);
                }
                Into(ref grassIdx, matGrass, toGrass);
                Into(ref asphIdx, matAsphalt, toAsph);

                // --- 4) değişen köşelerin normalleri komşu üçgenlerden
                if (hasN && nch > 0)
                {
                    var acc = new Vector3[pos.Count];
                    foreach (var t in tris)
                        for (int k = 0; k + 2 < t.Count; k += 3)
                        {
                            int a = t[k], b = t[k + 1], c = t[k + 2];
                            if (!(changed[a] || changed[b] || changed[c])) continue;
                            var fn = Vector3.Cross(pos[b] - pos[a], pos[c] - pos[a]);
                            if (changed[a]) acc[a] += fn; if (changed[b]) acc[b] += fn; if (changed[c]) acc[c] += fn;
                        }
                    for (int v = 0; v < pos.Count; v++)
                        if (changed[v] && acc[v].sqrMagnitude > 1e-12f)
                        {
                            var nn = acc[v].normalized;
                            if (Vector3.Dot(nn, nrm[v]) < 0f) nn = -nn;
                            nrm[v] = nn;
                        }
                }

                var mesh = new Mesh { name = src.name + "_nizamiye", indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
                mesh.SetVertices(pos);
                if (hasN) mesh.SetNormals(nrm);
                if (hasT) mesh.SetTangents(tan);
                if (hasC) mesh.SetColors(col);
                for (int u = 0; u < uvs.Count; u++) mesh.SetUVs(uvCh[u], uvs[u]);
                mesh.subMeshCount = tris.Length;
                for (int s = 0; s < tris.Length; s++) mesh.SetTriangles(tris[s] ?? new List<int>(), s, false);
                mesh.RecalculateBounds();
                mf.sharedMesh = mesh;
                PrefabUtility.RecordPrefabInstancePropertyModifications(mf);
                if (mats.Count != mr.sharedMaterials.Length) { mr.sharedMaterials = mats.ToArray(); PrefabUtility.RecordPrefabInstancePropertyModifications(mr); }
                var mc = mf.GetComponent<MeshCollider>();
                if (mc != null) { mc.sharedMesh = null; mc.sharedMesh = mesh; PrefabUtility.RecordPrefabInstancePropertyModifications(mc); }
                nMesh++; nVert += nch; nGrass += movedHere; nMarkDel += delHere; nCurb += curbHere;
            }
            L($"Nizamiye: {nMesh} mesh düzenlendi: {nSplit} üçgen bölündü (kenar <= 1 m), {nVert} köşe yeniden yüksekliklendi ({nSkirt} yüzey altı köşe yüzeyle birlikte kaydırıldı) (yol seviyesi + 8 m şev), {nGrass} eski yol üçgeni çime, {nCurb} bordür/oluk/kaldırım üçgeni asfalta, {nMarkDel} eski çizgi üçgeni asfalta/çime çevrildi");

            // yeni asfalt ve çizgiler (dünya çerçevesinde; model kökünün dönüşünden bağımsız)
            var asf = MakeNizamiyeMesh("Nizamiye_Asfalt", d.asfalt_v, d.asfalt_t, model.transform, matAsphalt, true);
            var cz = matMark != null ? MakeNizamiyeMesh("Nizamiye_Cizgiler", d.cizgi_v, d.cizgi_t, model.transform, matMark, false) : null;
            Physics.SyncTransforms();

            // yeni yolun ya da kazılan/doldurulan zeminin üstünde kalan RoadRunner ağaç/çalıları gizlenir
            int nTree = 0;
            foreach (var t in model.GetComponentsInChildren<Transform>(true))
            {
                var nm = t.name;
                if (!(nm.StartsWith("Bush_") || nm.StartsWith("Eucalyptus_")) || !nm.EndsWith("Node")) continue;
                if (t.parent != null && (t.parent.name.StartsWith("Bush_") || t.parent.name.StartsWith("Eucalyptus_"))) continue;
                if (!t.gameObject.activeSelf || !box.Contains(t.position)) continue;
                var ros = UnityToRos(t.position);
                bool hide = g.Maske(g.yol, ros.x, ros.y) || g.Maske(g.cim, ros.x, ros.y);
                if (!hide && g.Zemin(ros.x, ros.y, out var hz, out var wt)) hide = Mathf.Abs(hz - t.position.y) * wt > 0.3f;
                if (hide) { t.gameObject.SetActive(false); nTree++; }
            }
            L($"Nizamiye: yeni asfalt {d.asfalt_t.Length / 3} üçgen, çizgi {(cz != null ? d.cizgi_t.Length / 3 : 0)} üçgen" +
              (matMark == null ? " (çizgi malzemesi yok!)" : "") + $"; {nTree} ağaç/çalı gizlendi");

            if (d.durak != null) AddBusStop(d.durak, parent);
        }

        static GameObject MakeNizamiyeMesh(string name, float[] v, int[] t, Transform parent, Material mat, bool collider)
        {
            int n = v.Length / 3;
            var pos = new Vector3[n]; var uv = new Vector2[n];
            for (int i = 0; i < n; i++) { pos[i] = RosToUnity(new Vector3(v[3 * i], v[3 * i + 1], v[3 * i + 2])); uv[i] = new Vector2(v[3 * i], v[3 * i + 1]); }
            var tri = (int[])t.Clone();
            for (int k = 0; k + 2 < tri.Length; k += 3)   // yukarı bakan yüz (Unity'de saat yönü)
                if (Vector3.Cross(pos[tri[k + 1]] - pos[tri[k]], pos[tri[k + 2]] - pos[tri[k]]).y < 0f) { var s = tri[k + 1]; tri[k + 1] = tri[k + 2]; tri[k + 2] = s; }
            var mesh = new Mesh { name = name, indexFormat = UnityEngine.Rendering.IndexFormat.UInt32 };
            mesh.vertices = pos; mesh.uv = uv; mesh.triangles = tri;
            mesh.RecalculateNormals(); mesh.RecalculateTangents(); mesh.RecalculateBounds();
            var go = new GameObject(name);
            go.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            go.transform.SetParent(parent, true);
            go.AddComponent<MeshFilter>().sharedMesh = mesh;
            var mr = go.AddComponent<MeshRenderer>(); mr.sharedMaterial = mat;
            if (collider) go.AddComponent<MeshCollider>().sharedMesh = mesh;
            return go;
        }

        // Basit otobüs durağı kulübesi: çatı, arka ve yan camlar, dikmeler, oturak. Ön yüz yola (doğuya) bakar.
        static void AddBusStop(NizamiyeDurak s, Transform parent)
        {
            var mats = BuildingMaterials();
            float th = s.yaw_deg * Mathf.Deg2Rad;
            var along = RosToUnity(new Vector3(Mathf.Cos(th), Mathf.Sin(th), 0f)).normalized;
            var front = RosToUnity(new Vector3(Mathf.Sin(th), -Mathf.Cos(th), 0f)).normalized;   // yolun sağ normali: doğuya
            var p = RosToUnity(new Vector3(s.x, s.y, s.z));
            if (Physics.Raycast(p + Vector3.up * 5f, Vector3.down, out var hit, 10f)) p.y = hit.point.y;
            var root = new GameObject("Nizamiye_Durak");
            root.transform.SetParent(parent, false);
            root.transform.SetPositionAndRotation(p, Quaternion.LookRotation(front, Vector3.up));   // yerel +z = yol tarafı, +x = yol boyu
            void Part(string n, Vector3 c, Vector3 sz, Material m)
            {
                var b = GameObject.CreatePrimitive(PrimitiveType.Cube); b.name = n;
                b.transform.SetParent(root.transform, false); b.transform.localPosition = c; b.transform.localScale = sz;
                if (m != null) b.GetComponent<MeshRenderer>().sharedMaterial = m;
            }
            const float L_ = 4.0f, D = 1.6f, H = 2.5f;
            Part("Cati", new Vector3(0f, H + 0.05f, 0f), new Vector3(L_ + 0.3f, 0.1f, D + 0.3f), mats.cati);
            Part("ArkaCam", new Vector3(0f, H / 2f + 0.05f, -D / 2f), new Vector3(L_, H - 0.2f, 0.04f), mats.cam);
            Part("YanCam_1", new Vector3(-L_ / 2f, H / 2f + 0.05f, -D / 4f), new Vector3(0.04f, H - 0.2f, D / 2f), mats.cam);
            Part("YanCam_2", new Vector3(L_ / 2f, H / 2f + 0.05f, -D / 4f), new Vector3(0.04f, H - 0.2f, D / 2f), mats.cam);
            foreach (var sx in new[] { -1f, 1f })
                foreach (var sz in new[] { -1f, 1f })
                    Part("Dikme", new Vector3(sx * L_ / 2f, H / 2f, sz * D / 2f), new Vector3(0.08f, H, 0.08f), mats.detay);
            Part("Oturak", new Vector3(0f, 0.45f, -D / 2f + 0.3f), new Vector3(L_ * 0.75f, 0.06f, 0.4f), mats.detay);
            L($"Nizamiye: otobüs durağı kulübesi ROS ({s.x:F1}, {s.y:F1}), yön {s.yaw_deg:F0}°");
        }

        // Sahne kaydedilmeden hemen önce çağrılır: bölge kutusundaki tüm görünen yüzeyleri (zemin, yol, ağaç, bina, kasis, durak)
        // alan ağırlıklı rastgele örnekler, ROS harita çerçevesinde float32 x,y,z olarak yazar.
        static void NizamiyeBulutYaz()
        {
            if (!File.Exists(GateDataPath)) return;
            var d = JsonUtility.FromJson<NizamiyeVeri>(File.ReadAllText(GateDataPath));
            if (d.bulut_kutu == null || d.bulut_kutu.Length != 4) return;
            float x0 = d.bulut_kutu[0], x1 = d.bulut_kutu[1], y0 = d.bulut_kutu[2], y1 = d.bulut_kutu[3], dens = Mathf.Max(0.5f, d.bulut_yogunluk);
            var box = new Bounds(); box.SetMinMax(new Vector3(-y1, -1000f, x0), new Vector3(-y0, 1000f, x1));
            var ego = UnityEngine.Object.FindFirstObjectByType<Awsim.Scene.AutowareSimulationDemo.EgoVehicle>();
            var rnd = new System.Random(12345);
            var outPts = new List<Vector3>(400000); int nObj = 0;
            foreach (var mr in UnityEngine.Object.FindObjectsByType<MeshRenderer>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
            {
                if (!mr.enabled || !mr.bounds.Intersects(box)) continue;
                if (ego != null && mr.transform.IsChildOf(ego.transform)) continue;
                var mf = mr.GetComponent<MeshFilter>(); var m = mf != null ? mf.sharedMesh : null;
                if (m == null || !m.isReadable) continue;
                var l2w = mr.transform.localToWorldMatrix; var vs = m.vertices; var tr = m.triangles; nObj++;
                for (int k = 0; k + 2 < tr.Length; k += 3)
                {
                    var a = l2w.MultiplyPoint3x4(vs[tr[k]]); var b = l2w.MultiplyPoint3x4(vs[tr[k + 1]]); var c = l2w.MultiplyPoint3x4(vs[tr[k + 2]]);
                    var cen = (a + b + c) / 3f;
                    if (!box.Contains(cen)) continue;
                    float area = Vector3.Cross(b - a, c - a).magnitude * 0.5f;
                    double want = area * dens; int nPts = (int)want; if (rnd.NextDouble() < want - nPts) nPts++;
                    for (int i = 0; i < nPts; i++)
                    {
                        float r1 = (float)rnd.NextDouble(), r2 = (float)rnd.NextDouble();
                        if (r1 + r2 > 1f) { r1 = 1f - r1; r2 = 1f - r2; }
                        var q = a + (b - a) * r1 + (c - a) * r2;
                        var ros = UnityToRos(q);
                        if (ros.x < x0 || ros.x > x1 || ros.y < y0 || ros.y > y1) continue;
                        outPts.Add(ros);
                    }
                }
            }
            var path = GateCloudPath;
            Directory.CreateDirectory(Path.GetDirectoryName(path));
            using (var bw = new BinaryWriter(File.Create(path)))
                foreach (var q in outPts) { bw.Write(q.x); bw.Write(q.y); bw.Write(q.z); }
            L($"Nizamiye nokta bulutu: {outPts.Count} nokta ({nObj} nesne, {dens} nokta/m², ROS kutu x {x0}..{x1}, y {y0}..{y1}) -> {path}");
        }
    }
}
