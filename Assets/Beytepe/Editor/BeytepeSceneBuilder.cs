// Beytepe "Yeni yol" v2 sahnesini AWSIM demo sahnesinden otomatik kurar ve derler.
//
// Menü:  AWSIM > Beytepe > 1. Sahneyi Kur  /  2. Oynatıcıyı Derle
// Komut satırı (batchmode):
//   Unity -batchmode -quit -projectPath <AWSIM> -executeMethod Beytepe.EditorTools.BeytepeSceneBuilder.BuildAll -logFile <log>
//
// Yaptıkları:
//  1) AutowareSimulationDemo.unity -> Assets/Beytepe/BeytepeSimulation.unity (kopya; orijinale dokunmaz)
//  2) Shinjuku modellerini, Shinjuku trafik şeritlerini, yayaları, V2I'yi kaldırır
//  3) Beytepe.fbx'i ekler, zemin/yapı meshlerine MeshCollider koyar
//  4) Modelin doğru dönüşünü, v2 lanelet düğümlerine (local_x, local_y, ele) yukarıdan ışın atarak bulur
//     (harita ve FBX aynı çerçevede: ROS (x,y,z) -> Unity (-y, z, x), ofset 0)
//  5) MGRS ofsetini 0 yapar (Autoware "Local" projektörü: harita çerçevesi = local_x/local_y)
//  6) Ego aracını başlangıç şeridine koyar
//  7) Rapor yazar: ~/awsim/beytepe_build_report.txt
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

namespace Beytepe.EditorTools
{
    public static class BeytepeSceneBuilder
    {
        const string SrcScene = "Assets/Awsim/Scenes/AutowareSimulationDemo.unity";
        const string DstScene = "Assets/Beytepe/BeytepeSimulation.unity";
        const string FbxPath = "Assets/Beytepe/Model/Beytepe.fbx";

        static string Home => Environment.GetEnvironmentVariable("HOME") ?? "/tmp";
        static string OsmPath => Environment.GetEnvironmentVariable("BEYTEPE_OSM")
                                 ?? Path.Combine(Home, "autoware_map/beytepe_yeni_v2/lanelet2_map.osm");
        static string BuildPath => Environment.GetEnvironmentVariable("BEYTEPE_BUILD")
                                   ?? Path.Combine(Home, "awsim/Beytepe-Build/AWSIM-Beytepe.x86_64");
        static string ReportPath => Path.Combine(Home, "awsim/beytepe_build_report.txt");

        // Dünya seçimi (2026-11-09): "roadrunner" (varsayılan, Beytepe.fbx) ya da "gercek"
        // (gerçek araç kaydından üretilen zemin/nesne karoları, Assets/Beytepe/Model/Gercek).
        static string World => (Environment.GetEnvironmentVariable("BEYTEPE_WORLD") ?? "roadrunner").Trim().ToLowerInvariant();
        const string RealDir = "Assets/Beytepe/Model/Gercek";
        // Hizalama denemelerinde kullanılan temel ölçek (OBJ cm olarak gelirse 100'e çıkarılır).
        static float RootScale = 1f;

        // Ego başlangıcı (ROS harita çerçevesi): doğu yolu, lanelet 2000024 (planning testlerinde kullanılan)
        static readonly Vector3 EgoRosPos = new Vector3(181.45f, -192.69f, 0f);
        const float EgoRosYawDeg = 83.7f;

        static readonly string[] NoColliderKeywords = { "Foliage", "Leaves", "Leaf", "Bush", "Tree", "Eucalyptus", "Grass", "Trunk" };

        static readonly StringBuilder Log = new StringBuilder();
        static void L(string s) { Debug.Log("[Beytepe] " + s); Log.AppendLine(s); }

        // FBX'teki RoadRunner malzemelerinin temel rengi beyaz; asıl dokular FBX'e gömülmemiş (dışa aktaran
        // bilgisayarın geçici klasöründe kalmış). Bu yüzden her malzemeye adına göre CC0 PBR doku seti
        // (ambientCG; tools/make_beytepe_textures.py ile Assets/Beytepe/Textures altına üretilir) atanır.
        // Modelin yol/zemin UV'leri 1 birim = 1 m olduğundan dokular gerçek boyutlarıyla döşenir (tile = metre).
        // Doku seti bulunamazsa düz renge düşülür.
        const string MatDir = "Assets/Beytepe/Materials";
        const string TexDir = "Assets/Beytepe/Textures";
        static readonly (string key, Color color, float smooth, string tex, float tileM, Color tint, bool cutout)[] MatTable =
        {
            ("LaneMarkingYellow", new Color(0.85f, 0.68f, 0.08f), 0.35f, null,            0f,   Color.white, false),
            ("LaneMarking",       new Color(0.85f, 0.85f, 0.85f), 0.35f, null,            0f,   Color.white, false),
            ("Asphalt",           new Color(0.16f, 0.16f, 0.17f), 0.25f, "Asphalt",       2.5f, Color.white, false),
            ("Grass",             new Color(0.21f, 0.33f, 0.11f), 0.10f, "Grass",         1.4f, Color.white, false),
            ("Sidewalk",          new Color(0.55f, 0.54f, 0.51f), 0.15f, "ConcreteLight", 1.1f, Color.white, false),
            ("Curb",              new Color(0.62f, 0.61f, 0.58f), 0.15f, "ConcreteLight", 1.1f, new Color(1.1f, 1.1f, 1.1f), false),
            ("Gutter",            new Color(0.42f, 0.42f, 0.40f), 0.15f, "ConcreteRoad",  2.4f, new Color(0.8f, 0.8f, 0.8f), false),
            ("Bridge",            new Color(0.50f, 0.49f, 0.47f), 0.15f, "ConcreteRoad",  2.4f, Color.white, false),
            ("Concrete1_Road",    new Color(0.40f, 0.40f, 0.38f), 0.20f, "ConcreteRoad",  2.4f, Color.white, false),
            ("Leaves",            new Color(0.16f, 0.28f, 0.10f), 0.10f, "Leaves",        0f,   Color.white, true),
            ("Trunk",             new Color(0.30f, 0.22f, 0.15f), 0.10f, "Bark",          0f,   Color.white, false),
        };

        static Texture2D LoadTex(string path, bool normal, bool linear, bool alpha)
        {
            var imp = AssetImporter.GetAtPath(path) as TextureImporter;
            if (imp == null) return null;
            bool changed = false;
            var wantType = normal ? TextureImporterType.NormalMap : TextureImporterType.Default;
            if (imp.textureType != wantType) { imp.textureType = wantType; changed = true; }
            if (!normal && imp.sRGBTexture == linear) { imp.sRGBTexture = !linear; changed = true; }
            if (alpha && !imp.alphaIsTransparency) { imp.alphaIsTransparency = true; changed = true; }
            if (imp.maxTextureSize != 2048) { imp.maxTextureSize = 2048; changed = true; }
            if (changed) imp.SaveAndReimport();
            return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
        }

        static void ResetHdrpKeywords(Material m)
        {
            // HDRP'nin kendi anahtar kelime/geçiş ayarlayıcısı (editor API); bulunamazsa elle ayarlanır.
            var t = Type.GetType("UnityEditor.Rendering.HighDefinition.HDShaderUtils, Unity.RenderPipelines.HighDefinition.Editor");
            var mi = t?.GetMethod("ResetMaterialKeywords", new[] { typeof(Material) });
            if (mi != null) { mi.Invoke(null, new object[] { m }); return; }
            void K(string k, bool on) { if (on) m.EnableKeyword(k); else m.DisableKeyword(k); }
            K("_NORMALMAP", m.GetTexture("_NormalMap") != null);
            K("_MASKMAP", m.GetTexture("_MaskMap") != null);
            K("_ALPHATEST_ON", m.GetFloat("_AlphaCutoffEnable") > 0.5f);
            K("_DOUBLESIDED_ON", m.GetFloat("_DoubleSidedEnable") > 0.5f);
            if (m.GetFloat("_AlphaCutoffEnable") > 0.5f) m.renderQueue = (int)UnityEngine.Rendering.RenderQueue.AlphaTest;
        }

        // --- Gerçek veriden dünya (2026-11-09, ~/harita_gercek/awsim_dunya'dan kopyalanır):
        //   Zemin/zemin_tx_ty.obj + .png : 50 m karo, 0.5 m ızgara, ZED X ortofotosu (5 cm) doku olarak
        //   Nesne/nesne_tx_ty_yapi.obj   : lidar nesneleri, yerden 2.5 m'ye kadar (MeshCollider var)
        //   Nesne/nesne_tx_ty_yaprak.obj : lidar nesneleri, 2.5 m üstü (ağaç tepesi; collider yok)
        // OBJ köşeleri ROS (x,y,z) -> OBJ (y,z,x) yazıldı; Unity X'i aynalayınca RosToUnity ile aynı olur.
        // Yön/ayna yine de 4. adımdaki ışın testiyle doğrulanır.
        static GameObject AddRealWorld(Transform parent)
        {
            var shader = Shader.Find("HDRP/Lit") ?? throw new Exception("HDRP/Lit bulunamadı");
            string matDir = RealDir + "/Malzeme";
            if (!AssetDatabase.IsValidFolder(matDir)) AssetDatabase.CreateFolder(RealDir, "Malzeme");
            Material Make(string name, Color c, Texture2D tex, float smooth, bool twoSided)
            {
                var path = $"{matDir}/{name}.mat";
                var m = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (m == null) { m = new Material(shader); AssetDatabase.CreateAsset(m, path); }
                m.shader = shader;
                m.SetTexture("_BaseColorMap", tex);
                m.SetTexture("_NormalMap", null);
                m.SetTexture("_MaskMap", null);
                m.SetColor("_BaseColor", tex != null ? Color.white : c);
                m.SetFloat("_Metallic", 0f);
                m.SetFloat("_Smoothness", smooth);
                m.SetFloat("_AlphaCutoffEnable", 0f);
                m.SetFloat("_DoubleSidedEnable", twoSided ? 1f : 0f);
                m.SetTextureScale("_BaseColorMap", Vector2.one);
                m.SetTextureOffset("_BaseColorMap", Vector2.zero);
                ResetHdrpKeywords(m);
                EditorUtility.SetDirty(m);
                return m;
            }
            var yapi = Make("gercek_yapi", new Color(0.55f, 0.53f, 0.50f), null, 0.15f, true);
            var yaprak = Make("gercek_yaprak", new Color(0.25f, 0.38f, 0.18f), null, 0.10f, true);
            // 3. sürüm (2026-11-09): kasis tümsekleri ve tabelalar dosya adına göre renk alır
            var kasisSari = Make("gercek_kasis_sari", new Color(0.85f, 0.68f, 0.08f), null, 0.35f, false);
            var kasisBeyaz = Make("gercek_kasis_beyaz", new Color(0.85f, 0.85f, 0.85f), null, 0.35f, false);
            var tabelaPlaka = Make("gercek_tabela_plaka", new Color(0.90f, 0.90f, 0.90f), null, 0.50f, true);
            var tabelaDirek = Make("gercek_tabela_direk", new Color(0.50f, 0.50f, 0.52f), null, 0.60f, false);

            var root = new GameObject("BeytepeModel");
            root.transform.SetParent(parent, false);
            var paths = AssetDatabase.FindAssets("t:Model", new[] { RealDir })
                .Select(g => AssetDatabase.GUIDToAssetPath(g))
                .Where(p => p.EndsWith(".obj", StringComparison.OrdinalIgnoreCase))
                .OrderBy(p => p, StringComparer.Ordinal).ToList();
            if (paths.Count == 0) throw new Exception("Gerçek dünya modeli bulunamadı: " + RealDir);
            int nZemin = 0, nYapi = 0, nYaprak = 0, nCol = 0, nTex = 0;
            long nTri = 0;
            foreach (var path in paths)
            {
                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);
                if (prefab == null) { L("  uyarı: model yüklenemedi " + path); continue; }
                var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab, root.transform);
                var file = Path.GetFileNameWithoutExtension(path);
                bool isZemin = file.StartsWith("zemin_", StringComparison.Ordinal);
                bool isYaprak = file.EndsWith("_yaprak", StringComparison.Ordinal);
                Material mat;
                if (isZemin)
                {
                    var tex = AssetDatabase.LoadAssetAtPath<Texture2D>(Path.ChangeExtension(path, ".jpg"))
                              ?? AssetDatabase.LoadAssetAtPath<Texture2D>(Path.ChangeExtension(path, ".png"));
                    if (tex != null) nTex++;
                    mat = Make(file, new Color(0.36f, 0.42f, 0.34f), tex, 0.2f, false);
                    nZemin++;
                }
                else if (isYaprak) { mat = yaprak; nYaprak++; }
                else if (file.StartsWith("kasis_sari", StringComparison.Ordinal)) { mat = kasisSari; nYapi++; }
                else if (file.StartsWith("kasis_beyaz", StringComparison.Ordinal)) { mat = kasisBeyaz; nYapi++; }
                else if (file.StartsWith("tabela_plaka", StringComparison.Ordinal)) { mat = tabelaPlaka; nYapi++; }
                else if (file.StartsWith("tabela_direk", StringComparison.Ordinal)) { mat = tabelaDirek; nYapi++; }
                else { mat = yapi; nYapi++; }
                foreach (var mr in go.GetComponentsInChildren<MeshRenderer>(true))
                {
                    var mats = mr.sharedMaterials;
                    if (mats.Length == 0) mats = new Material[1];
                    for (int i = 0; i < mats.Length; i++) mats[i] = mat;
                    mr.sharedMaterials = mats;
                }
                foreach (var mf in go.GetComponentsInChildren<MeshFilter>(true))
                {
                    var mesh = mf.sharedMesh;
                    if (mesh == null) continue;
                    for (int s = 0; s < mesh.subMeshCount; s++) nTri += mesh.GetIndexCount(s) / 3;
                    if (isYaprak) continue;   // ağaç tepeleri: fizik yok, lidar/kamera görür
                    var mc = mf.gameObject.AddComponent<MeshCollider>();
                    mc.sharedMesh = mesh; nCol++;
                }
            }
            AssetDatabase.SaveAssets();

            // Ölçek denetimi: beklenen boyut yaklaşık 600 x 3200 m. OBJ santimetre sayıldıysa 100 kat büyüt.
            RootScale = 1f;
            var rs = root.GetComponentsInChildren<Renderer>(true);
            if (rs.Length > 0)
            {
                var b = rs[0].bounds;
                foreach (var r in rs) b.Encapsulate(r.bounds);
                L($"Gerçek dünya sınırları (ölçek 1): merkez {b.center}, boyut {b.size}");
                if (b.size.magnitude < 100f) { RootScale = 100f; L("  OBJ santimetre olarak gelmiş; temel ölçek 100 yapıldı."); }
            }
            L($"Gerçek dünya eklendi: {nZemin} zemin karosu ({nTex} dokulu), {nYapi} yapı, {nYaprak} yaprak karosu; " +
              $"{nTri / 1e6:F1} M üçgen; {nCol} MeshCollider ({RealDir})");
            return root;
        }

        // --- Gerçek dünyaya ağaç ve çalı (2026-11-09): Beytepe.fbx içindeki Eucalyptus_Sm01 (ağaç) ve Bush_Sm01 (çalı)
        //     şablonları, lidardan ölçülen yer / boy / taç yarıçapıyla çoğaltılır (Assets/Beytepe/Model/Gercek/agaclar.csv,
        //     satır: tur,x,y,z_zemin,yukseklik,tac_yaricap; ROS harita çerçevesi). Hizalamadan SONRA ayrı bir köke
        //     mutlak konumla eklenir, böylece zemin modelinin hizalama dönüşünden etkilenmez. Collider yok (eski dünyadaki gibi).
        static void AddRealTrees(Transform parent)
        {
            var csv = Path.Combine(Application.dataPath, "Beytepe/Model/Gercek/agaclar.csv");
            if (!File.Exists(csv)) { L("UYARI: ağaç listesi yok: " + csv); return; }
            AssetDatabase.ImportAsset(FbxPath, ImportAssetOptions.ForceSynchronousImport);
            var fbx = AssetDatabase.LoadAssetAtPath<GameObject>(FbxPath) ?? throw new Exception("FBX yüklenemedi: " + FbxPath);
            // FBX kendi içe aktarma dönüşüyle (270,0,0) örneklenir; eski dünyada da bu dönüşle ağaçlar dik duruyordu.
            var tmp = (GameObject)PrefabUtility.InstantiatePrefab(fbx);
            Transform Pick(string key, int minR, int maxR)
            {
                foreach (var t in tmp.GetComponentsInChildren<Transform>(true))
                {
                    if (t.name.IndexOf(key, StringComparison.OrdinalIgnoreCase) < 0) continue;
                    if (t.name.IndexOf("Leaves", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                    if (t.name.IndexOf("Trunk", StringComparison.OrdinalIgnoreCase) >= 0) continue;
                    int n = t.GetComponentsInChildren<MeshRenderer>(true).Length;
                    if (n >= minR && n <= maxR) return t;
                }
                return null;
            }
            (Quaternion rot, Vector3 scale, Vector3 baseOff, float height, float width) Measure(Transform t)
            {
                var rs = t.GetComponentsInChildren<Renderer>(true);
                var b = rs[0].bounds;
                foreach (var r in rs) b.Encapsulate(r.bounds);
                var trunk = rs.FirstOrDefault(r => r.name.IndexOf("Trunk", StringComparison.OrdinalIgnoreCase) >= 0);
                var c = trunk != null ? trunk.bounds.center : b.center;
                var basePt = new Vector3(c.x, b.min.y, c.z);   // gövde dibi
                return (t.rotation, t.lossyScale, basePt - t.position, b.size.y, Mathf.Max(b.size.x, b.size.z));
            }
            var tTree = Pick("Eucalyptus", 2, 3);
            var tBush = Pick("Bush", 1, 2);
            L($"Ağaç şablonu: {(tTree != null ? tTree.name : "YOK")}; çalı şablonu: {(tBush != null ? tBush.name : "YOK")}");
            if (tTree == null && tBush == null) { UnityEngine.Object.DestroyImmediate(tmp); return; }
            var mTree = tTree != null ? Measure(tTree) : default;
            var mBush = tBush != null ? Measure(tBush) : default;

            var root = new GameObject("BeytepeAgaclar");
            root.transform.SetParent(parent, false);
            var rng = new System.Random(12345);
            int nT = 0, nB = 0;
            foreach (var line in File.ReadAllLines(csv))
            {
                if (line.StartsWith("#") || line.Trim().Length == 0) continue;
                var f = line.Split(',');
                if (f.Length < 6) continue;
                bool isTree = f[0] == "agac";
                var tpl = isTree ? tTree : tBush;
                if (tpl == null) continue;
                var m = isTree ? mTree : mBush;
                var vals = new float[5];
                bool okRow = true;
                for (int q = 0; q < 5; q++)
                    if (!float.TryParse(f[q + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out vals[q]) || float.IsNaN(vals[q]))
                        okRow = false;
                if (!okRow) continue;   // ör. yükseklik modelinin dışında kalan (nan) satır
                float x = vals[0], y = vals[1], z = vals[2], hgt = vals[3], rad = vals[4];
                var holder = new GameObject(isTree ? "agac" : "cali");
                holder.transform.SetParent(root.transform, false);
                holder.transform.position = RosToUnity(new Vector3(x, y, z));
                var copy = UnityEngine.Object.Instantiate(tpl.gameObject, holder.transform);
                var yaw = Quaternion.Euler(0f, (float)(rng.NextDouble() * 360.0), 0f);
                copy.transform.rotation = yaw * m.rot;
                copy.transform.localScale = m.scale;
                copy.transform.localPosition = -(yaw * m.baseOff);   // gövde dibi holder'ın orijininde
                float s = Mathf.Clamp(hgt / Mathf.Max(m.height, 0.1f), 0.2f, 6f);
                float sh = Mathf.Clamp(2f * rad / Mathf.Max(m.width, 0.1f), 0.6f * s, 1.6f * s);
                holder.transform.localScale = new Vector3(sh, s, sh);
                if (isTree) nT++; else nB++;
            }
            UnityEngine.Object.DestroyImmediate(tmp);
            ApplyFallbackMaterials(root);
            L($"Ağaç {nT}, çalı {nB} eklendi (şablon boyu: ağaç {mTree.height:F1} m, çalı {mBush.height:F1} m)");
        }

        // --- Kasisler (2026-11-10): her kasis ayrı, düzenlenebilir bir nesne (Beytepe.Runtime.BeytepeKasis: genişlik, uzunluk,
        //     yükseklik, blok; sarı-beyaz, yol yönünde yuvarlak, MeshCollider). Hepsi Assets/Beytepe/Prefabs/Kasisler.prefab içinde.
        //     Prefab varsa (elle düzenlenmiş olabilir) aynen kullanılır; yoksa lanelet haritasındaki gerçek kasis alanlarından
        //     (type=speed_bump, height>=0.06) üretilip kaydedilir. Baştan üretmek: menü "AWSIM/Beytepe/Kasisleri haritadan yeniden üret".
        const string BumpPrefabPath = "Assets/Beytepe/Prefabs/Kasisler.prefab";

        static void AddBumps(Transform parent)
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(BumpPrefabPath);
            if (prefab == null)
            {
                prefab = CreateBumpPrefabFromMap();
                if (prefab == null) { L("UYARI: kasis prefab'ı üretilemedi"); return; }
                L("Kasisler: prefab haritadan üretildi -> " + BumpPrefabPath);
            }
            else L("Kasisler: mevcut (düzenlenmiş olabilir) prefab kullanıldı -> " + BumpPrefabPath);
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent);
            // Environments/Beytepe Y ekseninde 90° dönük (demo sahnesi): kasis konumları dünya koordinatında, kök dünyaya göre dönüşsüz olmalı
            inst.transform.position = Vector3.zero; inst.transform.rotation = Quaternion.identity; inst.transform.localScale = Vector3.one;
            Physics.SyncTransforms();
            var ks = inst.GetComponentsInChildren<Beytepe.Runtime.BeytepeKasis>(true);
            foreach (var k in ks) k.Olustur();
            L($"Kasisler: {ks.Length} nesne sahneye eklendi (" + string.Join(", ", ks.Select(k =>
                $"{k.name} {k.genislik:F1}x{k.uzunluk:F1} m h{k.yukseklik:F2}")) + ")");
            var road = parent.Find("BeytepeModel");
            if (road != null) FixOldBumpPatches(road.gameObject, ks);
        }

        // RoadRunner modelinde her kasis yerinde açık gri bir beton yama (malzeme Concrete1_Road) ve yolun bütün enine kesitinde
        // (yol, çizgi, oluk, bordür, kaldırım, kenardaki çim) 3-9 cm'lik bir tümsek var; haritadaki kasis alanından 0-4 m kaymış.
        // Yeni kasisle birlikte araç iki kez sarsılıyordu ve yama ikinci bir şerit gibi görünüyordu. Bu işlev, her kasisin çevresindeki
        // (kasis çerçevesinde enine genişlik/2+1 m, boyuna uzunluk/2+3 m) eski yamayı bulur ve:
        //  1) Yüzeyi düzler: yamanın iki ucunun 0.3 m dışındaki yol yüksekliğini birleştiren doğruya indirir (profil ±1.5 m'de ışınla
        //     ölçülür). Yol çevresi (yama yarı genişliği + 3.5 m) tam, ötesindeki çim 5.5 m'de sönerek düzlenir; 2 m'den farklı yükseklikteki
        //     köşeler (köprü, başka yol) atlanır. 1.5 cm'den alçak tümsek atlanır. Normal vektörleri yeni eğime göre döndürülür.
        //  2) Beton üçgenleri asfalt alt-meshine taşır (UV'ler dünya x/y'si olduğundan asfalt dokusu dikişsiz).
        // Değişen meshler sahneye gömülü kopyadır (FBX'e dokunulmaz); aynı nesnedeki MeshCollider da kopyayı kullanır (fizik, lidar).
        // Sonra kasisler yeni yüzeye yeniden oturtulur (yalnız yükseklik/eğim; yatay konum ve yön aynı).
        // Kasis eski yamasından çok uzağa taşınırsa yama bulunmaz ve o yama/tümsek eski haliyle kalır.
        // (-180,155) çevresindeki diğer beton yol alanı kasise yakın olmadığından değişmez.
        class EskiYama { public Beytepe.Runtime.BeytepeKasis k; public float a, b, qc, h; public float[] dz, ys; public Bounds wb; public int nTri; }

        static void FixOldBumpPatches(GameObject model, Beytepe.Runtime.BeytepeKasis[] ks)
        {
            if (ks.Length == 0) return;
            var cols = ks.Select(k => k.GetComponent<MeshCollider>()).ToArray();
            foreach (var c in cols) if (c != null) c.enabled = false;   // profil ışınları yeni kasislere çarpmasın
            Physics.SyncTransforms();

            // --- yol meshi ve beton alt-meshi
            MeshFilter roadMf = null; int ci = -1, ai = -1;
            foreach (var mf in model.GetComponentsInChildren<MeshFilter>(true))
            {
                var mr = mf.GetComponent<MeshRenderer>(); var src = mf.sharedMesh;
                if (mr == null || src == null || !src.isReadable) continue;
                var mats = mr.sharedMaterials;
                int c1 = Array.FindIndex(mats, m => m != null && m.name.IndexOf("Concrete1_Road", StringComparison.OrdinalIgnoreCase) >= 0);
                int a1 = Array.FindIndex(mats, m => m != null && m.name.IndexOf("Asphalt", StringComparison.OrdinalIgnoreCase) >= 0);
                if (c1 < 0 || a1 < 0 || c1 >= src.subMeshCount || a1 >= src.subMeshCount) continue;
                roadMf = mf; ci = c1; ai = a1; break;
            }
            if (roadMf == null) { L("UYARI: eski kasis yamaları: yol meshi (Concrete1_Road + Asphalt) bulunamadı"); foreach (var c in cols) if (c != null) c.enabled = true; return; }

            // --- her kasis için eski beton yamayı bul (kasis yerel çerçevesinde: x enine, z boyuna)
            var rVerts = roadMf.sharedMesh.vertices;
            var rL2w = roadMf.transform.localToWorldMatrix;
            var conc = roadMf.sharedMesh.GetTriangles(ci);
            var triOwner = new int[conc.Length / 3];
            var zones = new List<EskiYama>();
            var zoneOf = new EskiYama[ks.Length];
            var smin = Enumerable.Repeat(float.MaxValue, ks.Length).ToArray(); var smax = Enumerable.Repeat(float.MinValue, ks.Length).ToArray();
            var qmax = new float[ks.Length]; var ntri = new int[ks.Length];
            for (int t = 0; t < triOwner.Length; t++)
            {
                triOwner[t] = -1;
                var w0 = rL2w.MultiplyPoint3x4(rVerts[conc[3 * t]]); var w1 = rL2w.MultiplyPoint3x4(rVerts[conc[3 * t + 1]]); var w2 = rL2w.MultiplyPoint3x4(rVerts[conc[3 * t + 2]]);
                var cw = (w0 + w1 + w2) / 3f;
                for (int b = 0; b < ks.Length; b++)
                {
                    var lp = ks[b].transform.InverseTransformPoint(cw);
                    if (Mathf.Abs(lp.x) >= ks[b].genislik / 2f + 1f || Mathf.Abs(lp.z) >= ks[b].uzunluk / 2f + 3f || Mathf.Abs(lp.y) >= 1f) continue;
                    triOwner[t] = b; ntri[b]++;
                    foreach (var w in new[] { w0, w1, w2 })
                    {
                        var l = ks[b].transform.InverseTransformPoint(w);
                        smin[b] = Mathf.Min(smin[b], l.z); smax[b] = Mathf.Max(smax[b], l.z); qmax[b] = Mathf.Max(qmax[b], Mathf.Abs(l.x));
                    }
                    break;
                }
            }

            // --- tümsek profili: yamanın iki ucunun 0.3 m dışını birleştiren doğruya göre yükseklik farkı
            bool Down(Vector3 p, out float y)
            {
                if (Physics.Raycast(p + Vector3.up * 3f, Vector3.down, out var hit, 8f)) { y = hit.point.y; return true; }
                y = 0f; return false;
            }
            var notes = new List<string>();
            for (int b = 0; b < ks.Length; b++)
            {
                var nm = ks[b].name.Replace("Kasis_", "");
                if (ntri[b] == 0) { notes.Add($"{nm}: yama yok"); continue; }
                var z = new EskiYama { k = ks[b], a = smin[b] - 0.3f, b = smax[b] + 0.3f, qc = qmax[b], h = 0.05f, nTri = ntri[b] };
                int n = Mathf.CeilToInt((z.b - z.a) / z.h) + 1; z.h = (z.b - z.a) / (n - 1);
                var qs = z.qc > 2f ? new[] { -1.5f, 1.5f } : new[] { 0f };
                var prof = new float[n]; bool ok = true;
                for (int i = 0; i < n && ok; i++)
                {
                    float sum = 0f;
                    foreach (var q in qs) { if (!Down(ks[b].transform.TransformPoint(new Vector3(q, 0f, z.a + i * z.h)), out var y)) { ok = false; break; } sum += y; }
                    prof[i] = sum / qs.Length;
                }
                if (!ok) { notes.Add($"{nm}: yüzey ölçülemedi"); zoneOf[b] = null; continue; }
                z.dz = new float[n]; z.ys = prof;
                for (int i = 0; i < n; i++) z.dz[i] = prof[i] - Mathf.Lerp(prof[0], prof[n - 1], i / (float)(n - 1));
                float top = z.dz.Max();
                zoneOf[b] = z;   // beton üçgenleri her durumda asfalt olur
                if (top < 0.015f) { notes.Add($"{nm}: tümsek {top * 100f:F1} cm (düzlenmedi)"); z.dz = null; continue; }
                var wb = new Bounds(ks[b].transform.TransformPoint(new Vector3(0, 0, z.a)), Vector3.zero);
                foreach (var sx in new[] { -1f, 1f }) foreach (var sz in new[] { z.a, z.b }) foreach (var sy in new[] { -3f, 3f })
                    wb.Encapsulate(ks[b].transform.TransformPoint(new Vector3(sx * (z.qc + 9f), sy, sz)));
                z.wb = wb; zones.Add(z);
                notes.Add($"{nm}: tümsek {top * 100f:F1} cm düzlendi (s {z.a:F1}..{z.b:F1} m)");
            }

            float Interp(float[] arr, EskiYama z, float s)
            {
                float u = Mathf.Clamp((s - z.a) / z.h, 0f, arr.Length - 1.001f); int i = (int)u; float f = u - i;
                return arr[i] * (1 - f) + arr[i + 1] * f;
            }

            // --- köşe noktalarını düzle (yol meshi + yama içindeki diğer katmanlar)
            int nMesh = 0, nVert = 0;
            foreach (var mf in model.GetComponentsInChildren<MeshFilter>(true))
            {
                var src = mf.sharedMesh; var mr = mf.GetComponent<MeshRenderer>();
                if (src == null || mr == null || !src.isReadable) continue;
                if (NoColliderKeywords.Any(k => mf.name.IndexOf(k, StringComparison.OrdinalIgnoreCase) >= 0 && mf.name.IndexOf("Ground", StringComparison.OrdinalIgnoreCase) < 0)) continue;
                bool isRoad = mf == roadMf;
                var hitZones = zones.Where(z => z.wb.Intersects(mr.bounds)).ToList();
                if (hitZones.Count == 0 && !isRoad) continue;
                var l2w = mf.transform.localToWorldMatrix; var w2l = mf.transform.worldToLocalMatrix;
                var verts = src.vertices; var norms = src.normals; bool hasN = norms != null && norms.Length == verts.Length;
                int changed = 0;
                for (int v = 0; v < verts.Length && hitZones.Count > 0; v++)
                {
                    var w = l2w.MultiplyPoint3x4(verts[v]);
                    foreach (var z in hitZones)
                    {
                        if (!z.wb.Contains(w)) continue;
                        var lp = z.k.transform.InverseTransformPoint(w);
                        if (lp.z <= z.a || lp.z >= z.b || Mathf.Abs(lp.x) >= z.qc + 9f) continue;
                        float d = Interp(z.dz, z, lp.z);
                        if (Mathf.Abs(w.y - Interp(z.ys, z, lp.z)) > 2f) continue;   // köprü / başka yol
                        float ax = Mathf.Abs(lp.x), wq = Mathf.Clamp01(1f - (ax - z.qc - 3.5f) / 5.5f);
                        float dd = d * wq;
                        if (Mathf.Abs(dd) < 1e-4f) continue;
                        w.y -= dd;
                        if (hasN)
                        {
                            // yükseklik alanı normali: n ∝ (-∂y/∂x, 1, -∂y/∂z); düzeltmenin eğimi çıkarılır
                            float i0 = Interp(z.dz, z, lp.z - 0.05f), i1 = Interp(z.dz, z, lp.z + 0.05f);
                            float dds = (i1 - i0) / 0.1f * wq;
                            float ddq = (ax > z.qc + 3.5f && wq > 0f && wq < 1f) ? -d / 5.5f * Mathf.Sign(lp.x) : 0f;
                            var fwH = z.k.transform.forward; fwH.y = 0f; fwH.Normalize();
                            var rtH = z.k.transform.right; rtH.y = 0f; rtH.Normalize();
                            var grad = fwH * dds + rtH * ddq;
                            var nw = l2w.MultiplyVector(norms[v]).normalized;
                            if (nw.y > 0.3f)
                            {
                                float gx = -nw.x / nw.y - grad.x, gz = -nw.z / nw.y - grad.z;
                                norms[v] = w2l.MultiplyVector(new Vector3(-gx, 1f, -gz).normalized).normalized;
                            }
                        }
                        verts[v] = w2l.MultiplyPoint3x4(w);
                        changed++;
                        break;
                    }
                }
                bool moveTris = isRoad && triOwner.Any(o => o >= 0 && zoneOf[o] != null);
                if (changed == 0 && !moveTris) continue;
                var mesh = UnityEngine.Object.Instantiate(src);
                mesh.name = src.name + "_eski_kasissiz";
                if (changed > 0) { mesh.vertices = verts; if (hasN) mesh.normals = norms; }
                if (moveTris)
                {
                    var keep = new List<int>(conc.Length); var asphalt = new List<int>(mesh.GetTriangles(ai));
                    for (int t = 0; t < triOwner.Length; t++)
                    {
                        var dst = (triOwner[t] >= 0 && zoneOf[triOwner[t]] != null) ? asphalt : keep;
                        dst.Add(conc[3 * t]); dst.Add(conc[3 * t + 1]); dst.Add(conc[3 * t + 2]);
                    }
                    mesh.SetTriangles(asphalt, ai, false);
                    mesh.SetTriangles(keep, ci, false);
                }
                mesh.RecalculateBounds();
                mf.sharedMesh = mesh;   // sahneye gömülü kopya
                PrefabUtility.RecordPrefabInstancePropertyModifications(mf);
                var mc = mf.GetComponent<MeshCollider>();
                if (mc != null) { mc.sharedMesh = null; mc.sharedMesh = mesh; PrefabUtility.RecordPrefabInstancePropertyModifications(mc); }
                nMesh++; nVert += changed;
            }

            // --- kasisleri yeni yüzeye oturt
            foreach (var c in cols) if (c != null) c.enabled = true;
            Physics.SyncTransforms();
            var moves = new List<string>();
            foreach (var k in ks)
            {
                float y0 = k.transform.position.y;
                k.YereOturt();
                float dy = k.transform.position.y - y0;
                if (Mathf.Abs(dy) > 0.005f) moves.Add($"{k.name.Replace("Kasis_", "")} {dy * 100f:+0.0;-0.0} cm");
            }
            Physics.SyncTransforms();
            int nPatch = zoneOf.Count(z => z != null);
            L($"Eski RoadRunner kasisleri: {nPatch} yama asfalt yapıldı ({ntri.Sum()} üçgen), {zones.Count} tümsek düzlendi; {nMesh} mesh, {nVert} köşe değişti");
            L("  " + string.Join("; ", notes));
            L("  kasisler yeni yüzeye oturtuldu" + (moves.Count > 0 ? ": " + string.Join(", ", moves) : " (değişiklik yok)"));
        }

        // --- Binalar (2026-11-10): OpenStreetMap bina sınırlarından basit blok binalar, her biri ayrı düzenlenebilir nesne
        //     (Beytepe.Runtime.BeytepeBina: taban çokgeni + yükseklik). Hepsi Assets/Beytepe/Prefabs/Binalar.prefab içinde.
        //     Veri: Assets/Beytepe/Data/binalar.json (ROS harita x,y; yükseklik lidar / OSM / varsayılan; ~/harita_gercek/osm'den üretilir).
        //     Prefab varsa (elle düzenlenmiş olabilir) korunur; veride olup prefab'da olmayan binalar eklenir ve prefab'a kaydedilir.
        //     Bina tabanı içindeki (1.5 m pay) RoadRunner ağaç/çalıları her derlemede gizlenir.
        const string BuildingPrefabPath = "Assets/Beytepe/Prefabs/Binalar.prefab";
        const string BuildingDataPath = "Assets/Beytepe/Data/binalar.json";
#pragma warning disable 0649   // alanları JsonUtility doldurur
        // İsteğe bağlı cephe alanları (yalnız prefab veriden üretilirken kullanılır): detayli, catiBaca, catiKutu, renk [r,g,b]
        [Serializable] class BinaKaydi { public string id, ad, tur, kaynak; public float h; public float[] taban; public bool detayli; public int catiBaca, catiKutu; public float[] renk;
            public float katY, pencereG, pencereY, pencereA, pencereD, kenar, parapet;   // 0 = bileşenin varsayılanı
            public bool zeminYakin; }   // true: taban altında zemin modeli yoksa en yakın zemin yüksekliği kullanılır (model kenarı dışındaki binalar)
        [Serializable] class BinaListesi { public BinaKaydi[] binalar; }
#pragma warning restore 0649

        // Duvar: açık beton/sıva dokusu (ConcreteRoad, 2.4 m tekrar; bina başına renk BeytepeBina.duvarRengi ile çarpılır).
        // Cam: koyu, parlak. Detay: kat bandı, denizlik, parapet kapağı, çatı ekipmanı.
        static (Material duvar, Material cati, Material cam, Material detay) BuildingMaterials()
        {
            var shader = Shader.Find("HDRP/Lit");
            Material Mat(string name, Color c, float smooth, float metal, string tex, float tileM)
            {
                var path = $"{MatDir}/{name}.mat";
                var m = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (m == null) { m = new Material(shader); AssetDatabase.CreateAsset(m, path); }
                m.shader = shader;
                Texture2D bt = null, nt = null, mt = null;
                if (tex != null)
                {
                    bt = LoadTex($"{TexDir}/{tex}_Base.png", false, false, false);
                    nt = LoadTex($"{TexDir}/{tex}_Normal.png", true, true, false);
                    mt = LoadTex($"{TexDir}/{tex}_Mask.png", false, true, false);
                }
                m.SetTexture("_BaseColorMap", bt); m.SetTexture("_NormalMap", nt); m.SetTexture("_MaskMap", mt);
                m.SetFloat("_NormalScale", 1f);
                var tile = (bt != null && tileM > 0f) ? 1f / tileM : 1f;
                m.SetTextureScale("_BaseColorMap", new Vector2(tile, tile)); m.SetTextureOffset("_BaseColorMap", Vector2.zero);
                m.SetColor("_BaseColor", c);
                m.SetFloat("_Metallic", metal); m.SetFloat("_Smoothness", smooth);
                m.SetFloat("_SmoothnessRemapMin", 0f); m.SetFloat("_SmoothnessRemapMax", mt != null ? 0.6f : 1f);
                m.SetFloat("_AlphaCutoffEnable", 0f); m.SetFloat("_DoubleSidedEnable", 0f);
                ResetHdrpKeywords(m); EditorUtility.SetDirty(m); return m;
            }
            return (Mat("Bina_Duvar", new Color(0.97f, 0.93f, 0.94f), 0.2f, 0f, "ConcreteRoad", 2.4f),
                    Mat("Bina_Cati", new Color(0.42f, 0.42f, 0.44f), 0.2f, 0f, null, 0f),
                    Mat("Bina_Cam", new Color(0.22f, 0.28f, 0.34f), 0.88f, 0.1f, null, 0f),
                    Mat("Bina_Detay", new Color(0.66f, 0.66f, 0.64f), 0.25f, 0f, null, 0f));
        }

        // Tabandaki noktalarda zemin (bina çatıları hariç): en alçak ve ortalama yükseklik
        static bool BuildingGround(IEnumerable<Vector3> worldPts, out float min, out float mean)
        {
            min = float.MaxValue; float sum = 0f; int n = 0;
            foreach (var w in worldPts)
            {
                float best = float.MinValue; bool ok = false;
                foreach (var hit in Physics.RaycastAll(new Vector3(w.x, 400f, w.z), Vector3.down, 800f))
                    if (hit.collider.GetComponent<Beytepe.Runtime.BeytepeBina>() == null && hit.point.y > best) { best = hit.point.y; ok = true; }
                if (!ok) continue;
                min = Mathf.Min(min, best); sum += best; n++;
            }
            mean = n > 0 ? sum / n : 0f;
            return n > 0;
        }

        static readonly List<string> _skipNote = new List<string>();
        static readonly List<string> _nearNote = new List<string>();

        // Zemin modelinin dışında kalan binalar için (2026-11-11): merkezden dışa doğru halkalarda (5 m adım, 24 yön) zemine ışın atar;
        // vuruş olan ilk halkadaki yüksekliklerin ortancası zemin sayılır.
        static bool NearestGround(Vector3 c, float maxR, out float y, out float dist)
        {
            y = 0f; dist = 0f;
            for (float rr = 5f; rr <= maxR; rr += 5f)
            {
                var ys = new List<float>();
                for (int k = 0; k < 24; k++)
                {
                    float a = k * Mathf.PI * 2f / 24f;
                    var w = new Vector3(c.x + rr * Mathf.Cos(a), 400f, c.z + rr * Mathf.Sin(a));
                    float best = float.MinValue; bool ok = false;
                    foreach (var hit in Physics.RaycastAll(w, Vector3.down, 800f))
                        if (hit.collider.GetComponent<Beytepe.Runtime.BeytepeBina>() == null && hit.point.y > best) { best = hit.point.y; ok = true; }
                    if (ok) ys.Add(best);
                }
                if (ys.Count >= 2) { ys.Sort(); y = ys[ys.Count / 2]; dist = rr; return true; }
            }
            return false;
        }

        // Taban içindeki ızgara noktalarında en üstteki zemin yüzeyinin malzemesi yol (asfalt / yol betonu / çizgi) mi?
        static float RoadFraction(List<Vector3> ptsU)
        {
            var poly = ptsU.Select(p => new Vector2(p.x, p.z)).ToList();
            float mnx = poly.Min(q => q.x), mxx = poly.Max(q => q.x), mnz = poly.Min(q => q.y), mxz = poly.Max(q => q.y);
            float step = Mathf.Max(1.5f, Mathf.Sqrt((mxx - mnx) * (mxz - mnz) / 150f));
            int n = 0, road = 0;
            bool Inside(Vector2 pt)
            {
                bool ins = false;
                for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
                {
                    var a = poly[i]; var b = poly[j];
                    if ((a.y > pt.y) != (b.y > pt.y) && pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) ins = !ins;
                }
                return ins;
            }
            for (float x = mnx + step / 2f; x < mxx; x += step)
                for (float z = mnz + step / 2f; z < mxz; z += step)
                {
                    if (!Inside(new Vector2(x, z))) continue;
                    RaycastHit top = default; bool ok = false;
                    foreach (var h in Physics.RaycastAll(new Vector3(x, 400f, z), Vector3.down, 800f))
                        if (h.collider.GetComponent<Beytepe.Runtime.BeytepeBina>() == null && h.collider.GetComponent<Beytepe.Runtime.BeytepeKasis>() == null && (!ok || h.point.y > top.point.y)) { top = h; ok = true; }
                    if (!ok) continue;
                    n++;
                    var mc = top.collider as MeshCollider; var mr = top.collider.GetComponent<MeshRenderer>();
                    if (mc == null || mc.sharedMesh == null || mr == null || top.triangleIndex < 0) continue;
                    var mesh = mc.sharedMesh; int idx = top.triangleIndex * 3;
                    for (int sm = 0; sm < mesh.subMeshCount && sm < mr.sharedMaterials.Length; sm++)
                    {
                        var d = mesh.GetSubMesh(sm);
                        if (idx < d.indexStart || idx >= d.indexStart + d.indexCount) continue;
                        var nm = mr.sharedMaterials[sm] != null ? mr.sharedMaterials[sm].name : "";
                        if (nm.IndexOf("Asphalt", StringComparison.OrdinalIgnoreCase) >= 0 || nm.IndexOf("_Road", StringComparison.OrdinalIgnoreCase) >= 0
                            || nm.IndexOf("Marking", StringComparison.OrdinalIgnoreCase) >= 0) road++;
                        break;
                    }
                }
            return n > 0 ? (float)road / n : 0f;
        }

        static GameObject MakeBuilding(BinaKaydi r, Transform parent, (Material duvar, Material cati, Material cam, Material detay) mats)
        {
            if (r.taban == null || r.taban.Length < 6) return null;
            var ptsU = new List<Vector3>();
            for (int i = 0; i + 1 < r.taban.Length; i += 2) ptsU.Add(RosToUnity(new Vector3(r.taban[i], r.taban[i + 1], 0f)));
            var c = Vector3.zero; foreach (var p in ptsU) c += p; c /= ptsU.Count;
            var probe = new List<Vector3>(ptsU) { c };
            bool yakin = false;
            if (!BuildingGround(probe, out float gmin, out float gmean))
            {
                if (!r.zeminYakin || !NearestGround(c, 250f, out gmin, out float gd)) { _skipNote.Add($"{r.id} zemin yok"); return null; }
                gmean = gmin; yakin = true;
                _nearNote.Add($"{r.id}{(string.IsNullOrEmpty(r.ad) ? "" : " " + r.ad)} (zemin {gd:F0} m ötede, y {gmin:F1})");
            }
            float onRoad = RoadFraction(ptsU);
            if (onRoad > 0.15f) { _skipNote.Add($"{r.id}{(string.IsNullOrEmpty(r.ad) ? "" : " " + r.ad)} yol üstünde (%{onRoad * 100f:F0})"); return null; }
            var go = new GameObject("Bina_" + r.id + (string.IsNullOrEmpty(r.ad) ? "" : "_" + r.ad.Replace(' ', '_')));
            go.transform.SetParent(parent, false);
            go.transform.position = new Vector3(c.x, gmin, c.z);
            go.transform.rotation = Quaternion.identity;
            var b = go.AddComponent<Beytepe.Runtime.BeytepeBina>();   // MeshFilter/Renderer/Collider RequireComponent ile gelir
            b.taban = ptsU.Select(p => new Vector2(p.x - c.x, p.z - c.z)).ToList();
            b.yukseklik = r.h + (gmean - gmin);   // çatı = ortalama zemin + ölçülen yükseklik
            b.gomme = yakin ? 3f : 0.5f;   // zemin modeli dışında: alt kenar havada görünmesin
            b.duvar = mats.duvar; b.cati = mats.cati; b.cam = mats.cam; b.detay = mats.detay; b.osmKimlik = r.id; b.kaynak = r.kaynak;
            b.detayli = r.detayli; b.catiBaca = r.catiBaca; b.catiKutu = r.catiKutu;
            if (r.renk != null && r.renk.Length >= 3) b.duvarRengi = new Color(r.renk[0], r.renk[1], r.renk[2], 1f);
            if (r.katY > 0f) b.katYuksekligi = r.katY;
            if (r.pencereG > 0f) b.pencereGenislik = r.pencereG;
            if (r.pencereY > 0f) b.pencereYukseklik = r.pencereY;
            if (r.pencereA > 0f) b.pencereAraligi = r.pencereA;
            if (r.pencereD > 0f) b.pencereDerinlik = r.pencereD;
            if (r.kenar > 0f) b.kenarPayi = r.kenar;
            if (r.parapet > 0f) b.parapet = r.parapet;
            b.ZeminiOlc();
            b.Olustur();
            Physics.SyncTransforms();
            return go;
        }

        static void AddBuildings(Transform parent, GameObject model)
        {
            if (!File.Exists(BuildingDataPath)) { L("Binalar: veri yok (" + BuildingDataPath + "), atlandı"); return; }
            var data = JsonUtility.FromJson<BinaListesi>(File.ReadAllText(BuildingDataPath));
            if (data?.binalar == null) { L("UYARI: binalar.json okunamadı"); return; }
            var mats = BuildingMaterials();
            Physics.SyncTransforms();
            if (!AssetDatabase.IsValidFolder("Assets/Beytepe/Prefabs")) AssetDatabase.CreateFolder("Assets/Beytepe", "Prefabs");
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(BuildingPrefabPath);
            int nNew = 0, nSkip = 0; _skipNote.Clear(); _nearNote.Clear();
            if (prefab == null)
            {
                var tmp = new GameObject("Binalar");   // sahne kökünde, dünyaya göre dönüşsüz
                foreach (var r in data.binalar) { if (MakeBuilding(r, tmp.transform, mats) != null) nNew++; else nSkip++; }
                prefab = PrefabUtility.SaveAsPrefabAsset(tmp, BuildingPrefabPath);
                UnityEngine.Object.DestroyImmediate(tmp);
                L($"Binalar: prefab veriden üretildi -> {BuildingPrefabPath}");
            }
            var inst = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent);
            inst.transform.position = Vector3.zero; inst.transform.rotation = Quaternion.identity; inst.transform.localScale = Vector3.one;
            Physics.SyncTransforms();
            var have = new HashSet<string>(inst.GetComponentsInChildren<Beytepe.Runtime.BeytepeBina>(true).Select(b => b.osmKimlik));
            int nAdd = 0;
            foreach (var r in data.binalar)
            {
                if (have.Contains(r.id)) continue;
                if (MakeBuilding(r, inst.transform, mats) != null) nAdd++; else nSkip++;
            }
            if (nAdd > 0) PrefabUtility.ApplyPrefabInstance(inst, InteractionMode.AutomatedAction);
            var bs = inst.GetComponentsInChildren<Beytepe.Runtime.BeytepeBina>(true);
            int nZem = 0;
            foreach (var b in bs)
            {
                b.duvar = mats.duvar; b.cati = mats.cati; b.cam = mats.cam; b.detay = mats.detay;
                if (!b.ZeminiOlc()) nZem++;   // eğimli arazide zemin altında kalan pencereler çizilmez
                b.Olustur();
            }
            Physics.SyncTransforms();
            var dets = bs.Where(b => b.detayli).ToList();
            L($"Binalar: cephe detayı {dets.Count} binada" + (dets.Count > 0 ? " (" + string.Join(", ", dets.Select(b =>
                $"{b.osmKimlik}: {b.GetComponent<MeshFilter>().sharedMesh.vertexCount} köşe, baca {b.catiBaca}, kutu {b.catiKutu}")) + ")" : "")
                + (nZem > 0 ? $"; {nZem} binada zemin her köşede ölçülemedi" : ""));

            // Bina içindeki ağaç/çalıları gizle (dünya x/z'de tabana 1.5 m'den yakın)
            var polys = bs.Select(b => b.taban.Select(q => { var w = b.transform.TransformPoint(new Vector3(q.x, 0f, q.y)); return new Vector2(w.x, w.z); }).ToList()).ToList();
            var boxes = polys.Select(p => { var mn = new Vector2(p.Min(q => q.x), p.Min(q => q.y)) - Vector2.one * 1.5f; var mx = new Vector2(p.Max(q => q.x), p.Max(q => q.y)) + Vector2.one * 1.5f; return (mn, mx); }).ToList();
            bool Near(Vector2 pt, List<Vector2> poly)
            {
                bool inside = false;
                for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
                {
                    var a = poly[i]; var b = poly[j];
                    if ((a.y > pt.y) != (b.y > pt.y) && pt.x < (b.x - a.x) * (pt.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
                    var ab = b - a; float t = ab.sqrMagnitude > 1e-9f ? Mathf.Clamp01(Vector2.Dot(pt - a, ab) / ab.sqrMagnitude) : 0f;
                    if ((a + ab * t - pt).sqrMagnitude < 1.5f * 1.5f) return true;
                }
                return inside;
            }
            int nTree = 0;
            if (model != null)
                foreach (var t in model.GetComponentsInChildren<Transform>(true))
                {
                    var nm = t.name;
                    if (!(nm.StartsWith("Bush_") || nm.StartsWith("Eucalyptus_")) || !nm.EndsWith("Node")) continue;
                    if (t.parent != null && (t.parent.name.StartsWith("Bush_") || t.parent.name.StartsWith("Eucalyptus_"))) continue;
                    if (!t.gameObject.activeSelf) continue;
                    var pt = new Vector2(t.position.x, t.position.z);
                    for (int k = 0; k < polys.Count; k++)
                    {
                        if (pt.x < boxes[k].mn.x || pt.y < boxes[k].mn.y || pt.x > boxes[k].mx.x || pt.y > boxes[k].mx.y) continue;
                        if (Near(pt, polys[k])) { t.gameObject.SetActive(false); nTree++; break; }
                    }
                }
            L($"Binalar: {bs.Length} bina sahnede (yeni {nNew + nAdd}, atlanan {nSkip}); bina içinde kalan {nTree} ağaç/çalı gizlendi");
            if (_skipNote.Count > 0) L("  atlanan binalar: " + string.Join(", ", _skipNote));
            if (_nearNote.Count > 0) L($"  zemin modeli dışında, en yakın zemine oturtulan {_nearNote.Count} bina: " + string.Join(", ", _nearNote));
            L("  " + string.Join(", ", bs.Select(b => $"{b.osmKimlik} {b.yukseklik:F1} m ({b.kaynak})")));
        }

        // --- Gökyüzü (2026-11-10): demo sahnesinin HDRI gökyüzü görüntüsü projede yok (gri gökyüzü). Sahne köküne
        //     Beytepe.Runtime.BeytepeGokyuzu eklenir; oyun başında mavi gradyan gökyüzünü kurar (ayar: JSON "Gokyuzu" bölümü).
        static void AddSky()
        {
            var go = GameObject.Find("BeytepeGokyuzu") ?? new GameObject("BeytepeGokyuzu");
            go.transform.SetParent(null, false);
            if (go.GetComponent<Beytepe.Runtime.BeytepeGokyuzu>() == null) go.AddComponent<Beytepe.Runtime.BeytepeGokyuzu>();
            L("Gökyüzü: BeytepeGokyuzu eklendi (mavi gradyan; JSON 'Gokyuzu' bölümüyle ayarlanır, derleme gerekmez)");
        }

        [MenuItem("AWSIM/Beytepe/Binaları veriden yeniden üret")]
        public static void RegenerateBuildings()
        {
            // Beytepe sahnesi açıkken çalıştırın; elle yapılan bina düzenlemeleri silinir.
            if (AssetDatabase.LoadAssetAtPath<GameObject>(BuildingPrefabPath) != null) AssetDatabase.DeleteAsset(BuildingPrefabPath);
            var old = UnityEngine.Object.FindObjectsByType<Beytepe.Runtime.BeytepeBina>(FindObjectsSortMode.None);
            foreach (var r in old.Select(k => k.transform.parent).Where(t => t != null).Distinct().ToList())
                UnityEngine.Object.DestroyImmediate(r.gameObject);
            var models = FindPath("AutowreSimulationDemo/Environments/Beytepe/Models");
            if (models == null) { Debug.LogError("[Beytepe] Beytepe sahnesi açık değil"); return; }
            var model = models.Find("BeytepeModel");
            AddBuildings(models, model != null ? model.gameObject : null);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        static (Material sari, Material beyaz) BumpMaterials()
        {
            var shader = Shader.Find("HDRP/Lit");
            Material Mat(string name, Color c)
            {
                var path = $"{MatDir}/{name}.mat";
                var m = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (m == null) { m = new Material(shader); AssetDatabase.CreateAsset(m, path); }
                m.shader = shader; m.SetTexture("_BaseColorMap", null); m.SetColor("_BaseColor", c);
                m.SetFloat("_Metallic", 0f); m.SetFloat("_Smoothness", 0.35f); m.SetFloat("_AlphaCutoffEnable", 0f); m.SetFloat("_DoubleSidedEnable", 1f);
                ResetHdrpKeywords(m); EditorUtility.SetDirty(m); return m;
            }
            return (Mat("Kasis_Sari", new Color(0.85f, 0.68f, 0.08f)), Mat("Kasis_Beyaz", new Color(0.85f, 0.85f, 0.85f)));
        }

        // Haritadaki kasis alanlarından prefab üretir. Model (yol) sahnede ve collider'ları hazır olmalı (yüzey ışınla ölçülür).
        static GameObject CreateBumpPrefabFromMap()
        {
            if (!File.Exists(OsmPath)) { L("UYARI: kasis için OSM yok: " + OsmPath); return null; }
            var doc = new System.Xml.XmlDocument(); doc.Load(OsmPath);
            var nodes = new Dictionary<string, Vector3>();
            foreach (System.Xml.XmlNode n in doc.SelectNodes("/osm/node"))
            {
                float x = 0, y = 0, z = 0; bool hx = false, hy = false;
                foreach (System.Xml.XmlNode t in n.SelectNodes("tag"))
                {
                    var k = t.Attributes["k"].Value; var v = t.Attributes["v"].Value;
                    if (k == "local_x") { x = P(v); hx = true; } else if (k == "local_y") { y = P(v); hy = true; } else if (k == "ele") z = P(v);
                }
                if (hx && hy) nodes[n.Attributes["id"].Value] = new Vector3(x, y, z);
            }
            var (sari, beyaz) = BumpMaterials();
            var root = new GameObject("Kasisler");   // sahne kökünde, dünyaya göre dönüşsüz
            Physics.SyncTransforms();
            bool Down(Vector3 u, out Vector3 q)
            {
                if (Physics.Raycast(new Vector3(u.x, u.y + 5f, u.z), Vector3.down, out var hit, 10f)) { q = hit.point; return true; }
                q = u; return false;
            }
            int nOk = 0, nSkip = 0;
            foreach (System.Xml.XmlNode w in doc.SelectNodes("/osm/way"))
            {
                bool isBump = false; float h = 0; string note = "";
                foreach (System.Xml.XmlNode t in w.SelectNodes("tag"))
                {
                    var k = t.Attributes["k"].Value; var v = t.Attributes["v"].Value;
                    if (k == "type" && v == "speed_bump") isBump = true;
                    if (k == "height") h = P(v);
                    if (k == "note") note = v;
                }
                if (!isBump || h < 0.06f) continue;   // yaya geçidi sanal kasisları (0.05) nesne değil
                var pts = new List<Vector3>();
                foreach (System.Xml.XmlNode nd in w.SelectNodes("nd"))
                    if (nodes.TryGetValue(nd.Attributes["ref"].Value, out var pp)) pts.Add(pp);
                if (pts.Count < 4) { nSkip++; continue; }
                Vector3 A = pts[0], B = pts[1], C = pts[2], D = pts[3];
                if (Vector2.Distance(A, B) < Vector2.Distance(B, C)) { var a0 = A; A = B; B = C; C = D; D = a0; }   // AB: yol enine, AD: yol yönünde
                Vector3 m0 = (A + D) / 2f, m1 = (B + C) / 2f;                 // enine orta çizginin uçları (ROS)
                float len = (Vector2.Distance(A, D) + Vector2.Distance(B, C)) / 2f;
                // Yol yüzeyinde kalan enine aralık: ortadaki yüksekliğe göre 5 cm'den yüksek (bordür/kaldırım) noktalar dışarıda
                const int ns = 60; var hy2 = new float[ns + 1]; var hitOk = new bool[ns + 1];
                for (int i = 0; i <= ns; i++) { hitOk[i] = Down(RosToUnity(Vector3.Lerp(m0, m1, i / (float)ns)), out var q); hy2[i] = q.y; }
                int mid = ns / 2; if (!hitOk[mid]) { nSkip++; continue; }
                int lo = mid, hi = mid;
                while (lo > 0 && hitOk[lo - 1] && Mathf.Abs(hy2[lo - 1] - hy2[mid]) < 0.05f + 0.03f * Mathf.Abs(lo - 1 - mid) * Vector2.Distance(m0, m1) / ns) lo--;
                while (hi < ns && hitOk[hi + 1] && Mathf.Abs(hy2[hi + 1] - hy2[mid]) < 0.05f + 0.03f * Mathf.Abs(hi + 1 - mid) * Vector2.Distance(m0, m1) / ns) hi++;
                Vector3 e0 = Vector3.Lerp(m0, m1, lo / (float)ns), e1 = Vector3.Lerp(m0, m1, hi / (float)ns);
                float wid = Vector2.Distance(e0, e1);
                if (wid < 1.0f) { nSkip++; continue; }
                var cR = (e0 + e1) / 2f;
                var cU = RosToUnity(cR);
                var fR = (D - A); fR.z = 0f; var fU = RosToUnity(fR).normalized; fU.y = 0f; fU.Normalize();   // yön vektörü de aynı eksen çevirimiyle
                var rU = Vector3.Cross(Vector3.up, fU);
                if (!Down(cU, out var pc) || !Down(cU + fU * (len * 0.4f), out var pf) || !Down(cU + rU * (wid * 0.3f), out var pr)) { nSkip++; continue; }
                var nrm = Vector3.Cross(pf - pc, pr - pc).normalized; if (nrm.y < 0f) nrm = -nrm;
                var go = new GameObject($"Kasis_{w.Attributes["id"].Value}");
                go.transform.SetParent(root.transform, false);
                go.transform.position = pc;
                go.transform.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(fU, nrm).normalized, nrm);
                var kas = go.AddComponent<Beytepe.Runtime.BeytepeKasis>();   // MeshFilter/Renderer/Collider RequireComponent ile gelir
                kas.genislik = wid; kas.uzunluk = len; kas.yukseklik = h; kas.blokGenisligi = 0.6f;
                kas.sari = sari; kas.beyaz = beyaz; kas.haritaKimligi = w.Attributes["id"].Value + (note.Length > 0 ? " (" + note + ")" : "");
                kas.Olustur();
                nOk++;
            }
            if (!AssetDatabase.IsValidFolder("Assets/Beytepe/Prefabs")) AssetDatabase.CreateFolder("Assets/Beytepe", "Prefabs");
            var saved = PrefabUtility.SaveAsPrefabAsset(root, BumpPrefabPath);
            UnityEngine.Object.DestroyImmediate(root);
            Physics.SyncTransforms();
            L($"Kasis prefab'ı: {nOk} kasis üretildi, {nSkip} atlandı");
            return saved;
        }

        [MenuItem("AWSIM/Beytepe/Kasisleri haritadan yeniden üret")]
        public static void RegenerateBumps()
        {
            // Beytepe sahnesi (yol modeliyle) açıkken çalıştırın; elle yapılan kasis düzenlemeleri silinir.
            if (AssetDatabase.LoadAssetAtPath<GameObject>(BumpPrefabPath) != null) AssetDatabase.DeleteAsset(BumpPrefabPath);
            var old = UnityEngine.Object.FindObjectsByType<Beytepe.Runtime.BeytepeKasis>(FindObjectsSortMode.None);
            foreach (var r in old.Select(k => k.transform.parent).Where(t => t != null).Distinct().ToList())
                UnityEngine.Object.DestroyImmediate(r.gameObject);
            var models = FindPath("AutowreSimulationDemo/Environments/Beytepe/Models");
            if (models == null) { Debug.LogError("[Beytepe] Beytepe sahnesi açık değil"); return; }
            AddBumps(models);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        }

        static void ApplyFallbackMaterials(GameObject model)
        {
            var shader = Shader.Find("HDRP/Lit");
            if (shader == null) { L("UYARI: HDRP/Lit bulunamadı, malzemeler değiştirilmedi"); return; }
            if (!AssetDatabase.IsValidFolder(MatDir)) AssetDatabase.CreateFolder("Assets/Beytepe", "Materials");
            var cache = new Dictionary<string, Material>();
            var used = new SortedDictionary<string, string>();
            Material Get(Material src)
            {
                if (src == null) return null;
                if (cache.TryGetValue(src.name, out var m)) return m;
                var row = MatTable.FirstOrDefault(r => src.name.IndexOf(r.key, StringComparison.OrdinalIgnoreCase) >= 0);
                if (row.key == null) row = ("?", new Color(0.5f, 0.5f, 0.5f), 0.2f, null, 0f, Color.white, false);
                var path = $"{MatDir}/{src.name}.mat";
                m = AssetDatabase.LoadAssetAtPath<Material>(path);
                if (m == null) { m = new Material(shader); AssetDatabase.CreateAsset(m, path); }
                m.shader = shader;
                Texture2D baseTex = null, nrm = null, mask = null;
                if (row.tex != null)
                {
                    baseTex = LoadTex($"{TexDir}/{row.tex}_Base.png", false, false, row.cutout);
                    nrm = LoadTex($"{TexDir}/{row.tex}_Normal.png", true, true, false);
                    mask = LoadTex($"{TexDir}/{row.tex}_Mask.png", false, true, false);
                }
                m.SetTexture("_BaseColorMap", baseTex);
                m.SetTexture("_NormalMap", nrm);
                m.SetTexture("_MaskMap", mask);
                m.SetFloat("_NormalScale", 1f);
                m.SetColor("_BaseColor", baseTex != null ? row.tint : row.color);
                m.SetFloat("_Metallic", 0f);
                m.SetFloat("_Smoothness", row.smooth);
                m.SetFloat("_SmoothnessRemapMin", 0f);
                m.SetFloat("_SmoothnessRemapMax", mask != null ? 0.6f : 1f);
                var tile = (baseTex != null && row.tileM > 0f) ? 1f / row.tileM : 1f;
                m.SetTextureScale("_BaseColorMap", new Vector2(tile, tile));
                m.SetTextureOffset("_BaseColorMap", Vector2.zero);
                m.SetFloat("_AlphaCutoffEnable", row.cutout && baseTex != null ? 1f : 0f);
                m.SetFloat("_AlphaCutoff", 0.45f);
                m.SetFloat("_DoubleSidedEnable", row.cutout ? 1f : 0f);
                ResetHdrpKeywords(m);
                EditorUtility.SetDirty(m);
                used[src.name] = baseTex != null
                    ? $"doku {row.tex} (tekrar {(row.tileM > 0 ? row.tileM.ToString("F1", CultureInfo.InvariantCulture) + " m" : "UV")}{(row.cutout ? ", alfa kesme" : "")})"
                    : $"renk ({row.color.r:F2}, {row.color.g:F2}, {row.color.b:F2})";
                return cache[src.name] = m;
            }
            int n = 0;
            foreach (var r in model.GetComponentsInChildren<Renderer>(true))
            {
                var mats = r.sharedMaterials;
                for (int i = 0; i < mats.Length; i++) mats[i] = Get(mats[i]);
                r.sharedMaterials = mats; n++;
            }
            AssetDatabase.SaveAssets();
            L($"Malzemeler: {n} renderer, {used.Count} malzeme -> " + string.Join("; ", used.Select(kv => $"{kv.Key}: {kv.Value}")));
        }

        [MenuItem("AWSIM/Beytepe/1. Sahneyi Kur")]
        public static void BuildScene()
        {
            Log.Clear();
            L($"Başlangıç {DateTime.Now:yyyy-MM-dd HH:mm:ss}");

            // --- 1) sahne kopyası
            if (!File.Exists(SrcScene)) throw new Exception("Demo sahnesi bulunamadı: " + SrcScene);
            if (File.Exists(DstScene)) AssetDatabase.DeleteAsset(DstScene);   // yeniden kurulumda temiz başla
            if (!AssetDatabase.CopyAsset(SrcScene, DstScene)) throw new Exception("Sahne kopyalanamadı: " + DstScene);
            var scene = EditorSceneManager.OpenScene(DstScene, OpenSceneMode.Single);
            L("Sahne kopyalandı: " + DstScene);

            // --- 2) Shinjuku'ya özgü içerikleri kaldır
            var models = FindPath("AutowreSimulationDemo/Environments/Shinjuku/Models")
                         ?? throw new Exception("Environments/Shinjuku/Models bulunamadı");
            int removed = DestroyChildren(models);
            models.parent.name = "Beytepe";
            L($"Shinjuku modelleri kaldırıldı ({removed} nesne)");

            foreach (var p in new[] { "TrafficSimulator/TrafficLanes", "TrafficSimulator/StopLines",
                                      "TrafficSimulator/NPCPedestrians", "TrafficSimulator/TrafficIntersections" })
            {
                var t = FindPath("AutowreSimulationDemo/" + p);
                if (t != null) L($"  {p}: {DestroyChildren(t)} nesne kaldırıldı");
            }
            var v2i = FindPath("AutowreSimulationDemo/Environments/V2I");
            if (v2i != null) { UnityEngine.Object.DestroyImmediate(v2i.gameObject); L("  V2I kaldırıldı"); }

            var demo = UnityEngine.Object.FindFirstObjectByType<Awsim.Scene.AutowareSimulationDemo.AutowareSimulationDemo>();
            var so = new SerializedObject(demo);
            so.FindProperty("_useV2i").boolValue = false;
            so.FindProperty("_simplePedestrianWalkerControllers").arraySize = 0;
            so.ApplyModifiedPropertiesWithoutUndo();

            var traffic = UnityEngine.Object.FindFirstObjectByType<Awsim.Usecase.TrafficSimulation.TrafficSimulator>();
            var st = new SerializedObject(traffic);
            foreach (var f in new[] { "_randomTrafficSims", "_routeTrafficSims", "_trafficIntersections" })
            {
                var pr = st.FindProperty(f);
                if (pr != null) pr.arraySize = 0; else L("  uyarı: TrafficSimulator alanı yok: " + f);
            }
            st.FindProperty("_maxVehicleCount").intValue = 0;
            st.ApplyModifiedPropertiesWithoutUndo();
            L("Trafik: şeritsiz, NPC sayısı 0 (ilk aşama)");

            // --- 3) Beytepe modeli
            L($"Dünya: {World}");
            GameObject model;
            if (World == "gercek")
            {
                model = AddRealWorld(models);
            }
            else
            {
                AssetDatabase.ImportAsset(FbxPath, ImportAssetOptions.ForceSynchronousImport);
                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FbxPath)
                             ?? throw new Exception("FBX yüklenemedi: " + FbxPath);
                model = (GameObject)PrefabUtility.InstantiatePrefab(prefab, models);
                model.name = "BeytepeModel";
                var importRot = model.transform.localEulerAngles;
                int nCol = 0, nMesh = 0;
                foreach (var mf in model.GetComponentsInChildren<MeshFilter>(true))
                {
                    nMesh++;
                    if (NoColliderKeywords.Any(k => FullName(mf.transform).IndexOf(k, StringComparison.OrdinalIgnoreCase) >= 0))
                        continue;
                    var mc = mf.gameObject.AddComponent<MeshCollider>();
                    mc.sharedMesh = mf.sharedMesh; nCol++;
                }
                L($"Model eklendi: {nMesh} mesh, {nCol} tanesine MeshCollider (ağaç/çalı hariç). İçe aktarma dönüşü {importRot}");
                ApplyFallbackMaterials(model);
            }

            // --- 4) hizalama: lanelet düğümlerine ışın
            var nodes = ReadLaneletNodes(OsmPath);
            L($"Lanelet düğümü: {nodes.Count} ({OsmPath})");
            var sample = nodes.Where((n, i) => i % Math.Max(1, nodes.Count / 400) == 0).ToList();
            var cands = new List<(Vector3 rot, Vector3 scale)>();
            foreach (var x in new[] { -90f, 0f, 90f })
                foreach (var y in new[] { 0f, 90f, 180f, 270f })
                    foreach (var mirror in new[] { false, true })
                        cands.Add((new Vector3(x, y, 0), mirror ? new Vector3(-1, 1, 1) : Vector3.one));
            (Vector3 rot, Vector3 scale, int ok, float medDz) best = (Vector3.zero, Vector3.one, -1, 0);
            foreach (var c in cands)
            {
                model.transform.localPosition = Vector3.zero;
                model.transform.localEulerAngles = c.rot;
                model.transform.localScale = c.scale * RootScale;
                Physics.SyncTransforms();
                var dzs = new List<float>();
                foreach (var n in sample)
                {
                    var u = RosToUnity(n);
                    if (Physics.Raycast(u + Vector3.up * 30f, Vector3.down, out var hit, 80f))
                        dzs.Add(hit.point.y - u.y);
                }
                int ok = dzs.Count(d => Mathf.Abs(d) < 0.3f);
                float med = dzs.Count > 0 ? dzs.OrderBy(d => d).ElementAt(dzs.Count / 2) : float.NaN;
                if (ok > best.ok) best = (c.rot, c.scale, ok, med);
            }
            model.transform.localPosition = Vector3.zero;
            model.transform.localEulerAngles = best.rot;
            model.transform.localScale = best.scale * RootScale;
            Physics.SyncTransforms();
            float okRatio = best.ok / (float)sample.Count;
            L($"Hizalama: dönüş {best.rot}, ölçek {best.scale}; {best.ok}/{sample.Count} düğüm yüzeyde (|dz|<0.3 m) = {okRatio:P0}, medyan dz {best.medDz:F3} m");
            if (okRatio < 0.6f)
                L("UYARI: hizalama zayıf (<%60). Model ve harita aynı çerçevede olmayabilir; sahneyi kontrol edin.");
            if (World == "gercek") AddRealTrees(models);
            else { AddBumps(models); AddBuildings(models, model); }
            AddSky();

            // --- 5) MGRS ofseti = 0
            var mgrs = UnityEngine.Object.FindFirstObjectByType<Awsim.Common.MgrsPosition>();
            var sm = new SerializedObject(mgrs);
            sm.FindProperty("_mgrs._position").vector3Value = Vector3.zero;
            sm.FindProperty("_mgrs._gridZone").stringValue = "36SVK";
            sm.ApplyModifiedPropertiesWithoutUndo();
            L("MGRS ofseti 0 (Local projektör)");

            // --- 5b) NPC trafiği: lanelet haritasından TrafficLane üret
            AddNpcTraffic();
            AddCrosswalkPedestrians();

            // --- 6) ego aracı
            var ego = UnityEngine.Object.FindFirstObjectByType<Awsim.Scene.AutowareSimulationDemo.EgoVehicle>();
            var vehicle = new SerializedObject(ego).FindProperty("_vehicle").objectReferenceValue as Component;
            var egoT = vehicle != null ? vehicle.transform : ego.transform;
            var start = RosToUnity(EgoRosPos);
            if (Physics.Raycast(new Vector3(start.x, 400f, start.z), Vector3.down, out var gh, 600f))
                start.y = gh.point.y + 0.3f;
            egoT.position = start;
            egoT.rotation = Quaternion.Euler(0, -EgoRosYawDeg, 0);   // ROS yaw (CCW, x'ten) -> Unity Y (-yaw)
            L($"Ego: {egoT.name} Unity {start} (ROS {EgoRosPos.x},{EgoRosPos.y}, yaw {EgoRosYawDeg}°)");
            AddHusmartSensors(egoT, ego.transform);

            // --- 7) kaydet
            EditorSceneManager.MarkSceneDirty(scene);
            EditorSceneManager.SaveScene(scene);
            EditorBuildSettings.scenes = new[] { new EditorBuildSettingsScene(DstScene, true) };
            AssetDatabase.SaveAssets();
            L("Sahne kaydedildi.");
            WriteReport();
        }

        // --- Hüsmart sensör seti (Hesai JT128 + ZED X + Xsens MTi-7). Nesneler KAPALI eklenir; seçim çalışma anında
        //     Beytepe.Runtime.BeytepeSensorRig ile (--json_path dosyasındaki "SensorRig" bölümü). Bölüm yoksa ya da
        //     "Name": "lexus" ise özgün Lexus sensörleri aynen çalışır.
        const string CameraSensorPrefabGuid = "3780c25cb0cac05fd9081fd1d51ae3b0";

        static Transform FindDeep(Transform t, string name)
        {
            if (t == null) return null;
            if (t.name == name) return t;
            foreach (Transform c in t) { var r = FindDeep(c, name); if (r != null) return r; }
            return null;
        }

        const string SensorMaterialPath = "Assets/Awsim/Models/Entity/Sensor/Velodyne VLP-16/Materials/SensorMetal.mat";

        /// <summary>Sensörün AWSIM ekranında görünmesi için basit model (çarpıştırıcısız). Lidarın altına konursa RGL onu
        /// hiçbir lidar için sahneye almaz (ataları arasında LidarSensor olan nesneler dışlanır); kendi ışınlarını kesmez.</summary>
        static void AddSensorModel(Transform parent, PrimitiveType type, Vector3 scale, Vector3 localPos)
        {
            var m = GameObject.CreatePrimitive(type);
            m.name = "sensor_model";
            UnityEngine.Object.DestroyImmediate(m.GetComponent<Collider>());
            m.transform.SetParent(parent, false);
            m.transform.localPosition = localPos;
            m.transform.localScale = scale;
            var mat = AssetDatabase.LoadAssetAtPath<Material>(SensorMaterialPath);
            if (mat != null) m.GetComponent<MeshRenderer>().sharedMaterial = mat;
            var r = m.GetComponent<MeshRenderer>();
            r.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        }

        static void AddHusmartSensors(Transform vehicleT, Transform egoT)
        {
            Transform root = null, baseLink = null;
            foreach (var cand in new[] { vehicleT, egoT, vehicleT.root })
            {
                baseLink = FindDeep(cand, "base_link");
                if (baseLink != null) { root = cand; break; }
            }
            if (baseLink == null) { L("UYARI: ego base_link bulunamadı; Hüsmart sensörleri eklenmedi"); return; }
            var vlp = root.GetComponentsInChildren<RGLUnityPlugin.LidarSensor>(true).FirstOrDefault();
            var vlpPub = vlp != null ? vlp.GetComponent<Awsim.Entity.RglLidarPublisher>() : null;
            var kit = FindDeep(root, "XX1 Sensor Kit");
            var gnss = root.GetComponentInChildren<Awsim.Entity.GnssSensor>(true);

            // 1) Hesai JT128 (VLP-16 ile aynı konular: /sensing/lidar/top/pointcloud_raw(_ex); çerçeve hesai_top)
            var jt = new GameObject("hesai_top");
            jt.SetActive(false);
            jt.transform.SetParent(baseLink, false);
            var lidar = jt.AddComponent<RGLUnityPlugin.LidarSensor>();
            lidar.modelPreset = RGLUnityPlugin.LidarModel.HesaiJT128;
            lidar.configuration = RGLUnityPlugin.LidarConfigurationLibrary.ByModel[RGLUnityPlugin.LidarModel.HesaiJT128]();
            lidar.AutomaticCaptureHz = 10;
            if (vlp != null)
            {
                lidar.returnMode = vlp.returnMode;
                lidar.applyDistanceGaussianNoise = vlp.applyDistanceGaussianNoise;
                lidar.applyAngularGaussianNoise = vlp.applyAngularGaussianNoise;
                lidar.applyVelocityDistortion = vlp.applyVelocityDistortion;
                lidar.simulateBeamDivergence = vlp.simulateBeamDivergence;
            }
            AddSensorModel(jt.transform, PrimitiveType.Cylinder, new Vector3(0.06f, 0.034f, 0.06f), Vector3.zero);   // JT128 Φ60x68 mm
            var pub = jt.AddComponent<Awsim.Entity.RglLidarPublisher>();
            pub.frameId = "hesai_top";
            if (vlpPub != null)
            {
                pub.qos = vlpPub.qos;
                pub.pointCloud2Publishers = vlpPub.pointCloud2Publishers.Select(x => new Awsim.Entity.PointCloud2Publisher
                    { topic = x.topic, publish = x.publish, fieldsPreset = x.fieldsPreset, fields = x.fields?.ToArray() }).ToList();
            }

            // 2) ZED X: AWSIM CameraSensor prefabından iki göz
            var camPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(CameraSensorPrefabGuid));
            // CameraSensor prefabının katman maskesi yalnız "Default"; Beytepe modeli BeytepeGround katmanında.
            // Lexus'un trafik ışığı kamerasının maskesi (247) kopyalanır, bulunamazsa Visualization dışındaki her şey.
            int camMask = ~(1 << 11);
            var legacyCam = root.GetComponentsInChildren<Awsim.Entity.CameraSensor>(true)
                                .Select(c => c.GetComponentInChildren<Camera>(true)).FirstOrDefault(c => c != null);
            if (legacyCam != null) camMask = legacyCam.cullingMask;
            var zc = new GameObject("zedx_camera_center");
            zc.SetActive(false);
            zc.transform.SetParent(baseLink, false);
            var stereo = zc.AddComponent<Beytepe.Runtime.BeytepeStereoCamera>();
            // ZED X gövdesi ~166 x 44 x 42 mm; lensler ön yüzde, kamera yakın kırpma (0.1 m) içinde kaldığı için kendi görüntüsünde görünmez
            AddSensorModel(zc.transform, PrimitiveType.Cube, new Vector3(0.166f, 0.044f, 0.042f), new Vector3(0f, 0f, -0.022f));
            var sso = new SerializedObject(stereo);
            UnityEngine.Object shader = null;
            foreach (var (eyeName, side) in new[] { ("zedx_left_camera_frame", "left"), ("zedx_right_camera_frame", "right") })
            {
                if (camPrefab == null) break;
                var go = (GameObject)PrefabUtility.InstantiatePrefab(camPrefab, zc.transform);
                go.name = eyeName;
                var cs = go.GetComponent<Awsim.Entity.CameraSensor>();
                var cam = go.GetComponentInChildren<Camera>(true);
                if (cam != null) { cam.transform.localPosition = Vector3.zero; cam.transform.localRotation = Quaternion.identity; cam.cullingMask = camMask; }
                var rp = go.GetComponent<Awsim.Entity.CameraRos2Publisher>();
                if (rp != null)
                {
                    shader = shader ?? new SerializedObject(rp).FindProperty("_rosImageShader").objectReferenceValue;
                    rp.enabled = false;   // yayını BeytepeStereoCamera yapar
                }
                var cso = new SerializedObject(cs);
                cso.FindProperty("_width").intValue = 1920;
                cso.FindProperty("_height").intValue = 1200;
                cso.ApplyModifiedPropertiesWithoutUndo();
                sso.FindProperty(side == "left" ? "_leftSensor" : "_rightSensor").objectReferenceValue = cs;
                sso.FindProperty(side == "left" ? "_leftCamera" : "_rightCamera").objectReferenceValue = cam;
            }
            sso.FindProperty("_rosImageShader").objectReferenceValue = shader;
            sso.ApplyModifiedPropertiesWithoutUndo();

            // 3) Xsens MTi-7 + GNSS anteni
            var mti = new GameObject("mti7_imu_link");
            mti.SetActive(false);
            mti.transform.SetParent(baseLink, false);
            var mc = mti.AddComponent<Beytepe.Runtime.BeytepeMti7>();
            var ant = new GameObject("mti7_gnss_antenna");
            ant.transform.SetParent(baseLink, false);
            var mso = new SerializedObject(mc);
            mso.FindProperty("_antenna").objectReferenceValue = ant.transform;
            mso.ApplyModifiedPropertiesWithoutUndo();

            // 4) Sensör seti seçici
            var rig = root.gameObject.AddComponent<Beytepe.Runtime.BeytepeSensorRig>();
            var rso = new SerializedObject(rig);
            rso.FindProperty("_baseLink").objectReferenceValue = baseLink;
            rso.FindProperty("_lexusVlp16").objectReferenceValue = vlp != null ? vlp.gameObject : null;
            rso.FindProperty("_lexusSensorKitVisual").objectReferenceValue = kit != null ? kit.gameObject : null;
            rso.FindProperty("_lexusGnss").objectReferenceValue = gnss != null ? gnss.transform : null;
            rso.FindProperty("_jt128").objectReferenceValue = jt;
            rso.FindProperty("_zedx").objectReferenceValue = stereo;
            rso.FindProperty("_mti7").objectReferenceValue = mc;
            rso.ApplyModifiedPropertiesWithoutUndo();
            L($"Hüsmart sensörleri (kapalı, JSON 'SensorRig' ile seçilir): JT128 {(vlpPub != null ? "VLP-16 konularıyla" : "KONU YOK")}, " +
              $"ZED X {(camPrefab != null && shader != null ? "2 göz" : "EKSİK")} (katman maskesi {camMask}), MTi-7; VLP-16 {(vlp != null ? "bulundu" : "YOK")}, " +
              $"XX1 kit {(kit != null ? "bulundu" : "YOK")}, GNSS {(gnss != null ? "bulundu" : "YOK")}; kök {root.name}");
        }

        // AWSIM demosundaki NPC araç prefab'ları (Hatchback, SmallCar, Taxi, Truck_2t, Van)
        // AWSIM HDRP yaya modelleri: humanCasual, humanElegant
        static readonly string[] PedestrianPrefabGuids = { "45451c9dee0017949873d87da14d750a", "27e8a3736efefa447b1f65b33b30908a" };

        // --- Yaya geçitleri: haritadaki crosswalk laneletlerinden geçiş hattını çıkar, rastgele yaya yöneticisini kur
        static void AddCrosswalkPedestrians()
        {
            var root = FindPath("AutowreSimulationDemo");
            var old = root.Find("BeytepeCrosswalkPedestrians");
            if (old != null) UnityEngine.Object.DestroyImmediate(old.gameObject);
            var go = new GameObject("BeytepeCrosswalkPedestrians");
            go.transform.SetParent(root, false);
            var comp = go.AddComponent<Beytepe.Runtime.BeytepeCrosswalkPedestrians>();

            var osm = Awsim.Common.OsmData.Read(OsmPath);
            string Tag(Awsim.Common.Tag[] tags, string k) => tags == null ? null : tags.FirstOrDefault(x => x.Key == k).Value;
            float F(Awsim.Common.Tag[] tags, string k) => float.Parse(Tag(tags, k) ?? "0", CultureInfo.InvariantCulture);
            var nodes = osm.Nodes.ToDictionary(n => n.Id, n => n);
            var ways = osm.Ways.ToDictionary(w => w.Id, w => w);
            Vector3 P(long id)
            {
                var t = nodes[id].Tags;
                return Awsim.Common.Ros2Utility.Ros2ToUnityPosition(new Vector3(F(t, "local_x"), F(t, "local_y"), F(t, "ele")));
            }
            var cws = new List<(Vector3 a, Vector3 b, float w)>();
            foreach (var r in osm.Relations)
            {
                if (Tag(r.Tags, "type") != "lanelet" || Tag(r.Tags, "subtype") != "crosswalk") continue;
                var lw = r.Members.FirstOrDefault(m => m.Role == "left").RefId;
                var rw = r.Members.FirstOrDefault(m => m.Role == "right").RefId;
                if (!ways.ContainsKey(lw) || !ways.ContainsKey(rw)) continue;
                var Lp = ways[lw].NodeIds.Select(P).ToArray();
                var Rp = ways[rw].NodeIds.Select(P).ToArray();
                if (Lp.Length < 2 || Rp.Length < 2) continue;
                if (Vector3.Distance(Lp[0], Rp[0]) + Vector3.Distance(Lp[Lp.Length - 1], Rp[Rp.Length - 1]) >
                    Vector3.Distance(Lp[0], Rp[Rp.Length - 1]) + Vector3.Distance(Lp[Lp.Length - 1], Rp[0]))
                    Rp = Rp.Reverse().ToArray();
                var a = (Lp[0] + Rp[0]) * 0.5f;
                var b = (Lp[Lp.Length - 1] + Rp[Rp.Length - 1]) * 0.5f;
                var w = Vector3.Distance(Lp[Lp.Length / 2], Rp[Rp.Length / 2]);
                cws.Add((a, b, w));
            }

            var so = new SerializedObject(comp);
            var pf = so.FindProperty("_pedestrianPrefabs");
            var prefabs = PedestrianPrefabGuids.Select(g => AssetDatabase.LoadAssetAtPath<GameObject>(AssetDatabase.GUIDToAssetPath(g)))
                                               .Where(x => x != null).ToArray();
            pf.arraySize = prefabs.Length;
            for (int i = 0; i < prefabs.Length; i++) pf.GetArrayElementAtIndex(i).objectReferenceValue = prefabs[i];
            var list = so.FindProperty("_crosswalks");
            list.arraySize = cws.Count;
            for (int i = 0; i < cws.Count; i++)
            {
                var e = list.GetArrayElementAtIndex(i);
                e.FindPropertyRelative("start").vector3Value = cws[i].a;
                e.FindPropertyRelative("end").vector3Value = cws[i].b;
                e.FindPropertyRelative("width").floatValue = cws[i].w;
            }
            so.ApplyModifiedPropertiesWithoutUndo();
            L($"Yaya geçitleri: {cws.Count} geçit, yaya modeli {prefabs.Length}/{PedestrianPrefabGuids.Length}; rastgele geçişler (tek / grup / kalabalık)");
        }

        static readonly string[] NpcPrefabGuids = {
            "6d7d303c5e4defa16b508cece3ef599e", "a4db0a3241fc6cbcdb523a20abe25ec9", "cd6e20f712013db92912be3accdca402",
            "2aab0ce90015ebadfadf702af88b666e", "467a68bbdca38cddda59c3c6af539d63" };
        // Haritadaki hız limitleri RoadRunner varsayılanı (64/89 km/sa); NPC'ler kampüs için bununla sınırlanır.
        const float NpcSpeedCapKmh = 30f;

        const string GroundLayerName = "BeytepeGround";

        /// <summary>TagManager'da katmanı bulur, yoksa tercih edilen (boşsa) ya da ilk boş kullanıcı katmanına ekler.</summary>
        static int EnsureLayer(string name, int preferred)
        {
            int existing = LayerMask.NameToLayer(name);
            if (existing >= 0) return existing;
            var tm = new SerializedObject(AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/TagManager.asset")[0]);
            var layers = tm.FindProperty("layers");
            int idx = string.IsNullOrEmpty(layers.GetArrayElementAtIndex(preferred).stringValue) ? preferred : -1;
            for (int i = 8; idx < 0 && i < 32; i++)
                if (string.IsNullOrEmpty(layers.GetArrayElementAtIndex(i).stringValue)) idx = i;
            if (idx < 0) throw new Exception("Boş katman kalmadı");
            layers.GetArrayElementAtIndex(idx).stringValue = name;
            tm.ApplyModifiedPropertiesWithoutUndo();
            AssetDatabase.SaveAssets();
            return idx;
        }

        static void AddNpcTraffic()
        {
            var tsRoot = FindPath("AutowreSimulationDemo/TrafficSimulator")
                         ?? throw new Exception("AutowreSimulationDemo/TrafficSimulator bulunamadı");
            foreach (var n in new[] { "TrafficLanes", "StopLines" })
            {
                var old = tsRoot.Find(n);
                if (old != null) UnityEngine.Object.DestroyImmediate(old.gameObject);
            }

            // Zemin katmanı: NPC'lerin öndeki engel taraması (boxcast) eğimli yolda yol yüzeyine çarpıp
            // aracı durduruyordu. Beytepe modeli ayrı katmana alınır ve engel maskesinden çıkarılır.
            int groundLayer = EnsureLayer(GroundLayerName, 6);
            var modelRoot = FindPath("AutowreSimulationDemo/Environments/Beytepe/Models")
                            ?? throw new Exception("Environments/Beytepe/Models bulunamadı");
            int nLayer = 0;
            foreach (var t in modelRoot.GetComponentsInChildren<Transform>(true)) { t.gameObject.layer = groundLayer; nLayer++; }

            var loader = new Awsim.Common.LaneletLoader();
            // Şerit noktaları en fazla 10 m arayla (varsayılan 30 m): eğimi daha iyi izler
            loader.SetWaypointSettings(new Awsim.Common.LaneletLoader.WaypointSettings(1f, 10f, 15f));
            loader.Load(Awsim.Common.OsmData.Read(OsmPath), Vector3.zero, tsRoot.gameObject);   // MGRS ofseti 0

            var lanes = tsRoot.GetComponentsInChildren<Awsim.Usecase.TrafficSimulation.TrafficLane>(true);
            float cap = NpcSpeedCapKmh / 3.6f;
            foreach (var lane in lanes)
            {
                var so = new SerializedObject(lane);
                var sp = so.FindProperty("_speedLimit");
                sp.floatValue = sp.floatValue > 0f ? Mathf.Min(sp.floatValue, cap) : cap;
                so.ApplyModifiedPropertiesWithoutUndo();
            }
            // Araçlar yalnızca giriş şeritlerinden (öncesi olmayan: harita kenarı / çıkmaz uç) doğar;
            // devamı olmayan şeridin sonunda AWSIM aracı siler. Böylece haritaya girip çıkıyormuş gibi davranırlar.
            var spawnable = lanes.Where(l => l.PrevLanes.Count == 0 && l.Waypoints.Length >= 2).ToArray();
            if (spawnable.Length < 3)
            {
                L($"UYARI: yalnızca {spawnable.Length} giriş şeridi var; tüm düz şeritler doğma noktası yapıldı.");
                spawnable = lanes.Where(l => !l._intersectionLane
                                             && l.TurnDirection == Awsim.Usecase.TrafficSimulation.TrafficLane.TurnDirectionType.Straight
                                             && l.Waypoints.Length >= 2).ToArray();
            }
            int exits = lanes.Count(l => l.NextLanes.Count == 0);
            L("Giriş şeritleri: " + string.Join(", ", spawnable.Select(l =>
            {
                var p = l.Waypoints[0];
                return $"{l.name.Replace("TrafficLane.", "")} ({p.z:F0},{-p.x:F0})";   // ROS x,y
            })) + $"; çıkış (devamı olmayan) şerit {exits}");
            // Davranış testleri için: araçlar yalnızca giriş şeritlerinde değil, kavşak dışındaki her uzun şeridin
            // başında da doğabilsin (AWSIM, egonun 50 m yakınında doğurmaz). Böylece trafik en baştan haritaya yayılır.
            const float MinSpawnLaneLength = 30f;
            float LaneLength(Awsim.Usecase.TrafficSimulation.TrafficLane l)
            {
                float s = 0f;
                for (int i = 1; i < l.Waypoints.Length; i++) s += Vector3.Distance(l.Waypoints[i - 1], l.Waypoints[i]);
                return s;
            }
            var extraSpawn = lanes.Where(l => !l._intersectionLane && l.Waypoints.Length >= 2 && !spawnable.Contains(l)
                                              && LaneLength(l) >= MinSpawnLaneLength).ToArray();
            spawnable = spawnable.Concat(extraSpawn).ToArray();
            L($"Ek doğma şeridi (kavşak dışı, >= {MinSpawnLaneLength:F0} m): {extraSpawn.Length}; toplam doğma şeridi {spawnable.Length}");

            var prefabs = NpcPrefabGuids
                .Select(g => AssetDatabase.LoadAssetAtPath<Awsim.Usecase.TrafficSimulation.TrafficSimNpcVehicle>(AssetDatabase.GUIDToAssetPath(g)))
                .Where(p => p != null).ToArray();

            var traffic = UnityEngine.Object.FindFirstObjectByType<Awsim.Usecase.TrafficSimulation.TrafficSimulator>();
            var st = new SerializedObject(traffic);
            var sims = st.FindProperty("_randomTrafficSims");
            sims.arraySize = 1;
            var sim = sims.GetArrayElementAtIndex(0);
            sim.FindPropertyRelative("_enableSimulation").boolValue = true;
            sim.FindPropertyRelative("_enableSpawnCountLimit").boolValue = false;
            sim.FindPropertyRelative("_spawnCountLimit").intValue = 0;
            var pp = sim.FindPropertyRelative("_trafficSimNpcVehiclePrefabs");
            pp.arraySize = prefabs.Length;
            for (int i = 0; i < prefabs.Length; i++) pp.GetArrayElementAtIndex(i).objectReferenceValue = prefabs[i];
            var sl = sim.FindPropertyRelative("_spawnableTrafficLanes");
            sl.arraySize = spawnable.Length;
            for (int i = 0; i < spawnable.Length; i++) sl.GetArrayElementAtIndex(i).objectReferenceValue = spawnable[i];
            st.FindProperty("_maxVehicleCount").intValue = 10;   // çalışırken JSON'daki MaxVehicleCount geçerli
            st.FindProperty("_obstacleLayerMask.m_Bits").uintValue = ~(1u << groundLayer);
            st.FindProperty("_groundLayerMask.m_Bits").uintValue = 1u << groundLayer;
            foreach (var f in new[] { "_showGizmos", "_showYieldingPhase", "_showObstacleChecking", "_showSpawnPoints" })
            {
                var g = st.FindProperty(f);
                if (g != null) g.boolValue = false;
            }
            st.ApplyModifiedPropertiesWithoutUndo();

            var slRoot = tsRoot.Find("StopLines");
            L($"Trafik: {lanes.Length} TrafficLane (kavşak şeridi {lanes.Count(l => l._intersectionLane)}), " +
              $"{(slRoot != null ? slRoot.childCount : 0)} durma çizgisi, doğma şeridi {spawnable.Length}, " +
              $"NPC prefab {prefabs.Length}/{NpcPrefabGuids.Length}, NPC hız sınırı {NpcSpeedCapKmh} km/sa; " +
              $"model katmanı '{GroundLayerName}' ({groundLayer}, {nLayer} nesne), engel maskesinden hariç");
            if (prefabs.Length == 0) L("UYARI: NPC prefab'ı bulunamadı, trafik oluşmaz.");
        }

        [MenuItem("AWSIM/Beytepe/2. Oynatıcıyı Derle")]
        public static void BuildPlayer()
        {
            Directory.CreateDirectory(Path.GetDirectoryName(BuildPath));
            PlayerSettings.graphicsJobs = false;   // AWSIM TroubleShooting: segfault önlemi
            var opts = new BuildPlayerOptions
            {
                scenes = new[] { DstScene },
                locationPathName = BuildPath,
                target = BuildTarget.StandaloneLinux64,
                options = BuildOptions.None,
            };
            var r = BuildPipeline.BuildPlayer(opts);
            L($"Derleme: {r.summary.result}, {r.summary.totalSize / 1e6:F0} MB, {r.summary.totalTime}, hata {r.summary.totalErrors} -> {BuildPath}");
            WriteReport(append: true);
            if (r.summary.result != UnityEditor.Build.Reporting.BuildResult.Succeeded && Application.isBatchMode)
                EditorApplication.Exit(1);
        }

        public static void BuildAll()
        {
            try { BuildScene(); BuildPlayer(); }
            catch (Exception e)
            {
                L("HATA: " + e); WriteReport(append: true);
                if (Application.isBatchMode) EditorApplication.Exit(1);
                throw;
            }
        }

        // ------------------------------------------------------------ yardımcılar
        static Vector3 RosToUnity(Vector3 r) => new Vector3(-r.y, r.z, r.x);

        static List<Vector3> ReadLaneletNodes(string path)
        {
            var res = new List<Vector3>();
            if (!File.Exists(path)) { L("UYARI: OSM yok: " + path); return res; }
            var txt = File.ReadAllText(path);
            var rx = new Regex("<node [^>]*>\\s*<tag k=\"local_x\" v=\"([-0-9.eE]+)\"\\s*/>\\s*<tag k=\"local_y\" v=\"([-0-9.eE]+)\"\\s*/>\\s*<tag k=\"ele\" v=\"([-0-9.eE]+)\"");
            foreach (Match m in rx.Matches(txt))
                res.Add(new Vector3(P(m.Groups[1].Value), P(m.Groups[2].Value), P(m.Groups[3].Value)));
            return res;
        }

        static float P(string s) => float.Parse(s, CultureInfo.InvariantCulture);   // Türkçe yerel ayarda ondalık virgül sorunu

        static Transform FindPath(string path)
        {
            var parts = path.Split('/');
            var root = EditorSceneManager.GetActiveScene().GetRootGameObjects().FirstOrDefault(g => g.name == parts[0]);
            if (root == null) return null;
            var t = root.transform;
            for (int i = 1; i < parts.Length && t != null; i++) t = t.Find(parts[i]);
            return t;
        }

        static int DestroyChildren(Transform t)
        {
            int n = t.childCount;
            for (int i = n - 1; i >= 0; i--) UnityEngine.Object.DestroyImmediate(t.GetChild(i).gameObject);
            return n;
        }

        static string FullName(Transform t)
        {
            var sb = new StringBuilder(t.name);
            for (var p = t.parent; p != null && p.name != "BeytepeModel"; p = p.parent) sb.Insert(0, p.name + "/");
            return sb.ToString();
        }

        static void WriteReport(bool append = false)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(ReportPath));
            if (append) File.AppendAllText(ReportPath, Log.ToString()); else File.WriteAllText(ReportPath, Log.ToString());
            Log.Clear();
        }
    }
}
