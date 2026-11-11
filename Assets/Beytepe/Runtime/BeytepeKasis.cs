// Beytepe kasisi (2026-11-10): tek başına düzenlenebilir hız tümseği nesnesi.
// - Yerel eksenler: x = yol enine (genişlik), z = yol yönünde (uzunluk), y = yukarı. Nesnenin kendisi yol yüzeyine oturur.
// - Şekil: yol yönünde yuvarlak tümsek, yükseklik(z) = yukseklik * sin²(π (z/uzunluk + 0.5)); enine düz.
// - Görünüm: enine sarı/beyaz bloklar (blokGenisligi). Malzeme 0 = sarı, 1 = beyaz.
// - Fizik: MeshCollider (araç üstünden geçerken sarsılır).
// Inspector'da değer değişince şekil hemen yenilenir. Sağ tık menüsü: "Yere oturt" (yol yüzeyine ve eğimine hizalar).
// Kasislerin hepsi Assets/Beytepe/Prefabs/Kasisler.prefab içinde durur; sahne kurucu her derlemede bu prefab'ı kullanır.
// Not: Autoware'in yavaşlaması lanelet haritasındaki kasis alanlarına göredir; bir kasisi taşırsanız haritayı da güncelleyin.
using UnityEngine;

namespace Beytepe.Runtime
{
    [ExecuteAlways]
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(MeshCollider))]
    public class BeytepeKasis : MonoBehaviour
    {
        [Tooltip("Yol enine genişlik (m)")] public float genislik = 7f;
        [Tooltip("Yol yönünde uzunluk (m)")] public float uzunluk = 4f;
        [Tooltip("Tepe yüksekliği (m)")] public float yukseklik = 0.08f;
        [Tooltip("Sarı/beyaz blok genişliği (m)")] public float blokGenisligi = 0.6f;
        [Tooltip("Yol yönündeki bölüm sayısı (eğrinin pürüzsüzlüğü)")] public int bolum = 16;
        public Material sari;
        public Material beyaz;
        [Tooltip("Lanelet haritasındaki kasis alanının kimliği (bilgi amaçlı)")] public string haritaKimligi = "";

        Mesh _mesh;

        void OnEnable() { Olustur(); }

#if UNITY_EDITOR
        void OnValidate()
        {
            // OnValidate içinde collider/mesh atamak uyarı verir; bir sonraki editör adımında yenile
            UnityEditor.EditorApplication.delayCall += () => { if (this != null) Olustur(); };
        }
#endif

        void OnDestroy()
        {
            if (_mesh != null)
            {
                if (Application.isPlaying) Destroy(_mesh); else DestroyImmediate(_mesh);
            }
        }

        public void Olustur()
        {
            float w = Mathf.Max(0.2f, genislik), L = Mathf.Max(0.2f, uzunluk), h = Mathf.Max(0f, yukseklik);
            int nv = Mathf.Clamp(bolum, 2, 64);
            int nb = Mathf.Max(1, Mathf.RoundToInt(w / Mathf.Max(0.1f, blokGenisligi)));
            if (_mesh == null) _mesh = new Mesh { name = "kasis", hideFlags = HideFlags.DontSave };
            _mesh.Clear();
            var verts = new System.Collections.Generic.List<Vector3>();
            var subs = new[] { new System.Collections.Generic.List<int>(), new System.Collections.Generic.List<int>() };
            float Prof(float z) { float s = Mathf.Sin(Mathf.PI * (z / L + 0.5f)); return 0.005f + h * s * s; }
            for (int k = 0; k < nb; k++)
            {
                float x0 = -w / 2 + w * k / nb, x1 = -w / 2 + w * (k + 1) / nb;
                var tri = subs[k % 2];
                int b0 = verts.Count;
                for (int j = 0; j <= nv; j++)
                {
                    float z = -L / 2 + L * j / nv, y = Prof(z);
                    verts.Add(new Vector3(x0, y, z)); verts.Add(new Vector3(x1, y, z));
                }
                for (int j = 0; j < nv; j++)
                {
                    int a = b0 + 2 * j, b = a + 2, c = a + 1, d = a + 3;   // a=(x0,zj) b=(x0,zj+1) c=(x1,zj) d=(x1,zj+1): üst yüz +y
                    tri.AddRange(new[] { a, b, c, c, b, d });
                }
            }
            // yan kapaklar (x = ±w/2): tümseğin yanı açık kalmasın, yüzeye kadar iner
            for (int side = 0; side < 2; side++)
            {
                float x = side == 0 ? -w / 2 : w / 2;
                var tri = subs[side == 0 ? 0 : (nb - 1) % 2];
                int b0 = verts.Count;
                for (int j = 0; j <= nv; j++)
                {
                    float z = -L / 2 + L * j / nv;
                    verts.Add(new Vector3(x, Prof(z), z)); verts.Add(new Vector3(x, -0.02f, z));
                }
                for (int j = 0; j < nv; j++)
                {
                    int a = b0 + 2 * j, lo = a + 1, b = a + 2, blo = a + 3;
                    if (side == 0) tri.AddRange(new[] { a, lo, b, b, lo, blo });   // dışa (-x) bakar
                    else tri.AddRange(new[] { a, b, lo, b, blo, lo });             // dışa (+x) bakar
                }
            }
            _mesh.SetVertices(verts);
            _mesh.subMeshCount = 2;
            _mesh.SetTriangles(subs[0], 0);
            _mesh.SetTriangles(subs[1], 1);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
            GetComponent<MeshFilter>().sharedMesh = _mesh;
            var mr = GetComponent<MeshRenderer>();
            if (sari != null && beyaz != null) mr.sharedMaterials = new[] { sari, beyaz };
            var mc = GetComponent<MeshCollider>();
            mc.sharedMesh = null; mc.sharedMesh = _mesh;
        }

        // Yol yüzeyine ve eğimine oturtur (yön, yani yol boyunca bakış korunur).
        [ContextMenu("Yere oturt (yol yüzeyi ve eğimi)")]
        public void YereOturt()
        {
            var col = GetComponent<MeshCollider>();
            bool was = col.enabled; col.enabled = false;
            Physics.SyncTransforms();
            var fwd = transform.forward; fwd.y = 0f;
            if (fwd.sqrMagnitude < 1e-6f) fwd = Vector3.forward;
            fwd.Normalize();
            var right = Vector3.Cross(Vector3.up, fwd);
            var c = transform.position;
            bool Hit(Vector3 p, out Vector3 q)
            {
                if (Physics.Raycast(new Vector3(p.x, p.y + 5f, p.z), Vector3.down, out var hit, 15f)) { q = hit.point; return true; }
                q = p; return false;
            }
            if (Hit(c, out var pc) && Hit(c + fwd * (uzunluk * 0.4f), out var pf) && Hit(c + right * (genislik * 0.3f), out var pr))
            {
#if UNITY_EDITOR
                UnityEditor.Undo.RecordObject(transform, "Yere oturt");
#endif
                var n = Vector3.Cross(pf - pc, pr - pc).normalized;
                if (n.y < 0f) n = -n;
                transform.position = pc;
                transform.rotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(fwd, n).normalized, n);
            }
            col.enabled = was;
        }
    }
}
