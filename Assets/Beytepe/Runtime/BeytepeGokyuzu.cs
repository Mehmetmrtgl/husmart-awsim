// Beytepe gökyüzü (2026-11-10): mavi gradyan gökyüzü.
// AWSIM demo sahnesinin gökyüzü bir HDRI görüntüsüne bağlı, o görüntü projede olmadığı için gökyüzü düz gri çıkıyordu.
// Bu bileşen oyun başlarken öncelikli bir global Volume kurar: VisualEnvironment -> Gradient sky (üst / orta / alt renk).
// Ayarlar: AWSIM'e verilen --json_path dosyasındaki "Gokyuzu" bölümü (yeniden derleme gerekmez; AWSIM'i yeniden başlatmak yeter):
//   "Gokyuzu": { "Acik": true, "Ust": [r,g,b], "Orta": [r,g,b], "Alt": [r,g,b], "Yayilma": 1.6, "Pozlama": 13.0 }
//   Ust = tepe, Orta = ufuk, Alt = ufkun altı; Yayilma = renk geçişinin yumuşaklığı; Pozlama = parlaklık (EV, 1 artış = 2 kat).
// Gökyüzü sahnenin ortam ışığını da verir (gölgeler biraz maviye ve aydınlığa döner).
using System;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using Awsim.Common;

namespace Beytepe.Runtime
{
    [DefaultExecutionOrder(-50)]
    public class BeytepeGokyuzu : MonoBehaviour
    {
        [Serializable]
        public class Ayar
        {
            public bool Acik = true;
            public float[] Ust = { 0.22f, 0.42f, 0.82f };
            public float[] Orta = { 0.58f, 0.74f, 0.96f };
            public float[] Alt = { 0.78f, 0.82f, 0.86f };
            public float Yayilma = 1.6f;
            public float Pozlama = 13.0f;
        }
        [Serializable] class JsonRoot { public Ayar Gokyuzu = new Ayar(); }

        [Tooltip("JSON'da Gokyuzu bölümü yoksa kullanılan değerler")] public Ayar varsayilan = new Ayar();

        Volume _vol;
        VolumeProfile _prof;

        void Awake()
        {
            var a = Oku(out var src);
            if (!a.Acik) { Debug.Log($"[Beytepe] Gökyüzü: kapalı ({src}); sahnenin kendi gökyüzü kullanılıyor"); return; }
            _prof = ScriptableObject.CreateInstance<VolumeProfile>();
            _prof.name = "BeytepeGokyuzu (çalışma anı)";
            var ve = _prof.Add<VisualEnvironment>(false);
            ve.skyType.Override((int)SkyType.Gradient);
            ve.skyAmbientMode.Override(SkyAmbientMode.Dynamic);
            var gs = _prof.Add<GradientSky>(false);
            gs.top.Override(Renk(a.Ust, varsayilan.Ust));
            gs.middle.Override(Renk(a.Orta, varsayilan.Orta));
            gs.bottom.Override(Renk(a.Alt, varsayilan.Alt));
            gs.gradientDiffusion.Override(Mathf.Max(0.01f, a.Yayilma));
            gs.skyIntensityMode.Override(SkyIntensityMode.Exposure);
            gs.exposure.Override(a.Pozlama);
            gs.multiplier.Override(1f);
            _vol = gameObject.AddComponent<Volume>();
            _vol.isGlobal = true;
            _vol.priority = 100f;
            _vol.weight = 1f;
            _vol.sharedProfile = _prof;
            Debug.Log($"[Beytepe] Gökyüzü: gradyan, üst {Yaz(a.Ust)}, orta {Yaz(a.Orta)}, alt {Yaz(a.Alt)}, yayılma {a.Yayilma:F2}, pozlama {a.Pozlama:F1} EV ({src})");
        }

        void OnDestroy()
        {
            if (_prof != null) Destroy(_prof);
        }

        static Color Renk(float[] v, float[] yedek)
        {
            var x = (v != null && v.Length >= 3) ? v : yedek;
            return new Color(x[0], x[1], x[2], 1f);
        }

        static string Yaz(float[] v) => v != null && v.Length >= 3 ? $"({v[0]:F2}, {v[1]:F2}, {v[2]:F2})" : "(varsayılan)";

        Ayar Oku(out string src)
        {
            src = "varsayılan (json yok)";
            var path = CommandLineUtility.GetCommandLineArg("--json_path");
            if (string.IsNullOrEmpty(path)) path = Environment.GetEnvironmentVariable("BEYTEPE_SENSOR_JSON");
            if (string.IsNullOrEmpty(path) || !File.Exists(path)) return varsayilan;
            try
            {
                var text = File.ReadAllText(path);
                if (!text.Contains("\"Gokyuzu\"")) { src = path + " (Gokyuzu bölümü yok, varsayılanlar)"; return varsayilan; }
                var root = JsonUtility.FromJson<JsonRoot>(text);
                src = path + " (Gokyuzu bölümü)";
                return root?.Gokyuzu ?? varsayilan;
            }
            catch (Exception e)
            {
                Debug.LogError($"[Beytepe] Gökyüzü ayarı okunamadı ({path}): {e.Message}; varsayılanlar kullanılıyor");
                src = "okuma hatası, varsayılan";
                return varsayilan;
            }
        }
    }
}
