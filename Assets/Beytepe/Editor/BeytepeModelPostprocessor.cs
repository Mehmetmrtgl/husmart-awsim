// Beytepe FBX içe aktarma ayarları.
// - isReadable: RGL (GPU LiDAR) mesh köşelerini okuyabilsin.
// - addCollider=false: çarpıştırıcılar BeytepeSceneBuilder'da yalnız zemin/yapı meshlerine eklenir
//   (14 bin ağaç örneğine collider eklemek fiziği gereksiz yavaşlatır).
// - bakeAxisConversion=false: eksen çevirimi kök nesnenin dönüşünde kalır; doğru açı
//   BeytepeSceneBuilder'da lanelet noktalarına ışın atılarak doğrulanır.
using UnityEditor;

namespace Beytepe.EditorTools
{
    public class BeytepeModelPostprocessor : AssetPostprocessor
    {
        void OnPreprocessModel()
        {
            if (!assetPath.StartsWith("Assets/Beytepe/Model/"))
                return;
            var importer = (ModelImporter)assetImporter;
            importer.isReadable = true;
            importer.addCollider = false;
            importer.bakeAxisConversion = false;
            importer.useFileScale = true;
            importer.globalScale = 1f;
            importer.importCameras = false;
            importer.importLights = false;
            importer.importAnimation = false;
            importer.animationType = ModelImporterAnimationType.None;
            importer.meshCompression = ModelImporterMeshCompression.Off;
            importer.indexFormat = ModelImporterIndexFormat.UInt32;   // büyük arazi meshleri için
        }

        // Gerçek veriden dünya (Assets/Beytepe/Model/Gercek, 2026-11-09): zemin karolarının ortofoto dokuları.
        // Clamp: karo kenarında karşı kenardan renk sızmasın (dikiş çizgisi olmasın).
        void OnPreprocessTexture()
        {
            if (!assetPath.StartsWith("Assets/Beytepe/Model/Gercek/"))
                return;
            var ti = (TextureImporter)assetImporter;
            ti.textureType = TextureImporterType.Default;
            ti.wrapMode = UnityEngine.TextureWrapMode.Clamp;
            ti.maxTextureSize = 2048;   // 50 m karo / 2048 px = 2.4 cm: şerit çizgileri keskin kalsın
            ti.sRGBTexture = true;
            ti.mipmapEnabled = true;
            ti.alphaSource = TextureImporterAlphaSource.None;
        }
    }
}
