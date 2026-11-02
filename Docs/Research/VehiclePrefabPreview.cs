// 별도 임시 Unity 프로젝트 전용 (Assets/Editor에 복사). 원본 프로젝트에서 실행하지 말 것:
// 그래픽 설정을 built-in으로 바꾸고 색 공간을 Linear로 설정한다.
// Unity -batchmode -projectPath <tmp> -executeMethod VehiclePrefabPreview.ValidateAndRender -vehiclePreviewOut <dir> -quit 은 쓰지 말고 -quit 없이 실행 (스스로 Exit).
using System;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;

public static class VehiclePrefabPreview
{
    const string PrefabPath = "Assets/LogisticsVehicles/WingBody5t_Coupang.prefab";
    const int W = 1400, H = 900;

    public static void ValidateAndRender()
    {
        var log = new StringBuilder();
        int failures = 0;
        try
        {
            string outDir = Arg("-vehiclePreviewOut") ?? Path.Combine(Directory.GetCurrentDirectory(), "VehiclePreview");
            Directory.CreateDirectory(outDir);

            GraphicsSettings.defaultRenderPipeline = null;
            QualitySettings.renderPipeline = null;
            PlayerSettings.colorSpace = ColorSpace.Linear;
            EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(PrefabPath);
            if (prefab == null) throw new Exception("prefab not found: " + PrefabPath);
            var go = (GameObject)UnityEngine.Object.Instantiate(prefab);
            go.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);

            var standard = Shader.Find("Standard");
            foreach (var mf in go.GetComponentsInChildren<MeshFilter>(true))
            {
                var mr = mf.GetComponent<MeshRenderer>();
                var mesh = mf.sharedMesh;
                string path = PathOf(mf.transform);
                int mats = mr != null ? mr.sharedMaterials.Length : 0;
                bool ok = mesh != null && mesh.vertexCount > 0 && mesh.subMeshCount == mats && mesh.normals.Length == mesh.vertexCount;
                log.AppendLine(string.Format("{0} {1}: mesh={2} verts={3} subMeshes={4} materials={5} normals={6}",
                    ok ? "OK  " : "FAIL", path, mesh ? mesh.name : "null", mesh ? mesh.vertexCount : 0,
                    mesh ? mesh.subMeshCount : 0, mats, mesh ? mesh.normals.Length : 0));
                if (!ok) failures++;
                if (mr == null) continue;
                // 원본 material asset은 건드리지 않고 메모리상 Standard 복제본만 할당
                var src = mr.sharedMaterials;
                var dst = new Material[src.Length];
                for (int i = 0; i < src.Length; i++)
                {
                    var m = new Material(standard) { hideFlags = HideFlags.DontSave };
                    if (src[i] != null)
                    {
                        m.color = src[i].HasProperty("_BaseColor") ? src[i].GetColor("_BaseColor") : src[i].HasProperty("_Color") ? src[i].color : Color.magenta;
                        if (src[i].HasProperty("_BaseMap")) m.mainTexture = src[i].GetTexture("_BaseMap");
                        m.SetFloat("_Metallic", src[i].HasProperty("_Metallic") ? src[i].GetFloat("_Metallic") : 0f);
                        m.SetFloat("_Glossiness", src[i].HasProperty("_Smoothness") ? src[i].GetFloat("_Smoothness") : 0.3f);
                    }
                    else { m.color = Color.magenta; failures++; log.AppendLine("FAIL null material slot " + i + " on " + path); }
                    dst[i] = m;
                }
                mr.sharedMaterials = dst;
            }

            RenderSettings.ambientMode = AmbientMode.Flat;
            RenderSettings.ambientLight = new Color(0.42f, 0.43f, 0.46f);
            RenderSettings.skybox = null;
            Light("Key", new Vector3(45f, 35f, 0f), 1.1f);
            Light("Fill", new Vector3(25f, -140f, 0f), 0.45f);

            Bounds b = new Bounds(go.transform.position, Vector3.zero);
            foreach (var r in go.GetComponentsInChildren<Renderer>(true)) b.Encapsulate(r.bounds);
            log.AppendLine("bounds center=" + b.center.ToString("F3") + " size=" + b.size.ToString("F3"));

            var cam = new GameObject("PreviewCamera").AddComponent<Camera>();
            cam.clearFlags = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0.36f, 0.38f, 0.41f);
            cam.fieldOfView = 30f; cam.nearClipPlane = 0.05f; cam.farClipPlane = 500f;
            var rt = new RenderTexture(W, H, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB) { antiAliasing = 8 };
            cam.targetTexture = rt;

            Shot(cam, rt, b, Vector3.forward, outDir, "front", log);
            Shot(cam, rt, b, Vector3.right, outDir, "side", log);
            Shot(cam, rt, b, new Vector3(8f, 5f, 12f), outDir, "oblique", log);
            Shot(cam, rt, b, new Vector3(-8f, 5f, -12f), outDir, "oblique_rear", log);

            cam.targetTexture = null; rt.Release();
            log.AppendLine("failures=" + failures);
            File.WriteAllText(Path.Combine(outDir, "preview_log.txt"), log.ToString());
            Debug.Log("[VehiclePrefabPreview]\n" + log);
        }
        catch (Exception e)
        {
            failures++;
            Debug.LogError("[VehiclePrefabPreview] " + e + "\n" + log);
        }
        EditorApplication.Exit(failures == 0 ? 0 : 1);
    }

    static void Shot(Camera cam, RenderTexture rt, Bounds b, Vector3 dir, string outDir, string name, StringBuilder log)
    {
        dir.Normalize();
        float radius = b.extents.magnitude;
        float vFov = cam.fieldOfView * Mathf.Deg2Rad * 0.5f;
        float hFov = Mathf.Atan(Mathf.Tan(vFov) * W / H);
        float dist = radius / Mathf.Sin(Mathf.Min(vFov, hFov)) * 1.02f;
        cam.transform.position = b.center + dir * dist;
        cam.transform.LookAt(b.center, Vector3.up);
        cam.Render();
        var prev = RenderTexture.active; RenderTexture.active = rt;
        var tex = new Texture2D(W, H, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, W, H), 0, 0); tex.Apply();
        RenderTexture.active = prev;
        string file = Path.Combine(outDir, name + ".png");
        File.WriteAllBytes(file, tex.EncodeToPNG());
        UnityEngine.Object.DestroyImmediate(tex);
        log.AppendLine("saved " + file + " camera=" + cam.transform.position.ToString("F2"));
    }

    static void Light(string name, Vector3 euler, float intensity)
    {
        var l = new GameObject(name).AddComponent<Light>();
        l.type = LightType.Directional; l.intensity = intensity; l.shadows = LightShadows.Soft;
        l.transform.rotation = Quaternion.Euler(euler);
    }

    static string PathOf(Transform t) => t.parent == null ? t.name : PathOf(t.parent) + "/" + t.name;

    static string Arg(string key)
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++) if (a[i] == key) return a[i + 1];
        return null;
    }
}
