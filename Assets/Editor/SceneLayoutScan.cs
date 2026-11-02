// 배치 모드 조회 전용: 씬을 저장하지 않고 최상위 오브젝트와 지정 영역의 렌더러 경계를 출력한다.
// Unity -batchmode -projectPath <project> -executeMethod SceneLayoutScan.Run -scanOut <file>
using System;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class SceneLayoutScan
{
    const string ScenePath = "Assets/Scenes/LogisticCenter_Automation.unity";

    public static void Run()
    {
        var log = new StringBuilder();
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        foreach (var root in scene.GetRootGameObjects())
        {
            var rs = root.GetComponentsInChildren<Renderer>();
            log.AppendLine($"ROOT {root.name} pos={root.transform.position:F2} children={root.transform.childCount} renderers={rs.Length} bounds={Union(rs)}");
            foreach (Transform c in root.transform)
            {
                var cr = c.GetComponentsInChildren<Renderer>();
                log.AppendLine($"  {c.name} pos={c.position:F2} renderers={cr.Length} bounds={Union(cr)}");
            }
        }
        // 도크 12·13·14 주변 영역의 개별 렌더러
        var area = new Bounds(new Vector3(-15f, 2f, 3f), new Vector3(40f, 6f, 44f));
        log.AppendLine("AREA " + area.min.ToString("F1") + " ~ " + area.max.ToString("F1"));
        foreach (var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None))
            if (r.bounds.Intersects(area) && r.bounds.size.magnitude > 0.3f)
                log.AppendLine($"  R {Path(r.transform)} min={r.bounds.min:F2} max={r.bounds.max:F2}");
        string outFile = Arg("-scanOut") ?? "scene_scan.txt";
        File.WriteAllText(outFile, log.ToString());
        EditorApplication.Exit(0);
    }

    // Lists every root and the full ConveyorSystem hierarchy with world poses, drive settings and colliders.
    // Unity -batchmode -projectPath <project> -executeMethod SceneLayoutScan.DumpConveyors -scanOut <file>
    public static void DumpConveyors()
    {
        var log = new StringBuilder();
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        foreach (var root in scene.GetRootGameObjects())
        {
            var rs = root.GetComponentsInChildren<Renderer>();
            log.AppendLine($"ROOT {root.name} active={root.activeSelf} pos={root.transform.position:F3} rot={root.transform.eulerAngles:F1} scale={root.transform.localScale:F3} bounds={Union(rs)}");
            if (root.name == "ConveyorSystem" || root.GetComponentInChildren<RoboLacyTech.Conveyors.LinearConveyor>(true) != null || root.GetComponentInChildren<RoboLacyTech.Conveyors.CurvedConveyor>(true) != null)
                DumpTree(root.transform, 1, log);
        }
        File.WriteAllText(Arg("-scanOut") ?? "conveyor_dump.txt", log.ToString());
        EditorApplication.Exit(0);
    }

    static void DumpTree(Transform t, int depth, StringBuilder log)
    {
        foreach (Transform c in t)
        {
            if (c.name == "Leg") continue;
            var sb = new StringBuilder($"{new string(' ', depth * 2)}{c.name}{(c.gameObject.activeInHierarchy ? "" : " [inactive]")} pos={c.position:F3} rot={c.eulerAngles:F1} lscale={c.localScale:F3}");
            foreach (var comp in c.GetComponents<Component>())
            {
                if (comp == null) { sb.Append(" MISSING_SCRIPT"); continue; }
                switch (comp)
                {
                    case Collider col:
                        sb.Append($" {col.GetType().Name}{(col.enabled ? "" : "(off)")}{(col.isTrigger ? "(trigger)" : "")}[{col.bounds.min:F2}..{col.bounds.max:F2}]");
                        break;
                    case MonoBehaviour mb when mb is RoboLacyTech.Conveyors.LinearConveyor || mb is RoboLacyTech.Conveyors.CurvedConveyor || mb.GetType().Namespace == "Logistics":
                        var so = new SerializedObject(mb);
                        sb.Append($" {mb.GetType().Name}{(mb.enabled ? "" : "(off)")}(");
                        var it = so.GetIterator();
                        for (bool enter = true; it.NextVisible(enter); enter = false)
                            if (it.name != "m_Script" && it.propertyType != SerializedPropertyType.Generic)
                                sb.Append($"{it.propertyPath}={Value(it)} ");
                            else if (it.name == "motor")
                                foreach (var p in new[] { "motor.baseSpeed", "motor.direction", "motor.startRunning" })
                                    sb.Append($"{p}={Value(so.FindProperty(p))} ");
                        sb.Append(")");
                        break;
                }
            }
            log.AppendLine(sb.ToString());
            if (c.childCount > 0 && !(PrefabUtility.IsAnyPrefabInstanceRoot(c.gameObject) && c.GetComponentsInChildren<Transform>().Length > 12 && c.GetComponent<Logistics.LaneDiverter>() == null && !c.name.StartsWith("Diverter") && !c.name.StartsWith("EntryMerge")))
                DumpTree(c, depth + 1, log);
        }
    }

    static string Value(SerializedProperty p) => p.propertyType switch
    {
        SerializedPropertyType.Float => p.floatValue.ToString("F3"),
        SerializedPropertyType.Integer => p.intValue.ToString(),
        SerializedPropertyType.Boolean => p.boolValue.ToString(),
        SerializedPropertyType.Enum => p.enumDisplayNames.Length > p.enumValueIndex && p.enumValueIndex >= 0 ? p.enumDisplayNames[p.enumValueIndex] : p.intValue.ToString(),
        SerializedPropertyType.String => p.stringValue,
        SerializedPropertyType.Vector3 => p.vector3Value.ToString("F3"),
        SerializedPropertyType.ObjectReference => p.objectReferenceValue ? p.objectReferenceValue.name : "null",
        _ => p.propertyType.ToString(),
    };

    // Top and oblique PNGs around the main loop, without saving the scene.
    // -scanOut <dir>; the view centres on ConveyorSystem/MainLoop renderers.
    public static void RenderLoop()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        string outDir = Arg("-scanOut") ?? ".";
        Directory.CreateDirectory(outDir);
        foreach (var r in UnityEngine.Object.FindObjectsByType<Renderer>(FindObjectsSortMode.None))
            if (r.bounds.min.y > 7f) r.enabled = false;   // ceiling
        QualitySettings.lodBias = 20f;   // keep small LOD meshes (turns) visible in the overview shots
        var loop = GameObject.Find("ConveyorSystem/MainLoop").GetComponentsInChildren<Renderer>();
        var b = loop[0].bounds;
        foreach (var r in loop) b.Encapsulate(r.bounds);
        var cam = new GameObject("ScanCamera").AddComponent<Camera>();
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = Color.black;
        var rt = new RenderTexture(1600, 1200, 24) { antiAliasing = 4 };
        cam.targetTexture = rt;
        cam.orthographic = true;
        cam.orthographicSize = Mathf.Max(b.extents.z, b.extents.x * 0.75f) + 4f;
        cam.transform.SetPositionAndRotation(b.center + Vector3.up * 30f, Quaternion.Euler(90f, 0f, 0f));
        Save(cam, rt, System.IO.Path.Combine(outDir, "loop_top.png"));
        cam.orthographic = false;
        cam.fieldOfView = 50f;
        // Inside the hall: from the north-east of the loop, then close on the lane mouths from inside the loop.
        Vector3 eye = b.center + new Vector3(b.extents.x * 0.6f, 6f, b.extents.z + 4f);
        cam.transform.SetPositionAndRotation(eye, Quaternion.LookRotation(b.center - eye));
        Save(cam, rt, System.IO.Path.Combine(outDir, "loop_oblique.png"));
        Vector3 mouth = new Vector3(b.min.x, b.max.y, b.center.z);
        eye = mouth + new Vector3(5f, 3f, 3f);
        cam.transform.SetPositionAndRotation(eye, Quaternion.LookRotation(mouth + new Vector3(-4f, -1.5f, 0f) - eye));
        Save(cam, rt, System.IO.Path.Combine(outDir, "loop_lanes.png"));
        File.WriteAllText(System.IO.Path.Combine(outDir, "loop_bounds.txt"), $"min={b.min:F2} max={b.max:F2}");
        EditorApplication.Exit(0);
    }

    // Static, Rotor and Mixed straight conveyors side by side in a new unsaved scene. -scanOut <dir>
    public static void RenderVariants()
    {
        EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
        string[] v = { "Static/ConveyorBeltStraight", "Rotor/ConveyorBeltStraightRotor", "Mixed/ConveyorBeltStraightMixed" };
        for (int i = 0; i < v.Length; i++)
        {
            var go = (GameObject)PrefabUtility.InstantiatePrefab(AssetDatabase.LoadAssetAtPath<GameObject>($"Assets/Conveyor Works/Prefabs/Conveyor/{v[i]}.prefab"));
            go.transform.position = new Vector3(i * 1.4f, 0f, 0f);
        }
        var cam = Camera.main;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.backgroundColor = new Color(0.35f, 0.37f, 0.4f);
        cam.fieldOfView = 50f;
        cam.transform.SetPositionAndRotation(new Vector3(1.4f, 1.5f, -1.9f), Quaternion.LookRotation(new Vector3(1.4f, 0.45f, -0.6f) - new Vector3(1.4f, 1.5f, -1.9f)));
        var rt = new RenderTexture(1400, 900, 24) { antiAliasing = 4 };
        cam.targetTexture = rt;
        string outDir = Arg("-scanOut") ?? ".";
        Directory.CreateDirectory(outDir);
        Save(cam, rt, System.IO.Path.Combine(outDir, "conveyor_variants.png"));
        EditorApplication.Exit(0);
    }

    static void Save(Camera cam, RenderTexture rt, string file)
    {
        cam.Render();
        var prev = RenderTexture.active; RenderTexture.active = rt;
        var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0); tex.Apply();
        RenderTexture.active = prev;
        File.WriteAllBytes(file, tex.EncodeToPNG());
        UnityEngine.Object.DestroyImmediate(tex);
    }

    static string Union(Renderer[] rs)
    {
        if (rs.Length == 0) return "-";
        var b = rs[0].bounds;
        foreach (var r in rs) b.Encapsulate(r.bounds);
        return $"min={b.min:F2} max={b.max:F2}";
    }

    static string Path(Transform t) => t.parent == null ? t.name : Path(t.parent) + "/" + t.name;

    static string Arg(string key)
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++) if (a[i] == key) return a[i + 1];
        return null;
    }
}
