using System;
using System.IO;
using System.Text;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Places the four FR-02 view cameras (overall, sorting, FR5, dock) under ViewCameras, copying the scene's Main Camera
// settings. They are left inactive so the current view is unchanged; the camera switcher (TC-02) activates one at a time.
// Poses are planning values for the conveyor layout from ConveyorLayoutBuilder; the hall ceiling is at 8.83 m.
// Batch: Unity -batchmode -projectPath <project> -executeMethod ViewCameraBuilder.Build|Render [-viewOut <dir>]
public static class ViewCameraBuilder
{
    const string ScenePath = "Assets/Scenes/LogisticCenter_Automation.unity";
    const string RootName = "ViewCameras";

    static readonly (string name, Vector3 eye, Vector3 target, float fov)[] Views =
    {
        // Whole loop, lanes and docks diagonally from the open east side, under the ceiling trusses.
        ("Cam_Overview", new Vector3(10f, 8.3f, 10f), new Vector3(-12f, 1.5f, -2f), 65f),
        // West straight with the three flipper diverters, from inside the loop.
        ("Cam_Sorting", new Vector3(-4f, 6.5f, 4f), new Vector3(-11.5f, 3f, 6f), 55f),
        // Lane pickup ends at x -21.9 (z -0.36, 9.64, 12.00) where the FR5 cells go, looking west from the loop side.
        // Kept south of the hall column at (-14.2, 7.2), which otherwise blocks the right of the view.
        ("Cam_FR5", new Vector3(-12.5f, 5.5f, 2.5f), new Vector3(-21f, 1.2f, 6f), 65f),
        // Docks 12 and 13 on the west wall and 14 on the north wall.
        ("Cam_Dock", new Vector3(-15f, 6.5f, -6f), new Vector3(-26f, 1f, 9f), 60f),
    };

    [MenuItem("Tools/Logistics/Build View Cameras")]
    public static void Build()
    {
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var old = GameObject.Find(RootName);
        if (old != null) UnityEngine.Object.DestroyImmediate(old);
        var main = Camera.main;
        var root = new GameObject(RootName).transform;
        var log = new StringBuilder();
        foreach (var v in Views)
        {
            var go = new GameObject(v.name);
            go.transform.SetParent(root, false);
            go.transform.SetPositionAndRotation(v.eye, Quaternion.LookRotation(v.target - v.eye, Vector3.up));
            var cam = go.AddComponent<Camera>();
            if (main != null) cam.CopyFrom(main);
            go.transform.SetPositionAndRotation(v.eye, Quaternion.LookRotation(v.target - v.eye, Vector3.up));
            cam.fieldOfView = v.fov;
            cam.targetTexture = null;
            go.SetActive(false);
            log.AppendLine($"{v.name} pos={v.eye} target={v.target} rot={go.transform.eulerAngles:F1} fov={v.fov}");
        }
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Write("view_cameras.txt", log.ToString());
        if (Application.isBatchMode) EditorApplication.Exit(0);
    }

    // Renders each view camera to a PNG without saving the scene.
    public static void Render()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var root = GameObject.Find(RootName).transform;
        var rt = new RenderTexture(1600, 900, 24) { antiAliasing = 4 };
        foreach (Transform t in root)
        {
            t.gameObject.SetActive(true);
            var cam = t.GetComponent<Camera>();
            cam.targetTexture = rt;
            cam.Render();
            var prev = RenderTexture.active; RenderTexture.active = rt;
            var tex = new Texture2D(rt.width, rt.height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0); tex.Apply();
            RenderTexture.active = prev;
            File.WriteAllBytes(Path.Combine(OutDir(), t.name + ".png"), tex.EncodeToPNG());
            UnityEngine.Object.DestroyImmediate(tex);
            cam.targetTexture = null;
            t.gameObject.SetActive(false);
        }
        if (Application.isBatchMode) EditorApplication.Exit(0);
    }

    static void Write(string file, string text) => File.WriteAllText(Path.Combine(OutDir(), file), text);

    static string OutDir()
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++) if (a[i] == "-viewOut") { Directory.CreateDirectory(a[i + 1]); return a[i + 1]; }
        return Directory.GetCurrentDirectory();
    }
}
