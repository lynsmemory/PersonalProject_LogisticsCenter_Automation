using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using RoboLacyTech.Conveyors;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// Builds the raised loop conveyor in the factory centre, the upper-floor entry decline, three flipper diverters
// and three gravity lanes down to floor-level pickup (docks: gate 12, 13, 14).
// Plan values in metres (world, floor top Y=0), heights on round figures; retune here and rebuild.
// Rebuilding deletes and recreates ConveyorSystem, so edit these constants rather than the scene objects.
// Batch: Unity -batchmode -projectPath <project> -executeMethod ConveyorLayoutBuilder.Build|Simulate [-layoutOut <dir>]
public static class ConveyorLayoutBuilder
{
    const string ScenePath = "Assets/Scenes/LogisticCenter_Automation.unity";
    const string RootName = "ConveyorSystem";
    const string Rotor = "Assets/Conveyor Works/Prefabs/Conveyor/Rotor/";
    const string PhysicsFolder = "Assets/LogisticsLayout";
    const string LaneMaterialPath = PhysicsFolder + "/LaneRollerLowFriction.physicMaterial";
    const string GuideMaterialPath = PhysicsFolder + "/GuideNoFriction.physicMaterial";
    const string SteelMaterialPath = "Assets/LogisticsVehicles/Materials/Steel.mat";

    // Measured from the Rotor prefabs (pivot at bottom centre).
    const float BeltTop = 0.5f, StraightLen = 2.647f, ShortLen = 1.323f, LegHeight = 0.717f, BeltUnderside = 0.215f;
    const float BeltHalf = 0.437f;
    static readonly Vector3 TurnCentre = new Vector3(0.616f, 0f, 0.586f);
    const float TurnRadius = 0.83f;

    // Main loop: belt top 3.00 m, counter-clockwise seen from above (west -Z, south +X, east +Z, north -X).
    // Widened north-west so each lane ends in front of its dock; the north straight (z 14.63) stays clear of the
    // stair platform at z >= 15.6 and both hall columns, (-14.2, 7.2) and (-1.0, 6.1), stay off the belts.
    const float MainTop = 3.00f, Lift = MainTop - BeltTop;
    const float WestX = -11.50f, EastX = 2.97f, SouthZ = -17.70f, NorthZ = 13.80f;
    const float BeltSpeed = 0.5f, ReadZ = -5.0f;
    // Entry: upper floor 5.00 m, powered 20 degree decline onto the merge branch on the inner side of the east straight.
    const float EntryZ = -11.64f, UpperTop = 5.00f, EntrySlopeDeg = 20f, EntryApron = 0.45f;
    // Lanes leave the west straight toward -X. Section top heights: 3.00 > 2.90 (4 deg, middle stopper) > 1.00 (15 deg) > 0.90 (4 deg, pickup).
    const float LaneEaseDeg = 4f, LaneSteepDeg = 15f;
    static readonly float[] LaneTops = { MainTop, 2.90f, 1.00f, 0.90f };
    static readonly float[] LaneSlopes = { LaneEaseDeg, LaneSteepDeg, LaneEaseDeg };
    const float BrakeSpeed = 0.6f;
    // Lanes 1 and 2 on the lines of docks 12 and 13 (west wall), lane 3 just south of dock 14 (north wall, x -21.66).
    // Pickup ends at x = -21.9: about 7.3 m short of docks 12/13 and 4.7 m south of dock 14 for FR5, pallet and AMR.
    static readonly (string name, string id, float z, string gate)[] Lanes =
    {
        ("Lane1", "L01", -0.36f, "Gate_12"), ("Lane2", "L02", 9.64f, "Gate_13"), ("Lane3", "L03", 12.00f, "Gate_14"),
    };
    // Flipper: hinge on the east guide of the west straight, HingeUpstream ahead of the lane centre; at 55 degrees the tip
    // reaches 0.36 m past the belt edge into the lane mouth (a tip stopping at the edge left boxes half on the belt).
    // The west guide is open over the whole arm span plus 0.30 m upstream, where boxes reach the edge, and the
    // lane's upstream guide funnels from that opening into the 0.80 m lane.
    const float ArmLength = 1.46f, ArmAngle = 55f, HingeUpstream = 0.30f, MouthUpstream = 0.30f, FunnelLength = 0.60f;
    const float GuideOffset = 0.40f, GuideHeight = 0.12f;   // 0.35 m box between guides 0.80 m apart
    const float LaneHalf = 0.44f, TransferStep = 0.01f;
    // Lane walls stand 50 mm clear of the floor; lane kinks are rounded with this radius (boxes stopped dead at sharp kinks).
    const float WallClearance = 0.05f, KinkRadius = 1.0f;

    static Transform root;
    static PhysicsMaterial laneMaterial, guideMaterial;
    static Material steel;

    [MenuItem("Tools/Logistics/Build Conveyor Layout")]
    public static void Build()
    {
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var old = GameObject.Find(RootName);
        if (old != null) UnityEngine.Object.DestroyImmediate(old);
        // Lane: free-running rollers, below tan(4 deg) = 0.07 so boxes start by gravity. Guides: roller side rails.
        laneMaterial = PhysicsMat(LaneMaterialPath, "LaneRollerLowFriction", 0.03f);
        guideMaterial = PhysicsMat(GuideMaterialPath, "GuideNoFriction", 0f);
        steel = AssetDatabase.LoadAssetAtPath<Material>(SteelMaterialPath);
        root = new GameObject(RootName).transform;

        var main = Group("MainLoop");
        WestStraight(main);
        FillStraight(main, "South", new Vector3(WestX + TurnRadius, 0f, SouthZ - TurnRadius), new Vector3(EastX - TurnRadius, 0f, SouthZ - TurnRadius));
        EastStraight(main);
        FillStraight(main, "North", new Vector3(EastX - TurnRadius, 0f, NorthZ + TurnRadius), new Vector3(WestX + TurnRadius, 0f, NorthZ + TurnRadius));
        Vector3 sw = new Vector3(WestX + TurnRadius, Lift, SouthZ), se = new Vector3(EastX - TurnRadius, Lift, SouthZ);
        Vector3 ne = new Vector3(EastX - TurnRadius, Lift, NorthZ), nw = new Vector3(WestX + TurnRadius, Lift, NorthZ);
        Turn(main, "TurnSW", sw, 0f);
        Turn(main, "TurnSE", se, -90f);
        Turn(main, "TurnNE", ne, 180f);
        Turn(main, "TurnNW", nw, 90f);

        var guides = Group("Guides");
        var laneGaps = new List<(float, float)>();
        foreach (var lane in Lanes) laneGaps.Add(MouthSpan(lane.z));
        StraightGuides(guides, "West", true, WestX, SouthZ, NorthZ, null, laneGaps);
        // The diagonal branch carries entry boxes north while they cross onto the band, so the opening runs 0.90 m downstream.
        StraightGuides(guides, "East", true, EastX, SouthZ, NorthZ, null, new List<(float, float)> { MergeSpan });
        StraightGuides(guides, "South", false, SouthZ - TurnRadius, WestX + TurnRadius, EastX - TurnRadius, null, null);
        StraightGuides(guides, "North", false, NorthZ + TurnRadius, WestX + TurnRadius, EastX - TurnRadius, null, null);
        TurnGuides(guides, "TurnSW", sw, 180f, 270f);
        TurnGuides(guides, "TurnSE", se, 270f, 360f);
        TurnGuides(guides, "TurnNE", ne, 0f, 90f);
        TurnGuides(guides, "TurnNW", nw, 90f, 180f);

        BuildEntry();

        var markers = Group("Markers");
        Marker(markers, "BarcodeReadPoint", new Vector3(EastX, MainTop, ReadZ));

        foreach (var lane in Lanes)
        {
            var group = Group(lane.name + "_" + lane.gate);
            Flipper(group, lane.name, lane.id, lane.z);
            BuildLane(group, lane.name, lane.z);
        }

        // The warehouse floor mesh has no collider; this catches dropped boxes and later carries the AMRs.
        var floor = new GameObject("FloorCollider").AddComponent<BoxCollider>();
        floor.transform.SetParent(root, false);
        floor.center = new Vector3(-5.26f, -0.05f, -0.39f);
        floor.size = new Vector3(48f, 0.1f, 74.4f);

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Report("build", Summary());
        if (Application.isBatchMode) EditorApplication.Exit(0);
    }

    // ---------- main loop pieces ----------

    // Straights from a to b (flow a -> b) at the main height, scaled to fill the run exactly.
    static void FillStraight(Transform parent, string side, Vector3 a, Vector3 b)
    {
        a.y = b.y = Lift;
        float len = Vector3.Distance(a, b);
        if (len < 0.05f) return;
        bool shortPiece = len < 0.75f * StraightLen;
        float unit = shortPiece ? ShortLen : StraightLen;
        int n = Mathf.Max(1, Mathf.RoundToInt(len / unit));
        var rot = Quaternion.LookRotation(b - a, Vector3.up);
        for (int i = 0; i < n; i++)
        {
            Vector3 pos = Vector3.Lerp(a, b, (i + 0.5f) / n);
            var go = Place(Rotor + (shortPiece ? "ConveyorBeltStraightShortRotor.prefab" : "ConveyorBeltStraightRotor.prefab"), parent, $"{side}_{parent.childCount + 1:00}", pos, rot, len / n / unit);
            Linear(go, ConveyorAxis.Forward);
            Leg(parent, pos, MainTop, rot.eulerAngles.y);
        }
    }

    // Rollers stay as visuals; boxes ride drive surfaces that stop short of each flipper section,
    // which the LaneDiverter drives itself.
    static void WestStraight(Transform parent)
    {
        int first = parent.childCount;
        FillStraight(parent, "West", new Vector3(WestX, 0f, NorthZ), new Vector3(WestX, 0f, SouthZ));
        for (int i = first; i < parent.childCount; i++)
            if (parent.GetChild(i).name.StartsWith("West"))
                foreach (var c in parent.GetChild(i).GetComponentsInChildren<Collider>()) c.enabled = false;
        var spans = new List<(float, float)>();
        foreach (var lane in Lanes) spans.Add(SectionSpan(lane.z));
        spans.Sort((a, b) => b.Item2.CompareTo(a.Item2));
        float from = NorthZ;
        int n = 0;
        foreach (var span in spans)
        {
            WestBand(parent, ++n, from, span.Item2);
            from = span.Item1;
        }
        WestBand(parent, ++n, from, SouthZ);
    }

    static void WestBand(Transform parent, int index, float fromZ, float toZ)
    {
        if (fromZ - toZ < 0.02f) return;
        DriveSurface(parent, $"WestBand_{index}", new Vector3(WestX, MainTop, (fromZ + toZ) * 0.5f), new Vector3(2f * BeltHalf, 0.1f, fromZ - toZ), 180f);
    }

    // Belt under a flipper arm: from just downstream of the tip to just upstream of the hinge.
    static (float, float) SectionSpan(float laneZ)
    {
        float hingeZ = laneZ + HingeUpstream, tipZ = hingeZ - ArmLength * Mathf.Cos(ArmAngle * Mathf.Deg2Rad);
        return (tipZ - 0.1f, hingeZ + 0.1f);
    }

    // Entry joins the east straight from the loop interior side, like a lane leaving the west straight: the rollers
    // run straight through, and boxes ride drive surfaces with a merge section at the entry mouth that drives
    // diagonally (+X+Z), carrying entry boxes onto the belt; loop boxes pass along the east guide.
    // (The V90 T-piece used here before read as a branch back toward the entry.)
    static readonly (float, float) MergeSpan = (EntryZ - LaneHalf, EntryZ + 0.90f);

    static void EastStraight(Transform parent)
    {
        int first = parent.childCount;
        FillStraight(parent, "East", new Vector3(EastX, 0f, SouthZ), new Vector3(EastX, 0f, NorthZ));
        for (int i = first; i < parent.childCount; i++)
            if (parent.GetChild(i).name.StartsWith("East"))
                foreach (var c in parent.GetChild(i).GetComponentsInChildren<Collider>()) c.enabled = false;
        float lo = MergeSpan.Item1, hi = MergeSpan.Item2;
        DriveSurface(parent, "EastBand_1", new Vector3(EastX, MainTop, (SouthZ + lo) * 0.5f), new Vector3(2f * BeltHalf, 0.1f, lo - SouthZ), 0f);
        DriveSurface(parent, "EastBand_2", new Vector3(EastX, MainTop, (hi + NorthZ) * 0.5f), new Vector3(2f * BeltHalf, 0.1f, NorthZ - hi), 0f);
        // Drive at 45 degrees on a body whose axis-aligned child collider is the belt footprint.
        var merge = DriveSurface(parent, "EntryMergeSection", new Vector3(EastX, MainTop, (lo + hi) * 0.5f), new Vector3(2f * BeltHalf, 0.1f, hi - lo), 45f);
        var box = merge.GetComponent<BoxCollider>();
        var footprint = new GameObject("Footprint").AddComponent<BoxCollider>();
        footprint.transform.SetParent(merge.transform, false);
        footprint.transform.SetPositionAndRotation(merge.transform.position, Quaternion.identity);
        footprint.center = box.center;
        footprint.size = box.size;
        UnityEngine.Object.DestroyImmediate(box);
    }

    static void Turn(Transform parent, string name, Vector3 centre, float yaw)
    {
        // CurvedConveyor rotates its Rigidbody about the pivot, so the pivot sits on the arc centre.
        var pivot = new GameObject(name).transform;
        pivot.SetParent(parent, false);
        pivot.SetPositionAndRotation(centre, Quaternion.Euler(0f, yaw, 0f));
        var model = Place(Rotor + "Conveyor90DegreeTurnRotor.prefab", pivot, name + "_Model", Vector3.zero, Quaternion.identity, 1f);
        model.transform.SetLocalPositionAndRotation(-TurnCentre, Quaternion.identity);
        var curved = pivot.gameObject.AddComponent<CurvedConveyor>();
        Kinematic(pivot.gameObject);
        var so = new SerializedObject(curved);
        so.FindProperty("motor.baseSpeed").floatValue = BeltSpeed;
        // Forward turns counter-clockwise seen from above (measured; the package moves surfaces against its +angle).
        so.FindProperty("motor.direction").intValue = (int)ConveyorDirection.Forward;
        so.FindProperty("curveAxis").enumValueIndex = (int)ConveyorAxis.Up;
        so.FindProperty("radius").floatValue = TurnRadius;
        so.FindProperty("curveAngle").floatValue = 90f;
        so.ApplyModifiedPropertiesWithoutUndo();
        Vector3 mid = centre + Quaternion.Euler(0f, yaw, 0f) * new Vector3(-TurnRadius, 0f, -TurnRadius).normalized * TurnRadius;
        Leg(parent, mid, MainTop, yaw + 45f);
    }

    // ---------- entry ----------

    // Upper landing straight at 5.00 m flowing +X, a powered 20 degree decline, then a short flat apron at 3.01 m
    // (10 mm transfer step) that pushes boxes to the edge of the main belt.
    static void BuildEntry()
    {
        var entry = Group("EntryLine");
        float slope = EntrySlopeDeg * Mathf.Deg2Rad;
        Vector3 edge = new Vector3(EastX - BeltHalf, MainTop + TransferStep, EntryZ);
        Vector3 bottom = edge + new Vector3(-EntryApron, 0f, 0f);
        float rise = UpperTop - bottom.y;
        float run = rise / Mathf.Tan(slope), along = rise / Mathf.Sin(slope);
        Vector3 top = bottom + new Vector3(-run, rise, 0f);
        var apron = Place(Rotor + "ConveyorBeltStraightShortRotor.prefab", entry, "EntryApron", (bottom + edge) * 0.5f - Vector3.up * BeltTop, Quaternion.Euler(0f, 90f, 0f), EntryApron / ShortLen);
        Linear(apron, ConveyorAxis.Forward);
        Leg(entry, (bottom + edge) * 0.5f, edge.y, 90f);
        var rot = Quaternion.Euler(EntrySlopeDeg, 90f, 0f);   // forward = downhill toward +X
        int n = Mathf.Max(1, Mathf.RoundToInt(along / StraightLen));
        for (int i = 0; i < n; i++)
        {
            Vector3 topCentre = Vector3.Lerp(top, bottom, (i + 0.5f) / n);
            var go = Place(Rotor + "ConveyorBeltStraightRotor.prefab", entry, "EntryDecline_" + (i + 1), topCentre - rot * new Vector3(0f, BeltTop, 0f), rot, along / n / StraightLen);
            Linear(go, ConveyorAxis.Forward);
            Leg(entry, topCentre, topCentre.y, 90f);
        }
        Vector3 landing = top + new Vector3(-StraightLen * 0.5f, 0f, 0f);
        var upper = Place(Rotor + "ConveyorBeltStraightRotor.prefab", entry, "UpperLanding", new Vector3(landing.x, UpperTop - BeltTop, EntryZ), Quaternion.Euler(0f, 90f, 0f), 1f);
        Linear(upper, ConveyorAxis.Forward);
        Leg(entry, landing, UpperTop, 90f);
        foreach (float side in new[] { -GuideOffset, GuideOffset })
        {
            Guide(entry, "Entry_Guide", top + new Vector3(-StraightLen, 0f, side), top + new Vector3(0f, 0f, side));
            Guide(entry, "Entry_Guide", top + new Vector3(0f, 0f, side), bottom + new Vector3(0f, 0f, side));
            Guide(entry, "Entry_Guide", bottom + new Vector3(0f, 0f, side), edge + new Vector3(0f, 0f, side));
        }
        // Spawn position is decided later; this marks the start of the upper landing.
        Marker(entry, "UpperEntryStart", top + new Vector3(-StraightLen, 0f, 0f));
    }

    // ---------- diverter and lanes ----------

    static void Flipper(Transform parent, string laneName, string laneId, float laneZ)
    {
        float along = ArmLength * Mathf.Cos(ArmAngle * Mathf.Deg2Rad);
        // Local frame: +Z with the west straight flow (-Z world), +X toward the lane (-X world).
        var hinge = new GameObject(laneName + "_Flipper").transform;
        hinge.SetParent(parent, false);
        hinge.SetPositionAndRotation(new Vector3(WestX + GuideOffset, MainTop, laneZ + HingeUpstream), Quaternion.Euler(0f, 180f, 0f));

        var armGo = new GameObject("Arm");
        armGo.transform.SetParent(hinge, false);
        var armBody = armGo.AddComponent<Rigidbody>();
        armBody.isKinematic = true;
        armBody.useGravity = false;
        var box = armGo.AddComponent<BoxCollider>();
        box.center = new Vector3(0f, GuideHeight * 0.5f + 0.01f, ArmLength * 0.5f);
        box.size = new Vector3(0.04f, GuideHeight, ArmLength);
        box.sharedMaterial = guideMaterial;
        var visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        UnityEngine.Object.DestroyImmediate(visual.GetComponent<Collider>());
        visual.name = "ArmVisual";
        visual.transform.SetParent(armGo.transform, false);
        visual.transform.localPosition = box.center;
        visual.transform.localScale = box.size;
        if (steel != null) visual.GetComponent<MeshRenderer>().sharedMaterial = steel;

        var span = SectionSpan(laneZ);
        var sectionGo = new GameObject("Section");
        sectionGo.transform.SetParent(hinge, false);
        sectionGo.transform.SetPositionAndRotation(new Vector3(WestX, MainTop, (span.Item1 + span.Item2) * 0.5f), Quaternion.identity);
        var sectionBody = sectionGo.AddComponent<Rigidbody>();
        sectionBody.isKinematic = true;
        sectionBody.useGravity = false;
        var sectionBox = sectionGo.AddComponent<BoxCollider>();
        sectionBox.center = new Vector3(0f, -0.05f, 0f);
        sectionBox.size = new Vector3(2f * BeltHalf, 0.1f, span.Item2 - span.Item1);

        var diverter = hinge.gameObject.AddComponent<Logistics.LaneDiverter>();
        var so = new SerializedObject(diverter);
        so.FindProperty("laneId").stringValue = laneId;
        so.FindProperty("arm").objectReferenceValue = armBody;
        so.FindProperty("section").objectReferenceValue = sectionBody;
        so.FindProperty("speed").floatValue = BeltSpeed;
        so.FindProperty("armLength").floatValue = ArmLength;
        so.FindProperty("extendedAngle").floatValue = ArmAngle;
        so.FindProperty("beltCentreX").floatValue = GuideOffset;
        so.FindProperty("beltHalfWidth").floatValue = BeltHalf;
        // The box slides along the arm at about belt speed x cos(45) = 0.35 m/s, so 1.7 m plus the hand-off takes ~5-6 s.
        so.FindProperty("divertTimeout").floatValue = 10f;
        // A follower 1.5 m back reaches the section only after the ~3.3 s divert and 0.3 s retract at 0.5 m/s.
        so.FindProperty("followerClearance").floatValue = 1.5f;
        so.ApplyModifiedPropertiesWithoutUndo();
        Marker(parent, "DiverterPoint", new Vector3(WestX, MainTop, laneZ));
    }

    // Lane from the west belt edge toward -X: ease (middle stopper), steep with brake rollers, pickup bed with end stop.
    static void BuildLane(Transform lane, string name, float z)
    {
        // 10 mm transfer step below the belt: a lane flush with the belt caught boxes on the seam.
        Vector3 start = new Vector3(WestX - BeltHalf, MainTop - TransferStep, z);
        string[] sections = { "Ease", "Steep", "Pickup" };
        var profile = new List<Vector3> { start };
        var upper = new List<Vector3>();
        for (int s = 0; s < 3; s++)
        {
            float slope = LaneSlopes[s] * Mathf.Deg2Rad, drop = start.y - LaneTops[s + 1];
            float run = drop / Mathf.Tan(slope), along = drop / Mathf.Sin(slope);
            Vector3 end = start + new Vector3(-run, -drop, 0f);
            var rot = Quaternion.Euler(LaneSlopes[s], -90f, 0f);   // forward = downhill toward -X
            // Visual rollers only; the lane mesh below carries the boxes.
            int n = Mathf.Max(1, Mathf.RoundToInt(along / StraightLen));
            for (int i = 0; i < n; i++)
            {
                Vector3 topCentre = Vector3.Lerp(start, end, (i + 0.5f) / n);
                var go = Place(Rotor + "ConveyorBeltStraightRotor.prefab", lane, $"{name}_{sections[s]}_{i + 1}", topCentre - rot * new Vector3(0f, BeltTop, 0f), rot, along / n / StraightLen);
                foreach (var c in go.GetComponentsInChildren<Collider>()) c.enabled = false;
                Leg(lane, topCentre, topCentre.y, -90f);
            }
            if (s > 0)
            {
                // Brake region only (trigger); the lane mesh carries the boxes.
                var zone = new GameObject($"{name}_{sections[s]}Brake").AddComponent<BoxCollider>();
                zone.transform.SetParent(lane, false);
                zone.transform.SetPositionAndRotation((start + end) * 0.5f, rot);
                zone.center = new Vector3(0f, -0.05f, 0f);
                zone.size = new Vector3(2f * BeltHalf, 0.1f, along);
                zone.isTrigger = true;
                var brake = zone.gameObject.AddComponent<Logistics.BrakeRollerZone>();
                var so = new SerializedObject(brake);
                so.FindProperty("maxSpeed").floatValue = BrakeSpeed;
                so.ApplyModifiedPropertiesWithoutUndo();
            }
            profile.Add(end);
            if (s == 0)
            {
                // Upstream side funnels from the wide mouth opening into the lane width.
                Vector3 mouth = new Vector3(start.x, start.y, MouthSpan(z).Item2);
                Vector3 narrow = Vector3.Lerp(start, end, FunnelLength / run) + new Vector3(0f, 0f, GuideOffset);
                upper.Add(mouth);
                upper.Add(narrow);
                Marker(lane, "MiddleStopperPoint", end);
            }
            upper.Add(end + new Vector3(0f, 0f, GuideOffset));
            if (s == 2)
            {
                Guide(lane, name + "_EndStop", end + new Vector3(0f, 0f, -GuideOffset), end + new Vector3(0f, 0f, GuideOffset));
                Marker(lane, "PickupPoint", end + new Vector3(0.3f, 0f, 0f));
            }
            start = end;
        }
        // One continuous surface over all three sections: separate box colliders met at the kinks with end faces
        // inside the contact offset, which stopped boxes dead.
        profile = Rounded(profile, KinkRadius, 6);
        var surface = new GameObject(name + "_Surface").AddComponent<MeshCollider>();
        surface.transform.SetParent(lane, false);
        surface.transform.position = profile[0];
        surface.sharedMesh = LaneMesh(profile);
        surface.sharedMaterial = laneMaterial;
        // Side guides as continuous walls along the profile: box guides per section left end faces at the kinks
        // within contact range of a box riding the guide, which stopped it the same way.
        var lower = new List<Vector3>();
        foreach (var p in profile) lower.Add(p + new Vector3(0f, 0f, -GuideOffset));
        Wall(lane, name + "_GuideDownstream", "LaneGuideDownstream", lower, profile[0], true);
        Wall(lane, name + "_GuideUpstream", "LaneGuideUpstream", Rounded(upper, KinkRadius, 6), profile[0], false);
    }

    // Frictionless vertical wall GuideHeight tall standing on the polyline (running toward -X), facing +Z or -Z only:
    // coincident front and back triangles gave contacts pointing into the wall and stopped boxes riding it.
    static void Wall(Transform parent, string name, string meshName, List<Vector3> line, Vector3 origin, bool facePlusZ)
    {
        string path = PhysicsFolder + "/" + meshName + ".asset";
        var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
        bool created = mesh == null;
        if (created) mesh = new Mesh { name = meshName };
        mesh.Clear();
        var verts = new List<Vector3>();
        var tris = new List<int>();
        foreach (var p in line)
        {
            verts.Add(p - origin + Vector3.up * WallClearance);
            verts.Add(p - origin + Vector3.up * (WallClearance + GuideHeight));
        }
        for (int i = 0; i + 1 < line.Count; i++)
        {
            int a = 2 * i;
            if (facePlusZ) tris.AddRange(new[] { a, a + 1, a + 2, a + 1, a + 3, a + 2 });
            else tris.AddRange(new[] { a, a + 2, a + 1, a + 1, a + 2, a + 3 });
        }
        mesh.SetVertices(verts);
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        if (created) AssetDatabase.CreateAsset(mesh, path);
        else EditorUtility.SetDirty(mesh);
        AssetDatabase.SaveAssets();
        var wall = new GameObject(name).AddComponent<MeshCollider>();
        wall.transform.SetParent(parent, false);
        wall.transform.position = origin;
        wall.sharedMesh = mesh;
        wall.sharedMaterial = guideMaterial;
    }

    // Replaces each interior corner of a polyline (in the XY plane) by an arc of the given radius, keeping the end points.
    static List<Vector3> Rounded(List<Vector3> pts, float radius, int steps)
    {
        var result = new List<Vector3> { pts[0] };
        for (int i = 1; i + 1 < pts.Count; i++)
        {
            Vector3 a = (pts[i - 1] - pts[i]).normalized, b = (pts[i + 1] - pts[i]).normalized;
            float half = Vector3.Angle(a, b) * 0.5f * Mathf.Deg2Rad;
            float cut = radius / Mathf.Tan(half);
            cut = Mathf.Min(cut, 0.45f * (pts[i] - pts[i - 1]).magnitude, 0.45f * (pts[i + 1] - pts[i]).magnitude);
            Vector3 p0 = pts[i] + a * cut, p1 = pts[i] + b * cut;
            // Quadratic Bezier through the corner: tangent to both segments, close to a circular arc for small angles.
            for (int k = 0; k <= steps; k++)
            {
                float t = k / (float)steps;
                result.Add((1 - t) * (1 - t) * p0 + 2 * (1 - t) * t * pts[i] + t * t * p1);
            }
        }
        result.Add(pts[pts.Count - 1]);
        return result;
    }

    // Top surface strip through the lane profile, in coordinates relative to its first point (same for every lane).
    static Mesh LaneMesh(List<Vector3> profile)
    {
        const string path = PhysicsFolder + "/LaneSurface.asset";
        var mesh = AssetDatabase.LoadAssetAtPath<Mesh>(path);
        bool created = mesh == null;
        if (created) mesh = new Mesh { name = "LaneSurface" };
        mesh.Clear();
        var verts = new List<Vector3>();
        var tris = new List<int>();
        foreach (var p in profile)
        {
            Vector3 local = p - profile[0];
            verts.Add(local + new Vector3(0f, 0f, -BeltHalf));
            verts.Add(local + new Vector3(0f, 0f, BeltHalf));
        }
        for (int i = 0; i + 1 < profile.Count; i++)
        {
            int a = 2 * i;
            // Wound so the normal faces up for a profile running toward -X.
            tris.AddRange(new[] { a, a + 2, a + 1, a + 1, a + 2, a + 3 });
        }
        mesh.SetVertices(verts);
        mesh.SetTriangles(tris, 0);
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        if (created)
        {
            Directory.CreateDirectory(PhysicsFolder);
            AssetDatabase.CreateAsset(mesh, path);
        }
        else EditorUtility.SetDirty(mesh);
        AssetDatabase.SaveAssets();
        return mesh;
    }

    // ---------- guides ----------

    // Opening in the west guide at a lane: from just downstream of the arm tip to MouthUpstream past the hinge.
    static (float, float) MouthSpan(float laneZ)
    {
        float hingeZ = laneZ + HingeUpstream, tipZ = hingeZ - ArmLength * Mathf.Cos(ArmAngle * Mathf.Deg2Rad);
        return (Mathf.Min(tipZ - 0.05f, laneZ - LaneHalf), hingeZ + MouthUpstream);
    }

    // Invisible rail standing on the belt plane between two points on it.
    static void Guide(Transform parent, string name, Vector3 from, Vector3 to)
    {
        var box = new GameObject(name).AddComponent<BoxCollider>();
        box.transform.SetParent(parent, false);
        Vector3 run = to - from;
        Vector3 side = Vector3.Cross(Vector3.up, run).normalized;
        box.transform.SetPositionAndRotation((from + to) * 0.5f, Quaternion.LookRotation(run, Vector3.Cross(run, side)));
        box.center = new Vector3(0f, GuideHeight * 0.5f + 0.01f, 0f);
        box.size = new Vector3(0.03f, GuideHeight, run.magnitude + 0.03f);
        box.sharedMaterial = guideMaterial;
    }

    // Guides on both sides of a straight at fixed x (alongZ) or fixed z, from lo to hi, with gaps (lo, hi) on the -side or +side.
    static void StraightGuides(Transform parent, string name, bool alongZ, float fixedCoord, float lo, float hi, List<(float, float)> plusGaps, List<(float, float)> minusGaps)
    {
        foreach (float side in new[] { -1f, 1f })
        {
            var gaps = new List<(float, float)>((side > 0 ? plusGaps : minusGaps) ?? new List<(float, float)>());
            gaps.Sort((a, b) => a.Item1.CompareTo(b.Item1));
            float from = lo;
            foreach (var g in gaps)
            {
                Rail(parent, name, alongZ, fixedCoord + side * GuideOffset, from, g.Item1);
                from = g.Item2;
            }
            Rail(parent, name, alongZ, fixedCoord + side * GuideOffset, from, hi);
        }
    }

    static void Rail(Transform parent, string name, bool alongZ, float at, float a, float b)
    {
        if (b - a < 0.02f) return;
        Vector3 p0 = alongZ ? new Vector3(at, MainTop, a) : new Vector3(a, MainTop, at);
        Vector3 p1 = alongZ ? new Vector3(at, MainTop, b) : new Vector3(b, MainTop, at);
        Guide(parent, name + "_Guide", p0, p1);
    }

    static void TurnGuides(Transform parent, string name, Vector3 centre, float fromDeg, float toDeg)
    {
        const int segments = 8;
        foreach (float r in new[] { TurnRadius - GuideOffset, TurnRadius + GuideOffset })
            for (int i = 0; i < segments; i++)
            {
                float a0 = Mathf.Lerp(fromDeg, toDeg, i / (float)segments) * Mathf.Deg2Rad, a1 = Mathf.Lerp(fromDeg, toDeg, (i + 1) / (float)segments) * Mathf.Deg2Rad;
                Guide(parent, name + "_Guide", new Vector3(centre.x + r * Mathf.Cos(a0), MainTop, centre.z + r * Mathf.Sin(a0)),
                                               new Vector3(centre.x + r * Mathf.Cos(a1), MainTop, centre.z + r * Mathf.Sin(a1)));
            }
    }

    // ---------- helpers ----------

    // Invisible flat belt: top face at topCentre, width along local X, length along local Z (the flow direction).
    static GameObject DriveSurface(Transform parent, string name, Vector3 topCentre, Vector3 size, float yaw)
    {
        var go = new GameObject(name);
        go.transform.SetParent(parent, true);
        go.transform.SetPositionAndRotation(topCentre, Quaternion.Euler(0f, yaw, 0f));
        go.transform.localScale = Vector3.one;
        var box = go.AddComponent<BoxCollider>();
        box.center = new Vector3(0f, -size.y * 0.5f, 0f);
        box.size = size;
        Linear(go, ConveyorAxis.Forward);
        return go;
    }

    static void Linear(GameObject go, ConveyorAxis axis)
    {
        var linear = go.AddComponent<LinearConveyor>();
        Kinematic(go);
        var so = new SerializedObject(linear);
        so.FindProperty("motor.baseSpeed").floatValue = BeltSpeed;
        // LinearConveyor moves contacts opposite to its axis (teleport forward, MovePosition back), so Backward carries along the axis.
        so.FindProperty("motor.direction").intValue = (int)ConveyorDirection.Backward;
        so.FindProperty("movementAxis").enumValueIndex = (int)axis;
        so.ApplyModifiedPropertiesWithoutUndo();
    }

    // The conveyors set this in Awake too, but a non-kinematic body in the saved scene disables the non-convex MeshColliders.
    static void Kinematic(GameObject go)
    {
        var rb = go.GetComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;
    }

    // Leg from the floor to the belt underside of a section whose top is at beltTop.
    static void Leg(Transform parent, Vector3 at, float beltTop, float yaw)
    {
        var go = Place("Assets/Conveyor Works/Prefabs/Conveyor/ConveyorLegs.prefab", parent, "Leg", new Vector3(at.x, 0f, at.z), Quaternion.Euler(0f, yaw, 0f), 1f);
        go.transform.localScale = new Vector3(1f, (beltTop - BeltTop + BeltUnderside) / LegHeight, 1f);
    }

    static GameObject Place(string prefabPath, Transform parent, string name, Vector3 pos, Quaternion rot, float scaleZ)
    {
        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
        if (prefab == null) throw new Exception("prefab not found: " + prefabPath);
        var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab, parent);
        go.name = name;
        go.transform.SetPositionAndRotation(pos, rot);
        go.transform.localScale = new Vector3(1f, 1f, scaleZ);
        return go;
    }

    static Transform Group(string name)
    {
        var t = new GameObject(name).transform;
        t.SetParent(root, false);
        return t;
    }

    static void Marker(Transform parent, string name, Vector3 pos)
    {
        var t = new GameObject(name).transform;
        t.SetParent(parent, false);
        t.position = pos;
    }

    static PhysicsMaterial PhysicsMat(string path, string name, float friction)
    {
        var mat = AssetDatabase.LoadAssetAtPath<PhysicsMaterial>(path);
        bool created = mat == null;
        if (created) mat = new PhysicsMaterial(name);
        mat.dynamicFriction = friction; mat.staticFriction = friction;
        mat.frictionCombine = PhysicsMaterialCombine.Minimum; mat.bounciness = 0f;
        if (created)
        {
            Directory.CreateDirectory(PhysicsFolder);
            AssetDatabase.CreateAsset(mat, path);
        }
        else EditorUtility.SetDirty(mat);
        AssetDatabase.SaveAssets();
        return mat;
    }

    static string Summary()
    {
        var sb = new StringBuilder();
        var r = GameObject.Find(RootName).transform;
        foreach (Transform g in r)
        {
            sb.AppendLine($"{g.name}: {g.childCount} children");
            foreach (Transform c in g)
                if (c.name != "Leg" && !c.name.EndsWith("_Guide"))
                    sb.AppendLine($"  {c.name} pos={c.position:F3} rot={c.eulerAngles:F1} scale={c.localScale:F3}");
        }
        var rs = r.GetComponentsInChildren<Renderer>();
        var b = rs[0].bounds;
        foreach (var x in rs) b.Encapsulate(x.bounds);
        sb.AppendLine($"bounds min={b.min:F2} max={b.max:F2}");
        return sb.ToString();
    }

    // ---------- test ----------

    // Steps physics with test boxes fed onto the upper landing; the scene is not saved.
    // The test writes read results directly in place of the barcode reader, which is not built yet.
    // Lane 2 refuses boxes after its first diversion until t=150 s, so the second L02 box must pass and circulate.
    public static void Simulate()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var log = new StringBuilder();
        var spawn = GameObject.Find(RootName + "/EntryLine/UpperEntryStart").transform.position + new Vector3(0.5f, 0.3f, 0f);
        // -spawnInterval <s> sets the entry spacing (default 6 s = 3 m at 0.5 m/s).
        float gap = float.TryParse(Arg("-spawnInterval"), out var si) ? si : 6f;
        var schedule = new (float t, string read)[] { (0f, "L03"), (gap, "L01"), (2f * gap, null), (3f * gap, "L02"), (4f * gap, "L02") };
        var boxes = new List<Rigidbody>();
        float dt = Time.fixedDeltaTime, now = 0f;

        var behaviours = new List<MonoBehaviour>();
        behaviours.AddRange(UnityEngine.Object.FindObjectsByType<LinearConveyor>(FindObjectsSortMode.None));
        behaviours.AddRange(UnityEngine.Object.FindObjectsByType<CurvedConveyor>(FindObjectsSortMode.None));
        behaviours.AddRange(UnityEngine.Object.FindObjectsByType<Logistics.BrakeRollerZone>(FindObjectsSortMode.None));
        var diverters = UnityEngine.Object.FindObjectsByType<Logistics.LaneDiverter>(FindObjectsSortMode.None);
        behaviours.AddRange(diverters);
        foreach (var c in behaviours) Invoke(c, "Awake");
        Logistics.LaneDiverter lane2 = null;
        foreach (var d in diverters)
        {
            var diverter = d;
            diverter.DivertStarted += b => log.AppendLine($"t={now,6:F1} {diverter.Lane} arm out for {b.name}");
            diverter.Diverted += b =>
            {
                log.AppendLine($"t={now,6:F1} {diverter.Lane} diverted {b.name}");
                if (diverter.Lane == Logistics.DestinationLane.Lane2) diverter.AcceptsBoxes = false;
            };
            diverter.DivertFailed += b => log.AppendLine($"t={now,6:F1} {diverter.Lane} DIVERT FAILED {b.name}");
            if (diverter.Lane == Logistics.DestinationLane.Lane2) lane2 = diverter;
        }
        Physics.simulationMode = SimulationMode.Script;
        Physics.SyncTransforms();

        // Optional fine trace: -traceBox Box1 -traceFrom 70
        string traceBox = Arg("-traceBox");
        float traceFrom = float.TryParse(Arg("-traceFrom"), out var tf) ? tf : 0f;
        float traceStep = float.TryParse(Arg("-traceStep"), out var ts) ? ts : 0.25f;
        var maxLaneSpeed = new Dictionary<Rigidbody, float>();
        int next = 0;
        for (int step = 0; step <= Mathf.RoundToInt(450f / dt); step++)
        {
            now = step * dt;
            if (next < schedule.Length && now >= schedule[next].t)
            {
                var b = TestBox($"Box{next + 1}_{schedule[next].read ?? "unread"}", spawn);
                if (schedule[next].read != null && Logistics.BarcodeScanner.TryParseLane(schedule[next].read, out var readLane)) b.gameObject.AddComponent<Logistics.LogisticsBox>().SetReadResult(readLane);
                boxes.Add(b);
                next++;
            }
            if (lane2 != null && !lane2.AcceptsBoxes && now >= 150f)
            {
                lane2.AcceptsBoxes = true;
                log.AppendLine($"t={now,6:F1} L02 accepts boxes again");
            }
            foreach (var c in behaviours) Invoke(c, "FixedUpdate");
            Physics.Simulate(dt);
            foreach (var b in boxes)
                if (b.position.x < WestX - BeltHalf)
                    maxLaneSpeed[b] = Mathf.Max(maxLaneSpeed.TryGetValue(b, out var v) ? v : 0f, b.linearVelocity.magnitude);
            if (traceBox != null && now >= traceFrom && now <= traceFrom + 12f && step % Mathf.Max(1, Mathf.RoundToInt(traceStep / dt)) == 0)
            {
                var b = boxes.Find(x => x.name.StartsWith(traceBox));
                if (b != null)
                {
                    var row = new StringBuilder($"  trace t={now,6:F2} {b.name} pos={b.position:F3} yaw={b.rotation.eulerAngles.y:F0} v={b.linearVelocity:F2}");
                    foreach (var d in diverters) row.Append($" {d.Lane}arm={d.transform.GetComponentInChildren<Rigidbody>().rotation.eulerAngles.y:F0}");
                    foreach (var c in Physics.OverlapBox(b.position, b.transform.localScale * 0.5f + Vector3.one * 0.02f, b.rotation))
                        if (c.attachedRigidbody != b) row.Append($" [{c.name}]");
                    log.AppendLine(row.ToString());
                }
            }
            if (step % Mathf.RoundToInt(15f / dt) == 0)
            {
                var row = new StringBuilder($"t={now,6:F1}");
                foreach (var b in boxes) row.Append($" | {b.name} {b.position:F2}");
                log.AppendLine(row.ToString());
            }
        }
        log.AppendLine("final:");
        foreach (var b in boxes)
        {
            string lane = Where(b.position);
            log.AppendLine($"  {b.name} at {b.position:F2} -> {lane}" + (maxLaneSpeed.TryGetValue(b, out var v) ? $" max lane speed {v:F2} m/s" : "") + (b.IsSleeping() ? " (resting)" : $" v={b.linearVelocity.magnitude:F2}"));
        }
        Report("simulate", log.ToString());
        if (Application.isBatchMode) EditorApplication.Exit(0);
    }

    // Surface heights along lane 1 across the ease-to-steep kink; the scene is not saved.
    public static void Probe()
    {
        EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Physics.SyncTransforms();
        var log = new StringBuilder();
        float z = Lanes[0].z;
        foreach (float dz in new[] { 0f, -0.15f })
            for (float x = -10.80f; x >= -11.20f; x -= 0.02f)
                foreach (var h in Physics.RaycastAll(new Vector3(x, 6f, z + dz), Vector3.down, 10f))
                    log.AppendLine($"dz={dz:F2} x={x:F2} y={h.point.y:F4} {h.collider.name} n={h.normal:F3}");
        foreach (var c in Physics.OverlapSphere(new Vector3(-11.0f, 3.0f, Lanes[2].z - 0.22f), 0.35f, ~0, QueryTriggerInteraction.Collide))
            log.AppendLine($"near kink: {c.name} {c.GetType().Name} trigger={c.isTrigger} enabled={c.enabled} bounds={c.bounds.min:F3}..{c.bounds.max:F3} parent={c.transform.parent?.name}");
        Report("probe", log.ToString());
        if (Application.isBatchMode) EditorApplication.Exit(0);
    }

    static string Where(Vector3 p)
    {
        if (p.y < 0.5f) return "FELL";
        foreach (var lane in Lanes)
            if (p.x < WestX - BeltHalf && Mathf.Abs(p.z - lane.z) < LaneHalf + 0.1f)
                return $"{lane.name} ({lane.id})" + (p.y < 1.5f ? " at pickup" : " on lane");
        return "main loop / entry";
    }

    static Rigidbody TestBox(string name, Vector3 pos)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.position = pos;
        go.transform.localScale = new Vector3(0.35f, 0.34f, 0.48f);
        var rb = go.AddComponent<Rigidbody>();
        rb.mass = 3f;
        return rb;
    }

    static void Invoke(object target, string method) =>
        target.GetType().GetMethod(method, BindingFlags.Instance | BindingFlags.NonPublic | BindingFlags.Public)?.Invoke(target, null);

    static void Report(string kind, string text)
    {
        File.WriteAllText(Path.Combine(OutDir(), "conveyor_" + kind + ".txt"), text);
        Debug.Log($"[ConveyorLayoutBuilder:{kind}]\n{text}");
    }

    static string OutDir()
    {
        string dir = Arg("-layoutOut") ?? Directory.GetCurrentDirectory();
        Directory.CreateDirectory(dir);
        return dir;
    }

    static string Arg(string key)
    {
        var a = Environment.GetCommandLineArgs();
        for (int i = 0; i < a.Length - 1; i++) if (a[i] == key) return a[i + 1];
        return null;
    }
}
