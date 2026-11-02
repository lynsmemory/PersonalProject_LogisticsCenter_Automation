using System;
using System.IO;
using UnityEditor;
using UnityEngine;

// Original simulation geometry. Dimensions are editable planning values, not manufacturer CAD.
public static class LogisticsVehiclePrefabBuilder
{
    const string Folder = "Assets/LogisticsVehicles";
    static Material white, blue, dark, steel, rubber, glass, amber, red, branding;

    [MenuItem("Tools/Logistics/Create Vehicle Prefabs")]
    public static void Build()
    {
        Directory.CreateDirectory(Folder + "/Materials");
        white = Mat("BodyWhite", new Color(.83f,.86f,.88f));
        blue = Mat("IndustrialBlue", new Color(.035f,.23f,.48f));
        dark = Mat("Chassis", new Color(.075f,.09f,.105f));
        steel = Mat("Steel", new Color(.43f,.48f,.52f), .65f);
        rubber = Mat("Rubber", new Color(.025f,.03f,.035f));
        glass = Mat("Glass", new Color(.08f,.19f,.24f), .3f);
        amber = Mat("Amber", new Color(1,.46f,.035f));
        red = Mat("Red", new Color(.7f,.025f,.025f));
        branding = Mat("Coupang", Color.white);
        branding.mainTexture = AssetDatabase.LoadAssetAtPath<Texture2D>(Folder + "/Textures/CoupangSide.png");
        BuildTruck();
        BuildForklift();
        AssetDatabase.SaveAssets();
        File.WriteAllText(Folder + "/UnityBuildNotes.txt",
            "# Logistics simulation vehicle prefabs\n\n" +
            "Original editable geometry. Units: metres; up: +Y; forward: +Z. Approximate planning models, not manufacturer CAD.\n\n" +
            "WingBody5t_Coupang: cargo interior 6.2 x 2.3 x 2.4 m, floor top Y=1.1 m. Ten 1.1 m pallet slots in two columns. Slot transforms describe pallet underside centre. Payload rating is a label, not a physics certification. Open left wing local Z=-110 degrees, right wing Z=110 degrees. Open rear doors outward before rear loading.\n\n" +
            "RobotForklift: no driver cabin; mast, carriage, forks and sensors separated. Move ForkCarriage local Y from 0 to 1.1 m to place fork top at 1.18 m. Fork entry and pallet pocket geometry must be matched before operation.\n\n" +
            "Static compound BoxColliders are provided. Vehicle roots have no Rigidbody or navigation controller; add the selected movement controller later. Wheel meshes, wings, rear doors and carriage are editable children. No automatic loading, obstacle detection or payload validation is implemented.\n");
        Debug.Log("LOGISTICS_VEHICLES_BUILD_OK");
    }

    static Material Mat(string name, Color color, float metallic=0)
    {
        string path = Folder + "/Materials/" + name + ".mat";
        Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (m == null)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Standard");
            m = new Material(shader);
            AssetDatabase.CreateAsset(m,path);
        }
        m.color = color;
        m.SetFloat("_Metallic",metallic);
        m.SetFloat("_Smoothness",.32f);
        return m;
    }

    static GameObject Group(string name, Transform parent, Vector3 position)
    {
        GameObject g = new GameObject(name);
        g.transform.SetParent(parent,false);
        g.transform.localPosition = position;
        return g;
    }

    static GameObject Box(string name, Transform parent, Vector3 position, Vector3 scale, Material material, bool collider=true)
    {
        GameObject g = GameObject.CreatePrimitive(PrimitiveType.Cube);
        g.name=name; g.transform.SetParent(parent,false);
        g.transform.localPosition=position; g.transform.localScale=scale;
        g.GetComponent<Renderer>().sharedMaterial=material;
        if (!collider) UnityEngine.Object.DestroyImmediate(g.GetComponent<Collider>());
        return g;
    }

    static void Wheel(Transform parent, float x, float y, float z, float radius, float width)
    {
        GameObject g=GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        g.name="Wheel"; g.transform.SetParent(parent,false);
        g.transform.localPosition=new Vector3(x,y,z);
        g.transform.localRotation=Quaternion.Euler(0,0,90);
        g.transform.localScale=new Vector3(radius*2,width/2,radius*2);
        g.GetComponent<Renderer>().sharedMaterial=rubber;
        UnityEngine.Object.DestroyImmediate(g.GetComponent<Collider>());
        GameObject hub=GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        hub.name="Hub"; hub.transform.SetParent(g.transform,false);
        hub.transform.localScale=new Vector3(.55f,1.025f,.55f);
        hub.GetComponent<Renderer>().sharedMaterial=steel;
        UnityEngine.Object.DestroyImmediate(hub.GetComponent<Collider>());
    }

    static void Save(GameObject root, string file)
    {
        PrefabUtility.SaveAsPrefabAsset(root,Folder+"/"+file+".prefab");
        UnityEngine.Object.DestroyImmediate(root);
    }

    static void Logo(Transform parent, Vector3 position, Vector3 scale, int side)
    {
        GameObject g = GameObject.CreatePrimitive(PrimitiveType.Quad);
        g.name = "CoupangLogo";
        g.transform.SetParent(parent, false);
        g.transform.localPosition = position;
        g.transform.localScale = scale;
        g.transform.localRotation = Quaternion.Euler(0, side < 0 ? 90 : -90, 0);
        g.GetComponent<Renderer>().sharedMaterial = branding;
        UnityEngine.Object.DestroyImmediate(g.GetComponent<Collider>());
    }

    static void BuildTruck()
    {
        Transform t=new GameObject("WingBody5t_Coupang").transform;
        Box("Chassis",t,new Vector3(0,.67f,.35f),new Vector3(1.9f,.26f,8.5f),dark);
        Box("CargoFloor",t,new Vector3(0,1.02f,-.5f),new Vector3(2.42f,.16f,6.32f),steel);
        Box("CargoFrontWall",t,new Vector3(0,2.3f,2.63f),new Vector3(2.42f,2.4f,.06f),white);
        Box("RoofCentre",t,new Vector3(0,3.54f,-.5f),new Vector3(.12f,.08f,6.32f),white);
        for(int side=-1;side<=1;side+=2)
        {
            Transform wing=Group(side<0?"LeftWingPivot":"RightWingPivot",t,new Vector3(0,3.5f,-.5f)).transform;
            Box("WingRoof",wing,new Vector3(side*.62f,0,0),new Vector3(1.18f,.08f,6.32f),white);
            Box("WingSide",wing,new Vector3(side*1.22f,-1.2f,0),new Vector3(.06f,2.4f,6.32f),white);
            Logo(wing, new Vector3(side*1.28f,-1.06f,0), new Vector3(3.1f,1.55f,1), side);
            for(int rib=0;rib<8;rib++)
                Box("SideRib",wing,new Vector3(side*1.26f,-1.2f,-2.8f+rib*.8f),new Vector3(.025f,2.3f,.035f),steel,false);
            Box("SideRail",t,new Vector3(side*1.05f,.64f,-.4f),new Vector3(.06f,.08f,3.8f),steel);
            Wheel(t,side*1.02f,.49f,3.85f,.49f,.26f);
            Wheel(t,side*1.02f,.49f,-2.3f,.49f,.32f);
            Transform door=Group(side<0?"RearDoorLeftPivot":"RearDoorRightPivot",t,new Vector3(side*1.18f,1.1f,-3.68f)).transform;
            Box("RearDoor",door,new Vector3(-side*.59f,1.2f,0),new Vector3(1.18f,2.4f,.06f),white);
            Box("LockBar",door,new Vector3(-side*.7f,1.2f,-.055f),new Vector3(.035f,2.1f,.035f),steel,false);
            Box("RearLamp",t,new Vector3(side*.96f,.86f,-3.73f),new Vector3(.28f,.13f,.06f),red,false);
        }
        Box("Cab",t,new Vector3(0,1.65f,3.86f),new Vector3(2.28f,1.9f,2.2f),white);
        Box("CabRoof",t,new Vector3(0,2.64f,3.85f),new Vector3(2.34f,.12f,2.22f),white);
        Box("Windshield",t,new Vector3(0,2.08f,4.975f),new Vector3(1.96f,.7f,.025f),glass,false);
        Box("FrontGrille",t,new Vector3(0,1.2f,4.985f),new Vector3(1.25f,.4f,.035f),dark,false);
        Box("FrontBumper",t,new Vector3(0,.8f,5.02f),new Vector3(2.34f,.22f,.16f),steel);
        for(int side=-1;side<=1;side+=2)
        {
            Box("SideWindow",t,new Vector3(side*1.15f,2.1f,3.95f),new Vector3(.025f,.65f,1.45f),glass,false);
            Box("Mirror",t,new Vector3(side*1.32f,2.12f,4.55f),new Vector3(.17f,.38f,.13f),dark,false);
            Box("Headlight",t,new Vector3(side*.83f,1.2f,5),new Vector3(.32f,.2f,.035f),white,false);
            Box("Step",t,new Vector3(side*1.19f,.83f,3.03f),new Vector3(.22f,.1f,.65f),steel);
            Logo(t, new Vector3(side*1.146f,1.45f,3.85f), new Vector3(.65f,.325f,1), side);
        }
        Transform slots=Group("PalletSlots",t,Vector3.zero).transform;
        for(int row=0;row<5;row++)
            for(int col=0;col<2;col++)
                Group("PalletSlot_"+(row*2+col+1).ToString("00"),slots,new Vector3(col==0?-.575f:.575f,1.1f,1.85f-row*1.175f));
        Group("RearLoadingApproach",t,new Vector3(0,0,-5.5f));
        Group("DockReference",t,new Vector3(0,1.1f,-3.66f));
        Save(t.gameObject,"WingBody5t_Coupang");
    }

    static void BuildForklift()
    {
        Transform t=new GameObject("RobotForklift").transform;
        Box("BaseChassis",t,new Vector3(0,.32f,-.35f),new Vector3(1.05f,.3f,1.75f),dark);
        Box("Counterweight",t,new Vector3(0,.78f,-.73f),new Vector3(1.16f,.72f,.9f),blue);
        Box("BatteryCover",t,new Vector3(0,1.18f,-.73f),new Vector3(1.1f,.1f,.85f),white);
        Box("ControllerHousing",t,new Vector3(0,.86f,-.1f),new Vector3(.8f,.55f,.36f),white);
        for(int side=-1;side<=1;side+=2)
        {
            Wheel(t,side*.51f,.24f,.15f,.24f,.16f);
            Wheel(t,side*.46f,.2f,-.96f,.2f,.14f);
            Box("MastRail",t,new Vector3(side*.39f,1.61f,.45f),new Vector3(.12f,3.02f,.16f),steel);
            Box("MastInnerRail",t,new Vector3(side*.26f,1.61f,.47f),new Vector3(.075f,2.95f,.1f),dark);
            Box("FrontSafetyScanner",t,new Vector3(side*.43f,.4f,.62f),new Vector3(.18f,.12f,.13f),red,false);
        }
        Box("MastTopBridge",t,new Vector3(0,3.12f,.45f),new Vector3(.9f,.12f,.19f),steel);
        Box("LiftCylinder",t,new Vector3(0,1.4f,.42f),new Vector3(.12f,2.5f,.13f),dark);
        Transform carriage=Group("ForkCarriage",t,Vector3.zero).transform;
        Box("ForkBackplate",carriage,new Vector3(0,.47f,.62f),new Vector3(.91f,.7f,.1f),steel);
        for(int side=-1;side<=1;side+=2)
        {
            Box("ForkUpright",carriage,new Vector3(side*.3f,.45f,.72f),new Vector3(.12f,.7f,.07f),dark);
            Box("ForkBlade",carriage,new Vector3(side*.3f,.06f,1.27f),new Vector3(.12f,.04f,1.15f),steel);
        }
        Box("LoadBackrest",carriage,new Vector3(0,.9f,.63f),new Vector3(.95f,.08f,.08f),dark);
        Box("SensorPole",t,new Vector3(0,1.49f,-.76f),new Vector3(.055f,.55f,.055f),steel,false);
        Box("Lidar",t,new Vector3(0,1.8f,-.76f),new Vector3(.16f,.12f,.16f),dark,false);
        Box("Beacon",t,new Vector3(.34f,1.28f,-.85f),new Vector3(.1f,.13f,.1f),amber,false);
        Box("RearScanner",t,new Vector3(0,.35f,-1.25f),new Vector3(.22f,.12f,.09f),red,false);
        Group("PalletAttachPoint",carriage,new Vector3(0,.08f,1.27f));
        Group("NavigationReference",t,Vector3.zero);
        Group("FrontSensorOrigin",t,new Vector3(0,.4f,.76f));
        Save(t.gameObject,"RobotForklift");
    }
}
