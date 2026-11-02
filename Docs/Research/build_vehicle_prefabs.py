"""Generate native Unity prefabs without starting the Editor; original editable geometry."""
from pathlib import Path
import hashlib, math, re, json
from PIL import Image, ImageDraw, ImageFont

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'Assets/LogisticsVehicles'
OUT.mkdir(parents=True,exist_ok=True)
HEADER='%YAML 1.1\n%TAG !u! tag:unity3d.com,2011:\n'
def guid(s): return hashlib.md5(('logistics-vehicles/'+s).encode()).hexdigest()
def meta(p,kind='DefaultImporter'):
    Path(str(p)+'.meta').write_text(f'fileFormatVersion: 2\nguid: {guid(p.relative_to(OUT).as_posix())}\n{kind}:\n  externalObjects: {{}}\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n',encoding='utf-8')
def vec(v): return '{'+', '.join(f'{k}: {float(n):.8g}' for k,n in zip('xyz',v))+'}'
def quat(deg,axis=1):
    q=[0.,0.,0.,math.cos(math.radians(deg)/2)];q[axis]=math.sin(math.radians(deg)/2)
    return '{'+', '.join(f'{k}: {n:.8g}' for k,n in zip('xyzw',q))+'}'

# Typography reconstruction from the supplied truck photo; replaceable branding texture.
tex=OUT/'Textures';tex.mkdir(exist_ok=True)
im=Image.new('RGB',(2048,1024),(212,219,224));d=ImageDraw.Draw(im)
font=ImageFont.truetype('C:/Windows/Fonts/arialbd.ttf',310)
kr=ImageFont.truetype('C:/Windows/Fonts/malgunbd.ttf',106)
word='coupang';colors=['#42372d']*3+['#ed1c24','#f5b51b','#82b744','#008acb']
w=sum(d.textlength(c,font=font) for c in word);x=(2048-w)/2
for c,color in zip(word,colors):
    d.text((x,370),c,font=font,fill=color);x+=d.textlength(c,font=font)
s='쿠팡은 로켓배송!';d.text(((2048-d.textlength(s,font=kr))/2,205),s,font=kr,fill='#174b93')
png=tex/'CoupangSide.png';im.save(png);meta(png,'TextureImporter')
# Explicit texture importer defaults, retaining normal UV direction and mipmaps.
Path(str(png)+'.meta').write_text(f'''fileFormatVersion: 2
guid: {guid('Textures/CoupangSide.png')}
TextureImporter:
  serializedVersion: 13
  mipmaps:
    mipMapMode: 0
    enableMipMap: 1
    sRGBTexture: 1
  isReadable: 0
  textureFormat: 1
  maxTextureSize: 2048
  textureSettings:
    serializedVersion: 2
    filterMode: 1
    aniso: 2
    mipBias: 0
    wrapU: 1
    wrapV: 1
    wrapW: 1
  textureType: 0
  textureShape: 1
  alphaIsTransparency: 0
  userData:
  assetBundleName:
  assetBundleVariant:
''',encoding='utf-8')
shader_meta=next((ROOT/'Library/PackageCache').glob('com.unity.render-pipelines.universal*/Shaders/Lit.shader.meta'))
shader=re.search(r'guid: (\w+)',shader_meta.read_text()).group(1)
palette={'BodyWhite':(.83,.86,.88),'IndustrialBlue':(.035,.23,.48),'Chassis':(.075,.09,.105),'Steel':(.43,.48,.52),'Rubber':(.025,.03,.035),'Glass':(.08,.19,.24),'Amber':(1,.46,.035),'Red':(.7,.025,.025),'Coupang':(1,1,1)}
for name,col in palette.items():
    p=OUT/'Materials'/f'{name}.mat';p.parent.mkdir(exist_ok=True)
    texture=f'{{fileID: 2800000, guid: {guid("Textures/CoupangSide.png")}, type: 3}}' if name=='Coupang' else '{fileID: 0}'
    p.write_text(HEADER+f'''--- !u!21 &2100000
Material:
  serializedVersion: 8
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_Name: {name}
  m_Shader: {{fileID: 4800000, guid: {shader}, type: 3}}
  m_ValidKeywords: []
  m_InvalidKeywords: []
  m_LightmapFlags: 4
  m_EnableInstancingVariants: 1
  m_DoubleSidedGI: 0
  m_CustomRenderQueue: -1
  stringTagMap: {{}}
  disabledShaderPasses: []
  m_SavedProperties:
    serializedVersion: 3
    m_TexEnvs:
    - _BaseMap:
        m_Texture: {texture}
        m_Scale: {{x: 1, y: 1}}
        m_Offset: {{x: 0, y: 0}}
    m_Ints: []
    m_Floats:
    - _Surface: 0
    - _Cull: 2
    - _ZWrite: 1
    - _SrcBlend: 1
    - _DstBlend: 0
    - _Metallic: {0.65 if name=='Steel' else 0}
    - _Smoothness: 0.32
    m_Colors:
    - _BaseColor: {{r: {col[0]}, g: {col[1]}, b: {col[2]}, a: 1}}
    - _EmissionColor: {{r: 0, g: 0, b: 0, a: 1}}
  m_BuildTextureStacks: []
''',encoding='utf-8');meta(p,'NativeFormatImporter')

class Model:
    def __init__(self,name): self.name=name; self.nodes=[];self.add(name,None)
    def add(self,name,parent=0,pos=(0,0,0),scale=(1,1,1),mat=None,mesh=10202,collider=True,angle=0,axis=1):
        n=dict(name=name,parent=parent,pos=pos,scale=scale,mat=mat,mesh=mesh,collider=collider,angle=angle,axis=axis);self.nodes.append(n);return len(self.nodes)-1
    def wheel(self,x,y,z,r,w):
        self.add('Wheel',pos=(x,y,z),scale=(2*r,w/2,2*r),mat='Rubber',mesh=10206,collider=False,angle=90,axis=2)
        self.add('Hub',pos=(x+(.01 if x>0 else -.01),y,z),scale=(r,w/2+.015,r),mat='Steel',mesh=10206,collider=False,angle=90,axis=2)
    def save(self):
        chunks=[HEADER]
        for i,n in enumerate(self.nodes):
            b=(i+1)*10;components=[b+1]+([b+2,b+3]+([b+4] if n['collider'] else []) if n['mat'] else [])
            chunks.append(f'''--- !u!1 &{b}
GameObject:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  serializedVersion: 6
  m_Component:
'''+''.join(f'  - component: {{fileID: {c}}}\n' for c in components)+f'''  m_Layer: 0
  m_Name: {n['name']}
  m_TagString: Untagged
  m_Icon: {{fileID: 0}}
  m_NavMeshLayer: 0
  m_StaticEditorFlags: 0
  m_IsActive: 1
--- !u!4 &{b+1}
Transform:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {b}}}
  serializedVersion: 2
  m_LocalRotation: {quat(n['angle'],n['axis'])}
  m_LocalPosition: {vec(n['pos'])}
  m_LocalScale: {vec(n['scale'])}
  m_ConstrainProportionsScale: 0
''')
            children=[(j+1)*10+1 for j,c in enumerate(self.nodes) if c['parent']==i]
            chunks.append('  m_Children:'+('\n'+''.join(f'  - {{fileID: {c}}}\n' for c in children) if children else ' []\n'))
            chunks.append(f"  m_Father: {{fileID: {(n['parent']+1)*10+1 if n['parent'] is not None else 0}}}\n  m_LocalEulerAnglesHint: {vec(tuple(n['angle'] if k==n['axis'] else 0 for k in range(3)))}\n")
            if not n['mat']:continue
            chunks.append(f'''--- !u!33 &{b+2}
MeshFilter:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {b}}}
  m_Mesh: {{fileID: {n['mesh']}, guid: 0000000000000000e000000000000000, type: 0}}
--- !u!23 &{b+3}
MeshRenderer:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {b}}}
  m_Enabled: 1
  m_CastShadows: 1
  m_ReceiveShadows: 1
  m_DynamicOccludee: 1
  m_MotionVectors: 1
  m_LightProbeUsage: 1
  m_ReflectionProbeUsage: 1
  m_RayTracingMode: 2
  m_RenderingLayerMask: 1
  m_RendererPriority: 0
  m_Materials:
  - {{fileID: 2100000, guid: {guid('Materials/'+n['mat']+'.mat')}, type: 2}}
  m_StaticBatchInfo:
    firstSubMesh: 0
    subMeshCount: 0
  m_StaticBatchRoot: {{fileID: 0}}
  m_ProbeAnchor: {{fileID: 0}}
  m_LightProbeVolumeOverride: {{fileID: 0}}
  m_ScaleInLightmap: 1
  m_ReceiveGI: 1
  m_SortingLayerID: 0
  m_SortingLayer: 0
  m_SortingOrder: 0
  m_AdditionalVertexStreams: {{fileID: 0}}
''')
            if n['collider']:chunks.append(f'''--- !u!65 &{b+4}
BoxCollider:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {b}}}
  m_Material: {{fileID: 0}}
  m_IncludeLayers:
    serializedVersion: 2
    m_Bits: 0
  m_ExcludeLayers:
    serializedVersion: 2
    m_Bits: 0
  m_LayerOverridePriority: 0
  m_IsTrigger: 0
  m_ProvidesContacts: 0
  m_Enabled: 1
  serializedVersion: 3
  m_Size: {{x: 1, y: 1, z: 1}}
  m_Center: {{x: 0, y: 0, z: 0}}
''')
        p=OUT/(self.name+'.prefab');p.write_text(''.join(chunks),encoding='utf-8');meta(p,'PrefabImporter')
        ids=set(re.findall(r'^--- !u!\d+ &(\d+)',p.read_text(),re.M))
        refs=re.findall(r'\{fileID: (\d+)\}',p.read_text())
        assert all(r=='0' or r in ids for r in refs)
        return dict(name=self.name,nodes=len(self.nodes),components=len(ids),local_references_valid=True)

t=Model('WingBody5t_Coupang')
def box(name,p,s,m='BodyWhite',parent=0,collider=True):return t.add(name,parent,p,s,m,collider=collider)
box('Chassis',(0,.67,.35),(1.9,.26,8.5),'Chassis')
box('CargoFloor',(0,1.02,-.5),(2.42,.16,6.32),'Steel')
box('CargoFrontWall',(0,2.3,2.63),(2.42,2.4,.06))
box('RoofCentre',(0,3.54,-.5),(.12,.08,6.32))
for side in [-1,1]:
    wing=t.add('LeftWingPivot' if side<0 else 'RightWingPivot',pos=(0,3.5,-.5))
    box('WingRoof',(side*.62,0,0),(1.18,.08,6.32),parent=wing)
    box('WingSide',(side*1.22,-1.2,0),(.06,2.4,6.32),parent=wing)
    # Unity quad front is local -Z. The two signs orient each readable side outward.
    t.add('CoupangSideLogo',wing,(side*1.28,-1.06,0),(3.1,1.55,1),'Coupang',mesh=10210,collider=False,angle=90 if side<0 else -90)
    for rib in range(8):box('SideRib',(side*1.26,-1.2,-2.8+rib*.8),(.025,2.3,.035),'Steel',wing,False)
    box('SideRail',(side*1.05,.64,-.4),(.06,.08,3.8),'Steel')
    t.wheel(side*1.02,.49,3.85,.49,.26);t.wheel(side*1.02,.49,-2.3,.49,.32)
    door=t.add('RearDoorLeftPivot' if side<0 else 'RearDoorRightPivot',pos=(side*1.18,1.1,-3.68))
    box('RearDoor',(-side*.59,1.2,0),(1.18,2.4,.06),parent=door)
    box('LockBar',(-side*.7,1.2,-.055),(.035,2.1,.035),'Steel',door,False)
    box('RearLamp',(side*.96,.86,-3.73),(.28,.13,.06),'Red',collider=False)
box('Cab',(0,1.65,3.86),(2.28,1.9,2.2))
box('CabRoof',(0,2.64,3.85),(2.34,.12,2.22))
box('Windshield',(0,2.08,4.975),(1.96,.7,.025),'Glass',collider=False)
box('FrontGrille',(0,1.2,4.985),(1.25,.4,.035),'Chassis',collider=False)
box('FrontBumper',(0,.8,5.02),(2.34,.22,.16),'Steel')
for side in [-1,1]:
    box('SideWindow',(side*1.15,2.1,3.95),(.025,.65,1.45),'Glass',collider=False)
    box('Mirror',(side*1.32,2.12,4.55),(.17,.38,.13),'Chassis',collider=False)
    box('Headlight',(side*.83,1.2,5),(.32,.2,.035),collider=False)
    box('Step',(side*1.19,.83,3.03),(.22,.1,.65),'Steel')
    t.add('CabDoorLogo',0,(side*1.146,1.45,3.85),(.65,.325,1),'Coupang',mesh=10210,collider=False,angle=90 if side<0 else -90)
slots=t.add('PalletSlots')
for row in range(5):
    for col in range(2):
        pos=(-.575 if col==0 else .575,1.1,1.85-row*1.175)
        assert abs(pos[0])+.55<=1.15+1e-8 and -.5-3.1<=pos[2]-.55 and pos[2]+.55<=-.5+3.1
        t.add(f'PalletSlot_{row*2+col+1:02}',slots,pos)
t.add('RearLoadingApproach',pos=(0,0,-5.5));t.add('DockReference',pos=(0,1.1,-3.66))
results=[t.save()]
t=Model('RobotForklift')
box('BaseChassis',(0,.32,-.35),(1.05,.3,1.75),'Chassis')
box('Counterweight',(0,.78,-.73),(1.16,.72,.9),'IndustrialBlue')
box('BatteryCover',(0,1.18,-.73),(1.1,.1,.85))
box('ControllerHousing',(0,.86,-.1),(.8,.55,.36))
for side in [-1,1]:
    t.wheel(side*.51,.24,.15,.24,.16);t.wheel(side*.46,.2,-.96,.2,.14)
    box('MastRail',(side*.39,1.61,.45),(.12,3.02,.16),'Steel')
    box('MastInnerRail',(side*.26,1.61,.47),(.075,2.95,.1),'Chassis')
    box('FrontSafetyScanner',(side*.43,.4,.62),(.18,.12,.13),'Red',collider=False)
box('MastTopBridge',(0,3.12,.45),(.9,.12,.19),'Steel')
box('LiftCylinder',(0,1.4,.42),(.12,2.5,.13),'Chassis')
carriage=t.add('ForkCarriage')
box('ForkBackplate',(0,.47,.62),(.91,.7,.1),'Steel',carriage)
for side in [-1,1]:
    box('ForkUpright',(side*.3,.45,.72),(.12,.7,.07),'Chassis',carriage)
    box('ForkBlade',(side*.3,.06,1.27),(.12,.04,1.15),'Steel',carriage)
box('LoadBackrest',(0,.9,.63),(.95,.08,.08),'Chassis',carriage)
box('SensorPole',(0,1.49,-.76),(.055,.55,.055),'Steel',collider=False)
box('Lidar',(0,1.8,-.76),(.16,.12,.16),'Chassis',collider=False)
box('Beacon',(.34,1.28,-.85),(.1,.13,.1),'Amber',collider=False)
box('RearScanner',(0,.35,-1.25),(.22,.12,.09),'Red',collider=False)
t.add('PalletAttachPoint',carriage,(0,.08,1.27));t.add('NavigationReference');t.add('FrontSensorOrigin',pos=(0,.4,.76))
results.append(t.save())
report={'prefabs':results,'pallet_slots':10,'pallet_slot_bounds_valid':True,'unity_import_verified':False,'branding':'Typography approximation from user reference; not official vector artwork'}
(OUT/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
(OUT/'README.md').write_text('''# 물류 시뮬레이션 차량 프리팹

Unity 단위 m, +Y 위, +Z 전방. 제조사 CAD가 아닌 기본 형상 조합 모델입니다.

- WingBody5t_Coupang.prefab: 흰색 5톤 윙바디 형태. 적재함 내부 기준 6.2×2.3×2.4m, 바닥 상면 1.1m. 1100mm 파레트 2열×5자리. 양쪽 윙과 운전석에 쿠팡 표기. 5톤은 모델 분류이며 물리 하중 검증값이 아닙니다.
- RobotForklift.prefab: 차체·마스트·승강 캐리지·포크·센서 형상. ForkCarriage의 local Y를 0~1.1m로 변경해 포크 상면을 0.08~1.18m로 이동할 수 있습니다. 포크와 실제 파레트 포켓의 간섭 검증이 필요합니다.
- LeftWingPivot: local Z=-110도, RightWingPivot: local Z=110도로 열 수 있습니다. 닫힌 상태는 0도입니다. 후방 상차 시 RearDoor 피벗도 별도로 열어야 합니다.
- 로고는 사용자가 제공한 사진을 참고한 글꼴 기반 근사본입니다. 공식 벡터 로고와 정확히 일치하지 않습니다. Textures/CoupangSide.png를 교체할 수 있습니다. 양쪽 로고는 윙 자식이므로 개폐 시 따라갑니다.
- PalletSlot은 파레트 밑면 중앙 기준입니다. 도크·접근·항법·파레트 부착 기준점을 포함합니다.
- 단순 BoxCollider가 있습니다. 주행·장애물 감지·상차·윙 개폐 애니메이션 제어와 Rigidbody는 연결 전입니다.

Unity 배치 실행이 프로젝트 초기 로딩에서 종료되어 파일을 직접 생성했습니다. 로컬 참조와 파레트 슬롯 경계는 검사했으나 Unity 임포트·시각 확인·물리 시험은 미실시입니다. Unity에서 프리팹을 열어 확인하세요.

생성기: Docs/Research/build_vehicle_prefabs.py. Unity에서 Tools/Logistics/Create Vehicle Prefabs 메뉴로도 같은 계열의 프리팹을 재생성할 수 있습니다. 로고 텍스처는 별도 파일로 유지됩니다.
''',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
