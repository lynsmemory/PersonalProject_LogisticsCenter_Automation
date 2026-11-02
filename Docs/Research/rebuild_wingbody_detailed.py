"""1차: WingBody5t_Coupang 운전석·바퀴·프레임을 실제 메시(Unity native Mesh YAML)로 교체.

기존 prefab을 읽어 적재함·윙/뒷문 피벗(61/271/231/441)·루트(10/11)·파레트 슬롯·기준점·로고를 그대로 두고
큐브 운전석/원통 바퀴/유리/그릴/범퍼/차대만 제거 후 새 메시 노드로 대체한다.
Mesh .asset 형식은 Library/PackageCache/com.unity.render-pipelines.core*/Runtime/PathTracing/Meshes/6FaceSkyboxMesh.mesh
(serializedVersion 11, NativeFormatImporter mainObjectFileID 4300000)를 따른다.

사용:
  python rebuild_wingbody_detailed.py --output <임시폴더>          # 생성+검증+프리뷰만 (Assets 변경 없음)
  python rebuild_wingbody_detailed.py --output <임시폴더> --apply  # 검증 통과 시 Assets/LogisticsVehicles로 복사
"""
from pathlib import Path
import argparse, hashlib, json, math, re, shutil, struct
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
VEH = ROOT / 'Assets/LogisticsVehicles'
PREFAB = VEH / 'WingBody5t_Coupang.prefab'
PREFAB_GUID = 'ec8a971b886b75927171a490e1a5b544'
KEEP_IDS = {10: 11, 60: 61, 270: 271, 230: 231, 440: 441}
REMOVE = {'Cab', 'CabRoof', 'Windshield', 'FrontGrille', 'FrontBumper', 'SideWindow', 'Mirror',
          'Headlight', 'Step', 'Wheel', 'Hub', 'Chassis', 'SideRail'}
HEADER = '%YAML 1.1\n%TAG !u! tag:unity3d.com,2011:\n'

def guid(s): return hashlib.md5(('logistics-vehicles/detailed/' + s).encode()).hexdigest()
def sub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def dot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a):
    l = math.sqrt(dot(a, a)); return (a[0]/l, a[1]/l, a[2]/l) if l > 1e-12 else (0., 0., 0.)

# ---------------------------------------------------------------- geometry
class Mesh:
    """삼각형 목록. 각 삼각형은 outward 벡터로 감기 방향을 맞춘다 (Unity: cross(b-a,c-a)가 앞면 법선)."""
    def __init__(self, name): self.name, self.tris = name, []
    def tri(self, a, b, c, mat, out):
        n = cross(sub(b, a), sub(c, a))
        if dot(n, n) < 1e-14: return
        self.tris.append((a, c, b, mat) if dot(n, out) < 0 else (a, b, c, mat))
    def quad(self, a, b, c, d, mat, out): self.tri(a, b, c, mat, out); self.tri(a, c, d, mat, out)
    def deform(self, f): self.tris = [(f(a), f(b), f(c), m) for a, b, c, m in self.tris]

def area2(poly):
    """표준 shoelace 부호: CCW(u 오른쪽, v 위) > 0."""
    return sum(poly[i-1][0]*poly[i][1] - poly[i][0]*poly[i-1][1] for i in range(len(poly))) / 2
def edge_out(poly, i):
    # CCW에서 변 (du,dv)의 외향 법선은 (dv,-du). CW이면 부호 반전. inset은 이 외향의 반대로 이동.
    (u0, v0), (u1, v1) = poly[i], poly[(i+1) % len(poly)]
    s = 1 if area2(poly) > 0 else -1
    l = math.hypot(u1-u0, v1-v0) or 1
    return (s*(v1-v0)/l, -s*(u1-u0)/l)

def inset(poly, c):
    out = []
    for i in range(len(poly)):
        n0, n1 = edge_out(poly, i-1), edge_out(poly, i)
        k = 1 + n0[0]*n1[0] + n0[1]*n1[1]
        k = max(k, 0.35)
        out.append((poly[i][0] - c*(n0[0]+n1[0])/k, poly[i][1] - c*(n0[1]+n1[1])/k))
    return out

def triangulate(poly):
    """Ear clipping. 반환: 인덱스 삼각형 (poly 방향과 동일한 감기)."""
    idx = list(range(len(poly))); s = 1 if area2(poly) > 0 else -1; res = []
    def inside(p, a, b, c):
        def sg(p1, p2, p3): return (p1[0]-p3[0])*(p2[1]-p3[1]) - (p2[0]-p3[0])*(p1[1]-p3[1])
        d1, d2, d3 = sg(p, a, b), sg(p, b, c), sg(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))
    guard = 0
    while len(idx) > 3 and guard < 10000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k-1], idx[k], idx[(k+1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if s*((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])) <= 1e-12: continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)): continue
            res.append((i0, i1, i2)); idx.pop(k); break
        else: break
    if len(idx) == 3: res.append(tuple(idx))
    return res

def prism(m, prof, xa, xb, c, mat, edge_mats=None):
    """측면 프로파일 prof[(z,y)]를 x=xa..xb로 두껍게 연결한 쉘. 모든 프로파일 모서리를 c 챔퍼."""
    ins = inset(prof, c) if c > 0 else prof
    L = [[(xa, y, z) for z, y in ins], [(xa+c, y, z) for z, y in prof],
         [(xb-c, y, z) for z, y in prof], [(xb, y, z) for z, y in ins]]
    n = len(prof)
    for i in range(n):
        j = (i+1) % n; nz, ny = edge_out(prof, i); em = (edge_mats or {}).get(i, mat)
        for k, ox in ((0, -0.7), (1, 0.0), (2, 0.7)):
            if c <= 0 and k != 1: continue
            m.quad(L[k][i], L[k+1][i], L[k+1][j], L[k][j], em, (ox, ny, nz))
    for t in triangulate(ins):
        for layer, ox in ((0, -1), (3, 1)):
            m.tri(*(L[layer][q] for q in t), mat, (ox, 0, 0))

def box(m, x0, x1, y0, y1, z0, z1, mat, c=0.01):
    prism(m, [(z0, y0), (z1, y0), (z1, y1), (z0, y1)], x0, x1, min(c, (x1-x0)/3, (y1-y0)/3, (z1-z0)/3), mat)

def lathe(m, sec, center, side, mat, segs=32, rot=0.0):
    """sec[(x,r)] 단면을 X축 회전. side=±1은 바깥쪽(+x/-x) 방향."""
    cx, cy, cz = center; n = len(sec)
    for i in range(n):
        j = (i+1) % n; nx, nr = edge_out(sec, i)
        for k in range(segs):
            a0, a1 = rot + 2*math.pi*k/segs, rot + 2*math.pi*(k+1)/segs
            def P(p, a): return (cx + side*p[0], cy + p[1]*math.cos(a), cz + p[1]*math.sin(a))
            am = (a0+a1)/2
            m.quad(P(sec[i], a0), P(sec[j], a0), P(sec[j], a1), P(sec[i], a1), mat,
                   (side*nx, nr*math.cos(am), nr*math.sin(am)))

# ---------------------------------------------------------------- 차량 형상 (단위 m, +Z 전방, 바닥 Y=0)
FRONT_AXLE, REAR_AXLE, TIRE_R, CAB_W = 3.90, -1.00, 0.46, 1.10

def cab_profile():
    """운전석 측면 윤곽(z,y). 앞바퀴 휠아치 절개 포함. 반환: (polygon, 휠아치 edge index 집합)."""
    ar, ay, by = 0.57, TIRE_R, 0.62
    th0 = math.asin((by-ay)/ar)
    pts = [(2.82, by)]
    arch_start = len(pts)
    for k in range(13):
        th = (math.pi - th0) + (th0 - (math.pi - th0)) * k/12
        pts.append((FRONT_AXLE + ar*math.cos(th), ay + ar*math.sin(th)))
    arch = set(range(arch_start, arch_start+12))
    pts += [(4.86, by), (4.95, 0.92), (4.97, 1.50), (4.93, 1.76),   # 하부 전면 경사
            (4.66, 2.56), (4.56, 2.70), (4.40, 2.78),                # 앞유리 경사 → 루프 페어링
            (3.05, 2.80), (2.86, 2.72), (2.82, 2.55)]
    return pts, arch

def taper(p):
    """벨트라인(1.85m) 위 측면 안쪽 기울기(텀블홈) + 전면 평면상 곡률."""
    x, y, z = p
    t = min(max((y-1.85)/0.95, 0), 1)
    x *= 1 - 0.07*t
    if z > 4.6: z -= 0.05*(x/CAB_W)**2 * min((z-4.6)/0.3, 1)
    return (x, y, z)

def build_meshes():
    W = CAB_W; meshes = []
    # 운전석 쉘
    cab = Mesh('CabShell'); prof, arch = cab_profile()
    prism(cab, prof, -W, W, 0.11, 'BodyWhite', {i: 'Chassis' for i in arch})
    prism(cab, [(2.72, 2.74), (3.70, 2.76), (3.05, 3.40), (2.72, 3.46)], -1.06, 1.06, 0.12, 'BodyWhite')  # 루프 에어 디플렉터
    for s in (-1, 1):  # 도어 분할선, 손잡이, 계단
        x0, x1 = (W-0.004, W+0.006) if s > 0 else (-W-0.006, -W+0.004)
        box(cab, x0, x1, 0.70, 2.50, 3.42, 3.44, 'Chassis', 0.002)
        box(cab, x0, x1, 0.70, 1.80, 4.62, 4.64, 'Chassis', 0.002)
        box(cab, *(sorted((s*(W+0.005), s*(W+0.035)))), 1.62, 1.66, 3.55, 3.75, 'Steel', 0.01)
        box(cab, *(sorted((s*(W-0.30), s*(W-0.02)))), 0.48, 0.53, 3.10, 3.42, 'Steel', 0.01)
        box(cab, *(sorted((s*(W-0.30), s*(W-0.02)))), 0.82, 0.86, 3.10, 3.42, 'Steel', 0.01)
    cab.deform(taper); meshes.append(cab)
    # 유리 (실제 표면 위 얇은 판)
    gl = Mesh('CabGlass')
    dn = (0.94, 0.33)  # 앞유리 경사 방향/바깥 법선 (z,y)
    p0, p1 = (4.925, 1.80), (4.66, 2.54)
    ws = [(p0[0]+dn[0]*0.004, p0[1]+dn[1]*0.004), (p0[0]+dn[0]*0.02, p0[1]+dn[1]*0.02),
          (p1[0]+dn[0]*0.02, p1[1]+dn[1]*0.02), (p1[0]+dn[0]*0.004, p1[1]+dn[1]*0.004)]
    prism(gl, ws, -0.96, 0.96, 0.006, 'Glass')
    for s in (-1, 1):
        xs = sorted((s*(W-0.005), s*(W+0.012)))
        prism(gl, [(3.55, 1.90), (4.68, 1.90), (4.48, 2.50), (3.55, 2.50)], *xs, 0.004, 'Glass')  # 도어창
        prism(gl, [(3.02, 2.02), (3.36, 2.02), (3.36, 2.48), (3.02, 2.48)], *xs, 0.004, 'Glass')  # 후방 측창
        prism(gl, [(4.78, 1.90), (4.88, 1.90), (4.70, 2.40)], *xs, 0.003, 'Glass')                 # 앞유리 모서리 쿼터창
    gl.deform(taper); meshes.append(gl)
    # 전면: 다단 그릴, 헤드램프, 범퍼, 와이퍼, 미러
    fr = Mesh('CabFront')
    for k in range(4):
        y = 1.08 + k*0.11
        box(fr, -0.62, 0.62, y, y+0.065, 4.94, 5.00, 'Chassis', 0.012)
    box(fr, -0.66, 0.66, 1.03, 1.52, 4.93, 4.965, 'Chassis', 0.01)
    for s in (-1, 1):
        box(fr, *(sorted((s*0.70, s*1.00))), 1.02, 1.24, 4.90, 4.985, 'Steel', 0.025)
        box(fr, *(sorted((s*0.72, s*0.86))), 1.05, 1.21, 4.95, 4.99, 'Glass', 0.01)
        box(fr, *(sorted((s*0.88, s*0.98))), 1.05, 1.13, 4.95, 4.99, 'Amber', 0.01)
        box(fr, *(sorted((s*0.05, s*0.80))), 1.82, 1.84, 4.95, 4.97, 'Chassis', 0.005)       # 와이퍼
        box(fr, *(sorted((s*1.05, s*1.30))), 2.30, 2.33, 4.40, 4.43, 'Steel', 0.01)           # 미러 암
        box(fr, *(sorted((s*1.28, s*1.32))), 1.95, 2.33, 4.40, 4.43, 'Steel', 0.01)
        box(fr, *(sorted((s*1.26, s*1.36))), 1.80, 2.22, 4.34, 4.50, 'Chassis', 0.03)         # 미러
    prism(fr, [(4.80, 0.42), (5.08, 0.46), (5.10, 0.70), (5.02, 0.84), (4.82, 0.84)], -1.12, 1.12, 0.05, 'Chassis')
    fr.deform(taper); meshes.append(fr)
    # 바퀴: 프로파일 타이어 + 접시형 림 + 허브 + 볼트
    wh = Mesh('Wheels')
    tire = [(-0.13, 0.285), (0.13, 0.285), (0.13, 0.39), (0.12, 0.43), (0.095, 0.455), (0.05, TIRE_R),
            (-0.05, TIRE_R), (-0.095, 0.455), (-0.12, 0.43), (-0.13, 0.39)]
    rim = [(-0.12, 0.09), (0.04, 0.09), (0.07, 0.17), (0.10, 0.25), (0.125, 0.275), (0.125, 0.295), (-0.12, 0.295)]
    hub = [(0.0, 0.0), (0.10, 0.0), (0.10, 0.065), (0.085, 0.085), (0.0, 0.095)]
    bolt = [(0.03, 0.0), (0.075, 0.0), (0.075, 0.016), (0.03, 0.016)]
    for s in (-1, 1):
        for x, z, outer in ((s*0.92, FRONT_AXLE, True), (s*0.79, REAR_AXLE, False), (s*1.07, REAR_AXLE, True)):
            c = (x, TIRE_R, z)
            lathe(wh, tire, c, s, 'Rubber', 32)
            lathe(wh, rim, c, s, 'Steel', 32)
            if outer:
                lathe(wh, hub, c, s, 'Chassis', 24)
                for b in range(8):
                    a = 2*math.pi*b/8
                    lathe(wh, bolt, (x, TIRE_R + 0.13*math.cos(a), z + 0.13*math.sin(a)), s, 'Steel', 8)
                for h in range(6):  # 림 통풍 홀 표시
                    a = 2*math.pi*(h+0.5)/6; yy, zz = TIRE_R + 0.20*math.cos(a), z + 0.20*math.sin(a)
                    box(wh, *(sorted((x+s*0.085, x+s*0.092))), yy-0.03, yy+0.03, zz-0.03, zz+0.03, 'Chassis', 0.008)
    meshes.append(wh)
    # 하부: 프레임 레일, 횡보, 차축, 연료탱크, 흙받이
    fm = Mesh('Underbody')
    for s in (-1, 1):
        box(fm, *(sorted((s*0.40, s*0.48))), 0.62, 0.88, -3.62, 4.78, 'Chassis', 0.008)
        box(fm, *(sorted((s*0.62, s*1.20))), 0.98, 1.02, -3.66, 2.66, 'Steel', 0.006)       # 적재함 하부 금속 띠
        box(fm, *(sorted((s*1.08, s*1.13))), 0.30, 1.00, -1.62, -1.56, 'Chassis', 0.01)    # 뒷바퀴 흙받이
        box(fm, *(sorted((s*1.08, s*1.13))), 0.30, 1.00, -0.44, -0.38, 'Chassis', 0.01)
        box(fm, *(sorted((s*0.62, s*1.21))), 0.92, 0.98, -1.62, -0.38, 'Chassis', 0.01)
        box(fm, *(sorted((s*1.10, s*1.14))), 0.45, 0.50, 0.40, 2.30, 'Steel', 0.01)          # 측면 언더가드
        box(fm, *(sorted((s*1.10, s*1.14))), 0.65, 0.70, 0.40, 2.30, 'Steel', 0.01)
    for z in (-3.3, -2.2, -1.0, 0.4, 1.6, 2.6, 4.4):
        box(fm, -0.48, 0.48, 0.66, 0.84, z-0.05, z+0.05, 'Chassis', 0.008)
    lathe(fm, [(0.0, 0.0), (1.85, 0.0), (1.85, 0.07), (0.0, 0.07)], (-0.925, TIRE_R, FRONT_AXLE), 1, 'Chassis', 12)
    lathe(fm, [(0.0, 0.0), (2.0, 0.0), (2.0, 0.10), (0.0, 0.10)], (-1.0, TIRE_R, REAR_AXLE), 1, 'Chassis', 12)
    lathe(fm, [(0.0, 0.0), (0.55, 0.0), (0.55, 0.22), (0.0, 0.22)], (0.50, 0.62, 1.30), 1, 'Steel', 24)
    box(fm, -1.18, 1.18, 0.40, 0.55, -3.70, -3.60, 'Chassis', 0.02)                         # 후방 언더런 가드
    meshes.append(fm)
    return meshes

def geometry_self_test(cab_final, wheel_final):
    """다각형 부호/분할/외향 법선 자체 시험. 하나라도 실패하면 ok=False."""
    r = {}
    sq = [(0, 0), (1, 0), (1, 1), (0, 1)]
    r['square_ccw_area'] = area2(sq); r['square_cw_area'] = area2(sq[::-1])
    r['square_ccw_tris'] = len(triangulate(sq)); r['square_cw_tris'] = len(triangulate(sq[::-1]))
    r['square_edge0_outward'] = edge_out(sq, 0)  # 아래 변 → (0,-1) 기대
    def tri_area(poly, ts):
        return sum(abs((poly[b][0]-poly[a][0])*(poly[c][1]-poly[a][1]) - (poly[b][1]-poly[a][1])*(poly[c][0]-poly[a][0]))/2 for a, b, c in ts)
    prof, _ = cab_profile(); ins = inset(prof, 0.11)
    for name, poly in (('cab_profile', prof), ('cab_inset', ins)):
        ts = triangulate(poly)
        r[name] = dict(vertices=len(poly), triangles=len(ts), polygon_area=abs(area2(poly)), triangle_area_sum=tri_area(poly, ts))
    # 손잡이/계단 등의 안쪽 면을 제외하도록 쉘 본체만 같은 방식(prism+taper)으로 다시 만들어 검사
    shell = Mesh('t'); prism(shell, prof, -CAB_W, CAB_W, 0.11, 'BodyWhite'); shell.deform(taper)
    side_bad = side_n = 0
    for a, b, c, _ in shell.tris:
        n = norm(cross(sub(b, a), sub(c, a)))
        if abs(n[0]) > 0.9 and min(abs(a[0]), abs(b[0]), abs(c[0])) > 0.8:
            side_n += 1; side_bad += (n[0] * a[0]) <= 0
    r['cab_sidewall_tris'] = side_n; r['cab_sidewall_inward'] = side_bad
    tread_bad = tread_n = 0
    for (a, b, c, mt), n in zip(wheel_final['_tris'], wheel_final['face_normals']):
        if mt != 'Rubber': continue
        cen = [(a[i]+b[i]+c[i])/3 for i in range(3)]
        ax = FRONT_AXLE if abs(cen[2]-FRONT_AXLE) < abs(cen[2]-REAR_AXLE) else REAR_AXLE
        rad = (0, cen[1]-TIRE_R, cen[2]-ax)
        if math.hypot(rad[1], rad[2]) > 0.44 and abs(n[0]) < 0.3:
            tread_n += 1; tread_bad += dot(n, rad) <= 0
    r['tire_tread_tris'] = tread_n; r['tire_tread_inward'] = tread_bad
    ok = (abs(r['square_ccw_area'] - 1) < 1e-9 and abs(r['square_cw_area'] + 1) < 1e-9
          and r['square_ccw_tris'] == 2 and r['square_cw_tris'] == 2
          and abs(r['square_edge0_outward'][0]) < 1e-9 and r['square_edge0_outward'][1] < 0
          and side_n > 0 and side_bad == 0 and tread_n > 0 and tread_bad == 0)
    for name in ('cab_profile', 'cab_inset'):
        d = r[name]
        ok = ok and d['vertices'] == 24 and d['triangles'] == 22 and abs(d['polygon_area'] - d['triangle_area_sum']) < 1e-6
    r['ok'] = ok
    return r

# ---------------------------------------------------------------- Unity native mesh (.asset)
def finalize(m, crease=40):
    """자동 스무딩 법선 + 재질별 서브메시. 반환 dict(verts, normals, uvs, submeshes[(mat, idx)])."""
    fn = [norm(cross(sub(b, a), sub(c, a))) for a, b, c, _ in m.tris]
    key = lambda p: (round(p[0], 4), round(p[1], 4), round(p[2], 4))
    groups = {}
    for t, (a, b, c, _) in enumerate(m.tris):
        for p in (a, b, c): groups.setdefault(key(p), []).append(t)
    cosl = math.cos(math.radians(crease)); mats = []
    for *_, mt in m.tris:
        if mt not in mats: mats.append(mt)
    verts, nrms, uvs, subs, lut = [], [], [], [], {}
    for mt in mats:
        idx = []
        for t, (a, b, c, tm) in enumerate(m.tris):
            if tm != mt: continue
            for p in (a, b, c):
                acc = [0., 0., 0.]
                for u in groups[key(p)]:
                    if dot(fn[u], fn[t]) >= cosl: acc = [acc[i] + fn[u][i] for i in range(3)]
                n = norm(acc)
                k = (key(p), round(n[0], 3), round(n[1], 3), round(n[2], 3), mt)
                if k not in lut:
                    lut[k] = len(verts); verts.append(p); nrms.append(n)
                    uvs.append(((p[0] + p[2]) * 0.5, p[1] * 0.5))
                idx.append(lut[k])
        subs.append((mt, idx))
    return dict(name=m.name, verts=verts, normals=nrms, uvs=uvs, subs=subs, face_normals=fn, _tris=m.tris)

def aabb(pts):
    lo = [min(p[i] for p in pts) for i in range(3)]; hi = [max(p[i] for p in pts) for i in range(3)]
    return [(lo[i]+hi[i])/2 for i in range(3)], [(hi[i]-lo[i])/2 for i in range(3)]

def v3(v): return '{x: %.6g, y: %.6g, z: %.6g}' % tuple(v)

def mesh_yaml(f):
    V = f['verts']; wide = len(V) > 65535; isz = 4 if wide else 2
    ib, subtxt, first = bytearray(), [], 0
    for _, idx in f['subs']:
        ib += struct.pack('<%d%s' % (len(idx), 'I' if wide else 'H'), *idx)
        c, e = aabb([V[i] for i in idx]); lo, hi = min(idx), max(idx)
        subtxt.append(f'''  - serializedVersion: 2
    firstByte: {first}
    indexCount: {len(idx)}
    topology: 0
    baseVertex: 0
    firstVertex: {lo}
    vertexCount: {hi-lo+1}
    localAABB:
      m_Center: {v3(c)}
      m_Extent: {v3(e)}
''')
        first += len(idx)*isz
    vb = b''.join(struct.pack('<8f', *p, *n, *uv) for p, n, uv in zip(V, f['normals'], f['uvs']))
    chans = {0: (0, 3), 1: (12, 3), 4: (24, 2)}  # position, normal, uv0 (stream 0, Float32)
    ch = ''.join('    - stream: 0\n      offset: %d\n      format: 0\n      dimension: %d\n' % chans.get(i, (0, 0)) for i in range(14))
    c, e = aabb(V)
    comp = ''.join(f'    {k}:\n      m_NumItems: 0\n' + ('      m_Range: 0\n      m_Start: 0\n' if r else '') +
                   '      m_Data: \n      m_BitSize: 0\n' for k, r in
                   [('m_Vertices', 1), ('m_UV', 1), ('m_Normals', 1), ('m_Tangents', 1), ('m_Weights', 0),
                    ('m_NormalSigns', 0), ('m_TangentSigns', 0), ('m_FloatColors', 1), ('m_BoneIndices', 0), ('m_Triangles', 0)])
    return HEADER + f'''--- !u!43 &4300000
Mesh:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_Name: {f['name']}
  serializedVersion: 11
  m_SubMeshes:
{''.join(subtxt)}  m_Shapes:
    vertices: []
    shapes: []
    channels: []
    fullWeights: []
  m_BindPose: []
  m_BoneNameHashes:
  m_RootBoneNameHash: 0
  m_BonesAABB: []
  m_VariableBoneCountWeights:
    m_Data:
  m_MeshCompression: 0
  m_IsReadable: 1
  m_KeepVertices: 1
  m_KeepIndices: 1
  m_IndexFormat: {1 if wide else 0}
  m_IndexBuffer: {ib.hex()}
  m_VertexData:
    serializedVersion: 3
    m_VertexCount: {len(V)}
    m_Channels:
{ch}    m_DataSize: {len(vb)}
    _typelessdata: {vb.hex()}
  m_CompressedMesh:
{comp}    m_UVInfo: 0
  m_LocalAABB:
    m_Center: {v3(c)}
    m_Extent: {v3(e)}
  m_MeshUsageFlags: 0
  m_CookingOptions: 30
  m_BakedConvexCollisionMesh:
  m_BakedTriangleCollisionMesh:
  m_MeshMetrics[0]: 1
  m_MeshMetrics[1]: 1
  m_MeshOptimizationFlags: 1
  m_StreamData:
    serializedVersion: 2
    offset: 0
    size: 0
    path:
'''

def reference_keys():
    ref = next((ROOT/'Library/PackageCache').glob('com.unity.render-pipelines.core*/Runtime/PathTracing/Meshes/6FaceSkyboxMesh.mesh'), None)
    if not ref: return None, []
    return ref, re.findall(r'^  (\w[\w\[\]]*):', ref.read_text(encoding='utf-8'), re.M)

# ---------------------------------------------------------------- prefab 편집
def parse_blocks(text):
    parts = re.split(r'^(--- !u!(\d+) &(\d+).*)$', text, flags=re.M)
    blocks = {}; order = []
    for i in range(1, len(parts), 4):
        fid = int(parts[i+2]); blocks[fid] = dict(cls=int(parts[i+1]), text=parts[i] + parts[i+3]); order.append(fid)
    return blocks, order

def go_info(blocks):
    info = {}
    for fid, b in blocks.items():
        if b['cls'] != 1: continue
        info[fid] = dict(name=re.search(r'm_Name: (.*)', b['text']).group(1).strip(),
                         comps=[int(c) for c in re.findall(r'component: \{fileID: (\d+)\}', b['text'])])
    return info

def go_yaml(gid, name, comps, father, mesh_guid=None, mats=(), collider=None):
    t = gid + 1
    s = f'''--- !u!1 &{gid}
GameObject:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  serializedVersion: 6
  m_Component:
''' + ''.join(f'  - component: {{fileID: {c}}}\n' for c in comps) + f'''  m_Layer: 0
  m_Name: {name}
  m_TagString: Untagged
  m_Icon: {{fileID: 0}}
  m_NavMeshLayer: 0
  m_StaticEditorFlags: 0
  m_IsActive: 1
--- !u!4 &{t}
Transform:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {gid}}}
  serializedVersion: 2
  m_LocalRotation: {{x: 0, y: 0, z: 0, w: 1}}
  m_LocalPosition: {{x: 0, y: 0, z: 0}}
  m_LocalScale: {{x: 1, y: 1, z: 1}}
  m_ConstrainProportionsScale: 0
  m_Children: []
  m_Father: {{fileID: {father}}}
  m_LocalEulerAnglesHint: {{x: 0, y: 0, z: 0}}
--- !u!33 &{gid+2}
MeshFilter:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {gid}}}
  m_Mesh: {{fileID: 4300000, guid: {mesh_guid}, type: 2}}
--- !u!23 &{gid+3}
MeshRenderer:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {gid}}}
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
''' + ''.join(f'  - {{fileID: 2100000, guid: {g}, type: 2}}\n' for g in mats) + '''  m_StaticBatchInfo:
    firstSubMesh: 0
    subMeshCount: 0
  m_StaticBatchRoot: {fileID: 0}
  m_ProbeAnchor: {fileID: 0}
  m_LightProbeVolumeOverride: {fileID: 0}
  m_ScaleInLightmap: 1
  m_ReceiveGI: 1
  m_SortingLayerID: 0
  m_SortingLayer: 0
  m_SortingOrder: 0
  m_AdditionalVertexStreams: {fileID: 0}
'''
    if collider:
        c, e = collider
        s += f'''--- !u!65 &{gid+4}
BoxCollider:
  m_ObjectHideFlags: 0
  m_CorrespondingSourceObject: {{fileID: 0}}
  m_PrefabInstance: {{fileID: 0}}
  m_PrefabAsset: {{fileID: 0}}
  m_GameObject: {{fileID: {gid}}}
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
  m_Size: {v3([2*x for x in e])}
  m_Center: {v3(c)}
'''
    return s

# ---------------------------------------------------------------- 소프트웨어 프리뷰 (실제 메시 삼각형 투영)
def quat_rot(q, v):
    x, y, z, w = q; u = (x, y, z)
    t = cross(u, v); t = (2*t[0], 2*t[1], 2*t[2]); c = cross(u, t)
    return (v[0] + w*t[0] + c[0], v[1] + w*t[1] + c[1], v[2] + w*t[2] + c[2])

def node_world(blocks, tid, p):
    while tid:
        t = blocks[tid]['text']
        f = lambda k: [float(x) for x in re.search(k + r': \{x: ([-\d.e]+), y: ([-\d.e]+), z: ([-\d.e]+)', t).groups()]
        q = [float(x) for x in re.search(r'm_LocalRotation: \{x: ([-\d.e]+), y: ([-\d.e]+), z: ([-\d.e]+), w: ([-\d.e]+)', t).groups()]
        s, pos = f('m_LocalScale'), f('m_LocalPosition')
        p = quat_rot(q, (p[0]*s[0], p[1]*s[1], p[2]*s[2])); p = (p[0]+pos[0], p[1]+pos[1], p[2]+pos[2])
        tid = int(re.search(r'm_Father: \{fileID: (\d+)', t).group(1))
    return p

def preview_tris(blocks, info, mat_color, finals):
    tris = []
    for f in finals:
        for mt, idx in f['subs']:
            for k in range(0, len(idx), 3):
                tris.append(([f['verts'][i] for i in idx[k:k+3]], mat_color[mt]))
    unit = Mesh('u'); box(unit, -.5, .5, -.5, .5, -.5, .5, 'x', 0)
    for gid, g in info.items():
        mf = [blocks[c]['text'] for c in g['comps'] if blocks[c]['cls'] == 33]
        mr = [blocks[c]['text'] for c in g['comps'] if blocks[c]['cls'] == 23]
        if not mf or 'e000000000000000' not in mf[0]: continue
        mid = int(re.search(r'm_Mesh: \{fileID: (\d+)', mf[0]).group(1))
        mg = re.search(r'm_Materials:\n  - \{fileID: 2100000, guid: (\w+)', mr[0]).group(1)
        col = mat_color.get(mg, (200, 200, 200))
        if mid == 10202: src = [(a, b, c) for a, b, c, _ in unit.tris]
        elif mid == 10210: src = [((-.5, -.5, 0), (.5, -.5, 0), (.5, .5, 0)), ((-.5, -.5, 0), (.5, .5, 0), (-.5, .5, 0))]
        else: continue
        for tr in src:
            w = [node_world(blocks, g['comps'][0], p) for p in tr]
            tris.append((w, col));
            if mid == 10210: tris.append(([w[0], w[2], w[1]], col))
    return tris

def render(tris, yaw, pitch, path, size=(1400, 820)):
    cy, sy, cp, sp = math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    def V(p):
        x, z = p[0]*cy - p[2]*sy, p[0]*sy + p[2]*cy
        return (x, p[1]*cp - z*sp, p[1]*sp + z*cp)
    L = norm((0.45, 0.8, 0.55)); items = []
    for pts, col in tris:
        n = norm(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])))
        q = [V(p) for p in pts]
        if V(n)[2] <= 0: continue
        sh = 0.38 + 0.62*max(0, dot(n, L))
        items.append((sum(p[2] for p in q)/3, q, tuple(int(min(255, c*sh)) for c in col)))
    xs = [p[0] for _, q, _ in items for p in q]; ys = [p[1] for _, q, _ in items for p in q]
    sc = 0.9*min(size[0]/(max(xs)-min(xs)), size[1]/(max(ys)-min(ys)))
    ox, oy = size[0]/2 - sc*(max(xs)+min(xs))/2, size[1]/2 + sc*(max(ys)+min(ys))/2
    im = Image.new('RGB', size, (58, 62, 70)); d = ImageDraw.Draw(im)
    for _, q, col in sorted(items, key=lambda t: t[0]):
        d.polygon([(ox + sc*p[0], oy - sc*p[1]) for p in q], fill=col)
    im.save(path)

# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--output', required=True, help='Assets 밖 임시 폴더')
    ap.add_argument('--apply', action='store_true', help='검증 통과 시 Assets/LogisticsVehicles에 복사')
    a = ap.parse_args()
    out = Path(a.output).resolve(); stage = out/'LogisticsVehicles'
    if VEH.resolve() in out.parents or out == VEH.resolve(): raise SystemExit('--output은 Assets/LogisticsVehicles 밖이어야 합니다')
    (stage/'Meshes').mkdir(parents=True, exist_ok=True); (out/'Preview').mkdir(parents=True, exist_ok=True)

    text = PREFAB.read_text(encoding='utf-8'); blocks, order = parse_blocks(text); info = go_info(blocks)
    orig = {fid: blocks[fid]['text'] for fid in order}
    mat_guid, mat_color = {}, {}
    for mp in (VEH/'Materials').glob('*.mat'):
        g = re.search(r'guid: (\w+)', Path(str(mp)+'.meta').read_text()).group(1)
        c = re.search(r'_BaseColor: \{r: ([\d.e-]+), g: ([\d.e-]+), b: ([\d.e-]+)', mp.read_text(encoding='utf-8'))
        mat_guid[mp.stem] = g; mat_color[mp.stem] = mat_color[g] = tuple(int(255*min(1, float(x))) for x in c.groups())

    # 기존 큐브 운전석/바퀴/차대 제거
    removed = []
    for gid, g in list(info.items()):
        if g['name'] in REMOVE and gid not in KEEP_IDS:
            for fid in [gid] + g['comps']: blocks.pop(fid, None)
            removed.append(g['name']); del info[gid]
    for gid, g in info.items():  # 운전석 로고를 새 측면(x=±1.10)에 밀착
        if g['name'] == 'CabDoorLogo':
            t = g['comps'][0]
            blocks[t]['text'] = re.sub(r'm_LocalPosition: \{x: (-?)[\d.]+', lambda m: f'm_LocalPosition: {{x: {m.group(1)}1.106', blocks[t]['text'])

    meshes = build_meshes(); finals = [finalize(m) for m in meshes]
    new_children, add_text, files = [], [], []
    for k, f in enumerate(finals):
        g = guid('Meshes/' + f['name'] + '.asset'); p = stage/'Meshes'/(f['name'] + '.asset')
        p.write_text(mesh_yaml(f), encoding='utf-8')
        Path(str(p)+'.meta').write_text(f'fileFormatVersion: 2\nguid: {g}\nNativeFormatImporter:\n  externalObjects: {{}}\n  mainObjectFileID: 4300000\n  userData: \n  assetBundleName: \n  assetBundleVariant: \n', encoding='utf-8')
        files += [p, Path(str(p)+'.meta')]
        gid = 900000 + k*10; coll = aabb(f['verts']) if f['name'] == 'CabShell' else None
        comps = [gid+1, gid+2, gid+3] + ([gid+4] if coll else [])
        add_text.append(go_yaml(gid, f['name'], comps, 11, g, [mat_guid[mt] for mt, _ in f['subs']], coll))
        new_children.append(gid+1)
        (out/'Preview'/(f['name'] + '.obj')).write_text(''.join('v %.5f %.5f %.5f\n' % (-v[0], v[1], v[2]) for v in f['verts']) +
            ''.join('f %d %d %d\n' % (i[k2]+1, i[k2+2]+1, i[k2+1]+1) for _, i in f['subs'] for k2 in range(0, len(i), 3)), encoding='utf-8')

    rt = blocks[11]['text']
    kids = [int(c) for c in re.findall(r'- \{fileID: (\d+)\}', rt.split('m_Children:')[1].split('m_Father')[0])]
    kids = [c for c in kids if c in blocks] + new_children
    blocks[11]['text'] = re.sub(r'  m_Children:.*?(?=  m_Father)', '  m_Children:\n' + ''.join(f'  - {{fileID: {c}}}\n' for c in kids), rt, flags=re.S)
    prefab_text = HEADER + ''.join(blocks[f]['text'] for f in order if f in blocks) + ''.join(add_text)
    (stage/'WingBody5t_Coupang.prefab').write_text(prefab_text, encoding='utf-8'); files.append(stage/'WingBody5t_Coupang.prefab')

    # 검증
    nb, _ = parse_blocks(prefab_text); ninfo = go_info(nb)
    ids = re.findall(r'^--- !u!\d+ &(\d+)', prefab_text, re.M)
    refs = {int(r) for r in re.findall(r'\{fileID: (\d+)\}', prefab_text)} - {0}
    keep_names = [n for n in ('PalletSlots', 'RearLoadingApproach', 'DockReference')] + [f'PalletSlot_{i:02}' for i in range(1, 11)]
    kept_ok = all(nb.get(f, {}).get('text') == orig[f] for gid, g in go_info(parse_blocks(text)[0]).items()
                  if g['name'] in keep_names for f in [gid] + g['comps'])
    piv_ok = all(nb.get(t, {}).get('cls') == 4 and re.search(r'm_LocalPosition.*', nb[t]['text']).group(0) == re.search(r'm_LocalPosition.*', orig[t]).group(0)
                 for t in KEEP_IDS.values())
    cf = next(g for g in ninfo.values() if g['name'] == 'CargoFloor')
    ct = nb[cf['comps'][0]]['text']
    floor_top = float(re.search(r'm_LocalPosition: \{x: [-\d.e]+, y: ([-\d.e]+)', ct).group(1)) + float(re.search(r'm_LocalScale: \{x: [-\d.e]+, y: ([-\d.e]+)', ct).group(1))/2
    scene_refs = {}
    for sc in (ROOT/'Assets').rglob('*.unity'):
        for r in set(re.findall(r'fileID: (\d+), guid: ' + PREFAB_GUID, sc.read_text(encoding='utf-8', errors='ignore'))):
            scene_refs.setdefault(sc.name, []).append(int(r))
    missing_scene = {k: sorted(r for r in v if r != 100100000 and r not in nb) for k, v in scene_refs.items()}
    mesh_checks = []
    for f in finals:
        n = len(f['verts']); all_idx = [i for _, idx in f['subs'] for i in idx]
        mesh_checks.append(dict(name=f['name'], vertices=n, triangles=len(all_idx)//3, submeshes=[m for m, _ in f['subs']],
            indices_valid=all(0 <= i < n for i in all_idx) and len(all_idx) % 3 == 0,
            zero_normals=sum(1 for v in f['normals'] if dot(v, v) < 0.5),
            bounds=aabb(f['verts'])))
    ref, ref_keys = reference_keys()
    my_keys = re.findall(r'^  (\w[\w\[\]]*):', mesh_yaml(finals[0]), re.M)
    allv = [v for f in finals for v in f['verts']] + [(0, 0, 0)]
    report = dict(
        removed_cube_objects=sorted(set(removed)), new_mesh_objects=[f['name'] for f in finals],
        root_and_pivots_preserved=piv_ok and nb[10]['cls'] == 1, pivot_ids=KEEP_IDS,
        slots_and_references_unchanged=kept_ok, cargo_floor_top=floor_top, cargo_floor_ok=abs(floor_top-1.1) < 1e-6,
        duplicate_ids=len(ids) != len(set(ids)), local_refs_valid=all(str(r) in ids for r in refs),
        scene_referenced_ids=scene_refs, scene_ids_missing=missing_scene,
        overall_length=max(v[2] for v in allv) - (-3.75), meshes=mesh_checks,
        mesh_format_reference=str(ref), mesh_top_level_keys_match=(my_keys == ref_keys) if ref else None,
        preview='실제 생성 메시 삼각형 + prefab에 남은 Unity 기본 Cube/Quad를 Transform대로 투영. 로고 텍스처는 생략(재질 단색).',
        unity_import_verified=False)
    fmap = {f['name']: f for f in finals}
    report['geometry_self_test'] = geometry_self_test(fmap['CabShell'], fmap['Wheels'])
    report['ok'] = all([report['geometry_self_test']['ok'],report['root_and_pivots_preserved'], kept_ok, report['cargo_floor_ok'], not report['duplicate_ids'],
                        report['local_refs_valid'], not any(missing_scene.values()), all(m['indices_valid'] for m in mesh_checks),
                        report['mesh_top_level_keys_match'] is not False])
    pv = preview_tris(nb, ninfo, mat_color, finals)
    for name, yaw, pitch in (('front', 0, 0), ('side', 90, 0), ('oblique', 35, 18), ('oblique_rear', 215, 18)):
        render(pv, yaw, pitch, out/'Preview'/f'{name}.png')
    report['files'] = [str(p.relative_to(out)) for p in files]
    (out/'validation.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: report[k] for k in ('ok', 'removed_cube_objects', 'cargo_floor_top', 'scene_ids_missing', 'mesh_top_level_keys_match')}, ensure_ascii=False))

    if a.apply:
        if not report['ok']: raise SystemExit('검증 실패: --apply 중단')
        bk = out/'backup'; bk.mkdir(exist_ok=True); shutil.copy2(PREFAB, bk/PREFAB.name)
        (VEH/'Meshes').mkdir(exist_ok=True)
        for p in files: shutil.copy2(p, VEH/p.relative_to(stage))  # prefab .meta(GUID)는 건드리지 않음
        print('applied; backup:', bk/PREFAB.name)

if __name__ == '__main__':
    main()
