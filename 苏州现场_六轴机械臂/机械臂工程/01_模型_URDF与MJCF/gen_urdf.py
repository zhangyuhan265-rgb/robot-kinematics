# -*- coding: utf-8 -*-
"""生成纳博特六轴机械臂 URDF。

运动学（由 12 组真机样本反解并验证，位置残差最大 0.003 mm，**不要改这一段**）：

    i    θ      d(mm)   a(mm)   α(度)   零位偏置(度)
    1   θ1      377     216     +90         0
    2   θ2        0     900       0       -90
    3   θ3        0     130     +90         0
    4   θ4      934       0     +90         0
    5   θ5        0       0     +90       +90
    6   θ6      154       0       0      +180

T_i = Rot_z(θ_i)·Trans_z(d_i)·Trans_x(a_i)·Rot_x(α_i)，θ_i = q_i - off_i。

URDF 关节 origin 的推导（关键点：DH 的 Rot_z 在最左边，URDF 的转动在最右边，
所以第 i 个关节的 origin 用的是第 i-1 个 DH 的常数项）：
    U_i = F_{i-1}·Rot_z(q_i)                  ← URDF 的 link i 坐标系
    joint i 的 origin = C_{i-1},  C_0 = I
    C_k = Rot_z(-off_k)·Trans_z(d_k)·Trans_x(a_k)·Rot_x(α_k)
    tcp = U_6 · C_6
结果：URDF 关节角 == 控制器关节角 q，零位偏置已经吃进 origin，不用再换。

外形（2026-11-06 第二版）：按现场照片把占位几何做像真机——
两级圆柱基座、弯曲的铸件大臂、侧置电机鼓包、锥形小臂、紧凑腕部和法兰盘。
**纯视觉/碰撞，不影响运动学。**
"""
import math

MM = 0.001
DEG = math.pi / 180.0

A = [216.0, 900.0, 130.0, 0.0, 0.0, 0.0]
DD = [377.0, 0.0, 0.0, 934.0, 0.0, 0.0]
DD[5] = 154.0
ALPHA = [90.0, 0.0, 90.0, 90.0, 90.0, 0.0]
OFF = [0.0, -90.0, 0.0, 0.0, 90.0, 180.0]
LIMITS = [(-130.0, 130.0)] * 5 + [(-360.0, 360.0)]
# 控制器 get_robot_joint_param 读回来的额定关节速度（度/秒）：
#   ratedVel = 电机额定转速(rpm) / 减速比 × 6
#   99.1736 = 2000/121×6, 148.7603 = 3000/121×6, 225 = 3000/80×6, 111.8012 = 3000/161×6
RATED_VEL_DEG = [99.173553719008, 99.173553719008, 148.760330578512,
                 225.0, 111.801242236025, 225.0]
# 扭矩：控制器不提供，这里是占位值，量力而行前必须查驱动器铭牌
EFFORT_PLACEHOLDER = 500.0

# ---------------------------------------------------------------- 外形尺寸
# 单位米。都按现场照片的比例估的，不参与运动学。
BASE_R = 0.155          # 基座圆柱半径


def ident():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def mm(X, Y):
    return [[sum(X[i][k] * Y[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rotz(t):
    c, s = math.cos(t), math.sin(t)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def rotx(t):
    c, s = math.cos(t), math.sin(t)
    return [[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]]


def trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def C(k):
    T = rotz(-OFF[k] * DEG)
    T = mm(T, trans(0, 0, DD[k] * MM))
    T = mm(T, trans(A[k] * MM, 0, 0))
    T = mm(T, rotx(ALPHA[k] * DEG))
    return T


def rpy_from_R(R):
    pitch = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    if abs(abs(pitch) - math.pi / 2) < 1e-9:
        return 0.0, pitch, math.atan2(-R[0][1], R[1][1])
    return math.atan2(R[2][1], R[2][2]), pitch, math.atan2(R[1][0], R[0][0])


def xyz_rpy(T):
    return (T[0][3], T[1][3], T[2][3]), rpy_from_R([r[:3] for r in T[:3]])


def fmt(v):
    if abs(v) < 5e-10:
        v = 0.0
    s = ("%.9f" % v).rstrip("0").rstrip(".")
    return s if s not in ("", "-0") else "0"


def v3(t):
    return " ".join(fmt(x) for x in t)


def dir_rpy(d):
    """把 +z 对到方向 d 所需的 (roll=0, pitch, yaw)"""
    L = math.sqrt(sum(x * x for x in d))
    if L < 1e-12:
        return (0.0, 0.0, 0.0), 0.0
    dz = max(-1.0, min(1.0, d[2] / L))
    pitch = math.acos(dz)
    yaw = math.atan2(d[1], d[0]) if math.hypot(d[0], d[1]) > 1e-12 else 0.0
    return (0.0, pitch, yaw), L


# ---------------------------------------------------------------- 几何图元
def cyl(r, p0, p1):
    return dict(kind="cyl", r=r, p0=p0, p1=p1)


def sphere(r, c):
    return dict(kind="sphere", r=r, c=c)


def box(sx, sy, sz, c, rpy=(0.0, 0.0, 0.0)):
    return dict(kind="box", size=(sx, sy, sz), c=c, rpy=rpy)


def arc_boxes(p0, p1, bulge, sag, n, w, h):
    """沿二次贝塞尔曲线摆一串盒子，用来近似弯曲铸件。
    bulge：侧向单位向量；sag：拱高。"""
    p0 = [float(x) for x in p0]
    p1 = [float(x) for x in p1]
    ctrl = [(p0[i] + p1[i]) / 2.0 + bulge[i] * sag * 2.0 for i in range(3)]

    def bez(t):
        u = 1 - t
        return [u * u * p0[i] + 2 * u * t * ctrl[i] + t * t * p1[i] for i in range(3)]

    pts = [bez(k / float(n)) for k in range(n + 1)]
    out = []
    for k in range(n):
        a, b = pts[k], pts[k + 1]
        d = [b[i] - a[i] for i in range(3)]
        L = math.sqrt(sum(x * x for x in d))
        rpy, _ = dir_rpy(d)
        c = [(a[i] + b[i]) / 2.0 for i in range(3)]
        # 盒子沿局部 z 长度 L，横截面 w×h
        out.append(dict(kind="box", size=(w, h, L + 0.004), c=c, rpy=rpy))
    return out


# ---------------------------------------------------------------- 各 link 外形
# 坐标：link1 = 基座底面中心，z 向上
#       link2 = 肩点，肘在 (0, 0.9, 0)
#       link3 = 肘点，腕轴在 (0.13, 0, 0)
#       link4 = 小臂根，腕心在 (0, 0, 0.934)
#       link5 = 腕心（零长度）
#       link6 = 腕心，法兰端在 (0, 0, 0.154)
SHAPES = {
    "base_link": [
        cyl(0.215, (0, 0, 0.0), (0, 0, 0.040)),          # 安装法兰
        cyl(0.165, (0, 0, 0.040), (0, 0, 0.215)),        # 基座下段
        cyl(0.140, (0, 0, 0.215), (0, 0, 0.260)),        # 基座上段
    ],
    "link1": [
        cyl(0.128, (0, 0, 0.255), (0, 0, 0.340)),        # 回转体
        box(0.310, 0.255, 0.155, (0.095, 0, 0.372)),     # 肩部壳体
        cyl(0.112, (0.060, 0, 0.377), (0.216, 0, 0.377)),  # 肩部横臂（L6=216 的真实用途）
        cyl(0.070, (0.150, 0.130, 0.377), (0.150, 0.130, 0.470)),  # 线缆出口
    ],
    "link2": (
        # 弯曲的铸件大臂：从肩到肘，向 -x 方向拱 62mm
        arc_boxes((0, 0.0, 0), (0, 0.900, 0), (-1.0, 0.0, 0.0), 0.062, 7, 0.165, 0.190)
        + [
            cyl(0.108, (0, 0, -0.115), (0, 0, 0.115)),    # J2 电机鼓包（沿关节轴）
            cyl(0.100, (0, 0.900, -0.110), (0, 0.900, 0.110)),  # J3 侧鼓包
        ]
    ),
    "link3": [
        box(0.215, 0.215, 0.175, (0.065, 0, 0)),          # 肘部块
        cyl(0.100, (0, 0, -0.155), (0, 0, 0.110)),        # J3 电机（朝侧面，露出一截）
    ],
    "link4": [
        cyl(0.076, (0, 0, 0.000), (0, 0, 0.070)),         # 根部法兰
        cyl(0.062, (0, 0, 0.060), (0, 0, 0.560)),         # 小臂后段
        cyl(0.052, (0, 0, 0.550), (0, 0, 0.905)),         # 小臂前段（锥形管的近似）
    ],
    "link5": [
        box(0.145, 0.145, 0.165, (0, 0, 0.0)),            # 腕部壳体（近立方，绕轴转多少度都一样）
        cyl(0.075, (-0.150, 0, 0), (0.085, 0, 0)),        # J5 电机（侧面鼓包）
    ],
    "link6": [
        box(0.145, 0.120, 0.120, (0, 0, 0.075)),          # 腕端"鸭嘴"壳体
        cyl(0.058, (0, 0, 0.115), (0, 0, 0.152)),         # 法兰盘
        cyl(0.030, (0, 0, 0.150), (0, 0, 0.158)),         # 法兰凸台
    ],
}

# 碰撞体：比视觉简化，够 MoveIt 用
COLLIDE = {
    "base_link": [cyl(0.155, (0, 0, 0.0), (0, 0, 0.250))],
    "link1": [cyl(0.125, (0, 0, 0.245), (0, 0, 0.330)),
              box(0.320, 0.260, 0.160, (0.100, 0, 0.372)),
              cyl(0.112, (0.060, 0, 0.377), (0.216, 0, 0.377))],
    "link2": [cyl(0.075, (0, 0, 0), (0, 0.900, 0))],
    "link3": [box(0.240, 0.240, 0.225, (0.065, 0, 0))],
    "link4": [cyl(0.062, (0, 0, 0.0), (0, 0, 0.905))],
    "link5": [box(0.170, 0.145, 0.185, (0, 0, 0.0))],
    "link6": [cyl(0.055, (0, 0, 0.0), (0, 0, 0.150))],
}

# 惯量估算用的等效圆柱： (半径, 长度, 质量)
INERTIA = {
    "base_link": (0.155, 0.250, 45.0),
    "link1": (0.125, 0.330, 25.0),
    "link2": (0.080, 0.900, 30.0),
    "link3": (0.120, 0.230, 12.0),
    "link4": (0.060, 0.934, 14.0),
    "link5": (0.085, 0.180, 6.0),
    "link6": (0.055, 0.154, 3.0),
}

MAT = {
    "base_link": "metal_dark",
    "link1": "metal_dark",
    "link2": "metal_light",
    "link3": "metal_dark",
    "link4": "metal_light",
    "link5": "metal_dark",
    "link6": "metal_flange",
}


def emit_primitive(f, p, tag):
    if p["kind"] == "cyl":
        d = [p["p1"][i] - p["p0"][i] for i in range(3)]
        rpy, L = dir_rpy(d)
        c = [(p["p0"][i] + p["p1"][i]) / 2.0 for i in range(3)]
        geo = '<cylinder radius="%s" length="%s"/>' % (fmt(p["r"]), fmt(L))
    elif p["kind"] == "sphere":
        rpy, L, c = (0.0, 0.0, 0.0), 0.0, p["c"]
        geo = '<sphere radius="%s"/>' % fmt(p["r"])
    else:
        rpy, c = p["rpy"], p["c"]
        geo = '<box size="%s"/>' % v3(p["size"])
    f.write('      <origin xyz="%s" rpy="%s"/>\n' % (v3(c), v3(rpy)))
    f.write('      <geometry>%s</geometry>\n' % geo)


def emit_link(f, name):
    r, L, m = INERTIA[name]
    ixx = m * (3 * r * r + L * L) / 12.0
    izz = m * r * r / 2.0
    f.write('  <link name="%s">\n' % name)
    f.write('    <inertial>\n')
    f.write('      <origin xyz="0 0 0" rpy="0 0 0"/>\n')
    f.write('      <mass value="%s"/>\n' % fmt(m))
    f.write('      <inertia ixx="%s" ixy="0" ixz="0" iyy="%s" iyz="0" izz="%s"/>\n'
            % (fmt(ixx), fmt(ixx), fmt(izz)))
    f.write('    </inertial>\n')
    for p in SHAPES[name]:
        f.write('    <visual>\n')
        emit_primitive(f, p, "visual")
        f.write('      <material name="%s"/>\n' % MAT[name])
        f.write('    </visual>\n')
    for p in COLLIDE[name]:
        f.write('    <collision>\n')
        emit_primitive(f, p, "collision")
        f.write('    </collision>\n')
    f.write('  </link>\n\n')


def main():
    import io
    import sys
    buf = io.StringIO()

    # --lengths d1,a1,a2,a3,d4,d6  用实测值覆盖（单位 mm）
    if "--lengths" in sys.argv:
        raw = sys.argv[sys.argv.index("--lengths") + 1]
        vals = [float(v) for v in raw.split(",")]
        if len(vals) != 6:
            raise SystemExit("--lengths 需要 6 个数：d1,a1,a2,a3,d4,d6")
        global AA, DD
        AA = [vals[1], vals[2], vals[3], 0.0, 0.0, 0.0]
        DD = [vals[0], 0.0, 0.0, vals[4], 0.0, vals[5]]
        print("!! 已用实测长度覆盖：d=%s  a=%s" % (DD[:3], AA[:3]))
        print("!! 注意：控制器里的 L1..L6 没变，改完后 URDF 与控制器会不一致，"
              "Cartesian 指令会偏，必须同步改控制器参数表")

    def w(s=""):
        buf.write(s + "\n")

    w('<?xml version="1.0"?>')
    w('<!--')
    w('  纳博特六轴机械臂  控制器 192.168.2.14 / SDK 2.0.4-20260202')
    w('  运动学由 12 组真机 (关节角, TCP) 样本反解，位置残差最大 0.003 mm。')
    w('')
    w('  DH 表（标准/远端 DH: T = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)）')
    w('    i    θ     d(mm)   a(mm)  α(deg)  off(deg)')
    for i in range(6):
        w('    %d   θ%d   %6.1f  %6.1f   %+4.0f    %+5.0f'
          % (i + 1, i + 1, DD[i], A[i], ALPHA[i], OFF[i]))
    w('')
    w('  重要：URDF 关节角 == 控制器关节角 q，零位偏置已折进各 joint 的 origin。')
    w('        tcp 帧 == 控制器 get_current_position(fd, 1, ...) 的坐标系（TCP=法兰）。')
    w('')
    w('  外形按现场照片近似（两级基座 / 弯曲铸件大臂 / 侧置电机 / 锥形小臂），')
    w('  由基元拼装，只影响 visual 与 collision，不参与运动学。')
    w('  惯量按等效圆柱+假定质量推算，不是实测值，做动力学前必须替换。')
    w('-->')
    w('<robot name="nbt_arm6">')
    w('')
    for nm, rgba in (("metal_dark", "0.62 0.63 0.65 1"),
                     ("metal_light", "0.88 0.87 0.84 1"),
                     ("metal_flange", "0.46 0.47 0.50 1")):
        w('  <material name="%s">' % nm)
        w('    <color rgba="%s"/>' % rgba)
        w('  </material>')
    w('')

    Cs = [C(k) for k in range(6)]
    order = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]
    idx = {n: i for i, n in enumerate(order)}

    def write_link(name):
        if name == "base_link":
            r, L, m = INERTIA[name]
            ixx = m * (3 * r * r + L * L) / 12.0
            izz = m * r * r / 2.0
            w('  <link name="base_link">')
            w('    <inertial>')
            w('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            w('      <mass value="%s"/>' % fmt(m))
            w('      <inertia ixx="%s" ixy="0" ixz="0" iyy="%s" iyz="0" izz="%s"/>'
              % (fmt(ixx), fmt(ixx), fmt(izz)))
            w('    </inertial>')
            for p in SHAPES[name]:
                w('    <visual>')
                tmp = io.StringIO()
                emit_primitive(tmp, p, "visual")
                for line in tmp.getvalue().rstrip("\n").split("\n"):
                    w(line)
                w('      <material name="metal_dark"/>')
                w('    </visual>')
            for p in COLLIDE[name]:
                w('    <collision>')
                tmp = io.StringIO()
                emit_primitive(tmp, p, "collision")
                for line in tmp.getvalue().rstrip("\n").split("\n"):
                    w(line)
                w('    </collision>')
            w('  </link>')
            w('')
        else:
            f = io.StringIO()
            emit_link(f, name)
            buf.write(f.getvalue())

    write_link("base_link")

    for i in range(6):
        parent = order[i]
        child = "link%d" % (i + 1)
        T = ident() if i == 0 else Cs[i - 1]
        xyz, rpy = xyz_rpy(T)
        lo, hi = LIMITS[i]
        w('  <joint name="joint%d" type="revolute">' % (i + 1))
        w('    <parent link="%s"/>' % parent)
        w('    <child link="%s"/>' % child)
        w('    <origin xyz="%s" rpy="%s"/>' % (v3(xyz), v3(rpy)))
        w('    <axis xyz="0 0 1"/>')
        w('    <limit lower="%s" upper="%s" effort="%s" velocity="%s"/>'
          % (fmt(lo * DEG), fmt(hi * DEG), fmt(EFFORT_PLACEHOLDER),
             fmt(RATED_VEL_DEG[i] * DEG)))
        w('  </joint>')
        w('')
        write_link(child)

    xyz, rpy = xyz_rpy(Cs[5])
    w('  <!-- TCP = 法兰：与控制器 get_current_position(fd, 1, ...) 同一坐标系 -->')
    w('  <joint name="tcp_joint" type="fixed">')
    w('    <parent link="link6"/>')
    w('    <child link="tcp"/>')
    w('    <origin xyz="%s" rpy="%s"/>' % (v3(xyz), v3(rpy)))
    w('  </joint>')
    w('')
    w('  <link name="tcp">')
    w('    <!-- 空帧也有个极小惯量：不给的话 pybullet 等后端会按 mass=1、惯量=1 顶替 -->')
    w('    <inertial>')
    w('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    w('      <mass value="0.05"/>')
    w('      <inertia ixx="1e-5" ixy="0" ixz="0" iyy="1e-5" iyz="0" izz="1e-5"/>')
    w('    </inertial>')
    w('  </link>')
    w('</robot>')

    text = buf.getvalue()
    path = sys.argv[1] if len(sys.argv) > 1 else "nbt_arm6.urdf"
    with open(path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(text)
    nvis = sum(len(v) for v in SHAPES.values())
    print("已写 %s（%d 行，%d 个视觉基元）" % (path, text.count("\n"), nvis))
    for i in range(6):
        T = ident() if i == 0 else Cs[i - 1]
        x, r = xyz_rpy(T)
        print("  joint%d  xyz=(%9.4f %9.4f %9.4f) m   rpy=(%8.3f %8.3f %8.3f) deg"
              % (i + 1, x[0], x[1], x[2], r[0] / DEG, r[1] / DEG, r[2] / DEG))
    x, r = xyz_rpy(Cs[5])
    print("  tcp     xyz=(%9.4f %9.4f %9.4f) m   rpy=(%8.3f %8.3f %8.3f) deg"
          % (x[0], x[1], x[2], r[0] / DEG, r[1] / DEG, r[2] / DEG))


if __name__ == "__main__":
    main()
