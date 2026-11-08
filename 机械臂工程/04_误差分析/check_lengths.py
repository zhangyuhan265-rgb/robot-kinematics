# -*- coding: utf-8 -*-
"""
全链路 FK + 用"命令 300mm / 实走 260mm"这条硬证据去校长度。

为什么需要这个：
  彩页给 J4->J5 = 650，而现场量尺 J3->法兰 = 900，两者直接冲突
  （d4=650 时，home 位姿下肘到法兰只有 ~724，差 170+ mm）。
  量尺那次多半不是 home 位姿，所以量尺 900 不能直接反解 d4。

  改用最硬的那条证据：命令 300.009 mm，实际走 ≈260 mm。
  它的两个端点关节角是控制器读出来的（编码器），可信；
  距离是皮尺，也不依赖端点识别。→ 用它反解长度。

用法：python check_lengths.py
"""
import csv
import os
import numpy as np
from math import pi

# ---------------- 基础变换 ----------------
def Rz(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def Tr(x, y, z):
    M = np.eye(4); M[:3, 3] = (x, y, z); return M

def origin(x, y, z, rpy):
    """URDF <origin> = Trans(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll)"""
    roll, pitch, yaw = rpy          # 注意：不能叫 y，会和平移的 y 撞名
    M = Tr(x, y, z)
    M[:3, :3] = (Rz(yaw) @ Ry(pitch) @ Rx(roll))[:3, :3]
    return M

# ---------------- URDF 链路（单位 mm）----------------
# j1  (0,0,0)  rpy(0,0,0)
# j2  (a1,0,d1) rpy(90,0,0)
# j3  (0,a2,0)  rpy(0,0,90)
# j4  (a3,0,0)  rpy(90,0,0)
# j5  (0,0,d4)  rpy(90,0,0)
# j6  (0,0,0)   rpy(90,0,-90)
# tcp (0,0,d6)  rpy(0,0,-180)
def fk(P, q_deg, want_points=False):
    q = np.radians(q_deg)
    M = np.eye(4)
    pts = {}

    M = M @ origin(0, 0, 0, (0, 0, 0));            pts['J1'] = M[:3, 3].copy()
    M = M @ Rz(q[0])
    M = M @ origin(P['a1'], 0, P['d1'], (pi/2, 0, 0));   pts['J2'] = M[:3, 3].copy()
    M = M @ Rz(q[1])
    M = M @ origin(0, P['a2'], 0, (0, 0, pi/2));   pts['J3'] = M[:3, 3].copy()
    M = M @ Rz(q[2])
    M = M @ origin(P['a3'], 0, 0, (pi/2, 0, 0));   pts['J4'] = M[:3, 3].copy()
    M = M @ Rz(q[3])
    M = M @ origin(0, 0, P['d4'], (pi/2, 0, 0));   pts['J5'] = M[:3, 3].copy()
    M = M @ Rz(q[4])
    M = M @ origin(0, 0, 0, (pi/2, 0, -pi/2));     pts['J6'] = M[:3, 3].copy()
    M = M @ Rz(q[5])
    M = M @ origin(0, 0, P['d6'], (0, 0, -pi));    pts['tcp'] = M[:3, 3].copy()

    if want_points:
        return M[:3, 3].copy(), pts
    return M[:3, 3].copy()

CTRL = dict(a1=216.0, d1=377.0, a2=900.0, a3=130.0, d4=934.0, d6=154.0)
HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------- 1. 自检：12 组样本 ----------------
print("=" * 64)
print("1. 自检 —— 用控制器参数复现 fk_samples.csv（应 < 0.01 mm）")
print("=" * 64)
rows = []
_cands = [os.path.join(HERE, 'data', 'fk_samples.csv'),
          os.path.join(HERE, 'fk_samples.csv')]
_fp = next(p for p in _cands if os.path.exists(p))
with open(_fp, encoding='utf-8-sig') as f:
    for r in csv.reader(f):
        if len(r) < 10:
            continue
        try:
            q = [float(x) for x in r[1:7]]
            xyz = [float(x) for x in r[7:10]]
        except ValueError:
            continue
        rows.append((r[0], q, xyz))

worst = 0.0
for name, q, xyz in rows:
    e = float(np.linalg.norm(fk(CTRL, q) - np.array(xyz)))
    worst = max(worst, e)
    if e > 0.01:
        print("   !! %-14s 残差 %.3f mm" % (name, e))
print("   样本数 %d，最大位置残差 = %.4f mm   %s"
      % (len(rows), worst, "OK" if worst < 0.01 else "链路有问题"))

# ---------------- 2. home 位姿下 J3->法兰 ----------------
Q_HOME = [-11.382, -7.617, 7.674, -34.306, -30.333, -68.221]
print()
print("=" * 64)
print("2. home 位姿下 |J3 -> 法兰面|")
print("=" * 64)

def j3_flange(P, q):
    _, pts = fk(P, q, want_points=True)
    return float(np.linalg.norm(pts['tcp'] - pts['J3']))

BROCH = dict(a1=170.0, d1=431.0, a2=790.0, a3=156.0, d4=650.0, d6=132.5)
print("   控制器参数 (130/934/154)      : %7.2f mm" % j3_flange(CTRL, Q_HOME))
print("   彩页 d4=650 + a2=805          : %7.2f mm" % j3_flange(BROCH, Q_HOME))
print("   现场量尺                      :  900.00 mm")
print("   -> d4=650 时 home 位姿只有 %.0f，与 900 差 %.0f mm。"
      % (j3_flange(BROCH, Q_HOME), 900.0 - j3_flange(BROCH, Q_HOME)))
print("      说明量尺那次不是 home 位姿，900 不能直接拿来反解 d4。")

print()
print("   几何上限（a3 与 d4 共线、手腕伸直）:")
for d4 in (650.0, 828.0):
    print("     d4=%5.0f -> 上限 ≈ %.1f mm" % (d4, (156.0 + d4) + 132.5))

# ---------------- 3. 硬证据：300mm 命令 vs 260mm 实际 ----------------
print()
print("=" * 64)
print("3. 硬证据：命令 300.009 mm，实走 ≈260 mm")
print("=" * 64)
Q0 = [-14.553, -13.492, 17.343, -0.006, 3.831, -104.559]
Q1 = [-18.626, 5.219, -2.678, -0.005, 2.526, -108.634]

def step(P):
    return float(np.linalg.norm(fk(P, Q1) - fk(P, Q0)))

d_ctrl = step(CTRL)
print("   控制器参数下两点距离 = %.3f mm（控制器自报 300.009）" % d_ctrl)
print("   现场实测 ≈ 260 mm  →  比值 %.4f" % (260.0 / d_ctrl))

def bisect(fn, target, lo=300.0, hi=1500.0):
    flo = fn(lo) - target
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        fm = fn(mid) - target
        if flo * fm <= 0:
            hi = mid
        else:
            lo = mid; flo = fm
    return 0.5 * (lo + hi)

print()
print("   固定彩页 a3=156 / d4=650 / d6=132.5，反解 a2 使距离 = 260:")
def step_a2(a2):
    P = dict(BROCH); P['a2'] = a2
    return step(P)
print("     => a2 = %.1f mm   （现场量尺 805）" % bisect(step_a2, 260.0, 400, 1200))

print()
print("   固定 a2=805 / a3=156 / d6=132.5，扫 d4:")
for d4 in (500, 550, 600, 650, 700, 750, 800, 828, 900, 934):
    P = dict(BROCH); P['d4'] = float(d4)
    print("     d4=%5d -> 两点距离 %7.2f mm%s"
          % (d4, step(P), "   <= 命中 260" if abs(step(P) - 260) < 6 else ""))

# ---------------- 4. 统一缩放比 ----------------
print()
print("=" * 64)
print("4. 若全部长度按同一比例 s 缩放，s 应是多少？")
print("=" * 64)
def step_s(s):
    return step({k: v * s for k, v in CTRL.items()})

s_sol = bisect(step_s, 260.0, 0.5, 1.5)
print("   s = %.4f  →  控制器参数整体大 %.1f%%" % (s_sol, (1 / s_sol - 1) * 100))

# ---------------- 5. 最大臂展（数值搜索）----------------
print()
print("=" * 64)
print("5. 最大水平臂展（从 J1 轴起算）")
print("=" * 64)

def radius(P, q, which):
    _, pts = fk(P, q, want_points=True)
    p = pts[which]
    return float(np.hypot(p[0], p[1]))

def max_reach(P, which):
    best, bq = -1.0, None
    step = 30
    cand = range(-180, 180, step)
    for q2 in cand:
        for q3 in cand:
            for q4 in cand:
                for q5 in cand:
                    q = [0.0, q2, q3, q4, q5, 0.0]
                    v = radius(P, q, which)
                    if v > best:
                        best, bq = v, q
    # 局部细化
    for _ in range(6):
        step /= 3.0
        improved = True
        while improved:
            improved = False
            for i in range(1, 5):
                for d in (-step, step):
                    q = list(bq); q[i] += d
                    v = radius(P, q, which)
                    if v > best:
                        best, bq, improved = v, q, True
    return best, bq

for which, label in (('tcp', '到法兰面'), ('J5', '到腕心')):
    r, q = max_reach(BROCH, which)
    print("   %s : %.1f mm   （a1=216, a2=805, a3=156, d4=650, d6=132.5）"
          % (label, r))
    print("        最伸展时的 q2..q5 = %s" % ["%.0f" % x for x in q[1:5]])
    print("        要让它是 1800 需要 a1 = %.1f" % (216.0 + 1800.0 - r))

