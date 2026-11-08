# -*- coding: utf-8 -*-
"""
最大水平臂展的严格求解（scipy differential_evolution，带关节限位）。

为什么值得算：彩页型号是 CR20-1800，"1800" 大概率就是臂展。
如果按新的长度参数算出来的最大臂展正好 ≈1800，那 a1 也顺带被钉住了。

用法：python reach_scan.py
"""
import numpy as np
from math import pi
from scipy.optimize import differential_evolution

def Rz(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def origin(x, y, z, rpy):
    roll, pitch, yaw = rpy
    M = np.eye(4)
    M[:3, :3] = (Rz(yaw) @ Ry(pitch) @ Rx(roll))[:3, :3]
    M[:3, 3] = (x, y, z)
    return M

def fk_pts(P, q_deg):
    q = np.radians(q_deg)
    M = np.eye(4)
    pts = {}
    M = M @ origin(0, 0, 0, (0, 0, 0));                    pts['J2'] = M[:3, 3].copy()
    M = M @ Rz(q[0])
    M = M @ origin(P['a1'], 0, P['d1'], (pi/2, 0, 0));      pts['J2'] = M[:3, 3].copy()
    M = M @ Rz(q[1])
    M = M @ origin(0, P['a2'], 0, (0, 0, pi/2));           pts['J3'] = M[:3, 3].copy()
    M = M @ Rz(q[2])
    M = M @ origin(P['a3'], 0, 0, (pi/2, 0, 0));           pts['J4'] = M[:3, 3].copy()
    M = M @ Rz(q[3])
    M = M @ origin(0, 0, P['d4'], (pi/2, 0, 0));           pts['J5'] = M[:3, 3].copy()
    M = M @ Rz(q[4])
    M = M @ origin(0, 0, 0, (pi/2, 0, -pi/2));             pts['J6'] = M[:3, 3].copy()
    M = M @ Rz(q[5])
    M = M @ origin(0, 0, P['d6'], (0, 0, -pi));            pts['tcp'] = M[:3, 3].copy()
    return pts

LIM = 130.0   # J1~J5 限位 ±130°

def max_reach(P, which='tcp', seed=0):
    def neg(q):           # q = q2,q3,q4,q5（q1 不影响离轴半径，q6 不影响位置）
        qq = [0.0, q[0], q[1], q[2], q[3], 0.0]
        p = fk_pts(P, qq)[which]
        return -float(np.hypot(p[0], p[1]))

    res = differential_evolution(
        neg, bounds=[(-LIM, LIM)] * 4, seed=seed,
        maxiter=1500, popsize=40, tol=1e-10, polish=True, mutation=(0.4, 1.2),
        init='sobol')
    return -res.fun, res.x

CTRL = dict(a1=216.0, d1=377.0, a2=900.0, a3=130.0, d4=934.0, d6=154.0)
NEW = dict(a1=216.0, d1=431.0, a2=805.0, a3=156.0, d4=650.0, d6=132.5)

print("=" * 66)
print("最大水平臂展（从 J1 轴起算，J1~J5 限位 ±130°）")
print("=" * 66)
for P, tag in ((CTRL, "控制器参数"), (NEW, "彩页+实测参数（a1=216）")):
    for which, label in (('tcp', '到法兰面'), ('J5', '到腕心')):
        best = -1
        bq = None
        for sd in (1, 2, 3):
            v, q = max_reach(P, which, sd)
            if v > best:
                best, bq = v, q
        print("  %-22s %s = %7.1f mm   q2..q5 = %s"
              % (tag, label, best, ["%.0f" % x for x in bq]))
    print()

print("=" * 66)
print("反推：要让最大臂展正好等于 1800，a1 应该是多少")
print("=" * 66)
for which, label in (('tcp', '到法兰面'), ('J5', '到腕心')):
    r, _ = max_reach(NEW, which, 1)
    print("  %s : 现 a1=216 -> %.1f mm，需 a1 = %.1f 才是 1800"
          % (label, r, 216.0 + 1800.0 - r))
