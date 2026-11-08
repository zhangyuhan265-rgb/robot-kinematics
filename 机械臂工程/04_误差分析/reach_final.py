# -*- coding: utf-8 -*-
"""
定案参数下的最大臂展 —— 严格版。

上一版 reach_scan.py 的差分进化给出 1822，但手工分析显示
"q2=-90、a3 与 d4 共线" 的构型可能更大（理论上是 a1+a2+a3+d4+d6）。
这里用「结构化候选 + 多种子差分进化」两边夹，确认到底多大。

用法：python reach_final.py
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
    M = M @ Rz(q[0])
    M = M @ origin(P['a1'], 0, P['d1'], (pi/2, 0, 0));   pts['J2'] = M[:3, 3].copy()
    M = M @ Rz(q[1])
    M = M @ origin(0, P['a2'], 0, (0, 0, pi/2));         pts['J3'] = M[:3, 3].copy()
    M = M @ Rz(q[2])
    M = M @ origin(P['a3'], 0, 0, (pi/2, 0, 0));         pts['J4'] = M[:3, 3].copy()
    M = M @ Rz(q[3])
    M = M @ origin(0, 0, P['d4'], (pi/2, 0, 0));         pts['J5'] = M[:3, 3].copy()
    M = M @ Rz(q[4])
    M = M @ origin(0, 0, 0, (pi/2, 0, -pi/2));           pts['J6'] = M[:3, 3].copy()
    M = M @ Rz(q[5])
    M = M @ origin(0, 0, P['d6'], (0, 0, -pi));          pts['tcp'] = M[:3, 3].copy()
    return pts

FIN = dict(a1=170.0, d1=431.0, a2=790.0, a3=156.0, d4=650.0, d6=132.5)
LIM = 130.0

def radius(P, qq, which):
    p = fk_pts(P, qq)[which]
    return float(np.hypot(p[0], p[1]))

print("=" * 68)
print("定案参数: a1=170 a2=790 a3=156 d4=650 d6=132.5")
print("理论直臂上限 a1+a2+a3+d4+d6 = %.1f mm" %
      (170 + 790 + 156 + 650 + 132.5))
print("=" * 68)

# ---------- 1. 结构化候选 ----------
print("\n[1] 结构化候选扫描（q1=0, q6=0）")
best = (-1, None)
for q2 in (-90.0, -80.0, -100.0):
    for q3 in np.arange(-130, 131, 10):
        for q4 in np.arange(-130, 131, 10):
            for q5 in np.arange(-130, 131, 10):
                qq = [0.0, q2, q3, q4, q5, 0.0]
                v = radius(FIN, qq, 'tcp')
                if v > best[0]:
                    best = (v, qq)
print("    最佳: %.2f mm   q2..q5 = %s" % (best[0], ["%.0f" % x for x in best[1][1:5]]))

# 直接点名几个"看起来该最大"的构型
print("\n[2] 点名候选")
for q2, q3, q4, q5 in [(-90, 0, 90, 0), (-90, 0, 0, 0), (-90, 0, 90, -90),
                       (-90, 0, -90, 0), (-90, 0, 0, 180), (-90, 90, 90, 0),
                       (-90, 0, 90, 180), (-90, 0, 30, -103)]:
    qq = [0.0, q2, q3, q4, q5, 0.0]
    p = fk_pts(FIN, qq)['tcp']
    print("     q2=%4d q3=%4d q4=%4d q5=%4d -> R=%7.1f  z=%7.1f"
          % (q2, q3, q4, q5, np.hypot(p[0], p[1]), p[2]))

# ---------- 2. 多种子差分进化 ----------
print("\n[3] 差分进化（多种子）")
def neg(x):
    return -radius(FIN, [0.0, x[0], x[1], x[2], x[3], 0.0], 'tcp')

vals = []
for sd in range(1, 7):
    r = differential_evolution(neg, bounds=[(-LIM, LIM)] * 4, seed=sd,
                               maxiter=1200, popsize=40, tol=1e-12,
                               mutation=(0.5, 1.5), recombination=0.9,
                               polish=True, init='sobol')
    vals.append((-r.fun, r.x))
    print("     seed=%d -> %.2f mm  q2..q5 = %s"
          % (sd, -r.fun, ["%.0f" % x for x in r.x]))

print("\n     DE 最好 = %.2f mm ; 结构化最好 = %.2f mm"
      % (max(v for v, _ in vals), best[0]))
