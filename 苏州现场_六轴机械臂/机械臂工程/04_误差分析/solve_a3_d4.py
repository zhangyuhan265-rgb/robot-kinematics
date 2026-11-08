# -*- coding: utf-8 -*-
"""用两个实测位姿反解 a3 (J3->J4 偏置) 和 d4 (小臂长)。

约束：
  A  参考位姿   法兰中心离地 1710
  B  600测试终点 法兰中心离地 1015
其余参数不动：d1=431 a1=170 a2=790 d6=132.5 off5=179.67 垫高=336.5
"""
import numpy as np
from math import pi, radians, cos, sin, degrees

D1, A1, A2, D6 = 431.0, 170.0, 790.0, 132.5
OFF5 = 179.67
RISER = 336.5

QA = [-11.381, -7.617, 7.668, -0.001, 0.000, 291.779]
QB = [-8.596, -35.229, -5.487, -0.004, -40.764, 294.568]
MA, MB = 1710.0, 1015.0


def Rz(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def org(x, y, z, rpy):
    roll, pitch, yaw = rpy          # 不能叫 y，会和 y 平移撞名
    M = np.eye(4)
    M[:3, :3] = (Rz(yaw) @ Ry(pitch) @ Rx(roll))[:3, :3]
    M[:3, 3] = (x, y, z)
    return M


def fk_fz(q_deg, a3, d4):
    q = [radians(v) for v in q_deg]
    M = np.eye(4)
    M = M @ Rz(q[0])
    M = M @ org(A1, 0, D1, (pi / 2, 0, 0)) @ Rz(q[1])
    M = M @ org(0, A2, 0, (0, 0, pi / 2)) @ Rz(q[2])
    M = M @ org(a3, 0, 0, (pi / 2, 0, 0)) @ Rz(q[3])
    M = M @ org(0, 0, d4, (pi / 2, 0, 0)) @ Rz(q[4])
    M = M @ org(0, 0, 0, (pi / 2, 0, -radians(OFF5))) @ Rz(q[5])
    M = M @ org(0, 0, D6, (0, 0, -pi))
    return M[:3, 3][2] + RISER


def resid(a3, d4):
    return (fk_fz(QA, a3, d4) - MA) ** 2 + (fk_fz(QB, a3, d4) - MB) ** 2


print("现在 (a3=156, d4=650)： A z=%.1f (实测 %.0f)   B z=%.1f (实测 %.0f)"
      % (fk_fz(QA, 156.0, 650.0), MA, fk_fz(QB, 156.0, 650.0), MB))
print()

best = None
for a3 in np.arange(60, 360, 5.0):
    for d4 in np.arange(400, 1100, 5.0):
        v = resid(a3, d4)
        if best is None or v < best[0]:
            best = (v, a3, d4)
# 细化
v, a3b, d4b = best
step = 5.0
for _ in range(60):
    step *= 0.8
    for da in (-step, 0, step):
        for dd in (-step, 0, step):
            a3, d4 = a3b + da, d4b + dd
            vv = resid(a3, d4)
            if vv < v:
                v, a3b, d4b = vv, a3, d4
        v = resid(a3b, d4b)

print("=" * 60)
print("反解结果： a3 = %.1f mm   d4 = %.1f mm   （残差平方和 %.1f）" % (a3b, d4b, v))
print("  此时 A z=%.1f (实测 %.0f)   B z=%.1f (实测 %.0f)"
      % (fk_fz(QA, a3b, d4b), MA, fk_fz(QB, a3b, d4b), MB))
print()
print("参考：a3 旧值 156, d4 旧值 650；两者合成的肘→腕距离")
print("  旧: sqrt(156^2+650^2) = %.1f" % np.hypot(156, 650))
print("  新: sqrt(%.0f^2+%.0f^2) = %.1f" % (a3b, d4b, np.hypot(a3b, d4b)))
