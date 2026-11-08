# -*- coding: utf-8 -*-
"""300mm 检验：如果现场量的是【地面上的水平投影】，那模型里的 3D 300mm 就不能直接比。
先看那次运动的水平和竖直分量各是多少。"""
import numpy as np
from math import pi

def Rz(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = np.cos(a), np.sin(a); M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def origin(x, y, z, rpy):
    roll, pitch, yaw = rpy
    M = np.eye(4); M[:3, :3] = (Rz(yaw) @ Ry(pitch) @ Rx(roll))[:3, :3]; M[:3, 3] = (x, y, z)
    return M

def fk(P, q_deg):
    q = np.radians(q_deg)
    M = np.eye(4)
    M = M @ Rz(q[0])
    M = M @ origin(P['a1'], 0, P['d1'], (pi/2, 0, 0)) @ Rz(q[1])
    M = M @ origin(0, P['a2'], 0, (0, 0, pi/2)) @ Rz(q[2])
    M = M @ origin(P['a3'], 0, 0, (pi/2, 0, 0)) @ Rz(q[3])
    M = M @ origin(0, 0, P['d4'], (pi/2, 0, 0)) @ Rz(q[4])
    M = M @ origin(0, 0, 0, (pi/2, 0, -pi/2)) @ Rz(q[5])
    M = M @ origin(0, 0, P['d6'], (0, 0, -pi))
    return M[:3, 3]

Q0 = [-14.553, -13.492, 17.343, -0.006, 3.831, -104.559]
Q1 = [-18.626, 5.219, -2.678, -0.005, 2.526, -108.634]

SETS = {
    '控制器 216/377/900/130/934/154': dict(a1=216., d1=377., a2=900., a3=130., d4=934., d6=154.),
    '定案   170/431/790/156/650/132.5': dict(a1=170., d1=431., a2=790., a3=156., d4=650., d6=132.5),
    '假设 a2=926               ': dict(a1=170., d1=431., a2=926., a3=156., d4=650., d6=132.5),
    '假设 a2=805               ': dict(a1=170., d1=431., a2=805., a3=156., d4=650., d6=132.5),
}

print("=" * 84)
print("300mm 那次检验：位移的 3D / 水平 / 竖直分量")
print("=" * 84)
print("  %-34s  3D      水平    竖直" % "参数组")
for k, P in SETS.items():
    d = fk(P, Q1) - fk(P, Q0)
    h = np.hypot(d[0], d[1])
    print("  %-34s %6.1f  %6.1f  %+6.1f" % (k, np.linalg.norm(d), h, d[2]))

print()
print("现场记录：'控制器报 300.009 mm，物理实测约 260 mm'")
print("如果 260 是地面上的水平投影，就该跟【水平】那一列比。")
print()
print("=" * 84)
print("反解：要让【水平】分量等于 260，a2 该是多少？")
print("=" * 84)
P = dict(a1=170., d1=431., a2=790., a3=156., d4=650., d6=132.5)
def hstep(a2):
    P2 = dict(P); P2['a2'] = a2
    d = fk(P2, Q1) - fk(P2, Q0)
    return np.hypot(d[0], d[1])
lo, hi = 400., 1400.
for _ in range(200):
    mid = .5 * (lo + hi)
    if (hstep(lo) - 260.) * (hstep(mid) - 260.) <= 0:
        hi = mid
    else:
        lo = mid
print("  a2 = %.1f mm" % (.5 * (lo + hi)))

print()
print("=" * 84)
print("对照：要让【3D】等于 260，a2 该是多少？")
print("=" * 84)
def dstep(a2):
    P2 = dict(P); P2['a2'] = a2
    return np.linalg.norm(fk(P2, Q1) - fk(P2, Q0))
lo, hi = 400., 1400.
for _ in range(200):
    mid = .5 * (lo + hi)
    if (dstep(lo) - 260.) * (dstep(mid) - 260.) <= 0:
        hi = mid
    else:
        lo = mid
print("  a2 = %.1f mm" % (.5 * (lo + hi)))

print()
print("=" * 84)
print("高度约束：a2 多大时，home 位姿法兰离地 = 1765（模型现在 1612.8）")
print("=" * 84)
QH = [-11.382, -7.617, 7.674, -34.306, -30.333, -68.221]
def zhome(a2):
    P2 = dict(P); P2['a2'] = a2
    return fk(P2, QH)[2]
lo, hi = 700., 1100.
for _ in range(200):
    mid = .5 * (lo + hi)
    if (zhome(lo) - 1765.) * (zhome(mid) - 1765.) <= 0:
        hi = mid
    else:
        lo = mid
print("  a2 = %.1f mm   （当前 a2=790 时 z=%.1f）" % (.5 * (lo + hi), zhome(790.)))

print()
print("=" * 84)
print("高度约束：a2 多大时，当前位姿法兰离地 = 1710（模型现在 1574.6）")
print("=" * 84)
QN = [-11.382, -7.617, 7.671, -0.001, -0.330, 291.779]
def znow(a2):
    P2 = dict(P); P2['a2'] = a2
    return fk(P2, QN)[2]
lo, hi = 700., 1100.
for _ in range(200):
    mid = .5 * (lo + hi)
    if (znow(lo) - 1710.) * (znow(mid) - 1710.) <= 0:
        hi = mid
    else:
        lo = mid
print("  a2 = %.1f mm   （当前 a2=790 时 z=%.1f）" % (.5 * (lo + hi), znow(790.)))
