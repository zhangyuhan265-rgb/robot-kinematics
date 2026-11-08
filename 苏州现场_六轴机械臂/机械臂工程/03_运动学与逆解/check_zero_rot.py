# -*- coding: utf-8 -*-
"""核对模型在零位时的法兰姿态，和控制器上报的 (rx, ry, rz) 对比。
控制器约定：R = Rx(rx) · Ry(ry) · Rz(rz)，角度弧度。
"""
import numpy as np

from arm_env import UrdfArm

arm = UrdfArm("nbt_arm6.urdf")
T = arm.fk(np.zeros(6))[arm.tip]
R = T[:3, 3 - 1] if False else T[:3, :3]
p = T[:3, 3]

# 反解 Rx(a)Ry(b)Rz(c)
b = np.arcsin(np.clip(R[0, 2], -1, 1))
a = np.arctan2(-R[1, 2], R[2, 2])
c = np.arctan2(-R[0, 1], R[0, 0])

print("模型零位:")
print("  位置      = (%9.4f, %9.4f, %9.4f)  m" % tuple(p))
print("  R (3x3) =")
for row in R:
    print("      [%7.4f %7.4f %7.4f]" % tuple(row))
print("  rpy(RxRyRz) = (%7.4f, %7.4f, %7.4f) rad = (%.2f, %.2f, %.2f) deg"
      % (a, b, c, np.degrees(a), np.degrees(b), np.degrees(c)))
print()
print("控制器上报零位: (919.998, -0.001, 1244.501) mm, rpy = (3.142, 0.0, 0.0) rad")
print("                 = (180.00, 0.00, 0.00) deg")
print()
# 用反解出的角度重建，和控制器上报值比
ra, rb, rc = 3.142, 0.0, 0.0


def rx(t):
    return np.array([[1, 0, 0], [0, np.cos(t), -np.sin(t)], [0, np.sin(t), np.cos(t)]])


def ry(t):
    return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])


def rz(t):
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])


Rc = rx(ra) @ ry(rb) @ rz(rc)
print("控制器 R 重建 =")
for row in Rc:
    print("      [%7.4f %7.4f %7.4f]" % tuple(row))
ang = np.degrees(np.arccos(np.clip((np.trace(R.T @ Rc) - 1) / 2, -1, 1)))
print()
print("模型 vs 控制器 姿态夹角差 = %.4f 度" % ang)
print("法兰面法向（模型）= (%.4f, %.4f, %.4f)" % tuple(R[:, 2]))
