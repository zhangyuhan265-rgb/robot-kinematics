# -*- coding: utf-8 -*-
"""q5 转 ±90 后法兰中心的水平半径 / 高度，以及 q1 转 90 的弦长。
用来判断 J5 转不转、转哪边。
"""
import numpy as np

from arm_env import UrdfArm

arm = UrdfArm("nbt_arm6.urdf")
RISER = 0.3365


def info(qd):
    T = arm.fk(np.radians(qd))[arm.tip]
    p = T[:3, 3]
    R = np.hypot(p[0], p[1])
    n = T[:3, 2]  # 法兰面法向
    return R, p[2] + RISER, n


print("%-22s %9s %9s %9s   %s" % ("位姿", "R(mm)", "高(mm)", "弦长90°", "法兰面法向"))
for name, qd in [
    ("q5=0   (当前零位)", [0, 0, 0, 0, 0, 0]),
    ("q5=+90", [0, 0, 0, 0, 90, 0]),
    ("q5=-90", [0, 0, 0, 0, -90, 0]),
]:
    R, h, n = info(qd)
    print("%-22s %9.1f %9.1f %9.1f   (%6.3f, %6.3f, %6.3f)"
          % (name, R * 1000, h * 1000, 2 * R * np.sin(np.radians(45)) * 1000,
             n[0], n[1], n[2]))

print()
print("说明：法向 (0,0,-1) = 法兰面水平朝下；(1,0,0) = 朝水平方向。")
