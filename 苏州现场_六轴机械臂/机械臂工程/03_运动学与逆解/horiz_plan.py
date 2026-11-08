# -*- coding: utf-8 -*-
"""水平方向对齐度：算出模型在零位时的水平半径与方位，以及 q1 转动时
法兰中心的水平弦长（用于现场用铅垂线+卷尺做的两点测距试验）。

用法：python horiz_plan.py
"""
import numpy as np

from arm_env import UrdfArm

RISER = 0.3365  # 垫高件，桌面上是 0
arm = UrdfArm("nbt_arm6.urdf")

q0 = np.zeros(6)
p0 = arm.tcp(q0)
print("零位法兰中心（臂座坐标系）: x=%.3f y=%.3f z=%.3f m" % tuple(p0))
print("  水平半径 R = %.1f mm" % (np.hypot(p0[0], p0[1]) * 1000))
print("  方位角 atan2(y,x) = %.2f°" % np.degrees(np.arctan2(p0[1], p0[0])))
print("  离地高度 = %.1f mm（含垫高件 %.1f）" % ((p0[2] + RISER) * 1000, RISER * 1000))
print()

R = np.hypot(p0[0], p0[1])
az0 = np.arctan2(p0[1], p0[0])
print("q1 单独转动时法兰中心在同一水平圆上（半径 %.1f mm）:" % (R * 1000))
print("%8s %10s %10s %12s" % ("dq1(°)", "方位(°)", "R(mm)", "与q1=0的弦长(mm)"))
for dq in (0, 30, 45, 60, 90, 120, 180):
    a = np.radians(dq)
    chord = 2 * R * np.sin(abs(a) / 2)
    print("%8d %10.2f %10.1f %12.1f"
          % (dq, np.degrees(az0 + a), R * 1000, chord * 1000))

print()
print("其它位姿的水平半径（用来在多个半径上重复验证）:")
cands = [
    ("零位", [0, 0, 0, 0, 0, 0]),
    ("q2 抬 30", [0, 30, 0, 0, 0, 0]),
    ("q2 抬 60", [0, 60, 0, 0, 0, 0]),
    ("q2 低 15", [0, -15, 0, 0, 0, 0]),
    ("q3 转 40", [0, 0, 40, 0, 0, 0]),
    ("当前实机位姿", [20, -25, 35, 60, -30, 45]),
]
for name, qd in cands:
    p = arm.tcp(np.radians(qd))
    print("  %-14s R = %7.1f mm   高 = %7.1f mm   x=%7.1f y=%7.1f"
          % (name, np.hypot(p[0], p[1]) * 1000, (p[2] + RISER) * 1000,
             p[0] * 1000, p[1] * 1000))
