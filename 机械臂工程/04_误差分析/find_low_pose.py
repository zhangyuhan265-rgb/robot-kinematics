# -*- coding: utf-8 -*-
"""找一个“法兰盘水平朝下、但高度很低”的位姿，方便挂铅垂。
约束：q5=q6=0（否则盘面会歪），盘面法向 = (0,0,-1)，q1 任意（取 0）。
输出按高度从低到高排序，并给出该位姿的水平半径 R 和 q1 转 90° 的弦长。
"""
import numpy as np

from arm_env import UrdfArm

arm = UrdfArm("nbt_arm6.urdf")
RISER = 0.3365

cands = []
for q2 in range(-50, 51, 1):
    for q3 in range(-70, 71, 1):
        for q4 in range(-80, 81, 1):
            q = np.radians([0, q2, q3, q4, 0, 0])
            T = arm.fk(q)[arm.tip]
            n = T[:3, 2]
            if n[2] > -0.9998:      # 法向必须基本朝下
                continue
            p = T[:3, 3]
            R = np.hypot(p[0], p[1])
            h = p[2] + RISER
            if R < 0.55 or h < 0.55:   # 太靠近底座 / 太低
                continue
            cands.append((h, R, q2, q3, q4, n[2]))

cands.sort()
print("候选数 =", len(cands))
print()
print("%7s %8s %9s %9s %6s %6s" % ("高(mm)", "R(mm)", "弦长90°", "q1=180°弦", "q2", "q3"))
seen = []
for h, R, q2, q3, q4, nz in cands:
    # 每个 50mm 高度带里只留一个
    if any(abs(h - s) < 0.05 for s in seen):
        continue
    seen.append(h)
    print("%7.0f %8.1f %9.1f %9.1f %6d %6d   q4=%d"
          % (h * 1000, R * 1000,
             2 * R * np.sin(np.radians(45)) * 1000, 2 * R * 1000, q2, q3, q4))
    if len(seen) >= 10:
        break
