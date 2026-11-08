# -*- coding: utf-8 -*-
"""假设：水平半径少 52 mm，是不是 q5 零位偏了？

q5 在零位时轴是水平的，法兰中心绕它走的是水平圆弧，
高度只随 (1-cos δ) 二阶变化 —— 高度数据对 q5 零位极不敏感。
ΔR/Δq5 = -2.312 mm/度，所以 52 mm 对应约 +22.5 度。

用 12 个位姿的高度残差检验这个假设。
"""
import numpy as np

from arm_env import UrdfArm

RISER = 336.5
arm = UrdfArm("nbt_arm6.urdf")

DATA = [
    ([0, 0, 0, 0, 0, 0], 1581),
    ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474),
    ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068),
    ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549),
    ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569),
    ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666),
    ([20, -25, 35, 60, -30, 45], 1720),
]


def run(off5):
    Rs, errs = [], []
    for q, meas in DATA:
        qq = list(q)
        qq[4] += off5
        T = arm.fk(np.radians(qq))[arm.tip]
        Rs.append(np.hypot(T[0, 3], T[1, 3]) * 1000)
        errs.append((T[2, 3] + RISER / 1000.0) * 1000.0 - meas)
    return Rs, np.array(errs)


print("%10s %12s %10s %10s %10s" % ("q5零偏", "零位R", "平均误差", "RMS", "最大|e|"))
for off in (0, 5, 10, 15, 22.5, 30):
    Rs, errs = run(off)
    print("%9.1f° %12.1f %+10.1f %10.1f %10.1f"
          % (off, Rs[0], errs.mean(), np.sqrt((errs ** 2).mean()),
             np.abs(errs).max()))
    print("%22s%s" % ("逐位姿:", " ".join("%+5.0f" % e for e in errs)))
