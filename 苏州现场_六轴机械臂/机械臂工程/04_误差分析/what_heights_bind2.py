# -*- coding: utf-8 -*-
"""竖直数据能挡住多大「关节零位」误差？而关节零位对水平位移又有多敏感？

零位偏 δ 的物理含义：命令 q 时，实际角度是 q + δ。
所以真实位姿 = FK(q + δ·e_k)。
"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
TIP, NQ = arm.tip, arm.nq
RISER = 336.5
DATA = [
    ([0, 0, 0, 0, 0, 0], 1581), ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474), ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068), ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549), ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569), ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666), ([20, -25, 35, 60, -30, 45], 1720),
]
QA = [0, -23.65, 28.78, 0, 5.12, 0]
QB = [0, 21.28, -17.13, 0, 4.15, 0]
Q0 = [0, 0, 0, 0, 0, 0]


def shifted(q, k, d):
    qq = list(q)
    qq[k] += d
    return qq


def evaluate(k, deg):
    errs = np.array([arm.fk(np.radians(shifted(q, k, deg)))[TIP][2, 3] * 1000
                     + RISER - m for q, m in DATA])
    x0 = arm.fk(np.radians(shifted(Q0, k, deg)))[TIP][0, 3] * 1000
    xa = arm.fk(np.radians(shifted(QA, k, deg)))[TIP][0, 3] * 1000
    xb = arm.fk(np.radians(shifted(QB, k, deg)))[TIP][0, 3] * 1000
    return np.sqrt((errs ** 2).mean()), xa - x0, x0 - xb


b0 = evaluate(0, 0.0)
print("基准：高度 RMS %.2f mm，外推 %.2f mm，收回 %.2f mm" % b0)
print()
print("%-6s %8s %10s %12s %12s %10s"
      % ("轴", "零位偏", "高度RMS", "外推位移", "收回位移", "位移偏差"))
print("-" * 64)
for k, name in enumerate("J1 J2 J3 J4 J5 J6".split()):
    for deg in (-1.0, -0.5, -0.2, 0.2, 0.5, 1.0):
        rms, a, b = evaluate(k, deg)
        dev = max(abs(a - 299.98), abs(b - 299.97))
        flag = "  <-- 高度仍可接受" if rms < 6.5 else ""
        print("%-6s %+8.2f %10.2f %12.2f %12.2f %10.2f%s"
              % (name, deg, rms, a, b, dev, flag))
    print()
