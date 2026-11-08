# -*- coding: utf-8 -*-
"""检验：能不能用“某个轴的零位偏了”来解释水平半径差 52 mm。

对每个轴在零位上加 1° 误差，看水平半径 R 和法兰高度 h 各变多少。
如果某轴的 Δh/ΔR 很小，说明它可能偷偷改 R 而不被高度数据发现。
"""
import numpy as np

from arm_env import UrdfArm

arm = UrdfArm("nbt_arm6.urdf")
RISER = 336.5


def rh(qd):
    T = arm.fk(np.radians(qd))[arm.tip]
    return np.hypot(T[0, 3], T[1, 3]) * 1000, (T[2, 3] + RISER / 1000) * 1000


R0, h0 = rh([0, 0, 0, 0, 0, 0])
print("零位  R = %.1f mm   h = %.1f mm" % (R0, h0))
print()
print("%4s %10s %10s %12s %14s" % ("轴", "ΔR/mm", "Δh/mm", "Δh/ΔR", "R 差52时 h 差"))
for j in range(6):
    q = [0.0] * 6
    q[j] = 1.0
    R, h = rh(q)
    dR, dh = R - R0, h - h0
    ratio = (dh / dR) if abs(dR) > 1e-9 else float("inf")
    implied = abs(ratio) * 52 if np.isfinite(ratio) else float("nan")
    print("q%d %10.3f %10.3f %12.3f %14s"
          % (j + 1, dR, dh, ratio,
             ("%.0f mm" % implied) if np.isfinite(implied) else "—（ΔR=0）"))
print()
print("参考：高度实测残差 RMS 4.2 mm，最大 10.4 mm")
