# -*- coding: utf-8 -*-
"""从肩(J2)到法兰的最大距离到底是多少？

四段长度相加是 1.829 m，但它们方向被关节几何锁死，实际最大伸距可能远小于此。
如果实际最大伸距就是 1.42 m，那 x=1.10 上方 ~1.86 m 处的边界就是真实的。
"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
LO, HI, NQ, TIP = arm.lo.copy(), arm.hi.copy(), arm.nq, arm.tip
SHOULDER = np.array([0.170, 0.0, 0.431])

rng = np.random.default_rng(7)
N = 60000
Q = rng.uniform(LO, HI, size=(N, NQ))
Q[:, 0] = 0.0                      # q1 不影响肩到法兰的距离
d = np.empty(N)
P = np.empty((N, 3))
for i, q in enumerate(Q):
    P[i] = arm.fk(q)[TIP][:3, 3]
    d[i] = np.linalg.norm(P[i] - SHOULDER)

print("肩→法兰距离（%d 个随机姿态）" % N)
print("  最小 %.3f m   中位 %.3f m   最大 %.3f m"
      % (d.min(), np.median(d), d.max()))
print("  四段长度相加  %.3f m  ← 永远达不到，方向被锁死"
      % (0.790 + 0.156 + 0.750 + 0.1325))
print()
top = np.argmax(d)
print("最伸展姿态 q = %s" % " ".join("%+7.2f" % v for v in np.degrees(Q[top])))
print("  法兰 (%+.3f, %+.3f, %+.3f)  肩→法兰 %.3f m"
      % (P[top, 0], P[top, 1], P[top, 2], d[top]))
print()
zmax = P[:, 2].max()
print("法兰最高 z = %.3f m（%d 个姿态里）" % (zmax, N))
print("z 最高的那个姿态 x = %+.3f m" % P[np.argmax(P[:, 2]), 0])
