# -*- coding: utf-8 -*-
"""z=1.90 那个解到底卡在哪：是某个关节到限位了，还是臂展到边界了？"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
LO, HI, NQ, TIP = arm.lo.copy(), arm.hi.copy(), arm.nq, arm.tip


def solve(Pt, seed, iters=250, step=0.2):
    def fk(q):
        return arm.fk(q)[TIP][:3, 3]
    q = np.clip(np.array(seed, float), LO, HI)
    e = Pt - fk(q)
    cost = e @ e
    lam = 1e-3
    for _ in range(iters):
        if np.linalg.norm(e) < 1e-9:
            break
        J = np.zeros((3, NQ))
        h = 1e-7
        for i in range(NQ):
            qq = q.copy()
            qq[i] += h
            J[:, i] = (Pt - fk(qq) - e) / h
        dq = -np.linalg.solve(J.T @ J + lam * np.eye(NQ), J.T @ e)
        n = np.linalg.norm(dq)
        if n > step:
            dq *= step / n
        qn = np.clip(q + dq, LO, HI)
        en = Pt - fk(qn)
        cn = en @ en
        if cn < cost:
            q, e, cost = qn, en, cn
            lam = max(lam * 0.3, 1e-9)
        else:
            lam *= 10
            if lam > 1e7:
                break
    return q, e


rng = np.random.default_rng(5)
for z in (1.80, 1.90):
    Pt = np.array([1.10, 0.0, z])
    best, bq = None, None
    for _ in range(40):
        q, e = solve(Pt, rng.uniform(LO, HI))
        if best is None or np.linalg.norm(e) < best:
            best, bq = np.linalg.norm(e), q
    T = arm.fk(bq)[TIP]
    print("目标 z=%.2f  残差 %.2f mm  到达 (%+.3f, %+.3f, %+.3f)"
          % (z, best * 1000, T[0, 3], T[1, 3], T[2, 3]))
    print("   关节角与限位余量：")
    for i in range(NQ):
        g = np.degrees(bq[i])
        print("     J%d %+8.2f°   下限余量 %6.1f°  上限余量 %6.1f°"
              % (i + 1, g, g - np.degrees(LO[i]), np.degrees(HI[i]) - g))
    # 肩(0.170,0,0.7675) 到法兰的距离，和最远可能值比较
    d = np.linalg.norm(T[:3, 3] - np.array([0.170, 0.0, 0.7675]))
    print("   肩→法兰距离 %.3f m   （四段共线时上限 %.3f m）"
          % (d, 0.790 + 0.156 + 0.750 + 0.1325))
    print()
