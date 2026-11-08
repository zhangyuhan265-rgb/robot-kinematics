# -*- coding: utf-8 -*-
"""z>=1.9 不可达，是被 J5 的 -130° 限位卡住，还是真的够不着？

把 J5 限位放宽到 ±180 再解同样的点：
  收敛了  -> 是限位卡住的（工作空间被关节限位切掉）
  仍不收敛 -> 是真的够不着
"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
NQ, TIP = arm.nq, arm.tip


def solve(Pt, lo, hi, seed, iters=200, step=0.2):
    def fk(q):
        return arm.fk(q)[TIP][:3, 3]

    def jac(q, e0, h=1e-7):
        J = np.zeros((3, NQ))
        for i in range(NQ):
            qq = q.copy()
            qq[i] += h
            J[:, i] = (Pt - fk(qq) - e0) / h
        return J

    q = np.clip(np.array(seed, float), lo, hi)
    e = Pt - fk(q)
    cost = e @ e
    lam = 1e-3
    for _ in range(iters):
        if np.linalg.norm(e) < 1e-8:
            break
        J = jac(q, e)
        dq = -np.linalg.solve(J.T @ J + lam * np.eye(NQ), J.T @ e)
        n = np.linalg.norm(dq)
        if n > step:
            dq *= step / n
        qn = np.clip(q + dq, lo, hi)
        en = Pt - fk(qn)
        cn = en @ en
        if cn < cost:
            q, e, cost = qn, en, cn
            lam = max(lam * 0.3, 1e-9)
        else:
            lam *= 10
            if lam > 1e7:
                break
    return q, float(np.linalg.norm(e))


rng = np.random.default_rng(3)
LO0, HI0 = arm.lo.copy(), arm.hi.copy()
LO1, HI1 = LO0.copy(), HI0.copy()
LO1[4], HI1[4] = np.radians(-180), np.radians(180)

print("%8s %26s %26s"
      % ("z(m)", "URDF 限位 J5±130", "放宽 J5 到 ±180"))
for z in (1.80, 1.85, 1.90, 2.00, 2.10):
    Pt = np.array([1.10, 0.0, z])
    out = []
    for lo, hi in ((LO0, HI0), (LO1, HI1)):
        best, bq = 1e9, None
        for _ in range(30):
            q, e = solve(Pt, lo, hi, rng.uniform(lo, hi))
            if e < best:
                best, bq = e, q
        out.append((best, bq))
    print("%8.2f   残差 %7.2f mm J5=%+7.1f°   残差 %7.2f mm J5=%+7.1f°"
          % (z, out[0][0] * 1000, np.degrees(out[0][1][4]),
             out[1][0] * 1000, np.degrees(out[1][1][4])))
