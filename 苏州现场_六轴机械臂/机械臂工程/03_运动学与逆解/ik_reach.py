# -*- coding: utf-8 -*-
"""确认 x=1.10 面上 z>=2.0 到底是不可达，还是 IK 没解到。

1. 蒙特卡洛采样工作空间（20000 个随机 q），看 x≈1.10 处法兰能到多高
2. 对可疑点用 40 个起点硬解，看残差收敛到多少
3. 沿竖直线的热启动 IK（轨迹规划的真实用法），看耗时和成功率
"""
import sys
import time

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
LO, HI, NQ, TIP = arm.lo.copy(), arm.hi.copy(), arm.nq, arm.tip


def fk_T(q):
    return arm.fk(q)[TIP]


def numeric_jac(q, err, m, h=1e-7):
    e0 = err(q)
    J = np.zeros((m, NQ))
    for i in range(NQ):
        qq = q.copy()
        qq[i] += h
        J[:, i] = (err(qq) - e0) / h
    return J, e0


def lm_ik(seed, Pt, iters=150, step=0.2, tol=1e-8):
    err = lambda q: Pt - fk_T(q)[:3, 3]
    q = np.clip(np.array(seed, dtype=float), LO, HI)
    J, e = numeric_jac(q, err, 3)
    cost = e @ e
    lam = 1e-3
    for _ in range(iters):
        if np.linalg.norm(e) < tol:
            break
        dq = -np.linalg.solve(J.T @ J + lam * np.eye(NQ), J.T @ e)
        n = np.linalg.norm(dq)
        if n > step:
            dq *= step / n
        qn = np.clip(q + dq, LO, HI)
        Jn, en = numeric_jac(qn, err, 3)
        cn = en @ en
        if cn < cost:
            q, J, e, cost = qn, Jn, en, cn
            lam = max(lam * 0.3, 1e-9)
        else:
            lam *= 10.0
            if lam > 1e7:
                break
    return q, float(np.linalg.norm(e))


rng = np.random.default_rng(1)
print("=" * 62)
print("1. 蒙特卡洛工作空间（20000 个随机 q）")
print("=" * 62)
Q = rng.uniform(LO, HI, size=(20000, NQ))
P = np.array([fk_T(q)[:3, 3] for q in Q])
print("  x 范围  %+.2f ~ %+.2f m" % (P[:, 0].min(), P[:, 0].max()))
print("  z 范围  %+.2f ~ %+.2f m" % (P[:, 2].min(), P[:, 2].max()))
print("  离底座轴最大水平距离  %.2f m"
      % np.max(np.hypot(P[:, 0], P[:, 1])))
mask = (np.abs(P[:, 0] - 1.10) < 0.03) & (np.abs(P[:, 1]) < 0.05)
print()
print("  x≈1.10、y≈0 这个窄格里只落到 %d 个随机点（6 维空间太稀疏，"
      "蒙特卡洛判断不了单点可达性，下面第 2 节直接硬解）" % mask.sum())

print()
print("=" * 62)
print("2. 可疑点用 40 个起点硬解（x=1.10）")
print("=" * 62)
print("%8s %12s %12s" % ("z(m)", "最好残差mm", "收敛到<1mm"))
for z in (1.60, 1.70, 1.80, 1.90, 2.00, 2.10, 2.20):
    Pt = np.array([1.10, 0.0, z])
    best = 1e9
    okc = 0
    for k in range(40):
        _, e = lm_ik(rng.uniform(LO, HI), Pt)
        best = min(best, e)
        if e * 1000 < 1.0:
            okc += 1
    print("%8.2f %12.2f %12s"
          % (z, best * 1000, "%d / 40" % okc))

print()
print("=" * 62)
print("3. 竖直轨迹的热启动 IK（x=1.10, y=0, z 从 0.30 到 1.90，35 点）")
print("=" * 62)
zs = np.linspace(0.30, 1.90, 35)
q = np.zeros(NQ)
ts, errs = [], []
for z in zs:
    t = time.time()
    q, e = lm_ik(q, np.array([1.10, 0.0, z]))
    ts.append(time.time() - t)
    errs.append(e * 1000)
errs = np.array(errs)
print("  成功（<1mm）  %d / %d" % (np.sum(errs < 1.0), len(zs)))
print("  最大残差      %.3f mm" % errs.max())
print("  单点耗时      中位 %.1f ms   最大 %.1f ms"
      % (np.median(ts) * 1000, np.max(ts) * 1000))
print("  关节角行程（度）")
for i in range(NQ):
    vals = []
    q2 = np.zeros(NQ)
    for z in zs:
        q2, _ = lm_ik(q2, np.array([1.10, 0.0, z]))
        vals.append(np.degrees(q2[i]))
    print("    J%d  %+8.1f ~ %+8.1f" % (i + 1, min(vals), max(vals)))
