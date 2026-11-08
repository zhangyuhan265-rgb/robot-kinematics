# -*- coding: utf-8 -*-
"""对比「起点」对解的质量的影响：关节行程。

场景就是真机的日常：臂在 q_cur，目标就在旁边（比如沿墙走 20 mm）。
好的解应该离 q_cur 很近；随机起点容易收敛到另一个远处的解。
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
fk_T = lambda q: arm.fk(q)[TIP]
rng = np.random.default_rng(41)


def travel(q, q0):
    return float(np.degrees(np.abs(q - q0)).sum())


def run(label, target_step, tries, n=60):
    trav, ok, ts = [], 0, []
    for _ in range(n):
        q_cur = rng.uniform(LO + np.radians(30), HI - np.radians(30))
        P = fk_T(q_cur)[:3, 3] + np.array([target_step, 0.0, 0.0])
        t = time.time()
        q, e, good = arm._multi(q_cur, arm._err_pos(P), dim=3, tries=tries)
        ts.append(time.time() - t)
        if good:
            ok += 1
            trav.append(travel(q, q_cur))
    print("%-34s %6.0f%% %12s %9.1f"
          % (label, ok / n * 100,
             ("%.1f°" % np.median(trav)) if trav else "—",
             np.median(ts) * 1000))


def run_random_seed(label, target_step, n=60):
    trav, ok, ts = [], 0, []
    for _ in range(n):
        q_cur = rng.uniform(LO + np.radians(30), HI - np.radians(30))
        P = fk_T(q_cur)[:3, 3] + np.array([target_step, 0.0, 0.0])
        t = time.time()
        q, e = arm._lm(rng.uniform(LO, HI), arm._err_pos(P))
        ts.append(time.time() - t)
        if e < 1e-4:
            ok += 1
            trav.append(travel(q, q_cur))
    print("%-34s %6.0f%% %12s %9.1f"
          % (label, ok / n * 100,
             ("%.1f°" % np.median(trav)) if trav else "—",
             np.median(ts) * 1000))


print("%-34s %6s %12s %9s" % ("策略（目标 = 当前位姿平移 20 mm）", "成功", "关节行程", "中位ms"))
print("-" * 68)
run_random_seed("单起点 · 随机", 0.020)
run("暖启动 · 只用当前位姿（tries=1）", 0.020, 1)
run("多起点 · 优先离起点最近（tries=8）", 0.020, 8)

print()
print("%-34s %6s %12s %9s" % ("策略（目标 = 当前位姿平移 300 mm）", "成功", "关节行程", "中位ms"))
print("-" * 68)
run_random_seed("单起点 · 随机", 0.300)
run("暖启动 · 只用当前位姿（tries=1）", 0.300, 1)
run("多起点 · 优先离起点最近（tries=8）", 0.300, 8)
