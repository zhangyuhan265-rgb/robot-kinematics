# -*- coding: utf-8 -*-
"""起点策略对比：随机起点 vs 零位起点 vs 结构化种子库。

目标集仍是「均匀随机关节角正解出来的位姿」（最苛刻的分布）。
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
rng = np.random.default_rng(23)

MARGIN = np.radians(5.0)
N = 60
TARGETS, RSEEDS = [], []
for _ in range(N):
    q_ref = rng.uniform(LO + MARGIN, HI - MARGIN)
    TARGETS.append(fk_T(q_ref)[:3, 3])
    RSEEDS.append(rng.uniform(LO + MARGIN, HI - MARGIN))

# 结构化种子库：零位 + 几种典型姿态（肘上/肘下、腕翻转）
LIB = [
    np.zeros(NQ),
    np.radians([0, -30, 30, 0, 0, 0]),
    np.radians([0, 30, -30, 0, 0, 0]),
    np.radians([0, -60, 60, 0, 0, 0]),
    np.radians([0, 0, 60, 0, 0, 0]),
    np.radians([0, 0, -60, 0, 0, 0]),
    np.radians([0, -30, 30, 0, -60, 0]),
    np.radians([90, -30, 30, 0, 0, 0]),
    np.radians([-90, -30, 30, 0, 0, 0]),
    np.radians([180, -30, 30, 0, 0, 0]),
]


def trial(seed_fn, tries):
    okc, ts, errs = 0, [], []
    for Pt, rs in zip(TARGETS, RSEEDS):
        t = time.time()
        best = np.inf
        for k in range(tries):
            s = seed_fn(rs, k)
            _, e = arm.ik(s, Pt, tries=1)
            best = min(best, e)
            if best < 1e-6:
                break
        ts.append(time.time() - t)
        errs.append(best * 1000)
        okc += best < 1e-3
    return okc / N * 100, np.median(ts) * 1000, np.mean(ts) * 1000


print("%-22s %8s %10s %10s" % ("起点策略", "成功率", "中位ms", "平均ms"))
print("-" * 54)
for tries in (1, 2, 4, 6):
    r = trial(lambda rs, k: rs, tries)
    print("%-22s %7.0f%% %10.1f %10.1f"
          % ("随机 × %d" % tries, r[0], r[1], r[2]))
print()
for tries in (1, 2, 4, 6):
    r = trial(lambda rs, k: np.zeros(NQ), tries)
    print("%-22s %7.0f%% %10.1f %10.1f"
          % ("零位 × %d" % tries, r[0], r[1], r[2]))
print()
for tries in (1, 2, 3, 4, 6):
    r = trial(lambda rs, k: LIB[k % len(LIB)], tries)
    print("%-22s %7.0f%% %10.1f %10.1f"
          % ("种子库 × %d" % tries, r[0], r[1], r[2]))
print()
for tries in (2, 4, 6):
    r = trial(lambda rs, k: np.zeros(NQ) if k == 0 else rs, tries)
    print("%-22s %7.0f%% %10.1f %10.1f"
          % ("零位 + 随机 × %d" % tries, r[0], r[1], r[2]))
