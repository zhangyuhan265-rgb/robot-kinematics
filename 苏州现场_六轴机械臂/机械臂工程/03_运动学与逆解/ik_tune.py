# -*- coding: utf-8 -*-
"""IK 调优：先验几何 Jacobian，再扫起点数看成功率/耗时。

用法：python ik_tune.py
"""
import sys
import time

import numpy as np

from arm_env import UrdfArm, _rotvec

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
LO, HI, NQ, TIP = arm.lo.copy(), arm.hi.copy(), arm.nq, arm.tip
fk_T = lambda q: arm.fk(q)[TIP]
rng = np.random.default_rng(11)

print("=" * 64)
print("1. 几何 Jacobian vs 数值差分")
print("=" * 64)
err_p = err_w = 0.0
for _ in range(20):
    q = rng.uniform(LO, HI)
    Jg = arm.jacobian_geo(q)
    Jn = arm.jacobian(q)                      # 数值位置 Jacobian（3×nq）
    err_p = max(err_p, np.abs(Jg[:3] - Jn).max())
    h = 1e-7
    T0 = fk_T(q)
    Jw = np.zeros((3, NQ))
    for i in range(NQ):
        qq = q.copy()
        qq[i] += h
        Jw[:, i] = _rotvec(fk_T(qq)[:3, :3] @ T0[:3, :3].T) / h
    err_w = max(err_w, np.abs(Jg[3:] - Jw).max())
print("  位置部分最大偏差  %.2e" % err_p)
print("  姿态部分最大偏差  %.2e" % err_w)
print("  （应 ~1e-6 量级，只是数值差分的截断误差）")

MARGIN = np.radians(5.0)
N = 60
TARGETS, SEEDS = [], []
for _ in range(N):
    q_ref = rng.uniform(LO + MARGIN, HI - MARGIN)
    T = fk_T(q_ref)
    TARGETS.append((T[:3, 3], T[:3, :3]))
    SEEDS.append(rng.uniform(LO + MARGIN, HI - MARGIN))

print()
print("=" * 64)
print("2. 位置 IK：起点数 vs 成功率（N=%d，随机可达位姿）" % N)
print("=" * 64)
print("%6s %10s %9s %10s %9s %9s"
      % ("起点数", "冷启动成功率", "中位ms", "随机q0成功率", "中位ms", "最大ms"))
for tries in (1, 2, 4, 8, 12):
    ts, errs, ts2, errs2 = [], [], [], []
    for Pt, Rt in TARGETS:
        t = time.time()
        _, e = arm.ik(np.zeros(NQ), Pt, tries=tries)      # 冷启动：从零位
        ts.append(time.time() - t)
        errs.append(e * 1000)

        t = time.time()
        _, e2 = arm.ik(SEEDS[len(ts2)], Pt, tries=tries)  # 最坏情况：随机 q0
        ts2.append(time.time() - t)
        errs2.append(e2 * 1000)
    errs, errs2 = np.array(errs), np.array(errs2)
    print("%6d %9.0f%% %9.1f %9.0f%% %9.1f %9.1f"
          % (tries, np.mean(errs < 1.0) * 100, np.median(ts) * 1000,
             np.mean(errs2 < 1.0) * 100, np.median(ts2) * 1000,
             np.max(ts2) * 1000))

print()
print("=" * 64)
print("3. 位姿 IK：起点数 vs 成功率")
print("=" * 64)
print("%6s %8s %11s %9s %9s" % ("起点数", "成功率", "姿态中位°", "中位ms", "最大ms"))
for tries in (2, 4, 8, 12):
    ts, perr, rerr = [], [], []
    for Pt, Rt in TARGETS:
        t = time.time()
        _, ep, er = arm.ik_pose(SEEDS[len(ts)], Pt, Rt, tries=tries)
        ts.append(time.time() - t)
        perr.append(ep * 1000)
        rerr.append(er)
    perr, rerr = np.array(perr), np.array(rerr)
    ok = np.mean((perr < 1.0) & (rerr < 0.5)) * 100
    print("%6d %7.0f%% %11.3f %9.1f %9.1f"
          % (tries, ok, np.median(rerr), np.median(ts) * 1000,
             np.max(ts) * 1000))

print()
print("=" * 64)
print("4. 任务相关分布：壁面前 0.2 m 内的目标（x∈[0.9,1.3]）")
print("=" * 64)
Pt_list = []
for _ in range(N):
    Pt_list.append(np.array([rng.uniform(0.9, 1.3),
                             rng.uniform(-0.6, 0.6),
                             rng.uniform(0.4, 1.8)]))
for tries in (1, 4, 8):
    ok = 0
    errs = []
    t0 = time.time()
    for Pt in Pt_list:
        _, e, good = arm.ik_ex(np.zeros(NQ), Pt, tries=tries)
        ok += good
        errs.append(e * 1000)
    print("  起点 %2d  可达 %2d/%d  中位残差 %7.2f mm  用时 %.2f s"
          % (tries, ok, N, np.median(errs), time.time() - t0))
