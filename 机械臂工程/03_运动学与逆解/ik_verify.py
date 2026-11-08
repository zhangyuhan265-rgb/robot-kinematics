# -*- coding: utf-8 -*-
"""逆解回归测试（跑在 arm_env.UrdfArm 上）。

1. 关节限位
2. FK → IK 往返：单起点 vs 多起点 vs 位姿 IK
3. 壁面工作空间：x = 1.10 竖直面
4. 沿竖直轨迹的热启动

历史对照：改造前（固定阻尼 DLS、单起点）往返成功率 50%，90 分位误差 1557 mm，
且失败时不报错、静默返回错解。改造后见下面输出。
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

print("=" * 64)
print("1. 关节限位（来自 URDF）")
print("=" * 64)
print("  " + "  ".join("J%d[%+.0f,%+.0f]°"
                      % (i + 1, np.degrees(LO[i]), np.degrees(HI[i]))
                      for i in range(NQ)))

rng = np.random.default_rng(0)
MARGIN = np.radians(5.0)
N = 60


def round_trip(label, solver, pose=False):
    errs, rots, ts = [], [], []
    for _ in range(N):
        q_ref = rng.uniform(LO + MARGIN, HI - MARGIN)
        T = fk_T(q_ref)
        seed = rng.uniform(LO + MARGIN, HI - MARGIN)
        t = time.time()
        if pose:
            _, e, r = solver(seed, T[:3, 3], T[:3, :3])
            rots.append(r)
        else:
            _, e = solver(seed, T[:3, 3])
        ts.append(time.time() - t)
        errs.append(e * 1000)
    errs = np.array(errs)
    line = ("%-24s %7.2f %9.1f %9.1f %7.0f%% %7.0f"
            % (label, np.median(errs), np.percentile(errs, 90), errs.max(),
               np.mean(errs < 1.0) * 100, np.mean(ts) * 1000))
    if rots:
        line += "   姿态中位 %.3f°" % np.median(rots)
    print(line)


print()
print("=" * 64)
print("2. FK → IK 往返（N=%d，随机可达位姿）" % N)
print("=" * 64)
print("%-24s %7s %9s %9s %8s %7s"
      % ("", "中位mm", "90分位", "最大", "成功率", "ms/次"))
round_trip("位置 IK · 1 起点", lambda s, P: arm.ik(s, P, tries=1))
round_trip("位置 IK · 6 起点", lambda s, P: arm.ik(s, P, tries=6))
round_trip("位置 IK · 12 起点", lambda s, P: arm.ik(s, P, tries=12))
round_trip("位姿 IK · 6 起点", lambda s, P, R: arm.ik_pose(s, P, R, tries=6),
           pose=True)

print()
print("=" * 64)
print("3. 壁面工作空间：x = 1.10 m 竖直面（位置可达性）")
print("=" * 64)
ys = np.linspace(-0.60, 0.60, 7)
zs = np.linspace(0.30, 2.20, 9)
print("        z=" + " ".join("%4.1f" % z for z in zs))
tot = ok = 0
for y in ys:
    line = []
    for z in zs:
        tot += 1
        _, e, good = arm.ik_ex(np.zeros(NQ), np.array([1.10, y, z]), tries=4)
        ok += good
        line.append("   o" if good else "   .")
    print("  y=%+5.2f %s" % (y, "".join(line)))
print()
print("  可达 %d / %d = %.0f%%" % (ok, tot, ok / tot * 100))

print()
print("=" * 64)
print("4. 竖直轨迹热启动（x=1.10, y=0, z 0.30→1.90，35 点）")
print("=" * 64)
zs2 = np.linspace(0.30, 1.90, 35)
q = np.zeros(NQ)
ts, errs = [], []
for z in zs2:
    t = time.time()
    q, e = arm.ik(q, np.array([1.10, 0.0, z]), tries=1)   # 热启动只要 1 起点
    ts.append(time.time() - t)
    errs.append(e * 1000)
errs = np.array(errs)
print("  成功(<1mm) %d / %d   最大残差 %.2f mm"
      % (np.sum(errs < 1.0), len(zs2), errs.max()))
print("  单点耗时   中位 %.1f ms   最大 %.1f ms"
      % (np.median(ts) * 1000, np.max(ts) * 1000))
