# -*- coding: utf-8 -*-
"""FK 验证。重点是找一个**不依赖本实现**的外部参照。

参照分三类：
  A. 结构自检：旋转矩阵正交、零位手算
  B. 运动学不变量：绕 J1 / J2 轴转时某些量必须守恒（不依赖任何外部数据）
  C. 外部实测：控制器上报的 TCP（厂商自己的 FK 实现）+ 现场卷尺高度
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
fk_T = lambda q: arm.fk(q)[TIP]
rng = np.random.default_rng(31)

print("=" * 66)
print("A. 结构自检")
print("=" * 66)
worst_det = 0.0
worst_orth = 0.0
for _ in range(500):
    R = fk_T(rng.uniform(LO, HI))[:3, :3]
    worst_det = max(worst_det, abs(np.linalg.det(R) - 1))
    worst_orth = max(worst_orth, np.abs(R @ R.T - np.eye(3)).max())
print("  det(R) 离 1 的最大偏差     %.2e" % worst_det)
print("  R·Rᵀ 离 I 的最大偏差       %.2e" % worst_orth)

# 零位手算：法兰 = (a1+d4, 0, d1+a2+a3-d6)
a1, a2, a3, d1, d4, d6 = 0.170, 0.790, 0.156, 0.431, 0.750, 0.1325
manual = np.array([a1 + d4, 0.0, d1 + a2 + a3 - d6])
got = fk_T(np.zeros(NQ))[:3, 3]
print("  零位手算   (%8.1f, %8.1f, %8.1f) mm" % tuple(manual * 1000))
print("  零位 FK    (%8.1f, %8.1f, %8.1f) mm" % tuple(got * 1000))
print("  差         %.3e mm" % (np.abs(got - manual).max() * 1000))

print()
print("=" * 66)
print("B. 运动学不变量（不需要任何外部数据）")
print("=" * 66)
# B1 绕 J1 转：高度和水平半径必须守恒
h0 = fk_T(np.zeros(NQ))[2, 3]
r0 = np.hypot(*fk_T(np.zeros(NQ))[:2, 3])
eh = er = 0.0
for q1 in np.radians(np.arange(-130, 131, 10)):
    q = np.zeros(NQ)
    q[0] = q1
    T = fk_T(q)
    eh = max(eh, abs(T[2, 3] - h0))
    er = max(er, abs(np.hypot(T[0, 3], T[1, 3]) - r0))
print("  q1 扫描 [-130,130]°  高度最大变化 %.2e mm   半径最大变化 %.2e mm"
      % (eh * 1000, er * 1000))

# B2 绕 J2 转：法兰到 J2 轴的距离必须守恒，且始终在 y=0 平面内
pJ2 = np.array([a1, 0.0, d1])
dist0 = np.linalg.norm(fk_T(np.zeros(NQ))[:3, 3] - pJ2)
ed = ey = 0.0
for q2 in np.radians(np.arange(-100, 101, 5)):
    q = np.zeros(NQ)
    q[1] = q2
    P = fk_T(q)[:3, 3]
    ed = max(ed, abs(np.linalg.norm(P - pJ2) - dist0))
    ey = max(ey, abs(P[1]))
print("  q2 扫描 [-100,100]°  到 J2 轴距离最大变化 %.2e mm   |y| 最大 %.2e mm"
      % (ed * 1000, ey * 1000))

# B3 q6 是法兰自转，位置必须完全不变
e6 = 0.0
base = fk_T(np.radians([10, -20, 30, 40, -50, 0]))[:3, 3]
for q6 in np.radians(np.arange(-180, 181, 15)):
    q = np.radians([10, -20, 30, 40, -50, q6])
    e6 = max(e6, np.linalg.norm(fk_T(q)[:3, 3] - base))
print("  q6 扫描 [-180,180]°  位置最大变化 %.2e mm" % (e6 * 1000))

# B4 镜像：关于 x-z 平面做镜面反射 M = diag(1,-1,1)。
#     绕轴 a 转 θ 的旋转，镜像后变成绕 M·a 转 −θ。于是：
#       a 平行于 y（|a_y|=1）→ 角度不变；a 在 x-z 平面内 → 角度取反。
em = 0.0
axes0 = arm.jacobian_geo(np.zeros(NQ))[3:]      # 3×6，零位时各轴的世界方向
flip = np.abs(axes0[1]) < 0.5                   # a_y≈0 的要取反
print("  零位各轴方向：" + "  ".join(
    "J%d(%+.2f,%+.2f,%+.2f)%s" % (i + 1, *axes0[:, i], "反" if flip[i] else "同")
    for i in range(NQ)))
for _ in range(100):
    q = rng.uniform(LO, HI)
    A = fk_T(q)[:3, 3]
    q2 = q.copy()
    q2[flip] = -q2[flip]
    B = fk_T(q2)[:3, 3]
    em = max(em, np.linalg.norm(B - np.array([A[0], -A[1], A[2]])))
print("  镜像检验（按上面的规则翻转）最大偏差 %.2e mm" % (em * 1000))

print()
print("=" * 66)
print("C1. 对照控制器上报的 TCP（厂商自己的 FK 实现）")
print("=" * 66)
REC = [
    ([0, 0, 0, 0, 0, 0], (920.0, 0.0, 1244.5)),
    ([30, 0, 0, 0, 0, 0], (796.7, 460.0, 1244.5)),
    ([-30, 0, 0, 0, 0, 0], (796.7, -460.0, 1244.5)),
    ([0, -23.65, 28.78, 0, 5.12, 0], (1220.0, 0.0, 1244.6)),
    ([0, 21.28, -17.13, 0, 4.15, 0], (620.0, 0.0, 1244.5)),
]
print("%-32s %-22s %9s" % ("q(度)", "控制器 TCP(mm)", "差 mm"))
worst = 0.0
for qd, ctl in REC:
    P = fk_T(np.radians(qd))[:3, 3] * 1000
    e = np.abs(P - np.array(ctl)).max()
    worst = max(worst, e)
    print("%-32s (%7.1f,%7.1f,%7.1f) %9.3f"
          % (str(qd), ctl[0], ctl[1], ctl[2], e))
print("  最大偏差 %.3f mm   （控制器只报 0.1 mm，所以这已经是量化极限）" % worst)

print()
print("=" * 66)
print("C2. 对照现场卷尺实测的法兰高度（12 个位姿）")
print("=" * 66)
DATA = [
    ([0, 0, 0, 0, 0, 0], 1581),
    ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474),
    ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068),
    ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549),
    ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569),
    ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666),
    ([20, -25, 35, 60, -30, 45], 1720),
]
RISER = 336.5
errs = []
for qd, meas in DATA:
    h = fk_T(np.radians(qd))[2, 3] * 1000 + RISER
    errs.append(h - meas)
errs = np.array(errs)
print("  平均 %+.1f mm   RMS %.1f mm   最大|e| %.1f mm"
      % (errs.mean(), np.sqrt((errs ** 2).mean()), np.abs(errs).max()))
print("  逐位姿: %s" % " ".join("%+5.0f" % e for e in errs))
