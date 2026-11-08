# -*- coding: utf-8 -*-
"""求“法兰姿态不变、只平移 300 mm”对应的关节角。

为什么要恒姿态：挂点偏心 δ 是固定在法兰坐标系里的向量。
只有姿态不变，δ 在世界的方向才不变，两个地面点的距离才严格等于
法兰中心的实际平移量，δ 才能完全抵消。

用 6 维数值 Jacobian 做阻尼最小二乘 IK（位置 + 姿态都约束）。
"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
T0 = arm.fk(np.zeros(6))[arm.tip]
P0 = T0[:3, 3].copy()
R0 = T0[:3, :3].copy()
print("零位法兰中心 = (%8.1f, %8.1f, %8.1f) mm"
      % (P0[0] * 1000, P0[1] * 1000, P0[2] * 1000))


def rotvec(R):
    ang = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))
    if ang < 1e-12:
        return np.zeros(3)
    return ang / (2 * np.sin(ang)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def pose_err(q, Pt, Rt):
    T = arm.fk(q)[arm.tip]
    ep = Pt - T[:3, 3]
    er = rotvec(Rt @ T[:3, :3].T)
    return np.r_[ep, er]


def ik(q0, Pt, Rt, iters=400, lam=1e-4):
    q = np.array(q0, dtype=float)
    h = 1e-7
    for _ in range(iters):
        e = pose_err(q, Pt, Rt)
        if np.linalg.norm(e) < 1e-9:
            break
        J = np.zeros((6, 6))
        for i in range(6):
            qq = q.copy()
            qq[i] += h
            J[:, i] = (pose_err(qq, Pt, Rt) - e) / h
        dq = -np.linalg.solve(J.T @ J + lam * np.eye(6), J.T @ e)
        # 限步，避免跳出工作空间
        n = np.linalg.norm(dq)
        if n > 0.3:
            dq *= 0.3 / n
        q += dq
    return q, np.linalg.norm(pose_err(q, Pt, Rt))


CANDS = [
    ("沿 +X 平移 300（往外推）", P0 + np.array([0.300, 0, 0])),
    ("沿 +X 平移 150", P0 + np.array([0.150, 0, 0])),
    ("沿 +X 平移 200", P0 + np.array([0.200, 0, 0])),
    ("沿 -X 平移 300（收回）", P0 + np.array([-0.300, 0, 0])),
]

print()
print("%-28s %-9s %s" % ("目标", "残差", "关节角解"))
for name, Pt in CANDS:
    best = None
    for seed in (np.zeros(6), np.radians([0, -20, 20, 0, 0, 0]),
                 np.radians([0, 20, -20, 0, 0, 0]),
                 np.radians([0, -40, 40, 0, 0, 0])):
        q, r = ik(seed, Pt, R0)
        if best is None or r < best[1]:
            best = (q, r)
    q, r = best
    print("%-28s %8.2e  %s" % (name, r,
                               " ".join("%+7.2f" % v for v in np.degrees(q))))
    print("%-28s %8s  目标 (%7.1f, %7.1f, %7.1f) mm"
          % ("", "", Pt[0] * 1000, Pt[1] * 1000, Pt[2] * 1000))
