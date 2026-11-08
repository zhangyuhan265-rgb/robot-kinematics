# -*- coding: utf-8 -*-
"""红队：高度数据到底有几个「看不见的方向」？

参数 11 个：6 个关节零位（度）+ 5 段长度（毫米）。
观测量 12 个高度 + 2 个位移。
先算 12×11 的高度雅可比，取它的零空间；再看零空间里哪个方向对位移影响最大。
如果存在一个物理上说得通的组合能改 10 mm 位移而不改高度，那我之前的结论就塌了。
"""
import os
import sys
import tempfile

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

SRC = "nbt_arm6.urdf"
RISER = 336.5
DATA = [
    ([0, 0, 0, 0, 0, 0], 1581), ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474), ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068), ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549), ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569), ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666), ([20, -25, 35, 60, -30, 45], 1720),
]
QA = [0, -23.65, 28.78, 0, 5.12, 0]
QB = [0, 21.28, -17.13, 0, 4.15, 0]
Q0 = [0, 0, 0, 0, 0, 0]

J2 = 'xyz="0.17 0 0.431" rpy="1.570796327 0 0"'
STR = {
    "a2": ('xyz="0 0.79 0" rpy="0 0 1.570796327"',
           'xyz="0 %s 0" rpy="0 0 1.570796327"', 0.790),
    "a3": ('xyz="0.156 0 0" rpy="1.570796327 0 0"',
           'xyz="%s 0 0" rpy="1.570796327 0 0"', 0.156),
    "d4": ('xyz="0 0 0.75" rpy="1.570796327 0 0"',
           'xyz="0 0 %s" rpy="1.570796327 0 0"', 0.750),
    "d6": ('xyz="0 0 0.1325" rpy="0 0 -3.141592654"',
           'xyz="0 0 %s" rpy="0 0 -3.141592654"', 0.1325),
}
NAMES = ["δ1", "δ2", "δ3", "δ4", "δ5", "δ6",
         "Δa1", "Δa2", "Δa3", "Δd4", "Δd6"]


def make_arm(ds_deg, dlen_mm):
    """ds_deg: 6 个零位偏置（度）；dlen_mm: {a1,a2,a3,d4,d6} 增量（毫米）"""
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    a1 = (0.170 + dlen_mm.get("a1", 0.0) / 1000.0)
    d1 = 0.431
    txt = txt.replace(J2, 'xyz="%.6f 0 %.6f" rpy="1.570796327 0 0"' % (a1, d1))
    for k in ("a2", "a3", "d4", "d6"):
        old, fmt, base = STR[k]
        v = base + dlen_mm.get(k, 0.0) / 1000.0
        txt = txt.replace(old, fmt % ("%.6f" % v))
    fd, p = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    return p


def observe(ds_deg, dlen_mm):
    p = make_arm(ds_deg, dlen_mm)
    arm = UrdfArm(p)
    os.remove(p)
    ds = np.radians(np.array(ds_deg))

    def pose(q):
        qq = np.array(q, dtype=float) + ds
        return arm.fk(np.radians(qq))[arm.tip]

    def h(q):
        qq = np.array(q, dtype=float) + ds
        return arm.fk(np.radians(qq))[arm.tip][2, 3] * 1000 + RISER

    T0, TA, TB = pose(Q0), pose(QA), pose(QB)
    dist = lambda A, B: np.linalg.norm(A[:2, 3] - B[:2, 3]) * 1000
    return (np.array([h(q) for q, _ in DATA]),
            np.array([dist(TA, T0), dist(T0, TB)]))


base_h, base_d = observe([0] * 6, {})
EPS = [0.1] * 6 + [0.1] * 5          # 0.1 度 / 0.1 毫米


def f(p):
    ds = list(p[:6])
    dl = dict(zip(("a1", "a2", "a3", "d4", "d6"), p[6:]))
    return observe(ds, dl)


H, D = [], []
for i in range(11):
    pp = np.zeros(11)
    pp[i] = EPS[i]
    hh, dd = f(pp)
    H.append((hh - base_h) / EPS[i])
    D.append((dd - base_d) / EPS[i])
Jh = np.array(H).T          # 12×11
Jd = np.array(D).T          # 2×11

U, S, Vt = np.linalg.svd(Jh, full_matrices=True)
rank = int((S > S[0] * 1e-6).sum())
print("高度雅可比 %d×%d，秩 = %d → 零空间维数 = %d"
      % (Jh.shape[0], Jh.shape[1], rank, 11 - rank))
print("奇异值：", " ".join("%.3f" % s for s in S))

null = Vt[rank:]           # （11-rank）× 11
print("\n零空间里，对两个位移影响最大的方向：")
best = None
for v in null:
    g = Jd @ v
    n = np.linalg.norm(g)
    if best is None or n > best[0]:
        best = (n, v, g)
n, v, g = best
# 归一化到「最大参数改动 = 1」（即一个参数改 1 个单位，其它按比例）
scale = 1.0 / np.abs(v).max()
v = v * scale
g = Jd @ v
print("  参数方向（最大改动归一化为 1）：")
for name, val in zip(NAMES, v):
    if abs(val) > 1e-3:
        unit = "°" if name.startswith("δ") else "mm"
        print("      %-5s %+8.3f %s" % (name, val, unit))
print("  单位改动下：高度最大变化 %.3f mm，位移变化 %.3f / %.3f mm"
      % (np.abs(Jh @ v).max(), g[0], g[1]))

# 放大到「高度变化 ≤ 1 mm」的边界，看位移能变多少
lim = 1.0 / np.abs(Jh @ v).max()
print("\n把该方向放大到「高度最大变化 = 1 mm」：")
print("  位移变化 = %+.2f / %+.2f mm" % (g[0] * lim, g[1] * lim))
print("  对应参数改动：")
for name, val in zip(NAMES, v * lim):
    if abs(val) > 1e-3:
        unit = "°" if name.startswith("δ") else "mm"
        print("      %-5s %+8.3f %s" % (name, val, unit))
