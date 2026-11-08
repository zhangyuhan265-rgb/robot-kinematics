# -*- coding: utf-8 -*-
"""红队 2：在「可信尺度」下找高度看不见、但位移看得见的方向。

参数归一化：x_i = Δp_i / s_i，s = [0.5°×6, 2mm×5]。
即 |x_i| ≤ 1 表示「所有改动都在可信范围内」（零位 ≤0.5°，长度 ≤2 mm）。
目标：|A·x|∞ ≤ 1 mm（高度基本不变）时，|B·x| 最大是多少。
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
S = np.array([0.5] * 6 + [2.0] * 5)


def observe(p):
    ds = np.array(p[:6], dtype=float)
    dl = dict(zip(("a1", "a2", "a3", "d4", "d6"), p[6:]))
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    txt = txt.replace(J2, 'xyz="%.6f 0 0.431" rpy="1.570796327 0 0"'
                      % (0.170 + dl["a1"] / 1000.0))
    for k in ("a2", "a3", "d4", "d6"):
        old, fmt, base = STR[k]
        txt = txt.replace(old, fmt % ("%.6f" % (base + dl[k] / 1000.0)))
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    arm = UrdfArm(path)
    os.remove(path)
    dsr = np.radians(ds)

    def pose(q):
        return arm.fk(np.radians(np.array(q, dtype=float) + dsr))[arm.tip]

    def hh(q):
        return arm.fk(np.radians(np.array(q, dtype=float) + dsr))[arm.tip][2, 3] * 1000 + RISER

    T0, TA, TB = pose(Q0), pose(QA), pose(QB)
    dist = lambda A, B: np.linalg.norm(A[:2, 3] - B[:2, 3]) * 1000
    return (np.array([hh(q) - m for q, m in DATA]),
            np.array([dist(TA, T0) - 300.0, dist(T0, TB) - 300.0]))


base_h, base_d = observe(np.zeros(11))
H, D = [], []
for i in range(11):
    pp = np.zeros(11)
    pp[i] = S[i] * 1e-3          # 小扰动，单位=可信尺度×1e-3
    hh, dd = observe(pp)
    H.append((hh - base_h) / 1e-3)
    D.append((dd - base_d) / 1e-3)
A = np.array(H).T        # d(height)/d(x)，x 单位=可信尺度
B = np.array(D).T        # d(位移)/d(x)

U, sv, Vt = np.linalg.svd(A, full_matrices=True)
print("归一化后高度雅可比奇异值：" + " ".join("%.3f" % s for s in sv))
print()
print("逐个奇异方向：把高度变化压到 1 mm 时，位移能变多少")
print("%4s %10s %14s %14s %10s" % ("方向", "高度/mm", "位移x/mm", "位移y/mm", "参数幅度"))
best = None
for i in range(11):
    v = Vt[i]
    hv = np.abs(A @ v).max()
    if hv < 1e-9:
        c = 1.0
    else:
        c = 1.0 / hv
    amp = np.abs(c * v).max()
    bx, by = (B @ (c * v))
    tag = ""
    if amp <= 3.0 and max(abs(bx), abs(by)) > 2.0:
        tag = "  <== 红队可用"
        if best is None or max(abs(bx), abs(by)) > best[0]:
            best = (max(abs(bx), abs(by)), c * v, bx, by)
    print("%4d %10.4f %14.2f %14.2f %10.2f%s" % (i, hv, bx, by, amp, tag))

if best is not None:
    _, v, bx, by = best
    print()
    print("红队候选：高度变化 1 mm，位移变化 %+.2f / %+.2f mm" % (bx, by))
    print("对应参数改动（乘以 1 mm 高度预算）：")
    for name, val in zip(NAMES, v):
        if abs(val) > 1e-3:
            unit = "°" if name.startswith("δ") else "mm"
            print("      %-5s %+8.3f %s" % (name, val, unit))
    # 直接非线性验证
    hh, dd = observe(v)
    print()
    print("非线性直接验证（不是线性化）：")
    print("  高度残差最大变化 %.2f mm" % np.abs(hh).max())
    print("  位移 %.2f / %.2f mm" % (300 + dd[0], 300 + dd[1]))
else:
    print("\n没有找到「可信范围内、高度压得住、位移能改 >2 mm」的方向。")
