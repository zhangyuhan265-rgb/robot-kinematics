# -*- coding: utf-8 -*-
"""对「水平位移」那一块做独立复核。

查四件事：
  V1  d4 大范围扫描：位移真的一直是 300 吗？还是只在 ±10% 附近平坦？
  V2  三个位姿的姿态是否严格相同（这决定了 d4/a3/d6 是否抵消）
  V3  整体比例 k：位移与高度各怎么变
  V4  关节增益误差（不是零位！）：高度与位移各怎么变
  V5  三个位姿的 y 是否都为 0（否则 |Δx| ≠ 地面距离）
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

LEN = {
    "d1": ('xyz="0.17 0 0.431" rpy="1.570796327 0 0"',
           'xyz="0.17 0 %s" rpy="1.570796327 0 0"', 0.431),
    "a1": ('xyz="0.17 0 0.431" rpy="1.570796327 0 0"',
           'xyz="%s 0 0.431" rpy="1.570796327 0 0"', 0.170),
    "a2": ('xyz="0 0.79 0" rpy="0 0 1.570796327"',
           'xyz="0 %s 0" rpy="0 0 1.570796327"', 0.790),
    "a3": ('xyz="0.156 0 0" rpy="1.570796327 0 0"',
           'xyz="%s 0 0" rpy="1.570796327 0 0"', 0.156),
    "d4": ('xyz="0 0 0.75" rpy="1.570796327 0 0"',
           'xyz="0 0 %s" rpy="1.570796327 0 0"', 0.750),
    "d6": ('xyz="0 0 0.1325" rpy="0 0 -3.141592654"',
           'xyz="0 0 %s" rpy="0 0 -3.141592654"', 0.1325),
}


def build(repl=None, k=None):
    """repl: {参数名: 新值}；k: 所有长度同乘系数（d1/a1 共用一段文本，必须一起改）"""
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    J2 = 'xyz="0.17 0 0.431" rpy="1.570796327 0 0"'
    assert txt.count(J2) == 1
    if k is not None:
        a1 = repl["a1"] if repl and "a1" in repl else 0.170 * k
        d1 = repl["d1"] if repl and "d1" in repl else 0.431 * k
        txt = txt.replace(J2, 'xyz="%.6f 0 %.6f" rpy="1.570796327 0 0"'
                          % (a1, d1))
    else:
        a1 = (repl or {}).get("a1", 0.170)
        d1 = (repl or {}).get("d1", 0.431)
        txt = txt.replace(J2, 'xyz="%.6f 0 %.6f" rpy="1.570796327 0 0"'
                          % (a1, d1))
    for name in ("a2", "a3", "d4", "d6"):
        old, fmt, base = LEN[name]
        v = (repl or {}).get(name, base * k if k is not None else base)
        assert txt.count(old) == 1, name
        txt = txt.replace(old, fmt % ("%.6f" % v))
    fd, p = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    return p


def measure(repl=None, k=None, qshift=None, qgain=None):
    p = build(repl, k)
    arm = UrdfArm(p)
    os.remove(p)

    def pose(q):
        qq = np.array(q, dtype=float)
        if qgain is not None:
            qq = qq * qgain
        if qshift is not None:
            qq = qq + np.radians(qshift)
        return arm.fk(np.radians(qq))[arm.tip]

    def height(q):
        qq = np.array(q, dtype=float)
        if qgain is not None:
            qq = qq * qgain
        if qshift is not None:
            qq = qq + np.radians(qshift)
        return arm.fk(np.radians(qq))[arm.tip][2, 3] * 1000 + RISER

    T0, TA, TB = pose(Q0), pose(QA), pose(QB)
    errs = np.array([height(q) - m for q, m in DATA])
    d = lambda A, B: np.linalg.norm(A[:2, 3] - B[:2, 3]) * 1000
    return (np.sqrt((errs ** 2).mean()), d(TA, T0), d(T0, TB),
            T0, TA, TB)


print("=" * 66)
print("V2 三个位姿的姿态是否严格相同？（不同则 d4/a3/d6 不会抵消）")
print("=" * 66)
_, _, _, T0, TA, TB = measure()
for name, T in (("零位", T0), ("外推", TA), ("收回", TB)):
    ang = np.degrees(np.arccos(np.clip((np.trace(T0[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
    print("  %s：与零位的姿态夹角 = %.3e 度，y = %+.6f mm"
          % (name, ang, T[1, 3] * 1000))

print()
print("=" * 66)
print("V1  d4 大范围扫描（不只是 ±10%）")
print("=" * 66)
print("%10s %10s %12s %12s" % ("d4 (mm)", "高度RMS", "外推(mm)", "收回(mm)"))
for v in (0.600, 0.675, 0.7125, 0.750, 0.7875, 0.825, 0.900):
    r = measure({"d4": v})
    print("%10.1f %10.2f %12.2f %12.2f" % (v * 1000, r[0], r[1], r[2]))

print()
print("=" * 66)
print("V3  整体比例 k（所有长度同乘 k）")
print("=" * 66)
print("%8s %10s %12s %12s" % ("k", "高度RMS", "外推(mm)", "收回(mm)"))
for k in (0.990, 0.995, 0.997, 1.000, 1.003, 1.005, 1.010):
    r = measure(k=k)
    print("%8.3f %10.2f %12.2f %12.2f" % (k, r[0], r[1], r[2]))

print()
print("=" * 66)
print("V4  关节增益误差（实际角 = 命令角 × (1+g)）")
print("=" * 66)
print("%8s %10s %12s %12s" % ("g", "高度RMS", "外推(mm)", "收回(mm)"))
for g in (-0.04, -0.03, -0.02, -0.015, -0.01, -0.005, 0.0, 0.005, 0.01, 0.02, 0.03, 0.04):
    r = measure(qgain=1 + g)
    print("%8.3f %10.2f %12.2f %12.2f" % (g, r[0], r[1], r[2]))
