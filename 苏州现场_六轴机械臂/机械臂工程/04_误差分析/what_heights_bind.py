# -*- coding: utf-8 -*-
"""竖直数据能挡住多大的水平误差？

做法：逐个把六段长度改掉 Δ，看
  (a) 12 个位姿的高度 RMS 变成多少
  (b) 那两段 300 mm 平移算出来的位移变成多少
找出「RMS 仍可接受（≤6 mm）时，水平位移能错多少」。
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
QA = [0, -23.65, 28.78, 0, 5.12, 0]     # 外推 300
QB = [0, 21.28, -17.13, 0, 4.15, 0]     # 收回 300
Q0 = [0, 0, 0, 0, 0, 0]

# 参数 -> (模板, 格式化函数)
PARAM = {
    "a1  J1→J2 水平":  ('xyz="0.17 0 0.431" rpy="1.570796327 0 0"',
                       'xyz="%s 0 0.431" rpy="1.570796327 0 0"', 0.170),
    "a2  J2→J3":      ('xyz="0 0.79 0" rpy="0 0 1.570796327"',
                       'xyz="0 %s 0" rpy="0 0 1.570796327"', 0.790),
    "a3  J3→J4":      ('xyz="0.156 0 0" rpy="1.570796327 0 0"',
                       'xyz="%s 0 0" rpy="1.570796327 0 0"', 0.156),
    "d4  J4→J5":      ('xyz="0 0 0.75" rpy="1.570796327 0 0"',
                       'xyz="0 0 %s" rpy="1.570796327 0 0"', 0.750),
    "d6  J5→法兰":     ('xyz="0 0 0.1325" rpy="0 0 -3.141592654"',
                       'xyz="0 0 %s" rpy="0 0 -3.141592654"', 0.1325),
}


def build(key, val):
    old, fmt, _ = PARAM[key]
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    assert txt.count(old) == 1, key
    txt = txt.replace(old, fmt % ("%.6f" % val))
    fd, p = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    return p


def stats(key, val):
    p = build(key, val)
    arm = UrdfArm(p)
    os.remove(p)
    errs = np.array([arm.fk(np.radians(q))[arm.tip][2, 3] * 1000 + RISER - m
                     for q, m in DATA])
    x0 = arm.fk(np.radians(Q0))[arm.tip][0, 3] * 1000
    xa = arm.fk(np.radians(QA))[arm.tip][0, 3] * 1000
    xb = arm.fk(np.radians(QB))[arm.tip][0, 3] * 1000
    return (np.sqrt((errs ** 2).mean()),
            xa - x0, x0 - xb)


b_rms, b_a, b_b = stats("d4  J4→J5", 0.750)
print("基准：高度 RMS %.2f mm，外推位移 %.2f mm，收回位移 %.2f mm"
      % (b_rms, b_a, b_b))
print()
print("%-16s %8s %10s %10s %10s"
      % ("参数", "改动Δ", "高度RMS", "外推位移", "收回位移"))
print("-" * 60)
for key in PARAM:
    base = PARAM[key][2]
    for frac in (-0.10, -0.05, 0.05, 0.10):
        v = base * (1 + frac)
        rms, a, b = stats(key, v)
        print("%-16s %+8.1f %10.2f %10.2f %10.2f"
              % (key, (v - base) * 1000, rms, a, b))
    print()
