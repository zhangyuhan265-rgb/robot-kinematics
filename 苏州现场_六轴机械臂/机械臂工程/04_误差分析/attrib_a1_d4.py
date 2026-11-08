# -*- coding: utf-8 -*-
"""把 12 个对齐姿态的高度残差，分别归到 a1 和 d4 上。

直接在已知正确的 nbt_arm6.urdf 上替换两个数字：
  j2 的 origin x  = a1  (0.17)
  j5 的 origin z  = d4  (0.75)
a1 只把臂水平平移，理论上完全不影响高度；
d4 是小臂，小臂一旦不水平就顶高度，应该被高度数据卡住。
"""
import os
import re
import tempfile

import numpy as np

from arm_env import UrdfArm

RISER = 336.5
SRC = "nbt_arm6.urdf"

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

A1_OLD = 'xyz="0.17 0 0.431" rpy="1.570796327 0 0"'
D4_OLD = 'xyz="0 0 0.75" rpy="1.570796327 0 0"'


def make(a1, d4):
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    assert txt.count(A1_OLD) == 1, "a1 定位失败"
    assert txt.count(D4_OLD) == 1, "d4 定位失败"
    txt = txt.replace(A1_OLD, 'xyz="%s 0 0.431" rpy="1.570796327 0 0"' % a1)
    txt = txt.replace(D4_OLD, 'xyz="0 0 %s" rpy="1.570796327 0 0"' % d4)
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    return path


CASES = [
    ("基准   a1=170 d4=750", 0.17, 0.75),
    ("a1=118", 0.118, 0.75),
    ("d4=698", 0.17, 0.698),
    ("a1=118 d4=698", 0.118, 0.698),
    ("a1=170 d4=698 + 整体下移", 0.17, 0.698),
]

print("%-26s %9s %9s %10s" % ("假设", "平均误差", "RMS", "最大|误差|"))
for label, a1, d4 in CASES:
    path = make(a1, d4)
    arm = UrdfArm(path)
    errs = []
    for q, meas in DATA:
        T = arm.fk(np.radians(q))[arm.tip]
        errs.append((T[2, 3] + RISER / 1000.0) * 1000.0 - meas)
    errs = np.array(errs)
    print("%-26s %+9.1f %9.1f %10.1f"
          % (label, errs.mean(), np.sqrt((errs ** 2).mean()), np.abs(errs).max()))
    print("%28s%s" % ("逐位姿:", " ".join("%+5.0f" % e for e in errs)))
    os.remove(path)
