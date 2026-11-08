# -*- coding: utf-8 -*-
"""如果直接用厂家彩页的 d4=650 建模型，12 个位姿的高度误差会是多大？

这是报告里最有力的一句话：不是我们量不准，是照图纸建会差多少。
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
OLD = 'xyz="0 0 0.75" rpy="1.570796327 0 0"'
RISER = 336.5
DATA = [
    ([0, 0, 0, 0, 0, 0], 1581), ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474), ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068), ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549), ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569), ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666), ([20, -25, 35, 60, -30, 45], 1720),
]


def rms_for(d4):
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    assert txt.count(OLD) == 1
    txt = txt.replace(OLD, 'xyz="0 0 %s" rpy="1.570796327 0 0"' % d4)
    fd, p = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    arm = UrdfArm(p)
    errs = []
    for q, meas in DATA:
        h = arm.fk(np.radians(q))[arm.tip][2, 3] * 1000 + RISER
        errs.append(h - meas)
    os.remove(p)
    e = np.array(errs)
    return e


print("%-22s %9s %9s %9s" % ("d4 取值", "平均", "RMS", "最大|e|"))
print("-" * 52)
for label, v in [("彩页 650", 0.650), ("折中 700", 0.700),
                 ("标定 750", 0.750), ("假设 800", 0.800)]:
    e = rms_for(v)
    print("%-22s %+9.1f %9.1f %9.1f"
          % (label, e.mean(), np.sqrt((e ** 2).mean()), np.abs(e).max()))
e = rms_for(0.650)
print()
print("按彩页 650 的逐位姿残差：")
print("  " + " ".join("%+5.0f" % v for v in e))
