# -*- coding: utf-8 -*-
"""区分“a1 长度偏小”和“a1 方向偏了”这两种可能。

零位时 R = |a1向量 + d4向量|，单测一个 R 分不开长度和方向。
做法：构造三个模型，看它们在别的位姿下预测的 R 差多少。
  A 基准   a1 = 170 沿 +X
  B 长度小 a1 = 118 沿 +X
  C 方向偏 a1 = 170 但转 50.6°，使零位 R 也等于 868
"""
import os
import tempfile

import numpy as np

from arm_env import UrdfArm

SRC = "nbt_arm6.urdf"
A1_OLD = 'xyz="0.17 0 0.431" rpy="1.570796327 0 0"'


def make(ax, ay):
    with open(SRC, encoding="utf-8") as f:
        txt = f.read()
    assert txt.count(A1_OLD) == 1
    txt = txt.replace(A1_OLD,
                      'xyz="%.6f %.6f 0.431" rpy="1.570796327 0 0"' % (ax, ay))
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(txt)
    return path


MODELS = [
    ("A 基准 a1=170 / 0°", 0.170, 0.0),
    ("B 长度 118 / 0°", 0.118, 0.0),
    ("C 长度170 方向50.6°", 0.170 * np.cos(np.radians(50.6)),
     0.170 * np.sin(np.radians(50.6))),
]

POSES = [
    ("零位", [0, 0, 0, 0, 0, 0]),
    ("q2=+30", [0, 30, 0, 0, 0, 0]),
    ("q2=-15", [0, -15, 0, 0, 0, 0]),
    ("q3=+40", [0, 0, 40, 0, 0, 0]),
    ("现在这个位姿", [0, -30, 0, 0, 0, 0]),
]

arms = []
for label, ax, ay in MODELS:
    p = make(ax, ay)
    arms.append((label, UrdfArm(p)))
    os.remove(p)

print("%-22s %10s %10s %10s" % ("位姿", *[l for l, _ in arms]))
for name, q in POSES:
    row = []
    for _, arm in arms:
        T = arm.fk(np.radians(q))[arm.tip]
        row.append(np.hypot(T[0, 3], T[1, 3]) * 1000)
    print("%-22s %10.1f %10.1f %10.1f" % (name, *row))
    print("%-22s %10s %+10.1f %+10.1f"
          % ("  (相对 A)", "", row[1] - row[0], row[2] - row[0]))
