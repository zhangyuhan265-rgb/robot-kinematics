# -*- coding: utf-8 -*-
"""底座倾斜对「一维平移测距」到底有没有影响——严格计算。

关键：地面两点距离是「三维距离在地面上的投影」。
若整机刚性旋转 R，两个位姿的法兰位置变成 R·p1、R·p2，
测到的距离 = |proj_xy(R·p1) − proj_xy(R·p2)|。
"""
import sys

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

RISER = 0.3365
# 三个位姿的法兰位置（臂座坐标，来自控制器正解）
P = {
    "P3 收回(620)": np.array([0.620, 0.0, 1.2445]),
    "P1 零位(920)": np.array([0.920, 0.0, 1.2445]),
    "P2 外推(1220)": np.array([1.220, 0.0, 1.2446]),
}


def R_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def R_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def to_floor(p):
    q = p.copy()
    q[2] += RISER
    return q


def horiz_dist(a, b, R):
    pa = (R @ to_floor(a))[:2]
    pb = (R @ to_floor(b))[:2]
    return float(np.linalg.norm(pa - pb)) * 1000


def height(p, R):
    return float((R @ to_floor(p))[2]) * 1000


CASES = [
    ("无倾斜", np.eye(3)),
    ("绕 x（左右倒）+0.45°", R_x(np.radians(+0.45))),
    ("绕 x（左右倒）−0.45°", R_x(np.radians(-0.45))),
    ("绕 y（前后倒）+0.45°", R_y(np.radians(+0.45))),
    ("绕 y（前后倒）−0.45° 前面抬高", R_y(np.radians(-0.45))),
]

print("%-32s %9s %9s %9s %9s"
      % ("", "P1–P2", "P1–P3", "P2–P3", "零位高度"))
print("-" * 74)
for name, R in CASES:
    d12 = horiz_dist(P["P1 零位(920)"], P["P2 外推(1220)"], R)
    d13 = horiz_dist(P["P1 零位(920)"], P["P3 收回(620)"], R)
    d23 = horiz_dist(P["P2 外推(1220)"], P["P3 收回(620)"], R)
    h = height(P["P1 零位(920)"], R)
    print("%-32s %9.3f %9.3f %9.3f %9.2f" % (name, d12, d13, d23, h))

print()
print("实测：P1–P2 = 290，P1–P3 = 320，P2–P3 = 610（整数，卷尺 ±10~15 mm/点）")
print()
print("若倾斜真能解释 290/320（差 30 mm），需要的倾斜角：")
for axis, fn in (("绕 y（前后）", R_y), ("绕 x（左右）", R_x)):
    lo, hi = -30.0, 30.0
    for _ in range(80):
        mid = (lo + hi) / 2
        R = fn(np.radians(mid))
        d = horiz_dist(P["P1 零位(920)"], P["P2 外推(1220)"], R)
        if d > 290.0:
            lo = mid
        else:
            hi = mid
    print("  %s 需要 %.2f° —— 才能把 300 mm 缩成 290 mm" % (axis, abs(lo)))
