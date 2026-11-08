# -*- coding: utf-8 -*-
"""水平对齐度试验的俯视示意图：三个半径下 q1 转 ±90° 的法兰落点与弦长。

用法：python horiz_fig.py
输出：media/horiz_plan.png
"""
import numpy as np

from arm_env import UrdfArm

arm = UrdfArm("nbt_arm6.urdf")

CASES = [
    ("零位 q=(0,0,0,0,0,0)", [0, 0, 0, 0, 0, 0]),
    ("q3=+40", [0, 0, 40, 0, 0, 0]),
    ("q2=+30", [0, 30, 0, 0, 0, 0]),
]


def radius_of(qd):
    p = arm.tcp(np.radians(qd))
    return np.hypot(p[0], p[1])


import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

fig, axes = plt.subplots(1, 3, figsize=(12.6, 4.6), dpi=120)

for ax, (name, qd) in zip(axes, CASES):
    p = arm.tcp(np.radians(qd))
    R = np.hypot(p[0], p[1])
    az0 = np.arctan2(p[1], p[0])

    th = np.linspace(0, 2 * np.pi, 400)
    ax.plot(R * np.cos(th) * 1000, R * np.sin(th) * 1000,
            color="#c8d0da", lw=1.2, ls="--", zorder=1)

    pts = {}
    for dq, col in ((0, "#1f77b4"), (90, "#d62728"), (-90, "#2ca02c")):
        a = az0 + np.radians(dq)
        pts[dq] = (R * np.cos(a) * 1000, R * np.sin(a) * 1000)
        ax.plot(*pts[dq], "o", ms=7, color=col, zorder=4)
        ax.annotate("q1=%+d°" % dq, pts[dq], textcoords="offset points",
                    xytext=(8, 8), fontsize=9, color=col)

    for (a, b), t in (((0, 90), 0.5), ((0, -90), 0.5), ((90, -90), 0.30)):
        x = [pts[a][0], pts[b][0]]
        y = [pts[a][1], pts[b][1]]
        ax.plot(x, y, color="#888888", lw=1.0, ls=":", zorder=2)
        chord = 2 * R * np.sin(np.radians(abs(b - a)) / 2) * 1000
        mx, my = x[0] + (x[1] - x[0]) * t, y[0] + (y[1] - y[0]) * t
        # 标注沿“远离圆心”的方向往外挪，避免压到端点标签
        ux, uy = x[1] - x[0], y[1] - y[0]
        n = np.hypot(ux, uy) or 1.0
        nx, ny = -uy / n, ux / n
        if nx * mx + ny * my < 0:
            nx, ny = -nx, -ny
        if abs(nx * mx + ny * my) < 1e-9:
            nx, ny = 0.0, -1.0
        ax.annotate("%.0f" % chord, (mx, my), textcoords="offset points",
                    xytext=(nx * 18, ny * 18), fontsize=9, color="#444444",
                    ha="center", va="center",
                    bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.0))

    ax.plot(0, 0, "s", ms=9, color="#333333", zorder=5)
    ax.annotate("J1 轴", (0, 0), textcoords="offset points", xytext=(-10, -16),
                fontsize=9, color="#333333", ha="right")

    lim = 1180
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.6)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("%s\nR = %.1f mm" % (name, R * 1000), fontsize=10)

fig.suptitle("水平对齐度：q1 单轴转 ±90° 时法兰中心在地面的落点（单位 mm，数字为模型预测弦长）",
             fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig("media/horiz_plan.png")
print("已保存 media/horiz_plan.png")
