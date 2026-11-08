# -*- coding: utf-8 -*-
"""重画报告里的「12 位姿高度误差」图：中文标签、大字号、数字与正文一致。"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

# 直接从模型算精确残差，避免用表格里取整过的数
DATA = [
    ([0, 0, 0, 0, 0, 0], 1581), ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474), ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068), ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549), ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569), ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666), ([20, -25, 35, 60, -30, 45], 1720),
]
RISER = 336.5
arm = UrdfArm("nbt_arm6.urdf")
ERR = []
for q, meas in DATA:
    h = arm.fk(np.radians(q))[arm.tip][2, 3] * 1000 + RISER
    ERR.append(h - meas)
POSES = list(range(1, 13))
e = np.array(ERR)
mean, rms, mx = e.mean(), np.sqrt((e ** 2).mean()), np.abs(e).max()

fig, ax = plt.subplots(figsize=(8.4, 3.5), dpi=220)
colors = ["#c0504d" if v < -6 else "#d9a441" if v > 6 else "#4f81bd" for v in e]
bars = ax.bar(POSES, e, color=colors, width=0.66, edgecolor="#333333",
              linewidth=0.5)
for x, v in zip(POSES, e):
    ax.annotate("%+.1f" % v, (x, v), textcoords="offset points",
                xytext=(0, 4 if v >= 0 else -13), ha="center", fontsize=11)

ax.axhline(0, color="#333333", lw=1.0)
ax.axhspan(-rms, rms, color="#4f81bd", alpha=0.10, zorder=0)

ax.set_xlabel("位姿序号", fontsize=14)
ax.set_ylabel("模型 − 实测（mm）", fontsize=14)
ax.set_xticks(POSES)
ax.tick_params(labelsize=13)
ax.set_ylim(-15, 10.5)
ax.grid(axis="y", alpha=0.25, lw=0.6)
ax.set_title("12 个位姿的法兰高度误差（浅蓝带 = ±RMS）\n"
             "平均 %+.1f mm　　RMS %.1f mm　　最大 |误差| %.1f mm"
             % (mean, rms, mx), fontsize=14.5, pad=10)
fig.tight_layout()
fig.savefig("media/report_alignment.png")
print("已保存 media/report_alignment.png  平均 %+.1f  RMS %.1f  最大 %.1f"
      % (mean, rms, mx))
print("精确残差：" + " ".join("%+.2f" % v for v in e))
