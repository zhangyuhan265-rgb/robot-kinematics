# -*- coding: utf-8 -*-
"""重排「逆解过程」图：原生画布按 A4 实际尺寸设置，贴进文档接近 1:1，字才看得清。"""
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
matplotlib.rcParams["font.size"] = 9.5

arm = UrdfArm("nbt_arm6.urdf")
NQ = arm.nq
TARGET = np.array([1.10, 0.0, 1.30])
Q0 = np.radians([0, 45, -35, 0, 25, 0])


def arm_pts(q):
    frames, tip = arm._joint_frames(q)
    return np.array([p for p, _ in frames] + [tip])


def draw_arm(ax, q, **kw):
    P = arm_pts(q)
    ls = kw.pop("ls", "-")
    ax.plot(P[:, 0], P[:, 2], ls, marker="o", **kw)


def travel(q, q0):
    return float(np.degrees(np.abs(q - q0)).sum())


fig = plt.figure(figsize=(7.8, 5.5), dpi=220)
gs = fig.add_gridspec(2, 2, hspace=0.52, wspace=0.34,
                      left=0.075, right=0.945, top=0.86, bottom=0.12)

# A
axA = fig.add_subplot(gs[0, 0])
trace = []
q_sol, e_sol = arm._lm(Q0, arm._err_pos(TARGET), iters=150, trace=trace)
draw_arm(axA, Q0, color="#bbbbbb", ms=3, lw=1.5, ls="--", label="起点")
draw_arm(axA, q_sol, color="#1f6fd0", ms=3.5, lw=1.9, label="解")
fl = np.array([arm.tcp(t[0]) for t in trace])
axA.plot(fl[:, 0], fl[:, 2], "-", color="#e07b39", lw=1.0, label="法兰轨迹")
axA.plot(fl[0, 0], fl[0, 2], "o", color="#e07b39", ms=4)
axA.plot(TARGET[0], TARGET[2], "*", color="#d62728", ms=11, label="目标")
axA.set_title("A. LM 迭代：末端怎么挪过去\n残差 %.2f mm，%d 步，行程 %.0f°"
              % (e_sol * 1000, len(trace), travel(q_sol, Q0)), fontsize=9.5)
axA.set_xlabel("x (m)"); axA.set_ylabel("z (m)")
axA.legend(fontsize=7, loc="lower left", framealpha=0.9)

# B
axB = fig.add_subplot(gs[0, 1])
it = np.arange(len(trace))
err = np.array([t[1] for t in trace]) * 1000
lam = np.array([t[2] for t in trace])
axB.semilogy(it, np.maximum(err, 1e-12), color="#1f6fd0", lw=1.6)
axB.set_xlabel("迭代步"); axB.set_ylabel("位置误差 (mm)", color="#1f6fd0", fontsize=8.5)
axB.tick_params(axis="y", labelcolor="#1f6fd0", labelsize=8)
axB.tick_params(axis="x", labelsize=8)
ax2 = axB.twinx()
ax2.semilogy(it, lam, color="#2ca02c", lw=1.1, ls="--")
ax2.set_ylabel("阻尼 λ", color="#2ca02c", fontsize=8.5)
ax2.tick_params(axis="y", labelcolor="#2ca02c", labelsize=8)
axB.set_title("B. 误差收敛与阻尼自适应\n接受步 λ×0.3，拒绝步 λ×10", fontsize=9.5)

# C
axC = fig.add_subplot(gs[1, 0])
cands = [Q0] + arm._seed_lib()
runs = []
for k, s in enumerate(cands):
    q, e = arm._lm(s, arm._err_pos(TARGET), iters=150)
    runs.append((k, q, e, travel(q, Q0)))
ok = [r for r in runs if r[2] < 1e-4]
chosen = min(ok, key=lambda r: r[3])
trs = [r[3] for r in ok]
lo_t, hi_t = min(trs), max(trs)
cmap = plt.get_cmap("coolwarm")
for k, q, e, tr in ok:
    if k == chosen[0]:
        continue
    draw_arm(axC, q, color=cmap((tr - lo_t) / max(hi_t - lo_t, 1e-9)),
             ms=2, lw=1.0, alpha=0.8)
draw_arm(axC, Q0, color="#888888", ms=3, lw=1.4, ls="--")
draw_arm(axC, chosen[1], color="#d62728", ms=3.5, lw=2.0)
axC.plot(TARGET[0], TARGET[2], "*", color="#d62728", ms=11)
axC.set_title("C. 多起点落到不同关节解\n颜色＝行程，红＝选中的最近解", fontsize=9.5)
axC.set_xlabel("x (m)"); axC.set_ylabel("z (m)")

# D
axD = fig.add_subplot(gs[1, 1])
ks = [r[0] for r in ok]
trv = [r[3] for r in ok]
cols = ["#d62728" if k == chosen[0] else "#9fc5e8" for k in ks]
axD.barh(np.arange(len(ks)), trv, color=cols, height=0.7)
axD.set_yticks(np.arange(len(ks)))
axD.set_yticklabels(["起点 %d" % (k + 1) for k in ks], fontsize=7.5)
axD.invert_yaxis()
axD.tick_params(axis="x", labelsize=8)
axD.set_xlabel("关节行程 Σ|Δq| (°)", fontsize=8.5)
axD.set_title("D. 同样到位，行程差 %.1f 倍\n%.0f° vs %.0f°"
              % (hi_t / lo_t, hi_t, lo_t), fontsize=9.5)
for i, v in enumerate(trv):
    axD.text(v + 2, i, "%.0f" % v, va="center", fontsize=7)

for ax in (axA, axC):
    ax.set_xlim(-0.35, 1.55)
    ax.set_ylim(-0.15, 2.25)
    ax.set_aspect("equal")
    ax.grid(alpha=0.22, lw=0.5)
    ax.tick_params(labelsize=8)
    ax.plot(0, 0, "s", ms=6, color="#333333")

fig.suptitle("逆解在做什么：阻尼最小二乘（LM）+ 多起点 + 取最近解　目标 (1.10, 0, 1.30) m",
             fontsize=10.5)
fig.savefig("media/ik_viz.png", bbox_inches="tight")
print("已保存 media/ik_viz.png")
