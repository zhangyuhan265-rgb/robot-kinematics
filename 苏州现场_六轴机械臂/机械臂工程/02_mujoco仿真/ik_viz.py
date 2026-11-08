# -*- coding: utf-8 -*-
"""IK 求解过程可视化：一张图讲清 LM 在干什么、多起点在干什么。

输出 media/ik_viz.png
"""
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

arm = UrdfArm("nbt_arm6.urdf")
NQ = arm.nq
TARGET = np.array([1.10, 0.0, 1.30])
Q0 = np.radians([0, 45, -35, 0, 25, 0])        # 故意选一个偏的起点


def arm_pts(q):
    frames, tip = arm._joint_frames(q)
    return np.array([p for p, _ in frames] + [tip])


def draw_arm(ax, q, **kw):
    P = arm_pts(q)
    ls = kw.pop("ls", "-")
    ax.plot(P[:, 0], P[:, 2], ls, marker="o", **kw)


def travel(q, q0):
    return float(np.degrees(np.abs(q - q0)).sum())


fig = plt.figure(figsize=(13.2, 8.2), dpi=120)
gs = fig.add_gridspec(2, 2, hspace=0.30, wspace=0.22)

# ---------- A：单起点 LM 的轨迹 ----------
axA = fig.add_subplot(gs[0, 0])
trace = []
q_sol, e_sol = arm._lm(Q0, arm._err_pos(TARGET), iters=150, trace=trace)
draw_arm(axA, Q0, color="#bbbbbb", ms=4, lw=2, ls="--", label="起点 q0")
draw_arm(axA, q_sol, color="#1f6fd0", ms=5, lw=2.4, label="解 q*")
fl = np.array([arm.tcp(t[0]) for t in trace])
axA.plot(fl[:, 0], fl[:, 2], "-", color="#e07b39", lw=1.2, alpha=0.9,
         label="法兰轨迹（%d 步）" % len(trace))
axA.plot(fl[0, 0], fl[0, 2], "o", color="#e07b39", ms=7)
axA.plot(TARGET[0], TARGET[2], "*", color="#d62728", ms=18, label="目标")
axA.set_title("A. 单起点 LM：末端怎么一步步挪过去\n"
              "残差 %.2f mm，关节行程 %.1f°" % (e_sol * 1000, travel(q_sol, Q0)),
              fontsize=10)
axA.set_xlabel("x (m)"); axA.set_ylabel("z (m)")
axA.legend(fontsize=8, loc="lower left")

# ---------- B：误差与阻尼 ----------
axB = fig.add_subplot(gs[0, 1])
it = np.arange(len(trace))
err = np.array([t[1] for t in trace]) * 1000
lam = np.array([t[2] for t in trace])
axB.semilogy(it, np.maximum(err, 1e-12), color="#1f6fd0", lw=1.6, label="|e| (mm)")
axB.set_xlabel("迭代步"); axB.set_ylabel("位置误差 (mm)", color="#1f6fd0")
axB.tick_params(axis="y", labelcolor="#1f6fd0")
ax2 = axB.twinx()
ax2.semilogy(it, lam, color="#2ca02c", lw=1.2, ls="--", label="λ")
ax2.set_ylabel("阻尼 λ", color="#2ca02c")
ax2.tick_params(axis="y", labelcolor="#2ca02c")
axB.set_title("B. 误差收敛 与 阻尼自适应\n"
              "被接受的步让 λ 降 0.3×，被拒绝的步让 λ 升 10×", fontsize=10)

# ---------- C：多起点的不同分支 ----------
axC = fig.add_subplot(gs[1, 0])
cands = [Q0] + arm._seed_lib()
runs = []
for k, s in enumerate(cands):
    q, e = arm._lm(s, arm._err_pos(TARGET), iters=150)
    runs.append((k, s, q, e, travel(q, Q0)))
ok = [r for r in runs if r[3] < 1e-4]
chosen = min(ok, key=lambda r: r[4]) if ok else None
trs = [r[4] for r in ok] or [0.0]
lo_t, hi_t = min(trs), max(trs)
cmap = plt.get_cmap("coolwarm")
for k, s, q, e, tr in ok:
    if chosen and k == chosen[0]:
        continue
    c = cmap((tr - lo_t) / max(hi_t - lo_t, 1e-9))
    kw = dict(color=c, ms=4, lw=2.4, alpha=0.95) if tr in (lo_t, hi_t) else \
        dict(color=c, ms=3, lw=1.4, alpha=0.8)
    draw_arm(axC, q, **kw)
draw_arm(axC, Q0, color="#888888", ms=4, lw=2, ls="--")
if chosen:
    draw_arm(axC, chosen[2], color="#d62728", ms=5, lw=2.8)
axC.plot(TARGET[0], TARGET[2], "*", color="#d62728", ms=18)
axC.set_title("C. 多起点：%d 个起点全部收敛，但落到了不同分支\n"
              "颜色＝关节行程（%.0f°→%.0f°），红＝选中的最近解 %.1f°"
              % (len(ok), lo_t, hi_t, chosen[4] if chosen else float("nan")),
              fontsize=10)
axC.set_xlabel("x (m)"); axC.set_ylabel("z (m)")

# ---------- D：关节行程条形图 ----------
axD = fig.add_subplot(gs[1, 1])
ks = [r[0] for r in ok]
tr = [r[4] for r in ok]
cols = ["#d62728" if (chosen and k == chosen[0]) else "#9fc5e8" for k in ks]
axD.barh(np.arange(len(ks)), tr, color=cols)
axD.set_yticks(np.arange(len(ks)))
axD.set_yticklabels(["起点 %d" % (k + 1) for k in ks], fontsize=9)
axD.invert_yaxis()
axD.set_xlabel("关节行程 Σ|Δq| (度)")
axD.set_title("D. 为什么必须「优先取最近的解」\n"
              "同样到达目标，行程差 %.1f 倍（%.1f° vs %.1f°）"
              % (max(tr) / max(min(tr), 1e-6), max(tr), min(tr)), fontsize=10)
for i, v in enumerate(tr):
    axD.text(v + 1, i, "%.1f°" % v, va="center", fontsize=8)

for ax in (axA, axC):
    ax.set_xlim(-0.35, 1.55)
    ax.set_ylim(-0.15, 2.25)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25, lw=0.6)
    ax.plot(0, 0, "s", ms=8, color="#333333")

fig.suptitle("逆解在做什么：阻尼最小二乘（LM）+ 多起点 + 取最近解　"
             "目标 (1.10, 0, 1.30) m", fontsize=12)
fig.text(0.5, 0.015,
         "注：C 图的目标离起点较远，分支差别不夸张；真机场景（目标离当前位姿 20 mm）"
         "暖启动 4.1°，随机起点 483.6°，差 120 倍。",
         ha="center", fontsize=8.5, color="#555555")
for _ax in fig.axes:
    for _t in ([_ax.title, _ax.xaxis.label, _ax.yaxis.label]
               + list(_ax.get_xticklabels()) + list(_ax.get_yticklabels())
               + list(_ax.texts)):
        _t.set_fontsize(_t.get_fontsize() * 1.55)
for _t in fig.texts:
    _t.set_fontsize(_t.get_fontsize() * 1.55)
fig.savefig("media/ik_viz.png", bbox_inches="tight")
print("已保存 media/ik_viz.png")
print("A 面板：起点行程用 %.1f°，收敛残差 %.2f mm，%d 步"
      % (travel(q_sol, Q0), e_sol * 1000, len(trace)))
print("C 面板：%d 个起点，%d 个收敛，行程范围 %.1f° ~ %.1f°"
      % (len(runs), len(ok), min(tr), max(tr)))
