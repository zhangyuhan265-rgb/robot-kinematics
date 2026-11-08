# -*- coding: utf-8 -*-
"""机械臂交互查看器 —— 拖滑块改关节角，实时看模型。

不用装 pybullet，用 matplotlib 自带的滑块。适合快速看几何、试位姿。

用法：
    python view_arm.py                      # 打开查看器
    python view_arm.py --wall 1.10          # 同时显示一面墙（x = 1.10 m）

操作：
    下面的 6 个滑块 = 6 个关节角（度）
    鼠标左键拖动旋转视角，滚轮缩放
    关掉窗口即退出
"""
import argparse
import os

import numpy as np
import matplotlib
for _bk in ("TkAgg", "QtAgg", "Qt5Agg"):
    try:
        matplotlib.use(_bk)
        break
    except Exception:
        continue
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from render_urdf import parse, fk, draw_box, draw_cyl, draw_sph, PALETTE


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="nbt_arm6_on_riser.urdf")
    ap.add_argument("--wall", type=float, default=None, help="墙面在 x = ? 米")
    args = ap.parse_args()

    links, chain, root = parse(args.urdf)
    init = [0.0] * 6

    fig = plt.figure(figsize=(13, 8.5))
    ax = fig.add_axes([0.02, 0.28, 0.72, 0.70], projection="3d")
    info = fig.text(0.77, 0.90, "", fontsize=10, family="monospace", va="top")

    sliders = []
    labels = ["J1", "J2", "J3", "J4", "J5", "J6"]
    for i in range(6):
        y = 0.20 - i * 0.033
        sax = fig.add_axes([0.13, y, 0.52, 0.022])
        lim = 360 if i == 5 else 130
        s = Slider(sax, labels[i], -lim, lim, valinit=init[i], valstep=0.5)
        s.on_changed(lambda _v: redraw())
        sliders.append(s)

    state = {"first": True}
    _CM = [None]

    def redraw():
        q = [s.val for s in sliders]
        elev0, azim0 = ax.elev, ax.azim
        ax.cla()
        W = fk(chain, root, q)

        for lname, vis in links.items():
            T = W.get(lname)
            if T is None:
                continue
            col = PALETTE.get(lname, "#888888")
            for kind, prm, To in vis:
                M = T @ To
                if kind == "box":
                    draw_box(ax, M, prm, col)
                elif kind == "cyl":
                    draw_cyl(ax, M, prm[0], prm[1], col, n=12)
                elif kind == "sph":
                    draw_sph(ax, M, prm[0], col, n=10)

        # 骨架线
        pts = []
        for j in chain:
            lname = j["child"]
            if lname in W:
                pts.append(W[lname][:3, 3])
        if pts:
            pts = np.array(pts)
            ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="#d94f4f",
                    ms=3, lw=1.2, alpha=0.85, zorder=10)

        # 墙
        if args.wall is not None:
            y = np.linspace(-1.0, 1.0, 2); z = np.linspace(0, 2.2, 2)
            Y, Z = np.meshgrid(y, z)
            X = np.full_like(Y, args.wall)
            ax.plot_surface(X, Y, Z, color="#7fb3d5", alpha=0.28, linewidth=0)

        p = W["tcp"][:3, 3] * 1000
        ax.set_xlim(-1.0, 1.6); ax.set_ylim(-1.0, 1.0); ax.set_zlim(0, 2.2)
        ax.set_box_aspect((1.2, 1.0, 1.1))
        ax.view_init(elev=elev0, azim=azim0)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title("nbt_arm6  -  drag sliders to move joints", fontsize=10)

        msg = (" q (deg) = [%s]\n\n TCP (mm) = (%.1f, %.1f, %.1f)\n\n  radius = %.1f mm\n  height = %.1f mm"
               % (", ".join("%7.2f" % v for v in q),
                  p[0], p[1], p[2], np.hypot(p[0], p[1]), p[2]))
        if args.wall is not None:
            from arm_collision import CollisionModel
            cm = _CM[0] if _CM[0] is not None else CollisionModel(args.urdf)
            _CM[0] = cm
            dist, ln = cm.distance_to_plane(np.radians(q), [1, 0, 0], args.wall)
            msg += "\n\n wall at x=%.2f m\n clearance = %+.1f mm\n nearest = %s" % (
                args.wall, dist * 1000, ln)
        info.set_text(msg)
        if state["first"]:
            state["first"] = False
        fig.canvas.draw_idle()

    redraw()
    print("窗口已打开。拖下面的滑块改关节角；关窗口退出。")
    plt.show()


if __name__ == "__main__":
    main()
