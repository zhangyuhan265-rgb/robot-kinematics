# -*- coding: utf-8 -*-
"""用 matplotlib 把 URDF 的视觉几何画出来，检查模型有没有穿模 / 形状对不对。

不依赖 pybullet，只吃 URDF 里的 visual 基元（box / cylinder / sphere）。

用法：
    python render_urdf.py                      # 全零位姿
    python render_urdf.py --q 0,0,0,0,-90,0    # 指定关节角（度）
    python render_urdf.py --out media/xxx.png
"""
import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np
import matplotlib
if __name__ == "__main__":
    matplotlib.use("Agg")          # 只在"出 PNG"时用非交互后端；
                                   # 被 view_arm.py 导入时不动后端，否则窗口开不出来
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from math import pi, cos, sin, radians

DEFAULT_URDF = "nbt_arm6_on_riser.urdf"

PALETTE = {
    "base_link": "#3b3f46", "link1": "#4a5058", "link2": "#8a9199",
    "link3": "#565d66", "link4": "#9aa2aa", "link5": "#4a5058",
    "link6": "#c9a227", "tcp": "#d94f4f", "riser": "#6b7280",
}


def Rz(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M


def Rx(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M


def Ry(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M


def make_T(xyz, rpy):
    roll, pitch, yaw = rpy
    M = np.eye(4)
    M[:3, :3] = (Rz(yaw) @ Ry(pitch) @ Rx(roll))[:3, :3]
    M[:3, 3] = xyz
    return M


def rot_axis(axis, ang):
    x, y, z = axis
    n = (x * x + y * y + z * z) ** 0.5
    x, y, z = x / n, y / n, z / n
    c, s = cos(ang), sin(ang); C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M


# ---------------- 解析 ----------------
def parse(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        name = l.get("name")
        vis = []
        for v in l.findall("visual"):
            o = v.find("origin")
            xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
            rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
            g = v.find("geometry")
            if g is None:
                continue
            if g.find("box") is not None:
                size = [float(t) for t in g.find("box").get("size").split()]
                vis.append(("box", size, make_T(xyz, rpy)))
            elif g.find("cylinder") is not None:
                c = g.find("cylinder")
                vis.append(("cyl", (float(c.get("radius")), float(c.get("length"))),
                            make_T(xyz, rpy)))
            elif g.find("sphere") is not None:
                s = float(g.find("sphere").get("radius"))
                vis.append(("sph", (s,), make_T(xyz, rpy)))
        links[name] = vis
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
        rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(t) for t in (ax.get("xyz") if ax is not None else "0 0 1").split()]
        joints[j.get("name")] = dict(
            name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            T=make_T(xyz, rpy), axis=axis)
    childs = {v["child"] for v in joints.values()}
    root_link = [n for n in links if n not in childs][0]
    by_parent = {v["parent"]: v for v in joints.values()}
    chain = []
    cur = root_link
    while cur in by_parent:
        j = by_parent[cur]
        chain.append(j)
        cur = j["child"]
    return links, chain, root_link


def radians_list(rpy_deg):
    return [radians(v) for v in rpy_deg]


# ---------------- FK ----------------
def fk(chain, root_link, q_deg):
    """返回 {link: 4x4 世界变换}"""
    out = {root_link: np.eye(4)}
    M = np.eye(4)
    idx = 0
    for j in chain:
        M = M @ j["T"]
        if j["type"] == "revolute":
            ax = j["axis"]
            M = M @ rot_axis(ax, np.radians(q_deg[idx] if idx < len(q_deg) else 0.0))
            idx += 1
        out[j["child"]] = M.copy()
    return out


# ---------------- 画基元 ----------------
def draw_box(ax, T, size, color, alpha=0.85):
    sx, sy, sz = size
    c = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * 0.5
    c = c * np.array([sx, sy, sz])
    faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
             [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
    pts = (T[:3, :3] @ c.T).T + T[:3, 3]
    polys = [[pts[i] for i in f] for f in faces]
    ax.add_collection3d(Poly3DCollection(polys, facecolor=color, edgecolor="none", alpha=alpha))


def draw_cyl(ax, T, r, L, color, alpha=0.85, n=16):
    z = np.linspace(-L / 2, L / 2, 2)
    th = np.linspace(0, 2 * pi, n)
    TH, ZZ = np.meshgrid(th, z)
    X = r * np.cos(TH); Y = r * np.sin(TH)
    P = np.stack([X.ravel(), Y.ravel(), ZZ.ravel()])
    P = (T[:3, :3] @ P).T + T[:3, 3]
    X, Y, Z = P[:, 0].reshape(TH.shape), P[:, 1].reshape(TH.shape), P[:, 2].reshape(TH.shape)
    ax.plot_surface(X, Y, Z, color=color, alpha=alpha, linewidth=0, shade=True)


def draw_sph(ax, T, r, color, alpha=0.85, n=12):
    u = np.linspace(0, 2 * pi, n); v = np.linspace(0, pi, n)
    X, Y, Z = r * np.outer(np.cos(u), np.sin(v)), r * np.outer(np.sin(u), np.sin(v)), r * np.outer(np.ones_like(u), np.cos(v))
    P = np.stack([X.ravel(), Y.ravel(), Z.ravel()])
    P = (T[:3, :3] @ P).T + T[:3, 3]
    ax.plot_surface(P[:, 0].reshape(X.shape), P[:, 1].reshape(X.shape), P[:, 2].reshape(X.shape),
                    color=color, alpha=alpha, linewidth=0, shade=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default=DEFAULT_URDF)
    ap.add_argument("--q", default="0,0,0,0,0,0")
    ap.add_argument("--out", default="media/urdf_zero_pose.png")
    ap.add_argument("--title", default="")
    args = ap.parse_args()

    q = [float(v) for v in args.q.split(",")]
    links, chain, root_link = parse(args.urdf)
    W = fk(chain, root_link, q)

    fig = plt.figure(figsize=(16, 6))
    views = [(-65, 18, "iso"), (0, 0, "front +X"), (90, 0, "side +Y")]
    for idx, (azim, elev, tag) in enumerate(views, 1):
        ax = fig.add_subplot(1, 3, idx, projection="3d")
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
                    draw_cyl(ax, M, prm[0], prm[1], col)
                elif kind == "sph":
                    draw_sph(ax, M, prm[0], col)
        # 骨架
        pts = []
        for lname in W:
            pts.append(W[lname][:3, 3])
        pts = np.array(pts)
        ax.set_xlim(-0.9, 0.9); ax.set_ylim(-0.9, 0.9); ax.set_zlim(0, 2.0)
        ax.set_box_aspect((1, 1, 1.2))
        ax.view_init(elev=elev, azim=azim)
        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title("%s  q=%s" % (tag, args.q), fontsize=9)
        ax.grid(True, alpha=0.3)

    fig.suptitle(args.title or os.path.basename(args.urdf), fontsize=12)
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=110, bbox_inches="tight", facecolor="white")
    print("已保存 %s" % args.out)


if __name__ == "__main__":
    main()
