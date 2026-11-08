# -*- coding: utf-8 -*-
"""机械臂碰撞检测（纯几何，不依赖物理引擎）。

为什么要自己做：
    训练环境里最怕"末端撞墙"，而 pybullet 之类的库这里没有。
    壁面打印的碰撞对象很简单：**一面竖直的墙** + **臂自身的杆件**。
    用「胶囊 vs 半空间」和「胶囊 vs 胶囊」就能算得很准，而且有解析解。

几何近似：
    <cylinder>  →  胶囊（轴段 + 半径），精确
    <box>       →  对【平面】用 8 个角点，精确；对【自碰撞】用最长轴胶囊，保守
    <sphere>    →  退化胶囊（p0 == p1）

单位：米。角度：弧度。

用法：
    python arm_collision.py           # 零位下打印各项间隙
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------- 基础数学
def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M


def _T(xyz, rpy_rad):
    roll, pitch, yaw = rpy_rad
    M = np.eye(4)
    M[:3, :3] = (_Rz(yaw) @ _Ry(pitch) @ _Rx(roll))[:3, :3]
    M[:3, 3] = xyz
    return M


def _axis_rot(axis, ang):
    x, y, z = axis
    n = np.linalg.norm([x, y, z])
    x, y, z = x / n, y / n, z / n
    c, s = np.cos(ang), np.sin(ang); C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M


def seg_seg_dist(p1, q1, p2, q2):
    """两条线段的最近距离（经典算法）"""
    d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2
    a = d1 @ d1; e = d2 @ d2; f = d2 @ r
    eps = 1e-12
    if a <= eps and e <= eps:
        return np.linalg.norm(r)
    if a <= eps:
        s = 0.0; t = np.clip(f / e, 0, 1)
    else:
        c = d1 @ r
        if e <= eps:
            t = 0.0; s = np.clip(-c / a, 0, 1)
        else:
            b = d1 @ d2; den = a * e - b * b
            s = np.clip((b * f - c * e) / den, 0, 1) if den > eps else 0.0
            t = (b * s + f) / e
            if t < 0:
                t = 0.0; s = np.clip(-c / a, 0, 1)
            elif t > 1:
                t = 1.0; s = np.clip((b - c) / a, 0, 1)
    return np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t))


# ---------------------------------------------------------------- 模型
class CollisionModel:
    """从 URDF 的 <collision> 建碰撞体；给 q 就返回世界系的胶囊 / 盒子角点。"""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.links, joints = {}, {}
        for l in root.findall("link"):
            prims = []
            for c in l.findall("collision"):
                o = c.find("origin")
                xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
                rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
                g = c.find("geometry")
                if g is None:
                    continue
                if g.find("cylinder") is not None:
                    cc = g.find("cylinder")
                    prims.append(("cyl", float(cc.get("radius")), float(cc.get("length")),
                                  _T(xyz, rpy)))
                elif g.find("box") is not None:
                    sz = [float(t) for t in g.find("box").get("size").split()]
                    prims.append(("box", sz, _T(xyz, rpy)))
                elif g.find("sphere") is not None:
                    prims.append(("sph", float(g.find("sphere").get("radius")), _T(xyz, rpy)))
                # 其余（mesh）先忽略
            self.links[l.get("name")] = prims
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
            rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(t) for t in (ax.get("xyz") if ax is not None else "0 0 1").split()]
            joints[j.get("name")] = dict(
                name=j.get("name"), type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=_T(xyz, rpy), axis=axis)
        childs = {v["child"] for v in joints.values()}
        self.root_link = [n for n in self.links if n not in childs][0]
        by_parent = {v["parent"]: v for v in joints.values()}
        chain, cur = [], self.root_link
        while cur in by_parent:
            j = by_parent[cur]; chain.append(j); cur = j["child"]
        self.chain = chain
        self.rev = [i for i, j in enumerate(chain) if j["type"] == "revolute"]
        self.nq = len(self.rev)
        self._adj = self._adjacency()

    def _adjacency(self):
        """link 之间的跳数（父子=1 跳）。自碰撞要排除 ≤2 跳的对 ——
        隔一个 link 的两个件在机械上本来就是连着的，查了必然误报。"""
        import collections
        g = collections.defaultdict(set)
        for j in self.chain:
            g[j["parent"]].add(j["child"])
            g[j["child"]].add(j["parent"])
        hops = {}
        for s in g:
            d = {s: 0}
            dq = collections.deque([s])
            while dq:
                u = dq.popleft()
                for v in g[u]:
                    if v not in d:
                        d[v] = d[u] + 1
                        dq.append(v)
            hops[s] = d
        return hops

    # ---- FK ----
    def fk(self, q):
        out = {self.root_link: np.eye(4)}
        M = np.eye(4); k = 0
        for j in self.chain:
            M = M @ j["T"]
            if j["type"] == "revolute":
                M = M @ _axis_rot(j["axis"], q[k]); k += 1
            out[j["child"]] = M.copy()
        return out

    # ---- 几何体 ----
    def capsules(self, q, include_boxes=True):
        """[(link, p0, p1, r)] 世界系胶囊

        include_boxes=False 时跳过 box —— 对平面检测要这么做，
        因为胶囊半径会把盒子的端面也撑大，间隙算出来偏小。
        """
        W = self.fk(q)
        out = []
        for ln, prims in self.links.items():
            T = W.get(ln)
            if T is None:
                continue
            for p in prims:
                if p[0] == "cyl":
                    r, L, To = p[1], p[2], p[3]
                    M = T @ To
                    a = M[:3, 3] + M[:3, 2] * (-L / 2)
                    b = M[:3, 3] + M[:3, 2] * (L / 2)
                    out.append((ln, a, b, r))
                elif p[0] == "sph":
                    r, To = p[1], p[2]
                    M = T @ To
                    out.append((ln, M[:3, 3], M[:3, 3], r))
                elif p[0] == "box" and include_boxes:
                    sz, To = p[1], p[2]
                    M = T @ To
                    half = np.array(sz) / 2
                    ax = int(np.argmax(half))
                    d = np.zeros(3); d[ax] = half[ax]
                    r = float(np.linalg.norm(np.delete(half, ax)))
                    out.append((ln, M[:3, 3] - M[:3, :3] @ d,
                                M[:3, 3] + M[:3, :3] @ d, r))
        return out

    def box_corners(self, q):
        """[(link, 8x3)] 世界系盒子角点（对平面是精确的）"""
        W = self.fk(q)
        out = []
        for ln, prims in self.links.items():
            T = W.get(ln)
            if T is None:
                continue
            for p in prims:
                if p[0] != "box":
                    continue
                sz, To = p[1], p[2]
                M = T @ To
                half = np.array(sz) / 2
                c = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half
                out.append((ln, (M[:3, :3] @ c.T).T + M[:3, 3]))
        return out

    # ---- 墙（半空间）：dist = n·p - d，正数表示在外侧 ----
    def distance_to_plane(self, q, n, d):
        """返回到墙面的最小间隙（负数=穿透）和最接近的 link

        圆柱/球用胶囊（对平面精确）；盒子用 8 个角点（对平面也精确）。
        **不能**把盒子也按胶囊算 —— 胶囊半径会把盒子端面撑大，间隙偏小。
        """
        n = np.asarray(n, float); n /= np.linalg.norm(n)
        best = (1e9, None)
        for ln, a, b, r in self.capsules(q, include_boxes=False):
            # 间隙 = 墙位置 − 机器人最靠墙的点
            dist = d - (max(n @ a, n @ b) + r)
            if dist < best[0]:
                best = (dist, ln)
        for ln, corners in self.box_corners(q):
            dist = d - float(np.max(corners @ n))
            if dist < best[0]:
                best = (dist, ln)
        return best

    # ---- 自碰撞（胶囊-胶囊，排除父子相邻）----
    def self_distance(self, q, ignore_base=True):
        caps = self.capsules(q)
        best = (1e9, None)
        skip = {"world", "riser", "base_link"} if ignore_base else set()
        for i in range(len(caps)):
            li, a1, b1, r1 = caps[i]
            if li in skip:
                continue
            for j in range(i + 1, len(caps)):
                lj, a2, b2, r2 = caps[j]
                if lj in skip or li == lj:
                    continue
                if frozenset((li, lj)) in self._adj:
                    continue
                if self._adj.get(li, {}).get(lj, 99) <= 2:
                    continue
                d = seg_seg_dist(a1, b1, a2, b2) - r1 - r2
                if d < best[0]:
                    best = (d, (li, lj))
        return best


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, "..", "models", "nbt_arm6", "urdf", "nbt_arm6_on_riser.urdf"),
             os.path.join(here, "nbt_arm6_on_riser.urdf")]
    urdf = next((os.path.abspath(p) for p in cands if os.path.exists(p)), None)
    if urdf is None:
        raise SystemExit("找不到 URDF")
    cm = CollisionModel(urdf)
    print("URDF: %s" % os.path.basename(urdf))
    print("关节 %d   collision 基元 %d 个"
          % (cm.nq, sum(len(v) for v in cm.links.values())))
    q = np.zeros(cm.nq)
    caps = cm.capsules(q)
    print("零位下胶囊 %d 个" % len(caps))
    for d in (1.4, 1.2, 1.0, 0.9, 0.8):
        dist, ln = cm.distance_to_plane(q, [1, 0, 0], d)
        print("  墙在 x=%0.1f m（法向 +X，臂朝 +X）: 间隙 %+7.1f mm  最近 %s"
              % (d, dist * 1000, ln))
    dist, pair = cm.self_distance(q)
    print("自碰撞最小间隙 %+7.1f mm  (%s)" % (dist * 1000, pair))


if __name__ == "__main__":
    main()
