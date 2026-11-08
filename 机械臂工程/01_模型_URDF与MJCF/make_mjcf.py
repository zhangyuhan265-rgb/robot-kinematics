# -*- coding: utf-8 -*-
"""URDF → MJCF 转换器：给模型配上真正的材质、地面、墙和灯光。

为什么要转：
    MuJoCo 的 URDF 导入器**不认 <material>**，所有件都渲染成灰白色，
    也没有地面/墙/环境光。直接看就是"一堆灰管子"。
    转成 MJCF 才能控制颜色、纹理、灯光、天空盒。

用法：
    python make_mjcf.py                     # 生成 nbt_arm6.xml
    python make_mjcf.py --wall 1.10         # 加一面墙（x = 1.10 m）
"""
import argparse
import os
import xml.etree.ElementTree as ET

import numpy as np

FMT = lambda v: ("%.6f" % v).rstrip("0").rstrip(".")

# 每个 link 的颜色（rgba）
COLORS = {
    "riser":     "0.30 0.31 0.34 1",
    "base_link": "0.22 0.23 0.26 1",
    "link1":     "0.26 0.27 0.30 1",
    "link2":     "0.72 0.74 0.78 1",   # 大臂，浅色
    "link3":     "0.30 0.31 0.34 1",
    "link4":     "0.72 0.74 0.78 1",   # 小臂，浅色
    "link5":     "0.26 0.27 0.30 1",
    "link6":     "0.80 0.62 0.18 1",   # 法兰，金色
}

# 材质：给每个 link 一个带高光/反射的材质名
MATS = {
    "riser": "cast", "base_link": "cast", "link1": "cast", "link3": "cast",
    "link5": "cast", "link2": "paint", "link4": "paint", "link6": "gold",
}

# 细节件：在 link 自身坐标系里额外加的 geom，(type, param, pos, euler, rgba, mat)
#   flange 螺栓圈 + 关节缝圈 + 线缆管，让模型不像"一堆积木"
DETAILS = {
    "link6": [
        # 法兰盘本体（比 URDF 里那块"鸭嘴壳体"略大，保证从外面能看见）
        ("cylinder", (0.074, 0.020), (0, 0, 0.136), (0, 0, 0), "0.82 0.63 0.16 1", "gold"),
        # 6 颗螺栓，绕法兰面一圈。link6 坐标系里法兰面在 z≈0.131，半径 0.058
        *[("cylinder", (0.0050, 0.014),
           (0.048 * np.cos(np.radians(a)), 0.048 * np.sin(np.radians(a)), 0.148),
           (0, 0, 0), "0.40 0.42 0.46 1", "steel")
          for a in range(0, 360, 60)],
    ],
    "link4": [
        # 小臂上的线缆管（贴在侧面）
        ("cylinder", (0.010, 0.40), (0.050, 0.018, 0.32), (0, 0, 0), "0.10 0.10 0.11 1", "rubber"),
        # 两道加强箍
        ("cylinder", (0.066, 0.010), (0, 0, 0.22), (0, 0, 0), "0.60 0.62 0.66 1", "paint"),
        ("cylinder", (0.058, 0.010), (0, 0, 0.50), (0, 0, 0), "0.60 0.62 0.66 1", "paint"),
    ],
    "link5": [
        # 腕部缝圈
        ("cylinder", (0.078, 0.009), (0, 0, -0.060), (0, 0, 0), "0.20 0.21 0.23 1", "cast"),
    ],
}


def parse(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        geoms = []
        for v in l.findall("visual"):
            o = v.find("origin")
            xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
            rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
            g = v.find("geometry")
            if g is None:
                continue
            if g.find("box") is not None:
                geoms.append(("box", [float(t) for t in g.find("box").get("size").split()], xyz, rpy))
            elif g.find("cylinder") is not None:
                c = g.find("cylinder")
                geoms.append(("cylinder", (float(c.get("radius")), float(c.get("length"))), xyz, rpy))
            elif g.find("sphere") is not None:
                geoms.append(("sphere", float(g.find("sphere").get("radius")), xyz, rpy))
        links[l.get("name")] = geoms
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
        rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
        ax = j.find("axis")
        axis = [float(t) for t in (ax.get("xyz") if ax is not None else "0 0 1").split()]
        lim = j.find("limit")
        joints[j.get("name")] = dict(
            name=j.get("name"), type=j.get("type"),
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            xyz=xyz, rpy=rpy, axis=axis,
            lo=float(lim.get("lower")) if lim is not None and lim.get("lower") else -3.14,
            hi=float(lim.get("upper")) if lim is not None and lim.get("upper") else 3.14)
    childs = {v["child"] for v in joints.values()}
    root_link = [n for n in links if n not in childs][0]
    kids = {}
    for j in joints.values():
        kids.setdefault(j["parent"], []).append(j)
    order = []
    stack = [root_link]
    while stack:
        cur = stack.pop(0)
        for j in kids.get(cur, []):
            order.append(j)
            stack.append(j["child"])
    return links, joints, order, root_link


def rpy_to_quat(rpy):
    """URDF 的 rpy（固定轴 XYZ，R = Rz·Ry·Rx）→ MuJoCo 的 quat (w,x,y,z)

    为什么必须用四元数：
        MuJoCo 的 euler= 和 URDF 的 rpy= 旋转顺序约定不同。
        单轴旋转看不出来，但像 joint6 的 (90, 0, -90) 这种复合旋转会差 90°。
        四元数是唯一表示，不存在约定问题。
    """
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def quat_str(rpy):
    return " ".join(FMT(v) for v in rpy_to_quat(rpy))


def geom_xml(kind, prm, xyz, rpy, rgba, mat=None):
    if kind == "box":
        g = '<geom type="box" size="%s"' % " ".join(FMT(v / 2) for v in prm)
    elif kind == "cylinder":
        g = '<geom type="cylinder" size="%s %s"' % (FMT(prm[0]), FMT(prm[1] / 2))
    else:
        g = '<geom type="sphere" size="%s"' % FMT(prm)
    g += ' pos="%s" quat="%s"' % (" ".join(FMT(v) for v in xyz), quat_str(rpy))
    g += (' material="%s"' % mat) if mat else (' rgba="%s"' % rgba)
    g += ' contype="0" conaffinity="0"/>'
    return g


def detail_xml(d):
    kind, prm, pos, euler, rgba, mat = d
    return geom_xml(kind, prm, pos, euler, rgba, mat)


# 照照片重做的外形：被列进来的 link，**不再输出 URDF 原来的 visual**，改用这里的
# 尺寸按"比例"取自照片，绝对长度用实测值（a2=790 d4=750 a3=156 d6=132.5）
REPLACE = {
    # 小臂：真机是粗圆管（照片 03/04），不是三段细圆柱
    "link4": [
        ("cylinder", (0.078, 0.052), (0, 0, 0.026), (0, 0, 0), "0.74 0.76 0.80 1", "paint"),
        ("cylinder", (0.058, 0.640), (0, 0, 0.380), (0, 0, 0), "0.80 0.82 0.86 1", "paint"),
        ("cylinder", (0.066, 0.042), (0, 0, 0.716), (0, 0, 0), "0.70 0.72 0.76 1", "paint"),
        # 靠手腕处那块黑色盖板（照片 01/02 里很显眼）
        ("box", (0.052, 0.118, 0.150), (0.0, -0.082, 0.640), (0, 0, 0), "0.12 0.12 0.13 1", "rubber"),
        # 管身上的加强箍
        ("cylinder", (0.062, 0.012), (0, 0, 0.180), (0, 0, 0), "0.66 0.68 0.72 1", "steel"),
        ("cylinder", (0.062, 0.012), (0, 0, 0.520), (0, 0, 0), "0.66 0.68 0.72 1", "steel"),
    ],
    # 腕/法兰：去掉那个比法兰还宽的"鸭嘴盒子"，做成真正的法兰盘 + 螺栓圈
    "link6": [
        ("cylinder", (0.042, 0.100), (0, 0, 0.050), (0, 0, 0), "0.72 0.74 0.78 1", "paint"),
        ("cylinder", (0.062, 0.026), (0, 0, 0.113), (0, 0, 0), "0.82 0.63 0.16 1", "gold"),
        ("cylinder", (0.036, 0.012), (0, 0, 0.130), (0, 0, 0), "0.86 0.68 0.20 1", "gold"),
        *[("cylinder", (0.0050, 0.020),
           (0.047 * np.cos(np.radians(a)), 0.047 * np.sin(np.radians(a)), 0.120),
           (0, 0, 0), "0.42 0.44 0.48 1", "steel")
          for a in range(0, 360, 60)],
    ],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="nbt_arm6_on_riser.urdf")
    ap.add_argument("--out", default="nbt_arm6.xml")
    ap.add_argument("--wall", type=float, default=None)
    ap.add_argument("--floor", action="store_true", default=True)
    args = ap.parse_args()

    links, joints, order, root_link = parse(args.urdf)
    out = []
    w = out.append

    w('<mujoco model="nbt_arm6">')
    w('  <compiler angle="radian"/>')
    w('  <visual>')
    w('    <global offwidth="1280" offheight="960"/>')
    w('    <quality shadowsize="4096" offsamples="8"/>')
    w('    <headlight ambient="0.42 0.42 0.46" diffuse="0.45 0.45 0.45" specular="0.12 0.12 0.12"/>')
    w('    <map znear="0.02" zfar="60"/>')
    w('  </visual>')
    w('  <asset>')
    w('    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.28 0.34 0.45" rgb2="0.86 0.90 0.95" width="256" height="256"/>')
    w('    <texture name="grid" type="2d" builtin="checker" rgb1="0.72 0.74 0.77" rgb2="0.62 0.64 0.68" width="512" height="512"/>')
    w('    <material name="floor" texture="grid" texrepeat="8 8" reflectance="0.06"/>')
    w('    <texture name="walltex" type="2d" builtin="checker" rgb1="0.90 0.91 0.93" rgb2="0.84 0.86 0.89" width="256" height="256"/>')
    w('    <material name="wallmat" texture="walltex" texrepeat="6 6" reflectance="0.02"/>')
    # ---- 金属材质：高光 + 轻微反射，让件看起来有"金属感"而不是塑料 ----
    w('    <material name="cast"   rgba="0.26 0.27 0.30 1" specular="0.42" shininess="0.55" reflectance="0.05"/>')
    w('    <material name="paint"  rgba="0.73 0.75 0.79 1" specular="0.30" shininess="0.40" reflectance="0.03"/>')
    w('    <material name="gold"   rgba="0.82 0.63 0.16 1" specular="0.75" shininess="0.85" reflectance="0.10"/>')
    w('    <material name="steel"  rgba="0.40 0.42 0.46 1" specular="0.85" shininess="0.95" reflectance="0.12"/>')
    w('    <material name="rubber" rgba="0.10 0.10 0.11 1" specular="0.08" shininess="0.15" reflectance="0.01"/>')
    w('  </asset>')
    w('  <worldbody>')
    w('    <light pos="1.6 -1.6 3.2" dir="-0.6 0.6 -1" directional="true" diffuse="0.55 0.55 0.55"/>')
    w('    <light pos="-1.6 1.4 2.6" dir="0.6 -0.5 -1" directional="true" diffuse="0.32 0.33 0.36"/>')
    if args.floor:
        w('    <geom name="floor" type="plane" size="8 8 0.1" material="floor" pos="0 0 0"/>')
    if args.wall is not None:
        w('    <geom name="wall" type="box" size="0.02 3.0 1.6" pos="%s 0 1.6" material="wallmat"/>'
          % FMT(args.wall))

    # 从根 link 往下递归，父 link 到子 link 之间如果有 joint 就用 body+joint
    kids = {}
    for j in order:
        kids.setdefault(j["parent"], []).append(j)

    def emit_children(link, depth):
        pad = "  " * depth
        for j in kids.get(link, []):
            jt = ""
            if j["type"] == "revolute":
                jt = '<joint name="%s" type="hinge" axis="%s" pos="0 0 0" range="%s %s" damping="2"/>' % (
                    j["name"], " ".join(FMT(v) for v in j["axis"]), FMT(j["lo"]), FMT(j["hi"]))
            w('%s<body name="%s" pos="%s" quat="%s">' % (
                pad, j["child"],
                " ".join(FMT(v) for v in j["xyz"]), quat_str(j["rpy"])))
            if jt:
                w(pad + "  " + jt)
            if j["child"] in REPLACE:
                for det in REPLACE[j["child"]]:
                    w("%s  %s" % (pad, detail_xml(det)))
            else:
                for kind, prm, xyz, rpy in links.get(j["child"], []):
                    w("%s  %s" % (pad, geom_xml(kind, prm, xyz, rpy,
                                                COLORS.get(j["child"], "0.6 0.6 0.6 1"),
                                                MATS.get(j["child"]))))
            for det in DETAILS.get(j["child"], []):
                w("%s  %s" % (pad, detail_xml(det)))
            emit_children(j["child"], depth + 1)
            w('%s</body>' % pad)

    if root_link == "world":
        # world 是 MuJoCo 保留名，不能建同名 body —— 直接把它的子节点挂到 worldbody 下
        emit_children("world", 2)
    else:
        w('  <body name="%s" pos="0 0 0">' % root_link)
        for kind, prm, xyz, rpy in links.get(root_link, []):
            w("    " + geom_xml(kind, prm, xyz, rpy, COLORS.get(root_link, "0.6 0.6 0.6 1")))
        emit_children(root_link, 2)
        w('  </body>')
    w('  </worldbody>')
    w('</mujoco>')

    txt = "\n".join(out) + "\n"
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(txt)
    print("已写 %s（%d 行，%d 个 geom）" % (args.out, len(out), txt.count("<geom")))


if __name__ == "__main__":
    main()
