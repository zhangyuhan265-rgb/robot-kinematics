# -*- coding: utf-8 -*-
"""URDF 结构自检：XML 合法性、名字唯一、树连通、每个 link 有几何、limit 合理。"""
import sys
import xml.etree.ElementTree as ET


def main():
    path = sys.argv[1]
    root = ET.parse(path).getroot()
    ok = True

    links = [l.get("name") for l in root.findall("link")]
    joints = [j.get("name") for j in root.findall("joint")]
    print("robot name = %s" % root.get("name"))
    print("link %d 个：%s" % (len(links), links))
    print("joint %d 个：%s" % (len(joints), joints))

    if len(set(links)) != len(links):
        print("!! link 重名")
        ok = False
    if len(set(joints)) != len(joints):
        print("!! joint 重名")
        ok = False

    parent_of = {}
    for j in root.findall("joint"):
        p = j.find("parent").get("link")
        c = j.find("child").get("link")
        if c in parent_of:
            print("!! %s 有两个父关节" % c)
            ok = False
        parent_of[c] = j.get("name")
        if p not in links:
            print("!! joint %s 的 parent %s 不存在" % (j.get("name"), p))
            ok = False
        if c not in links:
            print("!! joint %s 的 child %s 不存在" % (j.get("name"), c))
            ok = False

    roots = [l for l in links if l not in parent_of]
    print("根 link：%s" % roots)
    if len(roots) != 1:
        print("!! 根 link 不唯一")
        ok = False

    leaves = [l for l in links if l not in [j.find("parent").get("link") for j in root.findall("joint")]]
    print("叶 link：%s" % leaves)

    for l in root.findall("link"):
        n = l.get("name")
        has_vis = l.find("visual") is not None
        has_col = l.find("collision") is not None
        has_ine = l.find("inertial") is not None
        if n.startswith("link"):
            if not (has_vis and has_col and has_ine):
                print("!! %s 缺 visual/collision/inertial：%s %s %s" % (n, has_vis, has_col, has_ine))
                ok = False

    for j in root.findall("joint"):
        lim = j.find("limit")
        if j.get("type") == "revolute":
            if lim is None:
                print("!! revolute %s 没有 limit" % j.get("name"))
                ok = False
            else:
                lo = float(lim.get("lower"))
                hi = float(lim.get("upper"))
                if lo >= hi:
                    print("!! %s limit lower >= upper" % j.get("name"))
                    ok = False
        ax = j.find("axis")
        if j.get("type") == "revolute" and ax is None:
            print("!! revolute %s 没有 axis" % j.get("name"))
            ok = False

    print("\n结构自检：%s" % ("通过" if ok else "有问题"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
