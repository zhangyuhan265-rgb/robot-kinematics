# -*- coding: utf-8 -*-
"""把某个位姿"长什么样"用文字描述出来：大臂/小臂朝哪、法兰朝哪、整体转了多少度。
用来和真机肉眼对照。"""
import sys
import numpy as np
import xml.etree.ElementTree as ET
from math import pi, radians, degrees, sin, cos, atan2

PATH = 'nbt_arm6_on_riser.urdf'
Q_HOME = [-11.382, -7.617, 7.674, -34.306, -30.333, -68.221]
Q_DOWN = [-14.56, -13.49, 17.32, 0.00, 3.83, -104.56]
Q_ZERO = [0.0] * 6

tree = ET.parse(PATH)
root = tree.getroot()
links = [l.get('name') for l in root.findall('link')]
joints = {}
for j in root.findall('joint'):
    o = j.find('origin')
    xyz = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') if o is not None else '0 0 0').split()]
    ax = j.find('axis')
    axis = [float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]
    joints[j.get('name')] = dict(name=j.get('name'), type=j.get('type'),
                                 parent=j.find('parent').get('link'),
                                 child=j.find('child').get('link'),
                                 xyz=xyz, rpy=rpy, axis=axis)
children = {j['parent']: j for j in joints.values()}
roots = [l for l in links if l not in [j['child'] for j in joints.values()]]
chain, cur = [], roots[0]
while cur in children:
    j = children[cur]; chain.append((j['name'], j)); cur = j['child']
JN = ['joint%d' % i for i in range(1, 7)]

def Rz(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def axis_rot(axis, ang):
    x, y, z = axis; n = (x * x + y * y + z * z) ** 0.5; x, y, z = x / n, y / n, z / n
    c, s = cos(ang), sin(ang); C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M

def fk(q_deg):
    q = dict(zip(JN, [radians(float(v)) for v in q_deg]))
    M = np.eye(4); out = {}
    for name, j in chain:
        r, p, y = j['rpy']
        O = np.eye(4); O[:3, :3] = (Rz(y) @ Ry(p) @ Rx(r))[:3, :3]; O[:3, 3] = j['xyz']
        M = M @ O
        if j['type'] == 'revolute':
            M = M @ axis_rot(j['axis'], q[name])
        out[j['child']] = M.copy()
    return out

def ang_from_vec(v, ref):
    v = v / np.linalg.norm(v); ref = ref / np.linalg.norm(ref)
    return degrees(np.arccos(np.clip(np.dot(v, ref), -1, 1)))

def describe(tag, q):
    T = fk(q)
    P = {k: T[k][:3, 3] for k in ('link2', 'link3', 'link5', 'tcp')}
    up = P['link3'] - P['link2']       # 大臂方向
    fore = P['link5'] - P['link3']     # 小臂方向
    n = T['tcp'][:3, 2]                # 法兰法线
    print("=" * 68)
    print("%s   q = %s" % (tag, ["%.2f" % v for v in q]))
    print("=" * 68)
    print("  J2 (%.0f, %.0f, %.0f)  肘 (%.0f, %.0f, %.0f)  腕心 (%.0f, %.0f, %.0f)  法兰 (%.0f, %.0f, %.0f)"
          % (P['link2'][0] * 1000, P['link2'][1] * 1000, P['link2'][2] * 1000,
             P['link3'][0] * 1000, P['link3'][1] * 1000, P['link3'][2] * 1000,
             P['link5'][0] * 1000, P['link5'][1] * 1000, P['link5'][2] * 1000,
             P['tcp'][0] * 1000, P['tcp'][1] * 1000, P['tcp'][2] * 1000))
    print("  大臂：离竖直 %.1f 度，水平投影朝向方位角 %.1f 度"
          % (ang_from_vec(up, [0, 0, 1]), degrees(atan2(up[1], up[0]))))
    print("        指向%s%s" % ("上" if up[2] > 0 else "下",
                                "，几乎竖直" if ang_from_vec(up, [0, 0, 1]) < 20 else ""))
    print("  小臂：离水平 %.1f 度（%s），方位角 %.1f 度"
          % (abs(90 - ang_from_vec(fore, [0, 0, 1])),
             "上翘" if fore[2] > 0 else "下垂", degrees(atan2(fore[1], fore[0]))))
    print("  法兰法线 (%.3f, %.3f, %.3f)：离竖直 %.1f 度，%s"
          % (n[0], n[1], n[2], ang_from_vec(n, [0, 0, 1]),
             "竖直向下" if n[2] < -0.99 else ("竖直向上" if n[2] > 0.99 else "斜的")))
    print("  法兰中心 离地 %.1f mm，离 J1 轴 %.1f mm"
          % (P['tcp'][2] * 1000, np.hypot(P['tcp'][0], P['tcp'][1]) * 1000))
    print()

describe("A. fk_samples.csv 里的 home（我把它摆过去的那个）", Q_HOME)
describe("B. 法兰竖直朝下（19_摆正法兰.py 的 down）", Q_DOWN)
describe("C. 全零位", Q_ZERO)
