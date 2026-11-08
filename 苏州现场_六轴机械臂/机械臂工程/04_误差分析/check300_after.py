# -*- coding: utf-8 -*-
"""300mm 检验之后：用两端关节角 + 校正后的 URDF 算真实位移。"""
import numpy as np
import xml.etree.ElementTree as ET
from math import pi, radians, degrees, cos, sin

PATH = 'nbt_arm6_on_riser.urdf'
Q0 = [-11.381, -7.617, 7.668, -0.000, 0.001, 291.779]
Q1 = [-8.599, -34.459, 46.506, -0.003, 11.998, 294.566]
CTRL0 = (906.512, -182.468, 1238.107)
CTRL1 = (1206.514, -182.460, 1238.109)

tree = ET.parse(PATH); root = tree.getroot()
links = [l.get('name') for l in root.findall('link')]
J = {}
for j in root.findall('joint'):
    o = j.find('origin')
    xyz = [float(v) for v in (o.get('xyz') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') if o is not None else '0 0 0').split()]
    ax = j.find('axis')
    axis = [float(v) for v in (ax.get('xyz') if ax is not None else '0 0 1').split()]
    J[j.get('name')] = dict(name=j.get('name'), type=j.get('type'),
                            parent=j.find('parent').get('link'),
                            child=j.find('child').get('link'), xyz=xyz, rpy=rpy, axis=axis)
ch = {j['parent']: j for j in J.values()}
roots = [l for l in links if l not in [j['child'] for j in J.values()]]
chain, cur = [], roots[0]
while cur in ch:
    j = ch[cur]; chain.append((j['name'], j)); cur = j['child']
JN = ['joint%d' % i for i in range(1, 7)]

def Rz(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M

def Rx(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M

def Ry(a):
    c, s = cos(a), sin(a); M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M

def ar(ax, an):
    x, y, z = ax; n = (x * x + y * y + z * z) ** 0.5; x, y, z = x / n, y / n, z / n
    c, s = cos(an), sin(an); C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M

def fk(qd):
    q = dict(zip(JN, [radians(float(v)) for v in qd]))
    M = np.eye(4); out = {}
    for nm, j in chain:
        r, p, y = j['rpy']
        O = np.eye(4); O[:3, :3] = (Rz(y) @ Ry(p) @ Rx(r))[:3, :3]; O[:3, 3] = j['xyz']
        M = M @ O
        if j['type'] == 'revolute':
            M = M @ ar(j['axis'], q[nm])
        out[j['child']] = M.copy()
    return out

for tag, Q in (("起点", Q0), ("终点", Q1)):
    T = fk(Q)
    f = T['tcp'][:3, 3] * 1000
    w = T['link5'][:3, 3] * 1000
    n = T['tcp'][:3, 2]
    print("%s  法兰 (%.1f, %.1f, %.1f)  腕心 (%.1f, %.1f, %.1f)  法线 (%.3f, %.3f, %.3f)"
          % (tag, f[0], f[1], f[2], w[0], w[1], w[2], n[0], n[1], n[2]))

f0 = fk(Q0)['tcp'][:3, 3] * 1000
f1 = fk(Q1)['tcp'][:3, 3] * 1000
d = f1 - f0
print()
print("=" * 70)
print("真实法兰位移（校正模型）  = (%.1f, %.1f, %.1f)   |d| = %.1f mm"
      % (d[0], d[1], d[2], np.linalg.norm(d)))
print("控制器自报位移            = (%.1f, %.1f, %.1f)   |d| = %.1f mm"
      % (CTRL1[0] - CTRL0[0], CTRL1[1] - CTRL0[1], CTRL1[2] - CTRL0[2],
         np.linalg.norm(np.array(CTRL1) - np.array(CTRL0))))
w0 = fk(Q0)['link5'][:3, 3] * 1000
w1 = fk(Q1)['link5'][:3, 3] * 1000
print("腕心位移                  = (%.1f, %.1f, %.1f)   |d| = %.1f mm"
      % (w1[0] - w0[0], w1[1] - w0[1], w1[2] - w0[2], np.linalg.norm(w1 - w0)))
print()
print("注意：这次移动过程中 q5 从 0 变到 12°，所以法兰朝向也变了 12°。")
print("      J5 零位还差 90° 时，Cartesian 直线在真机上不会是直线。")
