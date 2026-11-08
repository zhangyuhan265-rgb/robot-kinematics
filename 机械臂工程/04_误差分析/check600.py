# -*- coding: utf-8 -*-
"""600mm 垂直检验之后：用两端关节角算真实位移。"""
import numpy as np
import xml.etree.ElementTree as ET
from math import pi, radians, degrees, cos, sin

PATH = 'nbt_arm6_on_riser.urdf'
Q0 = [-8.596, -34.459, 46.501, -0.003, 11.997, 294.565]
Q1 = [-8.596, -35.229, -5.487, -0.004, -40.764, 294.568]

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

T0, T1 = fk(Q0), fk(Q1)
f0, f1 = T0['tcp'][:3, 3] * 1000, T1['tcp'][:3, 3] * 1000
w0, w1 = T0['link5'][:3, 3] * 1000, T1['link5'][:3, 3] * 1000
n0, n1 = T0['tcp'][:3, 2], T1['tcp'][:3, 2]

print("起点 法兰 (%.1f, %.1f, %.1f)  法线 (%.3f, %.3f, %.3f)" % (*f0, *n0))
print("终点 法兰 (%.1f, %.1f, %.1f)  法线 (%.3f, %.3f, %.3f)" % (*f1, *n1))
print()
print("腕心位移   dz = %+.1f mm   |d| = %.1f" % (w1[2] - w0[2], np.linalg.norm(w1 - w0)))
print("法兰位移   dz = %+.1f mm   |d| = %.1f" % (f1[2] - f0[2], np.linalg.norm(f1 - f0)))
print()
print("法线转过的角度 = %.1f°" % degrees(np.arccos(np.clip(np.dot(n0, n1), -1, 1))))
