# -*- coding: utf-8 -*-
"""底座倾斜对两组数据的影响。

倾斜模型：整个臂绕地面上的原点转 φ（绕 x 轴 = 侧向倒；绕 y 轴 = 前后倒）。
某点（臂座坐标 p）的地面坐标 = (p_x, p_y, p_z + riser)，再经 R(φ) 旋转。

竖直高度 h(φ) = p_y·sinφx + (p_z+riser)·cosφx   （绕 x）
           或 = −p_x·sinφy + (p_z+riser)·cosφy   （绕 y）
水平位移 Δy ≈ −(离地高度)·φx                      （绕 x）
"""
import sys

import numpy as np

from arm_env import UrdfArm

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

arm = UrdfArm("nbt_arm6.urdf")
TIP, NQ = arm.tip, arm.nq
RISER = 0.3365
fk_T = lambda q: arm.fk(q)[TIP]

DATA = [
    ([0, 0, 0, 0, 0, 0], 1581),
    ([0, -20, 10, 0, 0, 0], 1402),
    ([0, -30, 30, 0, 0, 0], 1474),
    ([0, 15, -15, 0, 0, 0], 1551),
    ([0, 0, 40, 0, 0, 0], 2068),
    ([0, -10, 20, 0, -45, 0], 1755),
    ([30, -15, 15, 0, 0, 0], 1549),
    ([0, -30, 50, 0, -30, 0], 1773),
    ([0, -20, 20, 45, 0, 0], 1569),
    ([0, -20, 20, 90, 0, 0], 1660),
    ([0, -20, 20, -90, 0, 0], 1666),
    ([20, -25, 35, 60, -30, 45], 1720),
]

P = []
for q, meas in DATA:
    p = fk_T(np.radians(q))[:3, 3].copy()
    P.append((p[0], p[1], p[2] + RISER, meas))
P = np.array(P)


def residuals(px, py):
    cx, sx = np.cos(px), np.sin(px)
    cy, sy = np.cos(py), np.sin(py)
    out = []
    for x, y, z, meas in P:
        # 先绕 x 再绕 y（小角度下顺序无所谓）
        y1 = y * cx - z * sx
        z1 = y * sx + z * cx
        x1 = x * cy + z1 * sy
        z2 = -x * sy + z1 * cy
        out.append(z2 * 1000 - meas)
    return np.array(out)


print("=" * 68)
print("1. 零倾斜基准")
print("=" * 68)
base = residuals(0, 0)
print("  平均 %+.1f mm   RMS %.1f mm   最大|e| %.1f mm"
      % (base.mean(), np.sqrt((base ** 2).mean()), np.abs(base).max()))

print()
print("=" * 68)
print("2. 绕 x 轴倒 0.5°（侧向倒 —— 臂的零位平面本来是 y=0）")
print("=" * 68)
r = residuals(np.radians(0.5), 0)
print("  平均 %+.1f mm   RMS %.1f mm   最大|e| %.1f mm"
      % (r.mean(), np.sqrt((r ** 2).mean()), np.abs(r).max()))
print("  逐位姿: %s" % " ".join("%+5.0f" % e for e in r))
print("  Δ RMS = %+.2f mm" % (np.sqrt((r ** 2).mean()) - np.sqrt((base ** 2).mean())))

print()
print("=" * 68)
print("3. 绕 y 轴倒 0.5°（前后倒 —— 臂的伸出方向）")
print("=" * 68)
r = residuals(0, np.radians(0.5))
print("  平均 %+.1f mm   RMS %.1f mm   最大|e| %.1f mm"
      % (r.mean(), np.sqrt((r ** 2).mean()), np.abs(r).max()))
print("  逐位姿: %s" % " ".join("%+5.0f" % e for e in r))

print()
print("=" * 68)
print("4. 网格搜索最佳倾斜角（只凭 12 个高度）")
print("=" * 68)
best = None
for px in np.radians(np.arange(-1.0, 1.01, 0.05)):
    for py in np.radians(np.arange(-1.0, 1.01, 0.05)):
        r = residuals(px, py)
        c = np.sqrt((r ** 2).mean())
        if best is None or c < best[0]:
            best = (c, px, py)
print("  最优：绕x %+.2f°   绕y %+.2f°   RMS %.2f mm（基准 4.2）"
      % (np.degrees(best[1]), np.degrees(best[2]), best[0]))

print()
print("  固定绕y=0，只扫绕x（看最小值有多确定）：")
for deg in (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0):
    r = residuals(np.radians(deg), 0)
    print("    绕x %+5.2f°  RMS %5.2f mm   平均 %+5.1f" %
          (deg, np.sqrt((r ** 2).mean()), r.mean()))

print()
print("=" * 68)
print("5. 倾斜对水平位置的影响")
print("=" * 68)
p0 = np.array([0.920, 0.0, 1.581])
print("  零位法兰，地面坐标 (%.3f, %.3f, %.3f) m" % tuple(p0))
for deg in (0.2, 0.5, 1.0):
    a = np.radians(deg)
    print("  绕x %+.1f° → Δy = %+6.2f mm（法兰被侧向推走）"
          % (deg, -(p0[2] * np.sin(a)) * 1000))
for deg in (0.2, 0.5, 1.0):
    a = np.radians(deg)
    print("  绕y %+.1f° → Δx = %+6.2f mm、Δz = %+6.2f mm"
          % (deg, (p0[2] * np.sin(a)) * 1000,
             (-p0[0] * np.sin(a)) * 1000))
