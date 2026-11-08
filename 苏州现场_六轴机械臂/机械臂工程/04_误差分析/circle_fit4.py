# -*- coding: utf-8 -*-
"""四个地面点的圆拟合。

输入：四个点两两之间的 6 条距离（mm）。
步骤：
  1) 经典 MDS 把 6 条距离铺成 2 维坐标（用最小二乘，不假设共圆）；
  2) 检查“多出来的那 1 个约束”有没有被满足（不可嵌入残差）；
  3) 对 4 个点做代数圆拟合，给出圆心、半径、每点残差；
  4) 把拟合出的角度和命令角度对比 —— 这一步能分出“挂点偏移”和“q1 有比例误差”。

只需要把下面的 DIST 填满。
"""
import itertools
import sys

import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

ORDER = ["M0", "M+30", "M-30", "Mnew"]
CMD_ANGLE = {"M0": 0.0, "M+30": 30.0, "M-30": -30.0, "Mnew": 41.308}

# ← 填这里（任意方向，脚本自己处理对称）
DIST = {
    ("M0", "M+30"): 530.0,
    ("M+30", "M-30"): 1050.0,
    ("M-30", "Mnew"): 1090.0,
    ("M0", "M-30"): 525.0,
    ("M0", "Mnew"): 635.0,
    ("M+30", "Mnew"): 135.0,
}


def build_matrix():
    n = len(ORDER)
    idx = {k: i for i, k in enumerate(ORDER)}
    D = np.zeros((n, n))
    missing = []
    for a, b in itertools.combinations(ORDER, 2):
        v = DIST.get((a, b), DIST.get((b, a)))
        if v is None:
            missing.append((a, b))
            continue
        D[idx[a], idx[b]] = D[idx[b], idx[a]] = v
    return D, missing, idx


def mds(D2, dim=2):
    n = D2.shape[0]
    J = np.eye(n) - np.ones((n, n)) / n
    B = -0.5 * J @ D2 @ J
    w, V = np.linalg.eigh(B)
    o = np.argsort(w)[::-1]
    w, V = w[o], V[:, o]
    pos = np.maximum(w[:dim], 0)
    X = V[:, :dim] * np.sqrt(pos)
    # 残差：被丢掉的维度占总能量的比例
    left = np.sum(np.maximum(w[dim:], 0)) / max(np.sum(np.abs(w)), 1e-12)
    return X, left


def fit_circle(P):
    x, y = P[:, 0], P[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    R = np.sqrt(c + cx ** 2 + cy ** 2)
    res = np.hypot(x - cx, y - cy) - R
    return np.array([cx, cy]), R, res


def main():
    D, missing, idx = build_matrix()
    if missing:
        print("还缺这些距离：")
        for a, b in missing:
            print("   %-6s ↔ %-6s" % (a, b))
        print()
        print("（先把 6 条填满再看结果）")
        return

    X, left = mds(D ** 2, 2)
    print("=== 1. 距离内部一致性 ===")
    print("丢掉第 3 维之后剩下的能量占比 = %.2f%%" % (left * 100))
    print("  （0% = 6 条距离严格可嵌入平面；>5% 说明有某条距离跟其它对不上）")
    Dfit = np.linalg.norm(X[:, None, :] - X[None, :, :], axis=2)
    err = Dfit - D
    np.fill_diagonal(err, 0)
    print("  逐条距离残差（展开后 − 实测）:")
    for a, b in itertools.combinations(ORDER, 2):
        i, j = idx[a], idx[b]
        print("    %-6s ↔ %-6s  实测 %7.1f   展开 %7.1f   差 %+6.1f"
              % (a, b, D[i, j], Dfit[i, j], err[i, j]))

    print()
    print("=== 2. 圆拟合 ===")
    C, R, res = fit_circle(X)
    print("半径 R = %.1f mm" % R)
    print("圆心在展开坐标里的位置 = (%.1f, %.1f)" % (C[0], C[1]))
    print()
    print("%-6s %10s %10s %10s %10s"
          % ("点", "到圆心", "残差mm", "拟合角°", "命令角°"))
    ang = np.degrees(np.arctan2(X[:, 1] - C[1], X[:, 0] - C[0]))
    for k, name in enumerate(ORDER):
        print("%-6s %10.1f %+10.1f %10.2f %10.2f"
              % (name, np.hypot(*(X[k] - C)), res[k], ang[k],
                 CMD_ANGLE[name]))
    print()
    print("残差 = 该点半径 − 拟合半径。正数表示这个点离圆心偏远。")
    print("如果某点残差明显大，就是那一次挂铅垂没挂在盘心上。")
    print()
    print("=== 3. q1 有没有比例误差 ===")
    order = np.argsort(ang)
    for a, b in zip(order[:-1], order[1:]):
        da = (ang[b] - ang[a] + 360) % 360
        dc = (CMD_ANGLE[ORDER[b]] - CMD_ANGLE[ORDER[a]]) % 360
        print("  %-6s → %-6s  拟合 %7.2f°   命令 %7.2f°   比 %.4f"
              % (ORDER[a], ORDER[b], da, dc, da / dc if dc else float("nan")))


if __name__ == "__main__":
    main()
