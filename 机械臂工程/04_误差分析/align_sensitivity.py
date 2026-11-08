# -*- coding: utf-8 -*-
"""对齐度分析：现有 12 组样本对各个几何参数到底有多大约束力？

思路：把某个参数故意改 1 mm / 1 度，看 TCP 在
  (a) 现有 12 组样本上   (b) 远端姿态上
分别偏多少。
若 (a) 小于我们的残差 0.003 mm → 说明这 1 mm 的误差在 home 附近**根本看不见**，
现有的 0.0027 mm 不能证明这个参数对。

再据此挑出信息量最大的测试姿态（D-最优贪心）。
"""
import csv
import math

import numpy as np

CSV = "/mnt/c/Users/yohan/dsh-workspace/arm_urdf/fk_samples.csv"
ALPHA = np.radians([90.0, 0.0, 90.0, 90.0, 90.0, 0.0])
PN = ["d1", "a1", "a2", "a3", "d4", "d6", "off1", "off2", "off3", "off4", "off5", "off6"]
P0 = np.array([377.0, 216.0, 900.0, 130.0, 934.0, 154.0,
               0.0, -90.0, 0.0, 0.0, 90.0, 180.0])


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], float)


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def fk_pos(q_deg, p):
    d1, a1, a2, a3, d4, d6 = p[:6]
    ds = [d1, 0.0, 0.0, d4, 0.0, d6]
    as_ = [a1, a2, a3, 0.0, 0.0, 0.0]
    T = np.eye(4)
    for i in range(6):
        th = math.radians(q_deg[i] - p[6 + i])
        Ti = np.eye(4)
        Ti[:3, :3] = (Rz(th) @ Rx(ALPHA[i]))[:3, :3]
        Ti[:3, 3] = (Rz(th)[:3, :3] @ np.array([as_[i], 0.0, ds[i]]))
        T = T @ Ti
    return T[:3, 3]


def load_q():
    qs = []
    for r in csv.DictReader(open(CSV, encoding="utf-8")):
        qs.append([float(r["q%d" % (i + 1)]) for i in range(6)])
    return np.array(qs)


def sens(qs, p, step):
    """返回 12 参数 × 每组姿态 的 |ΔTCP| 最大分量（即偏差向量长度）"""
    base = np.array([fk_pos(q, p) for q in qs])
    out = np.zeros((12, len(qs)))
    for k in range(12):
        pp = p.copy()
        pp[k] += step[k]
        d = np.array([fk_pos(q, pp) for q in qs]) - base
        out[k] = np.linalg.norm(d, axis=1)
    return out


def main():
    qs = load_q()
    step = np.array([1.0] * 6 + [1.0] * 6)      # 长度 1mm / 角度 1度
    S = sens(qs, P0, step)

    print("=== 单参数改 1 单位（长度 1 mm，偏置 1 度）时，TCP 偏差 ===")
    print("%-6s %14s %14s %s" % ("参数", "现有样本最大mm", "改1单位够不够看出来", ""))
    for k in range(12):
        m = S[k].max()
        flag = "**看不见**" if m < 0.05 else ("弱" if m < 0.5 else "能看见")
        print("%-6s %14.4f   %s" % (PN[k], m, flag))

    # ---- 远端姿态：把每个关节分别推离 home ----
    home = qs[0]
    print("\n=== 远端姿态（单关节推到 home ± 90 度，其它不动）===")
    print("%-6s %12s %12s %8s" % ("参数", "home±15内最大", "推90度后最大", "放大倍数"))
    far = []
    for i in range(6):
        for sgn in (-1, 1):
            q = home.copy()
            q[i] += sgn * 90.0
            if max(abs(q[i]), 0) > 130 and i < 5:
                continue
            far.append(q)
    far = np.array(far)
    Sf = sens(far, P0, step)
    for k in range(12):
        a = S[k].max()
        b = Sf[k].max()
        print("%-6s %12.4f %12.4f %8.1f×" % (PN[k], a, b, (b / a) if a > 1e-9 else float("inf")))

    # ---- 用 D-最优贪心挑测试姿态 ----
    print("\n=== D-最优贪心挑测试姿态 ===")
    rng = np.random.default_rng(0)
    cand = []
    for _ in range(4000):
        q = home + rng.uniform(-1, 1, 6) * np.array([100, 70, 70, 110, 90, 150])
        q = np.clip(q, -125, 125)
        # 避开腕奇异（θ5 = q5 - 90 ≈ 0 或 180）
        th5 = (q[4] - 90.0) % 360.0
        if min(abs(th5), abs(th5 - 180.0), abs(th5 - 360.0)) < 25.0:
            continue
        cand.append(q)
    cand = np.array(cand)
    print("候选姿态 %d 个（已排除腕奇异 ±25 度）" % len(cand))

    # 数值雅可比
    def jac(q):
        J = np.zeros((3, 12))
        for k in range(12):
            pp = P0.copy()
            h = 0.01 if k < 6 else 0.01
            pp[k] += h
            J[:, k] = (fk_pos(q, pp) - fk_pos(q, P0)) / h
        return J

    Js = np.array([jac(q) for q in cand])          # (N,3,12)
    Jf = Js.reshape(len(cand), -1)                 # (N,36)
    chosen = []
    sel = np.zeros((0, 12))
    for step_i in range(8):
        best, bi = -1, -1
        for j in range(len(cand)):
            if j in chosen:
                continue
            A = np.vstack([sel, Js[j]])
            sv = np.linalg.svd(A, compute_uv=False)
            v = sv[-1]                              # 最小奇异值 = 最弱方向
            if v > best:
                best, bi = v, j
        chosen.append(bi)
        sel = np.vstack([sel, Js[bi]])
        print("  第%d 个：最小奇异值 %.4f" % (step_i + 1, best))

    print("\n挑出的姿态（相对 home 的增量，度）：")
    for i, bi in enumerate(chosen, 1):
        d = cand[bi] - home
        print("  P%-2d  Δq = [%+7.1f %+7.1f %+7.1f %+7.1f %+7.1f %+7.1f]   |Δq|max=%5.1f"
              % (i, d[0], d[1], d[2], d[3], d[4], d[5], np.abs(d).max()))
    print("\n绝对角度（发给控制器用）：")
    for i, bi in enumerate(chosen, 1):
        q = cand[bi]
        print("  P%-2d  q = [%8.2f %8.2f %8.2f %8.2f %8.2f %8.2f]" % ((i,) + tuple(q)))

    # 这些姿态相比现有样本，对最弱方向的灵敏度提升
    S_sel = sens(cand[chosen], P0, step)
    print("\n=== 挑出来的姿态 vs 现有样本，参数灵敏度对比 ===")
    print("%-6s %12s %12s %8s" % ("参数", "现有样本", "新姿态", "提升"))
    for k in range(12):
        a = S[k].max()
        b = S_sel[k].max()
        print("%-6s %12.4f %12.4f %8.1f×" % (PN[k], a, b, (b / a) if a > 1e-9 else float("inf")))


if __name__ == "__main__":
    main()
