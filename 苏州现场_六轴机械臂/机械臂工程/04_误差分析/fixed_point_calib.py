# -*- coding: utf-8 -*-
"""固定点法标定：只靠「多组构型碰同一个物理点」，不需要任何量具。

原理
----
杆件上装一个尖头，碰同一个物理点 P，换 n 组差别很大的构型各碰一次。
物理上 n 组的尖端都在 P；模型给出的尖端位置是

    p_i(δ) = FK_i(δ)[:3,3] + FK_i(δ)[:3,:3] · t

δ = 待求的参数修正，t = 尖头相对法兰的偏移。
最小化 Σ|p_i − P|² 就能同时解出 δ、t、P。**不需要知道 P 在哪儿，也不用量尖头多长。**

⚠️ 这个方法的物理边界（自检里踩过，必须知道）
------------------------------------------------
1. **d6 和尖头的 z 向长度在数学上完全等价**——都在轴 6 方向上平移尖端。
   无论摆多少组构型都分不开。所以本工具不估 d6，把它并入 t_z。
   要单独验 d6(=154)，只能拿卡尺量「腕心→法兰面」。
2. **off6 在尖头落在轴线延长线上时对尖端位置没有任何影响**（尖端就在轴 6 上）。
   本工具也不估 off6。要验它得用姿态数据，或者把尖头故意装偏。
3. 剩下的都估：d1 a1 a2 a3 d4、off1..off5，外加 t 和 P。
4. **只有真碰到同一个点的数据才有意义**。碰歪一次就污染结果。

用法
----
    python3 fixed_point_calib.py --selftest      # 合成数据自检
    python3 fixed_point_calib.py data.csv        # 真实数据

data.csv（表头必须这样，角度单位度）：
    name,q1,q2,q3,q4,q5,q6
    touch1,-12.0,-7.6,7.7,-34.3,-30.3,-68.2
    ...
"""
import csv
import math
import sys

import numpy as np
from scipy.optimize import least_squares

ALPHA = np.radians([90.0, 0.0, 90.0, 90.0, 90.0, 0.0])
D_NOM = np.array([377.0, 0.0, 0.0, 934.0, 0.0, 154.0])
A_NOM = np.array([216.0, 900.0, 130.0, 0.0, 0.0, 0.0])
OFF_NOM = np.array([0.0, -90.0, 0.0, 0.0, 90.0, 180.0])

# 可辨识参数：d1 a1 a2 a3 d4，加 off1..off5
PN = ["Δd1", "Δa1", "Δa2", "Δa3", "Δd4",
      "Δoff1", "Δoff2", "Δoff3", "Δoff4", "Δoff5"]
NP_ = len(PN)


def _rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


def _rx(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], float)


def fk(q_deg, dpar):
    """dpar: 长度修正在 dpar[0:5]，零位修正在 dpar[5:10]"""
    ds = np.array([D_NOM[0] + dpar[0], 0.0, 0.0, D_NOM[3] + dpar[4], 0.0, D_NOM[5]])
    as_ = np.array([A_NOM[0] + dpar[1], A_NOM[1] + dpar[2], A_NOM[2] + dpar[3],
                    0.0, 0.0, 0.0])
    off = OFF_NOM.copy()
    off[:5] += dpar[5:10]
    T = np.eye(4)
    for i in range(6):
        th = math.radians(q_deg[i] - off[i])
        Ti = np.eye(4)
        Ti[:3, :3] = (_rz(th) @ _rx(ALPHA[i]))[:3, :3]
        Ti[:3, 3] = _rz(th)[:3, :3] @ np.array([as_[i], 0.0, ds[i]])
        T = T @ Ti
    return T


def tip(q, dpar, t):
    T = fk(q, dpar)
    return T[:3, 3] + T[:3, :3] @ t


def unpack(x):
    return x[:NP_], x[NP_:NP_ + 3], x[NP_ + 3:NP_ + 6]


def residuals(x, qs):
    dpar, P, t = unpack(x)
    r = []
    for q in qs:
        r.extend(tip(q, dpar, t) - P)
    return np.array(r)


def spread(qs, dpar, t):
    ps = np.array([tip(q, dpar, t) for q in qs])
    return ps, np.linalg.norm(ps - ps.mean(axis=0), axis=1)


def solve(qs, t0=None):
    t0 = np.array([0.0, 0.0, 100.0]) if t0 is None else np.asarray(t0, float)
    ps = np.array([tip(q, np.zeros(NP_), t0) for q in qs])
    x0 = np.concatenate([np.zeros(NP_), ps.mean(axis=0), t0])
    lo = np.concatenate([np.full(5, -150.0), np.full(5, -15.0),
                         np.full(3, -1e5), np.full(3, -800.0)])
    hi = np.concatenate([np.full(5, 150.0), np.full(5, 15.0),
                         np.full(3, 1e5), np.full(3, 800.0)])
    s = least_squares(residuals, x0, args=(qs,), bounds=(lo, hi),
                      x_scale="jac", ftol=1e-15, xtol=1e-15, gtol=1e-15, max_nfev=20000)
    return s


def observability(qs, x):
    """数值雅可比 -> 奇异值，看哪些方向定得住"""
    n = len(qs)
    J = np.zeros((3 * n, len(x)))
    f0 = residuals(x, qs)
    h = 1e-4
    for k in range(len(x)):
        xx = x.copy()
        xx[k] += h
        J[:, k] = (residuals(xx, qs) - f0) / h
    sv = np.linalg.svd(J, compute_uv=False)
    return sv


def report(qs, names, sb, sa, delta):
    print("\n样本 %d 组" % len(qs))
    print("拟合前：尖端散差 RMS %.3f mm  最大 %.3f mm"
          % (np.sqrt((sb ** 2).mean()), sb.max()))
    print("拟合后：尖端散差 RMS %.3f mm  最大 %.3f mm"
          % (np.sqrt((sa ** 2).mean()), sa.max()))
    print("\n解出的修正：")
    for n, v in zip(PN, delta):
        flag = "  ← 小于 2mm，基本是噪声" if abs(v) < 2.0 and n.startswith("Δd") else ""
        print("  %-6s %+9.3f%s" % (n, v, flag))


def selftest(noise=0.3, seed=7):
    print("=" * 64)
    print("自检：造 10 组「碰同一点」的构型（真值模型带已知误差 + %.1fmm 触碰噪声）" % noise)
    print("=" * 64)
    d_true = np.zeros(NP_)
    d_true[0] = -3.0      # Δd1
    d_true[2] = -16.0     # Δa2   大臂短 16mm
    d_true[4] = +2.0      # Δd4
    d_true[6] = +0.6      # Δoff2
    d_true[8] = -0.8      # Δoff4
    t_true = np.array([0.0, 0.0, 120.0])
    P_true = np.array([1250.0, 150.0, 850.0])
    rng = np.random.default_rng(seed)

    def ik(seed_q):
        def f(x):
            return tip(x, d_true, t_true) - P_true
        s = least_squares(f, seed_q, bounds=(-125.0, 125.0), xtol=1e-14, ftol=1e-14)
        return s.x, np.linalg.norm(s.fun)

    qs = []
    tries = 0
    while len(qs) < 10 and tries < 6000:
        tries += 1
        seed_q = np.array([rng.uniform(-110, 110) for _ in range(6)])
        q, err = ik(seed_q)
        if err > 1e-6:
            continue
        if any(np.abs(q - o).max() < 25 for o in qs):
            continue
        qs.append(np.round(q, 3))
    print("造出 %d 组构型" % len(qs))

    # 加触碰噪声：把 P 稍微抖一下（模拟人手触摸的重复性）
    qs_noisy = []
    for q in qs:
        qq = q + rng.normal(0, 0.01, 6)   # 关节读数抖动 0.01 度
        qs_noisy.append(qq)

    _, sb = spread(qs_noisy, np.zeros(NP_), t_true)
    s = solve(qs_noisy)
    delta, P, t = unpack(s.x)
    _, sa = spread(qs_noisy, delta, t)
    report(qs_noisy, ["p%d" % (i + 1) for i in range(len(qs))], sb, sa, delta)

    print("\n与真值对比：")
    ok = 0
    for n, v, tv in zip(PN, delta, d_true):
        d = v - tv
        good = abs(d) < max(2.0, abs(tv) * 0.25)
        ok += good
        print("  %-6s 解 %+9.3f   真值 %+9.3f   差 %+7.3f  %s"
              % (n, v, tv, d, "OK" if good else "偏"))
    print("  → %d/%d 项落在容差内" % (ok, NP_))
    sv = observability(qs_noisy, s.x)
    print("\n奇异值（最后几个=最弱方向）：%s" % np.round(sv[-5:], 3))
    print("数值越小越定不住；如果比最大奇异值小 5 个数量级以上，说明那个方向没信息")


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    if len(sys.argv) < 2:
        print(__doc__)
        return
    names, qs = [], []
    for r in csv.DictReader(open(sys.argv[1], encoding="utf-8")):
        names.append(r["name"])
        qs.append([float(r["q%d" % (i + 1)]) for i in range(6)])
    if len(qs) < 6:
        raise SystemExit("至少要 6 组，建议 8~10 组，且构型差别要大")
    t0 = np.array([0, 0, 100.0])
    if "--pointer" in sys.argv:
        t0 = np.array([float(v) for v in sys.argv[sys.argv.index("--pointer") + 1].split(",")])
    _, sb = spread(qs, np.zeros(NP_), t0)
    s = solve(qs, t0)
    delta, P, t = unpack(s.x)
    _, sa = spread(qs, delta, t)
    report(qs, names, sb, sa, delta)
    print("\n尖头偏移 t = %s mm" % np.round(t, 2))
    print("固定点 P    = %s mm（基座系）" % np.round(P, 1))
    sv = observability(qs, s.x)
    print("奇异值最后 5 个：%s" % np.round(sv[-5:], 3))
    print("\n注意：d6 与尖头 z 长度等价、off6 对尖端位置无影响，这两项本方法测不出。")


if __name__ == "__main__":
    main()
