# 03 DH 参数建模 —— 练习
#
# 单元 3 / 对应课件 L3（39 页）/ 视频分P 17–25
# 跑法：~/miniforge3/envs/dl/bin/python 03_DH参数.py

#%% 单元格 0｜先跑这个：工具函数（不用改）
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=110)
print("numpy", np.__version__)


def Rx(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def Ry(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def Rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def T_of(R, p=(0.0, 0.0, 0.0)):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def Tx(a):
    return T_of(np.eye(3), (a, 0.0, 0.0))


def Dz(d):
    return T_of(np.eye(3), (0.0, 0.0, d))


def dh_craig(a, alpha, d, theta):
    """Craig 版（改进 DH）单级：T = Rx(alpha_{i-1})·Dx(a_{i-1})·Rz(theta_i)·Dz(d_i)"""
    # 注意：Rx/Rz 返回 3x3，要用 T_of() 包成 4x4 才能和 Tx/Dz 相乘
    return T_of(Rx(alpha)) @ Tx(a) @ T_of(Rz(theta)) @ Dz(d)


def dh_standard(theta, d, a, alpha):
    """Standard 版单级：T = Rz(theta_i)·Dz(d_i)·Dx(a_i)·Rx(alpha_i)"""
    return T_of(Rz(theta)) @ Dz(d) @ Tx(a) @ T_of(Rx(alpha))


print("函数备好：Rx / Ry / Rz / Tx / Dz / dh_craig / dh_standard")


#%% 单元格 1｜列 Craig 版 DH 表
#
# 研究对象：2 连杆平面臂，l1 = 1.0，l2 = 0.8（两个关节都绕 z 轴转）
#
# 每一级只要三样：a_{i-1}、alpha_{i-1}、d_i（θ 是关节变量，另外给）
#
# 提示：
#   1. 两个关节都绕 z 轴 → 所有 alpha 都是 0
#   2. d_i 是沿 z 的偏置，平面臂里全是 0
#   3. a_{i-1} 是沿 x 的杆长 → a_1 = l1、a_2 = l2
#   4. 最后要加一个"工具帧"（i = 3），它的 θ3 恒等于 0
#
# ——— 留给你做 ① ———
l1, l2 = 1.0, 0.8

DH_craig = [
    (0.0, 0.0, 0.0),      # i=1  示范行：(a_0, alpha_0, d_1)
    (l1, 0.0, 0.0),                 # i=2  ← 你填 (a_1, alpha_1, d_2)
    (l2, 0.0, 0.0),                 # i=3  ← 你填 (a_2, alpha_2, d_3)
]

if any(r is None for r in DH_craig):
    print("DH_craig 还没填完")
else:
    for _i, (_a, _al, _d) in enumerate(DH_craig, 1):
        print(f"  i={_i}: a_{_i-1}={_a:5.2f}   alpha_{_i-1}={_al:5.1f}   d_{_i}={_d:5.2f}")


#%% 单元格 2｜连乘得到 T_0^n
#
# 连乘顺序：T_0^n = T_0^1 · T_1^2 · … · T_{n-1}^n
# 注意：矩阵乘法从左到右写，但作用到点上时是【从右往左】发生的。

q = np.array([30.0, 45.0, 0.0])      # q3 = 0（工具帧不转）


def fk_craig(DH, q):
    T = np.eye(4)
    for (_a, _al, _d), _th in zip(DH, q):
        T = T @ dh_craig(_a, _al, _d, _th)
    return T


if any(r is None for r in DH_craig):
    print("先填完 DH_craig")
else:
    T03 = fk_craig(DH_craig, q)
    print("T_0^3 =\n", np.round(T03, 4))
    print("末端位置 =", np.round(T03[:3, 3], 4))


#%% 单元格 3｜和解析式对一下（自动检查，不用改）
def fk_analytic(q1_deg, q2_deg, _l1=l1, _l2=l2):
    _q1, _q2 = np.radians(q1_deg), np.radians(q2_deg)
    return np.array([
        _l1 * np.cos(_q1) + _l2 * np.cos(_q1 + _q2),
        _l1 * np.sin(_q1) + _l2 * np.sin(_q1 + _q2),
        0.0,
    ])


if any(r is None for r in DH_craig):
    print("先填完 DH_craig")
else:
    for _q in [(30.0, 45.0), (0.0, 0.0), (90.0, -30.0), (-120.0, 75.0)]:
        _T = fk_craig(DH_craig, np.array([_q[0], _q[1], 0.0]))
        _p_dh = _T[:3, 3]
        _p_an = fk_analytic(*_q)
        _ok = np.allclose(_p_dh, _p_an, atol=1e-9)
        print(f"  q=({_q[0]:7.1f},{_q[1]:7.1f})  DH 得出 {np.round(_p_dh, 4)}  解析式 {np.round(_p_an, 4)}  {'OK' if _ok else '不一致'}")


#%% 单元格 4｜Standard 版：同一个臂，不同的表
#
# ——— 留给你做 ② ———
# Standard 版每行是 (d_i, a_i, alpha_i)，θ 单独给。
# 它与 Craig 版最大的差别：Standard 的 2 连杆只需要【2 帧】，不要工具帧。
#
# 提示：Standard 里 a_i 是本级的杆长，所以 a_1 = l1、a_2 = l2
DH_standard = [
    (0.0, 1.0, 0.0),      # i=1 示范：(d_1, a_1, alpha_1)
    (0.0, l2, 0.0),                 # i=2  ← 你填 (d_2, a_2, alpha_2)
]


def fk_standard(DH, q):
    T = np.eye(4)
    for (_d, _a, _al), _th in zip(DH, q):
        T = T @ dh_standard(_th, _d, _a, _al)
    return T


if any(r is None for r in DH_standard):
    print("DH_standard 还没填完")
else:
    _T_std = fk_standard(DH_standard, np.array([30.0, 45.0]))
    print("Standard 的 T_0^2 =\n", np.round(_T_std, 4))
    if not any(r is None for r in DH_craig):
        _T_craig = fk_craig(DH_craig, np.array([30.0, 45.0, 0.0]))
        print("和 Craig 版结果一致吗:", np.allclose(_T_std, _T_craig, atol=1e-9))


#%% 单元格 5｜自检（写在这里，别急着往下走）
#
# 1. 为什么平面 2 连杆在 Craig 版下要 3 个 frame，在 Standard 版下只要 2 个？
#
# 2. DH 表里哪个参数是"关节变量"，哪个是"几何常量"？两套约定分别怎么区分？
#
# 3. 如果把 l2 改成 0（退化成一根杆），T_0^3 会变成什么样？先猜，再改数字跑一遍。
