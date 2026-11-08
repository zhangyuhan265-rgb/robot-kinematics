# 02 旋转角与齐次变换 —— 练习
#
# 单元 2 / 对应课件 L2（33 页）/ 视频分P 08–16
# 跑法：~/miniforge3/envs/dl/bin/python 02_旋转角与齐次变换.py

#%% 单元格 0｜先跑这个
import numpy as np

np.set_printoptions(precision=3, suppress=True, linewidth=110)
print("numpy", np.__version__)


def Rx(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Ry(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def Rz(deg):
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


print("Rx / Ry / Rz 已备好，后面的格子直接用")


#%% 单元格 1｜K2  Fixed angles：由三个角推出 R
#
# 要点：
#   1. Fixed = 绕【固定不动】的轴转，顺序固定 x → y → z
#   2. 乘积写成 R = Rz(α) · Ry(β) · Rx(γ)
#      —— 先转的（x）写在【最右】
#   3. 旋转不可交换，顺序换一下结果就变
#
# ——— 留给你做 ① ———
# A. 先绕 X 转 60，再绕 Y 转 30
R_xyz = Ry(30)@Rx(60)# ← 用上面的 Rx/Ry/Rz 拼
# B. 先绕 Y 转 30，再绕 X 转 60
R_rev = Rx(60)@Ry(30)

for _n, _R in [("R_xyz", R_xyz), ("R_rev", R_rev)]:
    if _R is None:
        print(f"{_n:6s} 还没写")
        continue
    _R = np.asarray(_R, dtype=float)
    print(f"{_n:6s} =\n{_R}")
    print(f"       det = {np.linalg.det(_R):+.6f}")

if R_xyz is not None and R_rev is not None:
    print("两者相同吗（应该 False）:", np.allclose(R_xyz, R_rev, atol=1e-9))


#%% 单元格 2｜K3  由 R 反推三个角（Fixed angles）
#
# 要点：atan2 的第一个参数是「sin 那一项」，第二个是「cos 那一项」
#
#   β = atan2(-r31, sqrt(r11^2 + r21^2))     第二参数恒 >= 0  ⇒  β ∈ [-90, 90]
#   α = atan2(r21/cβ, r11/cβ)
#   γ = atan2(r32/cβ, r33/cβ)
#
# 注意：r_ij 从 1 开始编号，Python 下标要减 1（r31 就是 R[2][0]）
#
# ——— 留给你做 ② ———
def fixed_from_R(R):
    R = np.asarray(R, dtype=float)
    r11, r21, r31 = R[0][0], R[1][0], R[2][0]
    r32, r33 = R[2][1], R[2][2]

    beta = np.arctan2(-r31,np.hypot(r11,r21))          # ← 用 np.arctan2 和 np.hypot
    cbeta = np.cos(beta)         # ← 用 np.cos
    alpha = np.arctan2(r21/cbeta,r11/cbeta)         # ← 一行
    gamma = np.arctan2(r32/cbeta,r33/cbeta)         # ← 一行

    # ——— 下面别改 ———
    return np.degrees([alpha, beta, gamma])


if R_xyz is not None:
    try:
        a, b, g = fixed_from_R(R_xyz)
        print(f"反推 (α, β, γ) = {a:.2f}, {b:.2f}, {g:.2f}")
        print("期望          =  0.00, 30.00, 60.00")
    except TypeError:
        print("fixed_from_R 还没写完（有 None）")


#%% 单元格 3｜K3  万向节锁：β = 90 时 α 和 γ 分不开
#
# 这格不用写，跑就行。观察：三组完全不同的 (α, γ)，给出【一模一样】的 R。

print("β 固定 90，换三组 (α, γ)：")
for _a, _g in [(0, 0), (30, 30), (90, 90)]:
    _R = Rz(_a) @ Ry(90) @ Rx(_g)
    print(f"  α={_a:3d}, γ={_g:3d} -> {np.round(_R, 3).tolist()}")

_RA = Rz(0) @ Ry(90) @ Rx(0)
_RB = Rz(30) @ Ry(90) @ Rx(30)
print("  α=0,γ=0 和 α=30,γ=30 是同一个矩阵吗:", np.allclose(_RA, _RB, atol=1e-12))
print("  -> β=90 时 R 只跟 (γ−α) 有关，α 和 γ 无法分离（这就是万向节锁）")


#%% 单元格 4｜K5/K6  齐次变换矩阵与它的逆
#
# T = [[R, P],
#      [0, 0, 0, 1]]
#
# T⁻¹ = [[Rᵀ, −Rᵀ·P],
#        [0, 0, 0, 1]]        ← 不用求伴随矩阵，直接转置
#
# ——— 留给你做 ③ ———
def inv_T(T):
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    P = T[:3, 3]
    Rt = R.transpose()            # ← R 的转置
    Pt = -Rt @ P            # ← −Rᵀ·P
    M = np.eye(4)        # ← 填好再把 rt/pt 放进去
    '''     ⎡              ⎤
        ⎢   Rt     Pt  ⎥     ← 左上 3×3 放 Rt，右上那一列放 Pt
M  =    ⎢              ⎥
        ⎢  0 0 0   1   ⎥     ← 这行 eye(4) 已经给你了，别动
        ⎣              ⎦
    '''
    M[:3, :3] = Rt      # 前 3 行 × 前 3 列  → 左上角那块
    M[:3, 3]  = Pt      # 前 3 行 × 第 4 列（下标 3） → 右上那一列
    return M          # ← 换成 M


# 用精确的 cos30 / sin30，不要写 0.866 —— 否则 R 不是严格正交，检查会假报警
_c30, _s30 = np.cos(np.radians(30)), np.sin(np.radians(30))
T_demo = np.array([
    [_c30, -_s30, 0.0, 2.0],
    [_s30,  _c30, 0.0, 1.0],
    [0.0, 0.0, 1.0, 3.0],
    [0.0, 0.0, 0.0, 1.0],
])

inv = inv_T(T_demo)
if inv is None:
    print("inv_T 还没写")
else:
    print(f"T⁻¹ =\n{np.round(inv, 4)}")
    print("T·T⁻¹ 与 I 的最大偏差:", np.abs(T_demo @ inv - np.eye(4)).max())
    print("和 np.linalg.inv 一致:", np.allclose(inv, np.linalg.inv(T_demo), atol=1e-9))


#%% 单元格 5｜K6  先移后转 vs 先转后移
#
# 这格不用写，跑就行。重点看【平移列】：
#   先转后移  [R,Q]    平移列 = Q      （没被转）
#   先移后转  [R,0]·[I,Q] = [R, R·Q]   平移列 = R·Q   （被转了）


def T_of(R, P):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = P
    return M


_R = Rz(90)
_Q = np.array([2.0, 0.0, 0.0])
_I = np.eye(3)
_Z = np.zeros(3)

T_rot_then_move = T_of(_I, _Q) @ T_of(_R, _Z)     # [R, Q]
T_move_then_rot = T_of(_R, _Z) @ T_of(_I, _Q)     # [R, R·Q]

_p = np.array([1.0, 0.0, 0.0, 1.0])
print("先转后移: 平移列 =", np.round(T_rot_then_move[:3, 3], 3), " p ->", np.round((T_rot_then_move @ _p)[:3], 3))
print("先移后转: 平移列 =", np.round(T_move_then_rot[:3, 3], 3), " p ->", np.round((T_move_then_rot @ _p)[:3], 3))
print("后者等于 (p + Q) 再整体转:", np.allclose((T_move_then_rot @ _p)[:3], _R @ (_p[:3] + _Q)))

# %%
