# 04 逆运动学 —— 练习
#
# 单元 4 / 对应课件 L4（35 页）/ 视频分P 26–32
# 跑法：~/miniforge3/envs/dl/bin/python 04_逆运动学.py

#%% 单元格 0｜先跑这个：工具
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=110)
print("numpy", np.__version__)

l1, l2 = 1.0, 0.8


def fk2(q1_deg, q2_deg, _l1=l1, _l2=l2):
    """2 连杆正解：输入两个关节角（度），返回末端 (x, y)"""
    _q1, _q2 = np.radians(q1_deg), np.radians(q2_deg)
    return np.array([
        _l1 * np.cos(_q1) + _l2 * np.cos(_q1 + _q2),
        _l1 * np.sin(_q1) + _l2 * np.sin(_q1 + _q2),
    ])


print("fk2 已备好，可以先用它算正解")


#%% 单元格 1｜三角方程 a·cos(th) + b·sin(th) = c
#
# 这是整讲的咽喉：后面 RRR、PUMA、Pieper 解法都靠它。
#
# 解法（按课件 p18：半角替换，化成二次方程）
#
#   u = tan(theta/2)
#   cos(theta) = (1 - u^2)/(1 + u^2)
#   sin(theta) = 2u/(1 + u^2)
#
#   代入 a·cos + b·sin = c：
#       (a + c)u^2 - 2b·u + (c - a) = 0
#
#   判别式     disc = b^2 + a^2 - c^2        disc < 0 → 无解
#   求根       u = [ b ± sqrt(disc) ] / (a + c)
#   回角度     theta = 2 * arctan(u)
#
# 两个坑（先不用管，想加深再说）：
#   1. a + c = 0 时二次项消失，一支根飞到无穷远 —— 那一支正好对应 theta = 180°
#   2. 所以 theta = 180° 用半角永远求不出来（tan(90°) 无定义），得单独补一支
#
# ——— 留给你做 ① ———
def solve_trig(a, b, c):
    """用半角替换返回两个解 (deg1, deg2)；无解返回 None"""
    disc = b**2 + a**2 - c**2            # ← 判别式 b^2 + a^2 - c^2（一行）

    if disc is None or disc < 0:
        return None        # 无实数解，返回 None

    root = np.sqrt(disc)            # ← np.sqrt(disc)
    u1 = (b+root) / (a+c)              # ← (b + root) / (a + c)
    u2 = (b-root) / (a+c)              # ← (b - root) / (a + c)
    th1 = 2*np.arctan(u1)             # ← 2 * np.arctan(u1)
    th2 = 2*np.arctan(u2)             # ← 2 * np.arctan(u2)

    return np.degrees([th1, th2])


print("3cos + 4sin = 2  ->", solve_trig(3, 4, 2), " ← 填完后应是两个解；可回代 3cos+4sin 验证是否等于 2")
print("3cos + 4sin = 9  ->", solve_trig(3, 4, 9), "（应该 None，因为 9 > 5）")

# —— 回代自检：不管用辅助角还是半角，解必须真的满足原方程 ——
_res = solve_trig(3, 4, 2)
if _res is not None:
    for _t in _res:
        _v = 3.0 * np.cos(np.radians(_t)) + 4.0 * np.sin(np.radians(_t))
        _tag = "OK" if abs(_v - 2.0) < 1e-9 else "不对"
        print(f"  回代 theta={_t:9.4f}  ->  3cos+4sin = {_v:.10f}   {_tag}")


#%% 单元格 2｜2 连杆的解析逆解
#
# 已知末端 (x, y)，求 (q1, q2)。
#
# 提示：
#   cos(q2) = (x^2 + y^2 - l1^2 - l2^2) / (2*l1*l2)
#   q2 = ± arccos(...)                     ← 正负就是【肘上 / 肘下】两支
#   q1 = arctan2(y, x) - arctan2(l2*sin(q2), l1 + l2*cos(q2))
#
# ——— 留给你做 ② ———
def ik2(x, y, _l1=l1, _l2=l2):
    """返回两个解 [(q1a, q2a), (q1b, q2b)]（度）；够不着时返回 None"""
    # 余弦定理：先算 cos(q2)
    c2 = (x**2 + y**2 - _l1**2 - _l2**2) / (2*_l1*_l2)       # ← 一行：用 x, y, _l1, _l2（注意乘方是 **）

    if c2 is None or abs(c2) > 1:
        return None                             # 够不着

    sols = []
    for s in (1, -1):                           # s = +1 肘上一支，-1 肘下另一支
        q2 = s * np.arccos(c2)                  # ← s * np.arccos(c2)
        q1 = np.arctan2(y, x) - np.arctan2(_l2 * np.sin(q2), _l1 + _l2 * np.cos(q2))             # ← np.arctan2(y, x) 减去 np.arctan2(...)
        # 注意：q1 可能为负，不要套 abs
        sols.append((np.degrees(q1), np.degrees(q2)))
    return sols


print("x=1, y=0 ->", ik2(1.0, 0.0))
print("x=5, y=0 ->", ik2(5.0, 0.0), "（应该 None，够不着）")


#%% 单元格 3｜闭环验证（自动检查，不用改）
#
# 思路：随便取一组关节角 → 正解得到末端 → 逆解 → 应该能拿回原来那组角（或等价解）

def _wrap(deg):
    """把角度折到 (-180, 180]"""
    return (deg + 180.0) % 360.0 - 180.0


if ik2(1.0, 0.0) is None:
    print("ik2 还没写完")
else:
    rng = np.random.default_rng(0)
    ok = True
    for _ in range(5):
        _q = rng.uniform(-150, 150, size=2)
        _p = fk2(_q[0], _q[1])
        _sols = ik2(float(_p[0]), float(_p[1]))
        if _sols is None:
            print(f"  q={np.round(_q,1)} 逆解返回 None，不正常")
            ok = False
            continue
        _hit = any(
            abs(_wrap(s[0] - _q[0])) < 1e-6 and abs(_wrap(s[1] - _q[1])) < 1e-6
            for s in _sols
        )
        print(f"  q={np.round(_q,1)} -> p={np.round(_p,4)} -> 逆解 {[(round(s[0],2), round(s[1],2)) for s in _sols]}  含原解: {_hit}")
        ok = ok and _hit
    print("全部通过:", ok)


#%% 单元格 4｜多解：肘上 / 肘下
#
# 跑就行。观察同一组 (x, y) 对应的两组关节角，它们的 q2 符号相反。

if ik2(1.0, 0.0) is not None:
    _target = (1.2, 0.5)
    _sols = ik2(*_target)
    print("目标点:", _target)
    for _k, _s in enumerate(_sols):
        _p = fk2(_s[0], _s[1])
        _name = "肘上" if _s[1] >= 0 else "肘下"
        print(f"  解 {_k + 1}（{_name}）: q1={_s[0]:8.3f}  q2={_s[1]:8.3f}   正解回去 = {np.round(_p, 4)}")
else:
    print("ik2 还没写完")


#%% 单元格 5｜自检（写在这里）
#
# 1. 为什么 2 连杆会有两组解？几何上这"两组"分别对应手臂的什么姿态？
#
# 2. 工程上从两组里选一组，你会按什么标准？至少说两条。
#
# 3. 如果目标点在 (3, 0)，l1 = 1、l2 = 0.8，会发生什么？为什么？
#
# 4. 数值法（比如牛顿迭代）在什么情况下反而比解析法更合适？
