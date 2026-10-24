# 01 旋转矩阵与坐标变换 —— 练习
#
# 单元 1 / 共 6 个单元
# 位置：~/projects/07_机器人学/练习/
# 跑法：~/miniforge3/envs/dl/bin/python 01_旋转矩阵与坐标变换.py
#
# 单元格约定：
#   0 号格必须先跑（导入 + 环境自检）
#   一个知识点一个格，一共 5 格

#%% 单元格 0｜先跑这个
import numpy as np

np.set_printoptions(precision=3, suppress=True, linewidth=100)

print("numpy", np.__version__)
print("单位阵 I =\n", np.eye(3))


#%% 单元格 1｜知识点 1：第 j 列 = 第 j 根坐标轴转完之后指向哪
#
# 一句话：不用背 sin/cos。把三根箭头转过去，位置并排竖着写成列，就是矩阵。
#
# 三个 90 度的几何答案（聊天里给过的表，照抄不算抄）：
#   绕 z 转 90： e1(1,0,0)->(0,1,0)    e2(0,1,0)->(-1,0,0)   e3(0,0,1)->(0,0,1)
#   绕 x 转 90： e1(1,0,0)->(1,0,0)    e2(0,1,0)->(0,0,1)    e3(0,0,1)->(0,-1,0)
#   绕 y 转 90： e1(1,0,0)->(0,0,-1)   e2(0,1,0)->(0,1,0)    e3(0,0,1)->(1,0,0)
#
# 写 np.array 时最容易翻车的地方 ——「列」要横着填进每一行：
#   若三列分别是 (a,b,c)、(d,e,f)、(g,h,i)，则写成
#       [[a, d, g],
#        [b, e, h],
#        [c, f, i]]
#
# ——— 留给你做 1 ———
# 把上面三张表翻译成三个 3x3 矩阵
Rz90 = np.array([[0,-1,0],[1,0,0],[0,0,1]])
Rx90 = np.array([[1,0,0],[0,0,-1],[0,1,0]])
Ry90 = np.array([[0,0,1],[0,1,0],[-1,0,0]])


# ——— 自动检查：填完直接跑，这段别改 ———
for _name, _R in [("Rz90", Rz90), ("Rx90", Rx90), ("Ry90", Ry90)]:
    if _R is None:
        print(f"{_name:5s} 还没写")
        continue
    _R = np.asarray(_R, dtype=float)
    _det = np.linalg.det(_R)
    _orth = np.abs(_R.T @ _R - np.eye(3)).max()
    print(f"{_name:5s} det={_det:+.6f}   R^T R - I 最大偏差={_orth:.2e}   形状={_R.shape}")

# %%
