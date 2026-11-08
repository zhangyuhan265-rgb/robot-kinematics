# 05 姿态表示与四元数 —— 练习
#
# 单元 5 / 共 6 个单元
# 位置：~/projects/07_机器人学/练习/
# 跑法：~/miniforge3/envs/dl/bin/python 05_姿态表示与四元数.py
#
# 本文件包含两样东西：
#   一、一个 Pose 类（已经写好、验证过，先读懂再用）
#   二、10 个测试（留给你写）
#
# 目标：同一个姿态能在一组表示之间来回转，转坏了立刻知道。

#%% 单元格 0｜先跑这个
from __future__ import annotations

import unittest
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

print("numpy", np.__version__)
import scipy
print("scipy", scipy.__version__)
print("Rotation 可用：", Rotation.from_euler("z", 90, degrees=True).as_matrix().round(3).tolist())


#%% 单元格 1｜Pose 类：位置 + 姿态 + 参考坐标系
#
# 三个设计决定，先想明白再往下看：
#   1) 内部只存四元数 (x, y, z, w)，欧拉角只在进出口出现 —— 避开死锁和约定问题
#   2) 长度用米，角度内部统一弧度
#   3) 每个位姿都带 frame —— 位姿不标坐标系等于没有意义

@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)   # (x, y, z, w)
    frame: str = "base"

    def __post_init__(self):
        object.__setattr__(self, "quat", _normalize(self.quat))
        if not self.frame:
            raise ValueError("frame 不能为空，位姿必须注明参考坐标系")

    @classmethod
    def from_euler(cls, position, euler, *, seq="ZYX", degrees=False, frame="base"):
        rot = Rotation.from_euler(seq, euler, degrees=degrees)
        return cls(x=position[0],y=position[1],z=position[2], quat=tuple(rot.as_quat()), frame=frame)
    #print("Rz(90) 把 x 轴转到：", Pose.from_euler([0, 0, 0], [90, 0, 0], seq="ZYX", degrees=True).to_matrix()[:, 0])
    @classmethod
    def from_matrix(cls, position, matrix, *, frame="base"):
        rot = Rotation.from_matrix(np.asarray(matrix, dtype=float))
        return cls(x=position[0],y=position[1],z=position[2], quat=tuple(rot.as_quat()), frame=frame)

    @classmethod
    def from_quat(cls, position, quat, *, frame="base"):
        return cls(x=position[0],y=position[1],z=position[2], quat=tuple(quat), frame=frame)

    @property                  # 这只是一层加工，把要输出的结果加工一下变成外面能用的东西。这两个函数本身不涉及到任何运算
    def rotation(self) -> Rotation:
        return Rotation.from_quat(self.quat)

    @property                # 这只是一层加工，把要输出的结果加工一下变成外面能用的东西。这两个函数本身不涉及到任何运算
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=float)

    def to_matrix(self) -> np.ndarray:
        return self.rotation.as_matrix()

    def to_euler(self, seq="ZYX", degrees=False):
        return self.rotation.as_euler(seq, degrees=degrees)

    def to_homogeneous(self) -> np.ndarray:
        m = np.eye(4)
        m[:3, :3] = self.to_matrix()
        m[:3, 3] = self.position
        return m

    def compose(self, other: "Pose") -> "Pose":
        """先 other 后 self，等价于矩阵 A @ B。"""
        rot = self.rotation * other.rotation
        pos = self.position + self.rotation.apply(other.position)
        return Pose(x=pos[0],y=pos[1],z=pos[2],quat=tuple(rot.as_quat()), frame=self.frame)

    def inverse(self) -> "Pose":
        inv = self.rotation.inv()
        return Pose(*(-inv.apply(self.position)),quat=tuple(inv.as_quat()), frame=self.frame)

    def __repr__(self):
        e = self.to_euler("ZYX", degrees=True)
        return (f"Pose(xyz=({self.x:.3f}, {self.y:.3f}, {self.z:.3f}), "
                f"zyx_deg=({e[0]:.1f}, {e[1]:.1f}, {e[2]:.1f}), frame={self.frame!r})")


def _normalize(quat):
    """四元数必须是 4 个分量、非零，模长归一化到 1。"""
    if len(quat) != 4:
        raise ValueError(f"四元数必须是 4 个分量，收到 {len(quat)} 个")
    x, y, z, w = (float(v) for v in quat)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm < 1e-12:
        raise ValueError("四元数模长为 0，无法归一化")
    return (x / norm, y / norm, z / norm, w / norm)


IDENTITY = Pose()


#%% 单元格 2｜快速自检：先确认这个类本身是好的
#
# 三行，跑一下。都是上一轮验证过的结论。

print("Rz(90) 把 x 轴转到：", Pose.from_euler([0, 0, 0], [90, 0, 0], seq="ZYX", degrees=True).to_matrix()[:, 0])#[:, 0]所有行，第0列
print("欧拉角往返：", Pose.from_euler([0, 0, 0], [30, 45, 10], seq="ZYX", degrees=True).to_euler("ZYX", degrees=True).round(9))
print("A ∘ A⁻¹ 的位置：", Pose.from_euler([1, 2, 3], [10, 20, 30], seq="ZYX", degrees=True)
        .compose(Pose.from_euler([1, 2, 3], [10, 20, 30], seq="ZYX", degrees=True).inverse()).position.round(12))


#%% 单元格 3｜你的任务：把下面 10 个 raise 换成真正的断言
#
# 规矩：不许改上面的 Pose 类去凑测试。
# 过不了先怀疑测试写错了 —— 顺序、序列、内外旋，错一个就全错。

TOL = 1e-9        # 浮点比较的容差
class PoseTests(unittest.TestCase):
    def test_identity_is_unit(self):
        """IDENTITY 的旋转矩阵是单位阵，位置全为 0。"""
        self.assertTrue(np.allclose(IDENTITY.to_matrix(), np.eye(3), atol=TOL))
        self.assertTrue(np.allclose(IDENTITY.position, 0.0, atol=TOL))

    def test_rz90_maps_x_to_y(self):
        p = Pose.from_euler([0, 0, 0], [90, 0, 0], seq="ZYX", degrees=True)
        x_after = p.to_matrix()[:, 0]
        self.assertTrue(np.allclose(x_after, [0, 1, 0], atol=TOL))

    def test_euler_roundtrip_zyx(self):
        """任取一组欧拉角，转成位姿再转回来，误差应小于 1e-9。"""
        euler = [30.0, 45.0, 10.0]
        p = Pose.from_euler([0, 0, 0], euler, seq="ZYX", degrees=True)
        back = p.to_euler("ZYX", degrees=True)
        self.assertTrue(np.allclose(back, euler, atol=TOL))

    def test_quaternion_is_normalized(self):
        """用任意欧拉角构造出的 Pose，四元数模长应为 1。"""
        p = Pose.from_euler([0, 0, 0], [12.3, -45.6, 78.9], seq="ZYX", degrees=True)
        norm = float(np.linalg.norm(np.array(p.quat)))
        self.assertAlmostEqual(norm, 1.0, places=12)

    def test_quaternion_component_order(self):
        """绕 Z 转 90 度的四元数应是 (0, 0, √2/2, √2/2)，即 (x, y, z, w) 顺序。"""
        raise NotImplementedError("待实现")

    def test_compose_matches_homogeneous_product(self):
        """a.compose(b) 的齐次矩阵应等于 a 的齐次矩阵乘以 b 的。"""
        raise NotImplementedError("待实现")

    def test_inverse_gives_identity(self):
        """A 与 A 的逆复合，应回到单位阵且位置为 0。"""
        raise NotImplementedError("待实现")

    def test_compose_is_not_commutative(self):
        """复合顺序不能交换：a.compose(b) 与 b.compose(a) 应不同。"""
        raise NotImplementedError("待实现")

    def test_gimbal_lock_makes_euler_non_unique(self):
        """中间轴为 90 度时，两组不同欧拉角会得到同一个旋转矩阵。
        提示：用 xyz 外旋，比较 [30, 90, 40] 与 [0, 90, 10] 的矩阵。"""
        raise NotImplementedError("待实现")

    def test_empty_frame_is_rejected(self):
        """frame 为空字符串时应抛 ValueError。"""
        raise NotImplementedError("待实现")


#%% 单元格 4｜跑测试
if __name__ == "__main__":
    unittest.main(verbosity=2)
