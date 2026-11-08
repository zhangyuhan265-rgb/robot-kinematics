# -*- coding: utf-8 -*-
"""把臂本体 URDF 装到一个固定垫高上，生成训练用的整机模型。

为什么单独一个脚本：
    臂本体 `nbt_arm6.urdf` 的 `base_link` 原点是【安装面】，不含垫高。
    控制器的 DH 也是从安装面起算的（L1 = 431）。所以垫高不能写进本体，
    否则仿真和控制器就差一个垫高、互相漂。

    本脚本在本体下面接一段垫高，并钉到 world：

        world --(fixed)--> riser --(fixed)--> base_link

    换安装高度只改这一个数，本体一行不用动。

用法：
    python3 gen_mounted.py urdf/nbt_arm6.urdf 336.5 urdf/nbt_arm6_on_riser.urdf

    # 想挂到底盘上而不是 world，就生成后把 world_to_riser 的 parent 改成底盘 link，
    # 或者直接用 --parent chassis_link
"""
import io
import sys

MM = 0.001


def fmt(v):
    return ("%.6f" % v).rstrip("0").rstrip(".")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    parent = "world"
    for a in sys.argv[1:]:
        if a.startswith("--parent"):
            parent = a.split("=", 1)[1] if "=" in a else parent

    arm = args[0] if len(args) > 0 else "urdf/nbt_arm6.urdf"
    riser = float(args[1]) if len(args) > 1 else 336.5
    out = args[2] if len(args) > 2 else "urdf/nbt_arm6_on_riser.urdf"

    R = riser * MM
    mass = 20.0
    rad = 0.160
    ixx = mass * (3 * rad * rad + R * R) / 12.0
    izz = mass * rad * rad / 2.0

    src = io.open(arm, encoding="utf-8").read()
    if "</robot>" not in src:
        raise SystemExit("没找到 </robot>，输入不像 URDF")

    chunk = u"""
  <!-- ================= 装配层（gen_mounted.py 生成，勿手改）=================
       垫高 %(riser).1f mm：world -> riser -> %(parent)s ... -> base_link
       臂本体 base_link 原点仍是【安装面】，所以本体的运动学一行没动。
       =================================================================== -->
  <link name="world"/>

  <link name="riser">
    <inertial>
      <origin xyz="0 0 %(half)s" rpy="0 0 0"/>
      <mass value="%(mass)s"/>
      <inertia ixx="%(ixx)s" ixy="0" ixz="0" iyy="%(ixx)s" iyz="0" izz="%(izz)s"/>
    </inertial>
    <visual>
      <origin xyz="0 0 %(half)s" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="%(rad)s" length="%(R)s"/>
      </geometry>
      <material name="metal_dark"/>
    </visual>
    <collision>
      <origin xyz="0 0 %(half)s" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="%(rad)s" length="%(R)s"/>
      </geometry>
    </collision>
  </link>

  <joint name="riser_mount" type="fixed">
    <parent link="%(parent)s"/>
    <child link="riser"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <joint name="riser_to_arm_base" type="fixed">
    <parent link="riser"/>
    <child link="base_link"/>
    <origin xyz="0 0 %(R)s" rpy="0 0 0"/>
  </joint>
""" % dict(riser=riser, parent=parent, half=fmt(R / 2.0), mass=fmt(mass),
           ixx=fmt(ixx), izz=fmt(izz), rad=fmt(rad), R=fmt(R))

    dst = src.replace("</robot>", chunk + "</robot>", 1)
    io.open(out, "w", encoding="utf-8").write(dst)

    print("垫高 %.1f mm 已装配：" % riser)
    print("  %s --fixed--> riser --fixed--> base_link" % parent)
    print("  垫高件：圆柱 r=%.0f mm, h=%.1f mm, m=%.0f kg" % (rad * 1000, riser, mass))
    print("  写成 %s（%d 字节）" % (out, len(dst.encode("utf-8"))))
    print()
    print("  臂本体 base_link 原点 = 安装面，离地 %.1f mm" % riser)
    print("  想换安装高度：重跑本脚本，别去改 nbt_arm6.urdf")


if __name__ == "__main__":
    main()
