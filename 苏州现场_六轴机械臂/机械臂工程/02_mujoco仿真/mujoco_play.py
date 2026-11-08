# -*- coding: utf-8 -*-
"""自由拖动模式 —— 鼠标直接把机械臂拽到任意姿态。

和 mujoco_view.py 的区别：
    mujoco_view 只做 mj_forward（运动学显示），鼠标拖不动。
    这个跑真正的物理（mj_step），鼠标施加的是力，机械臂会真的动。

用法：
    python mujoco_play.py                    # 无重力，拖着走、松手就停
    python mujoco_play.py --gravity 9.81     # 有重力，像真的臂（会往下坠）
    python mujoco_play.py --xml nbt_arm6_wall.xml
    python mujoco_play.py --q 0,0,0,0,90,0   # 起始姿态

鼠标操作（MuJoCo 窗口通用）：
    左键拖 / 右键拖   = 转视角、平移
    滚轮              = 缩放
    双击某个连杆      = 选中它（会高亮）
    按住 Ctrl + 左键拖 = 给选中的连杆施加力 ← 这就是"拖着机械臂走"
    再双击空白处      = 取消选中
"""
import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="nbt_arm6.xml")
    ap.add_argument("--q", default="0,0,0,0,0,0")
    ap.add_argument("--gravity", type=float, default=0.0,
                    help="0 = 无重力（拖着玩）；9.81 = 真实重力（会下坠）")
    ap.add_argument("--damping", type=float, default=3.0, help="关节阻尼，越大越稳")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    path = args.xml if os.path.isabs(args.xml) else os.path.join(here, args.xml)
    m = mujoco.MjModel.from_xml_path(path)
    d = mujoco.MjData(m)

    # 重力
    m.opt.gravity[:] = [0.0, 0.0, -abs(args.gravity)]
    # 阻尼（URDF 里设的是 2，这里可以覆盖）
    m.dof_damping[:] = args.damping

    print("模型: %s   重力 = %.2f m/s^2   阻尼 = %.1f"
          % (os.path.basename(path), args.gravity, args.damping))
    print()
    print("  ★ 双击一个连杆选中它，然后按住 Ctrl + 左键拖 —— 机械臂就跟着动了")
    print("  ★ 再双击空白处取消选中")
    print("  ★ 左键拖/右键拖 = 转视角/平移，滚轮 = 缩放")
    if args.gravity > 0:
        print("  ★ 有重力：松手会往下坠，像真的")
    else:
        print("  ★ 无重力：拖到哪停到哪，方便摆姿态")
    print()

    d.qpos[:] = np.radians([float(v) for v in args.q.split(",")])
    mujoco.mj_forward(m, d)

    with mujoco.viewer.launch_passive(m, d) as v:
        v.cam.distance = 3.6
        v.cam.elevation = -12
        v.cam.azimuth = 145
        v.cam.lookat[:] = [0.45, 0.0, 1.05]
        t0 = time.time()
        while v.is_running():
            mujoco.mj_step(m, d)          # ← 真物理，鼠标的力才起作用
            v.sync()
            time.sleep(1.0 / 120.0)
        print("窗口已关闭。最后姿态 q(deg) = %s"
              % np.round(np.degrees(d.qpos[:6]), 2))


if __name__ == "__main__":
    main()
