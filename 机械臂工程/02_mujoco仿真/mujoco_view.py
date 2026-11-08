# -*- coding: utf-8 -*-
"""MuJoCo 3D 查看器 —— 真实的 3D 窗口，鼠标可以转视角/缩放。

用法：
    python mujoco_view.py                     # 停在零位
    python mujoco_view.py --q 0,0,0,0,90,0    # 指定位姿
    python mujoco_view.py --sweep 4           # J4 缓慢摆动，看关节怎么动
    python mujoco_view.py --sweep 4 --amp 60

鼠标：
    左键拖 = 转视角     右键拖 = 平移     滚轮 = 缩放
    （双击物体可以给它一个扰动）
"""
import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--urdf", default="nbt_arm6_on_riser.urdf")
    ap.add_argument("--q", default="0,0,0,0,0,0")
    ap.add_argument("--sweep", type=int, default=-1, help="要摆动的关节号 1..6，-1 不摆")
    ap.add_argument("--amp", type=float, default=45.0, help="摆动幅度（度）")
    ap.add_argument("--period", type=float, default=4.0, help="摆动周期（秒）")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    p = args.urdf if os.path.isabs(args.urdf) else os.path.join(here, args.urdf)
    m = mujoco.MjModel.from_xml_path(p)
    d = mujoco.MjData(m)

    q0 = np.radians([float(v) for v in args.q.split(",")])
    print("模型: %s   nq=%d" % (os.path.basename(p), m.nq))
    print("初始 q(deg) = %s" % np.round(np.degrees(q0), 2))
    print("窗口操作：左键转视角 / 右键平移 / 滚轮缩放；关窗口退出。")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        # 相机/光照等可选调整
        viewer.cam.distance = 4.0
        viewer.cam.elevation = -15
        viewer.cam.azimuth = 130
        t0 = time.time()
        while viewer.is_running():
            d.qpos[:] = q0
            if 1 <= args.sweep <= 6:
                k = args.sweep - 1
                w = 2 * np.pi / max(args.period, 0.5)
                d.qpos[k] = q0[k] + np.radians(args.amp) * np.sin(w * (time.time() - t0))
            mujoco.mj_forward(m, d)
            viewer.sync()
            time.sleep(1.0 / 60.0)
    print("窗口已关闭。")


if __name__ == "__main__":
    main()
