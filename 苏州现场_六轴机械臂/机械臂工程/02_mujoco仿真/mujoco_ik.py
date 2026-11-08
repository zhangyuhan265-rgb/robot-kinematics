# -*- coding: utf-8 -*-
"""在 MuJoCo 3D 窗口里现场演示逆解：目标点逐个走，LM 迭代逐帧放。

用法：
    python mujoco_ik.py                  # 全自动，沿墙面走蛇形
    python mujoco_ik.py --speed 0.04     # 每帧更短，动得更快
    python mujoco_ik.py --hold 0.8       # 每个目标到位后停多久

窗口里看到：
    红球   = 当前 IK 目标
    橙点   = 法兰中心走过的轨迹（迭代过程 + 历史）
    左键拖转视角 / 右键平移 / 滚轮缩放，关窗口退出。
"""
import argparse
import os
import time

import numpy as np
import mujoco
import mujoco.viewer

from arm_env import UrdfArm

RISER = 0.3365
WALL_X = 1.10

# 墙面上的蛇形路径（地坐标系，单位米）
TARGETS, flip = [], False
for z in np.linspace(0.60, 1.70, 6):
    for y in ([0.35, -0.35] if flip else [-0.35, 0.35]):
        TARGETS.append(np.array([WALL_X, y, z]))
    flip = not flip


def add_sphere(scn, pos, r, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([r, r, r]), np.asarray(pos, dtype=float),
        np.eye(3).flatten(), np.asarray(rgba, dtype=np.float32))
    scn.ngeom += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="nbt_arm6.xml")
    ap.add_argument("--speed", type=float, default=0.07, help="每帧秒数")
    ap.add_argument("--hold", type=float, default=0.45, help="到位后停留秒数")
    ap.add_argument("--tries", type=int, default=8)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    arm = UrdfArm(os.path.join(here, "nbt_arm6.urdf"))
    m = mujoco.MjModel.from_xml_path(os.path.join(here, args.xml))
    d = mujoco.MjData(m)
    tcp_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tcp")

    print("模型 %s   nq=%d   tcp body id=%d" % (args.xml, m.nq, tcp_id))
    print("目标点 %d 个，沿 x=%.2f 的墙面蛇形" % (len(TARGETS), WALL_X))
    print("鼠标：左键转视角 / 右键平移 / 滚轮缩放；关窗口退出。")

    q_cur = np.zeros(6)
    trail = []
    total = solved = 0

    with mujoco.viewer.launch_passive(m, d, show_left_ui=False,
                                      show_right_ui=False) as viewer:
        viewer.cam.lookat[:] = [0.60, 0.0, 1.10]
        viewer.cam.distance = 2.9
        viewer.cam.elevation = -8
        viewer.cam.azimuth = 132

        def repaint(tgt, cur=None):
            scn = viewer.user_scn
            scn.ngeom = 0
            for i, t in enumerate(TARGETS):          # 整条路径（淡）
                add_sphere(scn, t, 0.018, [0.55, 0.60, 0.70, 0.6])
            add_sphere(scn, tgt, 0.052, [0.90, 0.05, 0.05, 1.0])
            for p in trail:
                add_sphere(scn, p, 0.010, [1.0, 0.55, 0.10, 0.85])

        k = 0
        while viewer.is_running():
            tgt_floor = TARGETS[k % len(TARGETS)]
            k += 1
            tgt_arm = tgt_floor - np.array([0, 0, RISER])   # IK 用臂座坐标系

            # 先用当前位姿做单起点 LM，顺便录下轨迹用来逐帧播放
            trace = []
            q_new, e = arm._lm(q_cur, arm._err_pos(tgt_arm), trace=trace)
            if e > 1e-3:                                     # 没解到就多起点重来
                q_new, e, ok = arm.ik_ex(q_cur, tgt_arm, tries=args.tries)
                trace = []
            travel = float(np.degrees(np.abs(q_new - q_cur)).sum())

            for q, err, lam in trace:
                if not viewer.is_running():
                    break
                d.qpos[:6] = q
                mujoco.mj_forward(m, d)
                trail.append(d.xpos[tcp_id].copy())
                trail = trail[-60:]
                repaint(tgt_floor)
                viewer.sync()
                time.sleep(args.speed)

            d.qpos[:6] = q_new
            mujoco.mj_forward(m, d)
            trail.append(d.xpos[tcp_id].copy())
            trail = trail[-60:]
            repaint(tgt_floor)
            viewer.sync()

            total += 1
            solved += e < 1e-3
            print("目标 (%5.2f,%6.2f,%5.2f)  残差 %7.3f mm  %2d 帧  行程 %6.1f°"
                  % (tgt_floor[0], tgt_floor[1], tgt_floor[2],
                     e * 1000, len(trace), travel))

            q_cur = q_new
            t_end = time.time() + args.hold
            while time.time() < t_end and viewer.is_running():
                viewer.sync()
                time.sleep(1.0 / 60.0)

    print("\n窗口已关闭。共 %d 个目标，解到 %d 个（%.0f%%）"
          % (total, solved, (solved / max(total, 1)) * 100))


if __name__ == "__main__":
    main()
