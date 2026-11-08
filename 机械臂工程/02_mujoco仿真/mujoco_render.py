# -*- coding: utf-8 -*-
"""把 MuJoCo 场景离屏渲染成 PNG（带材质/地面/墙/灯光）。

用法：
    python mujoco_render.py                        # 零位
    python mujoco_render.py --q 0,0,0,0,90,0 --out media/mj_q5_90.png
"""
import argparse
import os

import numpy as np
import mujoco


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="nbt_arm6.xml")
    ap.add_argument("--q", default="0,0,0,0,0,0")
    ap.add_argument("--out", default="media/mujoco_zero.png")
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=860)
    ap.add_argument("--azim", type=float, default=135.0)
    ap.add_argument("--elev", type=float, default=-12.0)
    ap.add_argument("--dist", type=float, default=4.2)
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    d.qpos[:] = np.radians([float(v) for v in args.q.split(",")])
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.55, 0.0, 1.05]
    cam.distance = args.dist
    cam.azimuth = args.azim
    cam.elevation = args.elev
    opt = mujoco.MjvOption()

    r = mujoco.Renderer(m, args.h, args.w)
    r.update_scene(d, camera=cam, scene_option=opt)
    img = r.render()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    try:
        from PIL import Image
        Image.fromarray(img).save(args.out)
    except Exception:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.imsave(args.out, img)
    print("已保存 %s  (%dx%d)" % (args.out, args.w, args.h))


if __name__ == "__main__":
    main()
