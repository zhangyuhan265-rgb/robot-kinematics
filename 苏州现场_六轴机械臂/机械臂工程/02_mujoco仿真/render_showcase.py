# -*- coding: utf-8 -*-
"""渲染展示图：零位 + 当前位姿，各三个视角。

用法：
    python render_showcase.py
输出：
    media/showcase.png
"""
import os

import mujoco
import numpy as np

XML = "nbt_arm6.xml"
OUT = "media/showcase.png"

POSES = [
    ("零位（已定型的基准位）", [0, 0, 0, 0, 0, 0]),
    ("当前实机位姿", [20, -25, 35, 60, -30, 45]),
]

VIEWS = [
    ("右侧 45°", 135.0, -12.0),
    ("正视", 90.0, -4.0),
    ("左侧 45°", 45.0, -12.0),
]

W, H = 660, 520


def shoot(m, d, azim, elev, dist=3.1):
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.38, 0.0, 1.02]
    cam.distance = dist
    cam.azimuth = azim
    cam.elevation = elev
    r = mujoco.Renderer(m, H, W)
    r.update_scene(d, camera=cam, scene_option=mujoco.MjvOption())
    img = r.render()
    r.close()
    return img


def main():
    m = mujoco.MjModel.from_xml_path(XML)
    rows = []
    for title, q in POSES:
        d = mujoco.MjData(m)
        d.qpos[:] = np.radians(q)
        mujoco.mj_forward(m, d)
        tiles = [shoot(m, d, a, e) for _, a, e in VIEWS]
        rows.append((title, q, tiles))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei",
                                             "DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False

    fig, axes = plt.subplots(2, 3, figsize=(11.0, 6.6), dpi=110)
    for i, (title, q, tiles) in enumerate(rows):
        for j, img in enumerate(tiles):
            ax = axes[i][j]
            ax.imshow(img)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_color("#cccccc")
            if i == 0:
                ax.set_title(VIEWS[j][0], fontsize=10, color="#555555")
            if j == 0:
                ax.set_ylabel("%s\nq=%s" % (title, q), fontsize=10,
                              rotation=90, labelpad=8)
    fig.suptitle("纳博特 6 轴机械臂 · 仿真模型（MuJoCo，臂长 d1=431 / a1=170 / a2=790 / a3=156 / d4=750 / d6=132.5）",
                 fontsize=11)
    fig.tight_layout(rect=[0.01, 0, 1, 0.95])
    os.makedirs("media", exist_ok=True)
    fig.savefig(OUT)
    print("已保存", OUT)


if __name__ == "__main__":
    main()
