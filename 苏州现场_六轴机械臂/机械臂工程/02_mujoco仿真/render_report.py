# -*- coding: utf-8 -*-
"""给报告用的渲染图。

输出：
  media/report_hero.png    场景主图（含墙面、材质、阴影）
  media/report_views.png   三视角对比（零位 / 工作位姿）
"""
import os
import sys

import mujoco
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

XML = "nbt_arm6.xml"
W, H = 1280, 860


def shoot(m, q, azim, elev, dist=3.2, look=(0.55, 0.0, 1.12), w=W, h=H):
    d = mujoco.MjData(m)
    d.qpos[:6] = np.radians(q)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = look
    cam.distance = dist
    cam.azimuth = azim
    cam.elevation = elev
    r = mujoco.Renderer(m, h, w)
    r.update_scene(d, camera=cam, scene_option=mujoco.MjvOption())
    img = r.render()
    r.close()
    return img


def save(img, path):
    from PIL import Image
    Image.fromarray(img).save(path)
    print("已保存", path, img.shape)


m = mujoco.MjModel.from_xml_path(XML)
print("场景:", XML, " nq =", m.nq)

# 主图：工作位姿，斜侧 45°
hero = shoot(m, [0, -10, 22, 0, -18, 0], azim=128, elev=-7, dist=3.1)
save(hero, "media/report_hero.png")

# 三视角
POSES = [
    ("零位（校准基准）", [0, 0, 0, 0, 0, 0]),
    ("作业位姿", [0, -10, 22, 0, -18, 0]),
]
VIEWS = [("正视", 90.0, -4.0), ("斜侧 45°", 128.0, -7.0), ("俯视", 90.0, -42.0)]
tiles = []
for name, q in POSES:
    row = [shoot(m, q, a, e, w=640, h=430) for _, a, e in VIEWS]
    tiles.append((name, row))

fig, axes = plt.subplots(2, 3, figsize=(11.2, 5.9), dpi=120)
for i, (name, row) in enumerate(tiles):
    for j, img in enumerate(row):
        ax = axes[i][j]
        ax.imshow(img)
        ax.set_xticks([]); ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color("#dddddd")
        if i == 0:
            ax.set_title(VIEWS[j][0], fontsize=13, color="#555555")
        if j == 0:
            ax.set_ylabel(name, fontsize=14, labelpad=8)
fig.suptitle("纳博特六轴机械臂 · 仿真模型（MuJoCo，含 336.5 mm 安装垫高件）", fontsize=16)
fig.tight_layout(rect=[0.01, 0, 1, 0.95])
fig.savefig("media/report_views.png")
print("已保存 media/report_views.png")
