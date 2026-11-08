# -*- coding: utf-8 -*-
"""机械臂控制面板 —— 6 个滑块 + MuJoCo 实时画面。

比 MuJoCo 自带窗口好在：
    * 拖动滑块机械臂立刻动，不用记"双击选中 + Ctrl 拖"这种隐藏操作
    * 用的是 MuJoCo 的渲染器（有材质/地面/灯光/阴影）
    * 画面上直接拖鼠标可以转视角
    * 实时显示 TCP 坐标、半径、高度、到墙的间隙

用法：
    python arm_panel.py                     # 无墙
    python arm_panel.py --wall 1.10         # 带墙，实时算间隙
"""
import argparse
import os
import threading
import tkinter as tk
from tkinter import ttk

import numpy as np
import mujoco

BASE = os.path.dirname(os.path.abspath(__file__))
W, H = 780, 560
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]


class Panel:
    def __init__(self, root, xml, wall=None):
        self.root = root
        self.wall = wall
        self.m = mujoco.MjModel.from_xml_path(xml)
        self.d = mujoco.MjData(self.m)
        self.r = mujoco.Renderer(self.m, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.45, 0.0, 1.02]
        self.cam.distance = 3.3
        self.cam.azimuth = 148
        self.cam.elevation = -14
        self._drag = None

        root.title("机械臂控制面板  -  nbt_arm6")
        left = tk.Frame(root)
        left.pack(side="left", fill="y", padx=10, pady=10)
        right = tk.Frame(root)
        right.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        tk.Label(left, text="关节角（度）", font=("Microsoft YaHei UI", 11, "bold")).pack(pady=(0, 6))
        self.scales = []
        for i, nm in enumerate(JOINT_NAMES):
            lim = 360 if i == 5 else 130
            row = tk.Frame(left)
            row.pack(fill="x", pady=1)
            tk.Label(row, text=nm, width=3, font=("Consolas", 10)).pack(side="left")
            s = tk.Scale(row, from_=-lim, to=lim, orient="horizontal", length=230,
                         resolution=0.5, command=lambda _v: self.update_info())
            s.set(0)
            s.pack(side="left")
            self.scales.append(s)

        btns = tk.Frame(left)
        btns.pack(fill="x", pady=(10, 4))
        tk.Button(btns, text="全部归零", width=12, command=self.reset).pack(side="left", padx=2)
        tk.Button(btns, text="存图 PNG", width=12, command=self.save_png).pack(side="left", padx=2)

        self.info = tk.Label(left, text="", justify="left", font=("Consolas", 9),
                             fg="#1a4d7a", anchor="w")
        self.info.pack(fill="x", pady=(10, 0))

        self.canvas = tk.Canvas(right, width=W, height=H, bg="#dfe4ea",
                                highlightthickness=0, cursor="fleur")
        self.canvas.pack()
        self.canvas.bind("<Button-1>", self.on_press)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", lambda e: setattr(self, "_drag", None))
        self.canvas.bind("<MouseWheel>", self.on_wheel)

        self.photo = None
        self.msg = tk.Label(right, text="在画面上按住左键拖动 = 转视角；滚轮 = 缩放",
                            fg="#666").pack(pady=(4, 0))
        self.update_info()
        self.tick()

    def q(self):
        return np.radians([s.get() for s in self.scales])

    def clearance(self):
        if self.wall is None:
            return None
        try:
            from arm_collision import CollisionModel
            if not hasattr(self, "_cm"):
                self._cm = CollisionModel(os.path.join(BASE, "nbt_arm6_on_riser.urdf"))
            dist, ln = self._cm.distance_to_plane(self.q(), [1, 0, 0], self.wall)
            return dist, ln
        except Exception:
            return None

    def update_info(self):
        self.d.qpos[:] = self.q()
        mujoco.mj_forward(self.m, self.d)
        bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "link6")
        # 法兰在 link6 坐标系的 z 上
        p = self.d.xpos[bid] + self.d.xmat[bid].reshape(3, 3) @ np.array([0, 0, 0.1325])
        txt = ("q(deg) = %s\n\n法兰 (mm): (%.1f, %.1f, %.1f)\n离 J1 轴: %.1f mm\n离地: %.1f mm"
               % (", ".join("%6.1f" % s.get() for s in self.scales),
                  p[0] * 1000, p[1] * 1000, p[2] * 1000,
                  np.hypot(p[0], p[1]) * 1000, p[2] * 1000))
        cl = self.clearance()
        if cl is not None:
            d, ln = cl
            txt += "\n\n到墙 x=%.2f: %+.1f mm\n最近: %s" % (self.wall, d * 1000, ln)
            if d < 0.05:
                txt += "  ← 危险"
        self.info.config(text=txt, fg="#b03a2e" if (cl and cl[0] < 0.05) else "#1a4d7a")

    def tick(self):
        self.d.qpos[:] = self.q()
        mujoco.mj_forward(self.m, self.d)
        self.r.update_scene(self.d, camera=self.cam)
        img = self.r.render()
        try:
            from PIL import Image, ImageTk
            self.photo = ImageTk.PhotoImage(Image.fromarray(img))
            self.canvas.delete("all")
            self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        except Exception:
            pass
        self.root.after(40, self.tick)

    def reset(self):
        for s in self.scales:
            s.set(0)
        self.update_info()

    def save_png(self):
        try:
            from PIL import Image
            self.d.qpos[:] = self.q()
            mujoco.mj_forward(self.m, self.d)
            self.r.update_scene(self.d, camera=self.cam)
            out = os.path.join(BASE, "media", "panel_shot.png")
            os.makedirs(os.path.dirname(out), exist_ok=True)
            Image.fromarray(self.r.render()).save(out)
            print("已存 %s" % out)
        except Exception as e:
            print("存图失败:", e)

    # 鼠标转视角
    def on_press(self, e):
        self._drag = (e.x, e.y, self.cam.azimuth, self.cam.elevation)

    def on_drag(self, e):
        if not self._drag:
            return
        x0, y0, az, el = self._drag
        self.cam.azimuth = az + (e.x - x0) * 0.4
        self.cam.elevation = max(-89, min(89, el - (e.y - y0) * 0.3))

    def on_wheel(self, e):
        self.cam.distance = float(np.clip(self.cam.distance * (0.92 if e.delta > 0 else 1.08),
                                          1.2, 9.0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="nbt_arm6.xml")
    ap.add_argument("--wall", type=float, default=None)
    args = ap.parse_args()

    xml = args.xml if os.path.isabs(args.xml) else os.path.join(BASE, args.xml)
    if not os.path.exists(xml):
        raise SystemExit("找不到 %s —— 先跑一次 make_mjcf.py" % xml)
    root = tk.Tk()
    Panel(root, xml, wall=args.wall)
    root.mainloop()


if __name__ == "__main__":
    main()
