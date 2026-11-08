# -*- coding: utf-8 -*-
"""机械臂仿真工具箱 —— 一键启动器（双击 .bat 打开这个）。

窗口上一排按钮，点一下就在新控制台里跑对应命令，不用记命令行。
"""
import os
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox, scrolledtext

BASE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable

# (按钮文字, 命令行参数, 一行说明)
ACTIONS = [
    ("★★ 控制面板（滑块+实时3D）", ["arm_panel.py", "--wall", "1.10"], "最推荐：拖滑块机械臂立刻动，右边是 MuJoCo 实时画面，实时算墙间隙。"),
    ("★ 自由拖动（无重力）", ["mujoco_play.py"], "双击连杆选中 → Ctrl+左键拖，机械臂跟着鼠标走。拖到哪停到哪。"),
    ("★ 自由拖动（有重力）", ["mujoco_play.py", "--gravity", "9.81"], "同上，但有重力：松手会往下坠，能感受负载和惯性。"),
    ("① 3D 查看器（带材质）", ["mujoco_view.py", "--urdf", "nbt_arm6.xml"],
     "真实 3D 窗口，鼠标转视角起。零位姿态。"),
    ("② 3D 查看器（J4 摆动）", ["mujoco_view.py", "--urdf", "nbt_arm6.xml",
                               "--sweep", "4", "--amp", "60"],
     "J4 来回摆，看关节怎么动。"),
    ("③ 3D 查看器（含墙面）", ["mujoco_view.py", "--urdf", "nbt_arm6_wall.xml"],
     "墙在 x=1.10 m，能看出臂和墙的空间关系。"),
    ("④ 滑块查看器（算墙间隙）", ["view_arm.py", "--wall", "1.10"],
     "拖动 6 个滑块改关节角；右侧实时显示 TCP 和到墙的间隙。"),
    ("⑤ 一键出图（4 张）", None, "生成零位/q5=90 的 MJCF 图和 URDF 三视图，存到 media/。"),
    ("⑥ 跑训练环境", ["arm_env.py"], "跑 5 个 episode，显示碰撞判定和奖励。"),
    ("⑦ 重新生成 MJCF", None, "改了 URDF 之后点这个（含墙面版本一起生成）。"),
    ("⑧ 打开输出文件夹", None, "打开 media/ 看生成的图。"),
]

FIGS = [
    ["mujoco_render.py", "--out", "media/mujoco_zero.png"],
    ["mujoco_render.py", "--q", "0,0,0,0,90,0", "--azim", "150",
     "--elev", "-16", "--dist", "2.9", "--out", "media/mujoco_q5_90.png"],
    ["render_urdf.py", "--q", "0,0,0,0,0,0", "--out", "media/urdf_zero_pose.png"],
    ["render_urdf.py", "--q", "0,0,0,0,90,0", "--out", "media/urdf_q5_90.png"],
]

REGEN = [
    ["make_mjcf.py", "--out", "nbt_arm6.xml"],
    ["make_mjcf.py", "--wall", "1.10", "--out", "nbt_arm6_wall.xml"],
]


class App:
    def __init__(self, root):
        self.root = root
        root.title("机械臂仿真工具箱  -  nbt_arm6")
        root.geometry("720x600")

        hdr = tk.Label(root, text="六轴机械臂仿真 / 可视化工具箱",
                       font=("Microsoft YaHei UI", 13, "bold"))
        hdr.pack(pady=(10, 2))
        tk.Label(root, text="点按钮即可，所有命令都会在新控制台里跑（关掉那个黑窗口就停）",
                 fg="#555").pack(pady=(0, 8))

        self.status = tk.Label(root, text=self._env_status(), font=("Consolas", 9),
                               fg="#1a6b32", justify="left")
        self.status.pack(pady=(0, 8))

        frm = tk.Frame(root)
        frm.pack(fill="x", padx=12)
        frm.columnconfigure(0, weight=1, uniform="cell")
        frm.columnconfigure(1, weight=1, uniform="cell")
        self.buttons = []
        for i, (label, args, tip) in enumerate(ACTIONS):
            row, col = i // 2, i % 2
            cell = tk.Frame(frm)
            cell.grid(row=row, column=col, padx=6, pady=6, sticky="nsew")
            b = tk.Button(cell, text=label, height=2,
                          font=("Microsoft YaHei UI", 10),
                          command=lambda a=args, l=label: self.on_click(a, l))
            b.pack(fill="x")
            tk.Label(cell, text=tip, fg="#666", font=("Microsoft YaHei UI", 8),
                     anchor="w", justify="left", wraplength=300).pack(fill="x")
            self.buttons.append(b)

        tk.Label(root, text="运行日志：", anchor="w").pack(fill="x", padx=16, pady=(10, 0))
        self.log = scrolledtext.ScrolledText(root, height=9, font=("Consolas", 9))
        self.log.pack(fill="both", expand=True, padx=14, pady=(2, 12))
        self._w("就绪。所需文件都在：%s" % BASE)

    # ---------- 工具 ----------
    def _env_status(self):
        parts = []
        try:
            import mujoco
            parts.append("MuJoCo %s" % mujoco.__version__)
        except Exception:
            parts.append("MuJoCo 未安装（3D 功能不可用）")
        miss = [f for f in ("nbt_arm6_on_riser.urdf", "mujoco_view.py")
                if not os.path.exists(os.path.join(BASE, f))]
        parts.append("缺少文件：%s" % (", ".join(miss) if miss else "无"))
        return "   ".join(parts)

    def _w(self, s):
        self.log.insert("end", s + "\n")
        self.log.see("end")

    def _run(self, args, label):
        script = os.path.join(BASE, args[0])
        if not os.path.exists(script):
            messagebox.showerror("文件不存在", script)
            return
        cmd = [PY, script] + args[1:]
        self._w("[%s] -> %s" % (label, " ".join(args)))
        try:
            subprocess.Popen(cmd, cwd=BASE,
                             creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0))
        except Exception as e:
            self._w("  启动失败：%s" % e)
            messagebox.showerror("启动失败", str(e))

    # ---------- 按钮 ----------
    def on_click(self, args, label):
        if label.startswith("⑤"):
            self._seq(FIGS, "出图")
        elif label.startswith("⑦"):
            self._seq(REGEN, "重新生成 MJCF")
        elif label.startswith("⑧"):
            p = os.path.join(BASE, "media")
            os.makedirs(p, exist_ok=True)
            os.startfile(p)
            self._w("已打开 %s" % p)
        else:
            self._run(args, label)

    def _seq(self, seq, label):
        for args in seq:
            self._run(args, label)


def main():
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
