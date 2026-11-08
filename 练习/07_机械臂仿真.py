# 07 机械臂仿真 —— 在三维里看懂「关节角 → 手臂姿态 → 末端位置」
#
# 不需要厂家的 URDF，PyBullet 自带 KUKA iiwa（7 轴，真机大概率是 6 轴，原理一样）。
#
# 跑法（一行，从任何目录都能跑）：
#   只看数字（无窗口，稳）：
#     ~/miniforge3/envs/dl/bin/python ~/projects/07_机器人学/练习/07_机械臂仿真.py
#   看窗口里的机械臂：
#     ~/miniforge3/envs/dl/bin/python ~/projects/07_机器人学/练习/07_机械臂仿真.py --gui
#
# 嫌长就先 cd 进 练习 目录，激活 dl 后：python 07_机械臂仿真.py --gui
#
# 环境：dl（Python 3.12.15，已装 pybullet 3.2.7，和 07 项目其它练习统一）

#%% 单元格 0｜导入 + 自检
import math
import sys
import time

import pybullet as p
import pybullet_data

USE_GUI = "--gui" in sys.argv
AUTO = "--auto" in sys.argv
cid = p.connect(p.GUI if USE_GUI else p.DIRECT)
print("pybullet 模式:", "GUI（有窗口）" if USE_GUI else "DIRECT（只有数字）")


#%% 单元格 1｜加载模型，看 URDF 里到底有什么
p.setAdditionalSearchPath(pybullet_data.getDataPath())
arm = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
n = p.getNumJoints(arm)

if USE_GUI:
    # 关掉侧边那三个相机预览面板（RGB / Depth / Segmentation）。
    # 它们显示的是 getCameraImage() 最后一次的渲染结果，
    # 这里没接相机，所以一直是空白 —— 关掉省地方。
    # 将来做视觉（看墙面、找工件）时再打开，那时每帧调 getCameraImage。
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

print(f"\n这个模型有 {n} 个关节：")
for i in range(n):
    info = p.getJointInfo(arm, i)
    name = info[1].decode()
    lo, hi = info[8], info[9]
    print(f"  {i}: {name:16s} 限位 [{lo:+.2f}, {hi:+.2f}] rad")

print("\n注意：真机一般 6 轴，这个模型 7 轴。README 里会写清每个 joint 的名字和限位——")
print("     你明天要的 URDF，打开也是长这样。")


#%% 单元格 2｜只动一个关节，看末端怎么走
print("\n只动第 2 个关节（索引 1），其余不动：")
for angle in (0.0, 0.3, 0.6, 0.9):
    p.resetJointState(arm, 1, angle)
    pos = p.getLinkState(arm, n - 1)[0]
    print(f"  关节2 = {angle:.1f} rad → 末端 xyz = "
          f"({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")

print("\n一句话：关节角是『怎么弯』，末端位置是『弯完之后手在哪』。")
print("        这就是正运动学——你课上推的 FK，这里一行 getLinkState 就给你了。")


#%% 单元格 3｜自己操控它
#
# 带 --gui 跑，会出现七个滑块：拖滑块 = 改那个关节的角度，机械臂实时跟着动。
# 终端每秒打印一次末端位置，边拖边看，能对上「关节角 → 手在哪」。
#
# 不想手动，就加 --auto，让它自己按正弦摆动。

if USE_GUI and not AUTO:
    # ---- 手动模式：七个滑块 ----
    sliders = []
    for i in range(n):
        info = p.getJointInfo(arm, i)
        lo, hi = info[8], info[9]
        lo, hi = max(lo, -3.0), min(hi, 3.0)          # 限位太大不好拖，收窄一点
        sliders.append(p.addUserDebugParameter(f"joint_{i + 1}", lo, hi, 0.0))

    print("\n拖动左侧滑块控制关节。关节名后面显示的是当前角度（rad）。")
    print("关窗口 或 Ctrl+C 结束。\n")

    frame = 0
    try:
        while p.isConnected():
            for i in range(n):
                q = p.readUserDebugParameter(sliders[i])
                p.resetJointState(arm, i, q)
            p.stepSimulation()
            time.sleep(1 / 60)
            frame += 1
            if frame % 60 == 0:                        # 每秒打一次
                pos = p.getLinkState(arm, n - 1)[0]
                print(f"  末端 xyz = ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")
    except KeyboardInterrupt:
        print("收到 Ctrl+C")

else:
    # ---- 自动模式 / 无窗口模式：正弦摆动 ----
    print("\n开始摆动 —— 关窗口 或 Ctrl+C 结束")
    try:
        t = 0
        while True:
            for i in range(n):
                p.resetJointState(arm, i, 0.5 * math.sin(0.05 * t + i * 0.6))
            if USE_GUI:
                p.stepSimulation()
                time.sleep(1 / 60)
                if not p.isConnected():
                    break
            else:
                if t % 10 == 0:
                    pos = p.getLinkState(arm, n - 1)[0]
                    print(f"  t={t:3d}  末端 xyz = ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})")
                if t >= 30:
                    break
            t += 1
    except KeyboardInterrupt:
        print("收到 Ctrl+C")

p.disconnect()
print("结束。")
