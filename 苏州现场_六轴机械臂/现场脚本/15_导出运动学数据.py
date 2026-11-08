# -*- coding: utf-8 -*-
"""导出 URDF 需要的全部运动学数据（压测结束后再跑）。

产出：
  1. DH 参数原始字段
  2. 各关节参数（软限位 / 方向 / 减速比 / 额定速度）
  3. 工具坐标系参数
  4. 若干姿态下的 (关节角, TCP) 成对样本 —— 用来校验 URDF 的正运动学

用法：
  python 15_导出运动学数据.py
  结果写到 /tmp/kin_data.txt，同时打印到屏幕
"""
import time

import nrc_interface as nrc

OUT = []


def p(s=""):
    print(s)
    OUT.append(str(s))


fd = nrc.connect_robot("192.168.2.14", "6001")
if fd <= 0:
    raise SystemExit("连接失败")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)

p("""# 机械臂运动学数据导出
# 控制器 192.168.2.14  SDK %s
""" % nrc.get_library_version())

# ---------- 1. DH ----------
p("## 1. get_robot_dh_param 全部字段")
d = nrc.RobotDHParam()
r = nrc.get_robot_dh_param(fd, d)
p("ret = %d" % r)
for a in dir(d):
    if a.startswith("_"):
        continue
    v = getattr(d, a)
    if callable(v):
        continue
    p("  %-20s = %s" % (a, v))
p()

# ---------- 2. 关节参数 ----------
p("## 2. get_robot_joint_param（注意 id 从 1 开始）")
for i in range(1, 8):
    jp = nrc.RobotJointParam()
    r = nrc.get_robot_joint_param(fd, i, jp)
    if r != nrc.SUCCESS:
        p("  id=%d 读取失败 ret=%d" % (i, r))
        continue
    p("  id=%d  negSW=%.3f posSW=%.3f  direction=%s  reducRatio=%s"
      % (i, jp.negSWLimit, jp.posSWLimit, jp.direction, jp.reducRatio))
    p("        ratedVel=%s maxVel=%s ratedRotSpeed=%s maxRotSpeed=%s"
      % (jp.ratedVel, jp.maxVel if hasattr(jp, "maxVel") else "-",
         jp.ratedRotSpeed, jp.maxRotSpeed))
p()

# ---------- 3. 工具 ----------
p("## 3. 工具坐标系")
num = nrc.VectorInt([0]) if False else None
for tool in (1, 2, 3):
    tp = nrc.ToolParam()
    r = nrc.get_tool_hand_param(fd, tool, tp)
    if r != nrc.SUCCESS:
        p("  tool=%d 读取失败 ret=%d" % (tool, r))
        continue
    p("  tool=%d 字段: %s" % (tool, [a for a in dir(tp) if not a.startswith("_")]))
    for a in dir(tp):
        if a.startswith("_"):
            continue
        v = getattr(tp, a)
        if callable(v):
            continue
        p("      %-16s = %s" % (a, v))
p()

# ---------- 4. 姿态样本 ----------
p("## 4. (关节角, TCP) 样本 —— 用来校验 URDF 正运动学")


def sample(tag):
    q = nrc.VectorDouble()
    x = nrc.VectorDouble()
    r1 = nrc.get_current_position(fd, 0, q)
    r2 = nrc.get_current_position(fd, 1, x)
    if r1 == nrc.SUCCESS and r2 == nrc.SUCCESS:
        p("%s" % tag)
        p("  q = [%s]" % ", ".join("%.6f" % v for v in q))
        p("  x = [%s]" % ", ".join("%.6f" % v for v in x))
        return True
    p("%s 读取失败 %d %d" % (tag, r1, r2))
    return False


sample("样本0 当前姿态")
p()
p("（要更多样本，让臂换几个姿态后再跑一次本脚本，把样本追加进来）")

with open("/tmp/kin_data.txt", "w", encoding="utf-8") as f:
    f.write("\n".join(OUT))
print("\n已写入 /tmp/kin_data.txt")
nrc.disconnect_robot(fd)
