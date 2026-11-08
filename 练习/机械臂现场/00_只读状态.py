# -*- coding: utf-8 -*-
"""只读状态检查：不上电、不使能、不运动。拔插示教器前后都可以跑。"""
import time

import nrc_interface as nrc

SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}

fd = nrc.connect_robot("192.168.2.14", "6001")
if fd <= 0:
    raise SystemExit("连接失败")
d = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < d:
    time.sleep(0.2)

ret, st = nrc.get_servo_state(fd, 0)
print("伺服状态   %s  %s" % (st, SERVO.get(st, "?")))

pos = nrc.VectorDouble()
print("读关节角   ret=%s" % nrc.get_current_position(fd, 0, pos))
print("           %s" % [round(v, 3) for v in pos])

pos2 = nrc.VectorDouble()
nrc.get_current_position(fd, 1, pos2)
print("读直角坐标 %s" % [round(v, 3) for v in pos2])

din = nrc.VectorInt([0] * 32)
print("读数字输入 ret=%s" % nrc.get_digital_input(fd, din))
print("           DI = %s" % list(din))

dout = nrc.VectorInt([0] * 32)
nrc.get_digital_output(fd, dout)
print("           DO = %s" % list(dout))

for i in range(6):
    p = nrc.RobotJointParam()
    r = nrc.get_robot_joint_param(fd, i, p)
    print("轴%d 软限位 ret=%d  [%s, %s] 度" % (i, r, p.negSWLimit, p.posSWLimit))

nrc.disconnect_robot(fd)
print("已断开（全程只读）")
