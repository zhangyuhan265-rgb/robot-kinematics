# -*- coding: utf-8 -*-
"""上电故障诊断：急停按下并复位之后，为什么 poweron 一直 RECEIVE_FAILED。"""
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def S():
    return nrc.get_servo_state(fd, 0)[1]


def J():
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return [round(v, 2) for v in p[:6]]


fd = nrc.connect_robot("192.168.2.14", "6001")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)

din = nrc.VectorInt([0] * 32)
nrc.get_digital_input(fd, din)
dout = nrc.VectorInt([0] * 32)
nrc.get_digital_output(fd, dout)

print("伺服状态", S(), SERVO.get(S(), ""))
print("关节    ", J())
print("DI      ", list(din))
print("DO      ", list(dout))
print()

print("clear_error      ->", n(nrc.clear_error(fd)), " 状态", S())
time.sleep(0.5)
print("set_servo_poweroff ->", n(nrc.set_servo_poweroff(fd)), " 状态", S())
time.sleep(0.5)
print("set_servo_state(0) ->", n(nrc.set_servo_state(fd, 0)), " 状态", S())
time.sleep(0.5)
print("set_servo_state(1) ->", n(nrc.set_servo_state(fd, 1)), " 状态", S())
time.sleep(0.8)

for i in range(8):
    r = nrc.set_servo_poweron(fd)
    time.sleep(0.6)
    print("poweron 第%d次 -> %-20s 状态 %s %s" % (i + 1, n(r), S(), SERVO.get(S(), "")))
    if S() == 3:
        break

print()
print("再读一次 DI", list(din) if nrc.get_digital_input(fd, din) == 0 else "失败")
print("最终状态", S(), SERVO.get(S(), ""))
print("最终关节", J())
nrc.disconnect_robot(fd)
