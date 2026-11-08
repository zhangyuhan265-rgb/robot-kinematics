# -*- coding: utf-8 -*-
"""急停回路验证：上电后持续监视伺服状态，看按下急停能不能把状态 3 打下来。

只上电、不发任何运动指令。运行时长默认 90 秒。
"""
import sys
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行(已使能)"}
CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def J(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return [round(v, 3) for v in p[:6]]


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


fd = nrc.connect_robot("192.168.2.14", "6001")
if fd <= 0:
    raise SystemExit("连接失败")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)

print("连接成功，起始状态 %s %s" % (S(fd), SERVO.get(S(fd), "")))
print("起始关节", J(fd))
sys.stdout.flush()

print("清报警 ->", n(nrc.clear_error(fd)))
nrc.set_servo_state(fd, 1)
time.sleep(0.5)
ok = False
for i in range(6):
    r = nrc.set_servo_poweron(fd)
    t0 = time.time()
    while time.time() - t0 < 3:
        if S(fd) == 3:
            ok = True
            break
        time.sleep(0.2)
    if ok:
        print("上电成功（第 %d 次 poweron=%s）状态 3" % (i + 1, n(r)))
        break
    time.sleep(0.8)

if not ok:
    print("上电失败，最后状态 %s" % S(fd))
    nrc.disconnect_robot(fd)
    raise SystemExit(1)

time.sleep(3.0)          # 等重力下沉结束
print("稳定后关节", J(fd))
print(">>> 现在请按下急停 <<<")
sys.stdout.flush()

last = S(fd)
t0 = time.time()
while time.time() - t0 < 90:
    cur = S(fd)
    if cur != last:
        print("[%6.2fs] 状态 %s -> %s %s" % (time.time() - t0, last, cur, SERVO.get(cur, "")))
        print("          关节", J(fd))
        sys.stdout.flush()
        last = cur
        if cur in (0, 2):
            print("[%6.2fs] *** 检测到急停/断使能 ***" % (time.time() - t0))
            sys.stdout.flush()
    time.sleep(0.2)

print("监视结束，最终状态 %s %s" % (S(fd), SERVO.get(S(fd), "")))
print("最终关节", J(fd))
nrc.disconnect_robot(fd)
print("已断开（未自动下电）")
