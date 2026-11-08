# -*- coding: utf-8 -*-
"""点动（jog）诊断第二步：换模式试。

假设：mode 0=示教 时，手动点动指令只认示教器，不认 SDK。
      mode 1=远程 / 2=运行 下，SDK 的 robot_start_jogging 才被接受。

安全：只点 J6（TCP 在轴 6 上，末端不动），脉冲 0.8 秒，位移超 3 度立刻停；
      每个模式试完都恢复 mode 0；退出前 stop_jogging + queue_stop + 下电。
"""
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
MODE = {0: "示教", 1: "远程", 2: "运行"}


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


def J(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return list(p)


def power_on(fd, mode=0):
    nrc.set_current_mode(fd, mode)
    time.sleep(0.3)
    nrc.clear_error(fd)
    if S(fd) == 3:
        return True
    if S(fd) != 0:
        nrc.set_servo_state(fd, 0)
        time.sleep(0.4)
    nrc.set_servo_state(fd, 1)
    time.sleep(0.6)
    for i in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def jog_pulse(fd, axis=5, direction=True, seconds=0.8, limit=3.0):
    q0 = J(fd)
    r1 = nrc.robot_start_jogging(fd, axis, direction)
    t0 = time.time()
    peak = 0.0
    while time.time() - t0 < seconds:
        time.sleep(0.08)
        q = J(fd)
        peak = max(peak, abs(q[axis] - q0[axis]))
        if peak > limit:
            break
    r2 = nrc.robot_stop_jogging(fd, axis)
    time.sleep(0.3)
    return r1, r2, J(fd)[axis] - q0[axis]


fd = nrc.connect_robot("192.168.2.14", "6001")
if fd <= 0:
    raise SystemExit("连接失败")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)
print("已连接 %s" % time.strftime("%H:%M:%S"))

try:
    if not power_on(fd):
        raise SystemExit("上电失败")
    nrc.set_speed(fd, 50)
    print("上电成功（mode 0 示教），q6 = %.3f" % J(fd)[5])

    for mode in (1, 2):
        print("\n=== 切到 mode %d (%s) ===" % (mode, MODE[mode]))
        r = nrc.set_current_mode(fd, mode)
        print("  set_current_mode ret=%s   伺服 = %s" % (r, SERVO.get(S(fd), S(fd))))
        time.sleep(0.8)
        if S(fd) != 3:
            print("  伺服掉了，重新上电（在该模式下）")
            if not power_on(fd, mode):
                print("  该模式下上不了电，跳过")
                nrc.set_current_mode(fd, 0)
                continue
        nrc.set_speed(fd, 50)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, True)
        time.sleep(0.3)

        for ax in (5,):
            for d in (True, False):
                r1, r2, dq = jog_pulse(fd, ax, d)
                print("  axis=%d dir=%s  start=%s stop=%s  Δq6=%+.4f 度"
                      % (ax, d, r1, r2, dq))
        # 恢复
        try:
            nrc.set_current_mode(fd, 0)
            time.sleep(0.5)
        except Exception:
            pass
        print("  已恢复 mode 0，伺服 = %s" % SERVO.get(S(fd), S(fd)))
        if S(fd) != 3:
            print("  恢复后伺服掉了，重新上电")
            power_on(fd)
finally:
    for ax in range(8):
        try:
            nrc.robot_stop_jogging(fd, ax)
        except Exception:
            pass
    try:
        nrc.set_current_mode(fd, 0)
        time.sleep(0.3)
    except Exception:
        pass
    try:
        nrc.queue_motion_stop(fd)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, False)
    except Exception:
        pass
    if S(fd) in (2, 3):
        nrc.set_servo_poweroff(fd)
        time.sleep(0.4)
    if S(fd) != 0:
        nrc.set_servo_state(fd, 0)
    nrc.disconnect_robot(fd)
    print("\n已恢复 mode 0 并下电断开 %s" % time.strftime("%H:%M:%S"))
