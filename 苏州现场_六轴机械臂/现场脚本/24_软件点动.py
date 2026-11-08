# 24 软件点动 —— 用 moveJ 自己做点动，绕开没实现的 robot_start_jogging
#
# 背景：2026-11-06 实测，SDK 的 robot_start_jogging 在 rtl105-22.07 固件里
#       返回 0 但机械臂零位移（示教器已连、使能开关已按、六种组合都试过）。
#       但 moveJ 这条链路非常稳（50 轮 2300 段连续轨迹零故障、回位 14μm），
#       所以直接用 moveJ 做「按一下走一步」的点动来替代。
#
# 用法（必须在真终端里跑，要读键盘）：
#   ~/miniforge3/envs/arm38/bin/python 24_软件点动.py --go
#
# 按键：
#   1 2 3 4 5 6   选择要动的轴
#   +  或  =      该轴正向走一步
#   -  或  _      该轴负向走一步
#   [  /  ]       步长减半 / 加倍（0.02° ~ 2.0°，默认 0.2°）
#   p             打印当前关节角和 TCP
#   q             退出
#
# 安全：每步都是一条独立 moveJ，走完等它到位才接受下一次按键；
#       步长小、速度慢；退出时一定下电。

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
STEP_MIN, STEP_MAX = 0.02, 2.0


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


def J(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return list(p)


def T(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 1, p)
    return list(p)


def power_on(fd):
    nrc.set_current_mode(fd, 0)
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


def movej_step(fd, axis, delta_deg, speed=20, timeout=8.0):
    q = J(fd)
    for i in range(6):
        if abs(q[i]) > 355:
            print("    !! 第 %d 轴接近 ±360 上限，注意" % (i + 1))
    q6 = list(q[:6])
    q6[axis] += delta_deg
    tgt = q6 + [0.0]
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(tgt)
    cmd.velocity = speed
    cmd.acc = 20
    cmd.dec = 20
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        return False
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        return False
    t0 = time.time()
    while time.time() - t0 < timeout:
        if S(fd) != 3:
            return False
        cur = J(fd)
        if max(abs(cur[i] - tgt[i]) for i in range(6)) < 0.02:
            return True
        time.sleep(0.05)
    return False


def get_key(timeout=0.1):
    """非阻塞读一个键"""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--speed", type=int, default=20, help="moveJ 速度参数")
    ap.add_argument("--step", type=float, default=0.2, help="初始步长（度）")
    args = ap.parse_args()

    print("软件点动 —— 用 moveJ 做增量，绕开没实现的 robot_start_jogging")
    print("按键：1-6 选轴 / + 正向 / - 负向 / [ ] 调步长 / p 打印 / q 退出")
    print("初始步长 %.2f 度，moveJ 速度 %d" % (args.step, args.speed))
    if not args.go:
        print("\n（预演。加 --go 真跑，需要在真终端里跑才能读键盘）")
        return

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s" % time.strftime("%H:%M:%S"))

    old_attr = None
    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        nrc.set_speed(fd, 60)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, True)
        print("上电成功，q = %s" % ["%.3f" % v for v in J(fd)[:6]])
        print("\n选中轴 [1]，步长 %.2f°。按 q 退出。\n" % args.step)

        axis = 0
        step = max(STEP_MIN, min(STEP_MAX, args.step))
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        while True:
            sys.stdout.write("\r轴 J%d   步长 %.2f°   > " % (axis + 1, step))
            sys.stdout.flush()
            k = get_key(0.2)
            if k is None:
                continue
            if k in "123456":
                axis = int(k) - 1
                continue
            if k == "q":
                print("\n退出")
                break
            if k == "p":
                print("\n  q   = %s" % ["%.3f" % v for v in J(fd)[:6]])
                print("  TCP = %s" % ["%.3f" % v for v in T(fd)[:6]])
                continue
            if k == "[":
                step = max(STEP_MIN, step / 2)
                continue
            if k == "]":
                step = min(STEP_MAX, step * 2)
                continue
            if k in "+=":
                sign = 1.0
            elif k in "-_":
                sign = -1.0
            else:
                continue
            ok = movej_step(fd, axis, sign * step, speed=args.speed)
            q = J(fd)
            print("\n  %s J%d %+0.2f°  ->  q%d = %.3f   %s"
                  % ("✓" if ok else "✗", axis + 1, sign * step, axis + 1, q[axis],
                     "" if ok else "没走成"))
    finally:
        if old_attr is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
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
        print("\n已下电断开 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    sys.exit(main())
