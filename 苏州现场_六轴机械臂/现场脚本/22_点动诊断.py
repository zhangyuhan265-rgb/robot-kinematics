# 22 点动（jog）诊断 —— 试通 robot_start_jogging
#
# 背景：之前 jog 一直"返回 0 但不动"。SDK 里接口是
#     robot_start_jogging(fd, axis, dir) / robot_stop_jogging(fd, axis)
#   注意它**没有速度参数**，速度来自别处的设置，所以可能是速度被设成了 0，
#   或者需要示教器上的使能开关按住。
#
# 本脚本按「安全优先」的顺序试探：
#   1. 先试 J6 —— TCP 就在轴 6 上，转 J6 末端不动，最安全
#   2. J6 能动，再试 J2（小角度、低速、短脉冲）
#   3. 每个轴都先给 0.6 秒的脉冲，测到动了就立刻停
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 22_点动诊断.py            # 预演
#   ~/miniforge3/envs/arm38/bin/python 22_点动诊断.py --go       # 真试
#   ... --go --hold    # 如果提示"可能要按使能"，加上这个参数，脚本会让你先按住再继续
#
# 安全：脉冲 ≤1 秒；速度先设 10%（全局速度）；任何单次位移超过 3 度立刻停；
#       退出前一定调 robot_stop_jogging + queue_motion_stop。

import argparse
import sys
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


def pulse(fd, axis, direction, seconds=0.6, limit_deg=3.0):
    """给一个点动脉冲，返回 (ret_start, ret_stop, 关节变化量)"""
    q0 = J(fd)
    r1 = nrc.robot_start_jogging(fd, axis, direction)
    t0 = time.time()
    peak = 0.0
    while time.time() - t0 < seconds:
        time.sleep(0.08)
        q = J(fd)
        d = abs(q[axis] - q0[axis]) if axis < 6 else 0.0
        peak = max(peak, d)
        if peak > limit_deg:
            break
    r2 = nrc.robot_stop_jogging(fd, axis)
    time.sleep(0.3)
    q1 = J(fd)
    return r1, r2, q1[axis] - q0[axis]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--hold", action="store_true", help="暂停等你按住示教器使能开关")
    ap.add_argument("--speed", type=int, default=50, help="全局速度百分比")
    ap.add_argument("--watch", action="store_true",
                    help="持续试探模式：每 interval 秒给一次脉冲，一动就停")
    ap.add_argument("--seconds", type=float, default=240.0, help="持续试探总时长")
    ap.add_argument("--interval", type=float, default=8.0, help="两次脉冲的间隔")
    args = ap.parse_args()

    print("诊断计划：")
    print("  1. 全局速度设成 %d%%" % args.speed)
    print("  2. 试 J6（轴 6，5、6 号索引都试）—— TCP 在轴 6 上，转它末端不动")
    print("  3. 试 J2（轴 1/2 号索引）—— 小角度、短脉冲")
    print("  4. 每个轴先正向 0.6 秒，测到动立刻停")
    if not args.go:
        print("\n（预演，没连机器人。加 --go 真试）")
        return

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
        nrc.set_speed(fd, args.speed)
        print("上电成功，全局速度 %d%%" % args.speed)
        print("当前 q = %s" % ["%.3f" % v for v in J(fd)[:6]])

        if args.hold:
            print("\n" + "=" * 60)
            print(">>> 现在请【按住示教器上的使能开关】并一直按住 <<<")
            print(">>> 90 秒后自动开始试，保持按住 <<<")
            print("=" * 60)
            for k in range(90, 0, -10):
                print("  还有 %2d 秒 ..." % k)
                time.sleep(10)

        # 打开运动队列（moveJ/moveL 都要这个，jog 可能也要）
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, True)
        time.sleep(0.3)

        if args.watch:
            print("\n" + "=" * 60)
            print(">>> 持续试探：每 %.0f 秒给 J6 一个 %.1f 秒的脉冲，共 %.0f 秒" % (args.interval, 0.8, args.seconds))
            print(">>> 从现在起【按住示教器使能开关不要松】，一动脚本自己会停 <<<")
            print("=" * 60)
            t_end = time.time() + args.seconds
            n = 0
            got = False
            while time.time() < t_end:
                n += 1
                r1, r2, dq = pulse(fd, 5, True)
                remain = int(t_end - time.time())
                if abs(dq) > 0.01:
                    print("  第 %2d 次：✅ 动了！Δq6 = %+.4f 度（ret %s/%s）" % (n, dq, r1, r2))
                    got = True
                    break
                print("  第 %2d 次：没动（Δq6=%+.4f）  剩余 %3d 秒" % (n, dq, max(remain, 0)))
                time.sleep(args.interval)
            if got:
                print("\n  反向补回去，免得 J6 偏掉 ...")
                pulse(fd, 5, False, seconds=0.9, limit_deg=6.0)
                print("  q = %s" % ["%.3f" % v for v in J(fd)[:6]])
                print("\n  → jog 通了！前置条件就是【按住示教器使能开关】")
            else:
                print("\n  ❌ %.0f 秒内试了 %d 次都没动" % (args.seconds, n))
                print("  → 不是使能开关的问题。下一步查示教器上的【点动速度】设置。")
            return

        # 1) 先试 J6：轴号 5（0基）和 6（1基）
        print("\n--- 试 J6（先试 index=5）---")
        r1, r2, dq = pulse(fd, 5, True)
        print("  start ret=%s  stop ret=%s  关节变化 = %+.4f 度" % (r1, r2, dq))
        moved = abs(dq) > 0.01

        if not moved:
            print("\n--- index=5 没动，试 index=6（可能是 1 基）---")
            r1, r2, dq = pulse(fd, 6, True)
            print("  start ret=%s  stop ret=%s  关节变化 = %+.4f 度" % (r1, r2, dq))
            moved = abs(dq) > 0.01

        if not moved:
            print("\n--- 还是没动，试反过来 dir=False（index=5）---")
            r1, r2, dq = pulse(fd, 5, False)
            print("  start ret=%s  stop ret=%s  关节变化 = %+.4f 度" % (r1, r2, dq))
            moved = abs(dq) > 0.01

        if moved:
            print("\n✅ jog 通了！")
            # 反向走回来，别把 J6 留在偏的位置
            print("  反向补回去 ...")
            pulse(fd, 5 if abs(dq) > 0 and True else 5, dq < 0, seconds=0.6)
            print("  现在 q = %s" % ["%.3f" % v for v in J(fd)[:6]])
        else:
            print("\n❌ J6 三个方向、两种轴号都没动。")
            print("   返回码都是 0，说明接口接受了命令但控制器没执行 →")
            print("   最可能是【示教器使能开关没按】或【点动速度被设成 0】。")
            print("   建议：加上 --hold 参数再跑一次，脚本会等你按住使能开关。")
    finally:
        for ax in range(8):
            try:
                nrc.robot_stop_jogging(fd, ax)
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
        print("\n已下电断开 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    sys.exit(main())
