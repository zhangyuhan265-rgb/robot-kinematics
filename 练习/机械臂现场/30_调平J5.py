# 30 调平 J5 —— 一小步一小步动 J5，让法兰面变水平（配合水平仪标 J5 零位）
#
# 为什么要这样标：
#   模型说法兰朝下 45°，真机朝上 —— 说明 J5 的零位偏了。
#   "法兰面水平"是唯一一个不依赖任何长度假设的客观条件，
#   用它来定零位，比量距离可靠得多。
#
# 每次只动 J5，动完立刻退出，等现场看水平仪再说下一步。
#
# 跑法：~/miniforge3/envs/arm38/bin/python 30_调平J5.py --dj5 15
#       ~/miniforge3/envs/arm38/bin/python 30_调平J5.py --set5 -5

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JOINT = 4          # 0-based，J5
CAP = 40.0         # 单次最多动 40 度


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
                print("  上电成功（第 %d 次）" % (i + 1))
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def movej(fd, target, speed, limit=90.0):
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        return False
    if nrc.queue_motion_send_to_controller(fd, 1) != nrc.SUCCESS:
        return False
    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < limit:
        if S(fd) != 3:
            return False
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > 5.0:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.06:
            return True
        time.sleep(0.08)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dj5", type=float, default=None, help="J5 相对当前转多少度")
    ap.add_argument("--set5", type=float, default=None, help="J5 直接设成多少度")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--speed", type=int, default=5)
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s  伺服=%s" % (time.strftime("%H:%M:%S"), SERVO.get(S(fd), S(fd))))

    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        time.sleep(0.5)
        q0 = J(fd)
        x0 = T(fd)
        print("[转前] q5 = %.3f°   q4 = %.3f°" % (q0[JOINT], q0[3]))
        print("       控制器报 TCP = (%.1f, %.1f, %.1f)" % (x0[0], x0[1], x0[2]))

        if args.set5 is not None:
            new5 = args.set5
        elif args.dj5 is not None:
            new5 = q0[JOINT] + args.dj5
        else:
            raise SystemExit("要给 --dj5 或 --set5")

        delta = new5 - q0[JOINT]
        if abs(delta) > CAP:
            raise SystemExit("单次 %.1f 度超过上限 %.0f 度，拆成几步" % (abs(delta), CAP))
        print("J5: %.3f -> %.3f  （转 %+.1f 度，速度 %d%%）"
              % (q0[JOINT], new5, delta, args.speed))

        if not args.go:
            print("\n（预演，没动。确认后加 --go）")
            return

        tgt = list(q0[:6]) + [0.0]
        tgt[JOINT] = new5
        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)
        if not movej(fd, tgt, args.speed):
            raise SystemExit("J5 没转成")

        time.sleep(0.8)
        q1 = J(fd)
        x1 = T(fd)
        print("\n[转后] q5 = %.3f°（差 %.3f）  q4 = %.3f°"
              % (q1[JOINT], q1[JOINT] - new5, q1[3]))
        print("       控制器报 TCP = (%.1f, %.1f, %.1f)" % (x1[0], x1[1], x1[2]))
        print("\n现在看水平仪：气泡离中间还差多少？法兰还朝上还是朝下了？")
    finally:
        print("保持上电中。")


if __name__ == "__main__":
    main()
