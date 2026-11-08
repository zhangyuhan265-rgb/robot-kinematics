# 33 单轴移动 —— 指定某个轴转多少度（其余轴不动）
#
# 跑法：
#   python 33_单轴移动.py --joint 3 --delta 20 --go      # J3 转 +20 度
#   python 33_单轴移动.py --joint 3 --set 0 --go         # J3 设成 0

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}


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
    for _ in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
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
    ap.add_argument("--joint", type=int, required=True, help="1..6")
    ap.add_argument("--delta", type=float, default=None)
    ap.add_argument("--set", type=float, default=None)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--speed", type=int, default=8)
    args = ap.parse_args()

    idx = args.joint - 1
    fd = 0
    for attempt in range(8):
        fd = nrc.connect_robot("192.168.2.14", "6001")
        if fd > 0:
            break
        print("连接失败，等 12 秒重试（第 %d 次）..." % (attempt + 1))
        time.sleep(12)
    if fd <= 0:
        raise SystemExit("连接失败（重试 8 次仍失败）")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)

    try:
        ok = False
        for _ in range(3):
            if power_on(fd):
                time.sleep(1.0)
                if S(fd) == 3:
                    ok = True
                    break
            time.sleep(1.0)
        if not ok:
            raise SystemExit("上电失败（伺服到不了 3 运行）")
        time.sleep(0.5)
        q0 = J(fd)
        x0 = T(fd)
        print("转前 q = %s" % ["%.3f" % v for v in q0[:6]])
        print("     控制器 TCP = (%.1f, %.1f, %.1f)" % (x0[0], x0[1], x0[2]))

        new = args.set if args.set is not None else q0[idx] + args.delta
        print("J%d: %.3f -> %.3f  （转 %+.1f 度）" % (args.joint, q0[idx], new, new - q0[idx]))
        if abs(new - q0[idx]) > 40:
            raise SystemExit("单次超过 40 度，拆成几步")
        if not args.go:
            print("（预演，没动。加 --go 才动）")
            return

        tgt = list(q0[:6]) + [0.0]
        tgt[idx] = new
        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)
        done = False
        for attempt in range(3):
            if S(fd) != 3:
                print("  伺服掉到 %s，重新上电" % SERVO.get(S(fd), S(fd)))
                power_on(fd)
                time.sleep(0.8)
            if movej(fd, tgt, args.speed):
                done = True
                break
            print("  第 %d 次没走成，重试" % (attempt + 1))
            time.sleep(0.8)
        if not done:
            raise SystemExit("没转成（重试 3 次）")

        time.sleep(0.8)
        q1 = J(fd)
        x1 = T(fd)
        print("转后 q = %s" % ["%.3f" % v for v in q1[:6]])
        print("     J%d 实际 %.3f（差 %.3f）" % (args.joint, q1[idx], q1[idx] - new))
        print("     控制器 TCP = (%.1f, %.1f, %.1f)" % (x1[0], x1[1], x1[2]))
        print("保持上电中。")
    finally:
        pass


if __name__ == "__main__":
    main()
