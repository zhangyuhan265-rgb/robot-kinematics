# 29 单轴测试：把 J4 转一段，用来判断手腕零位到底错在哪
#
# 背景：home 位姿下模型说法兰朝下 45°，现场看到朝上 26°、离地高 152mm。
#       三个假设都能解释当前状态，转一下 J4 就能分开：
#
#         A  J4 零位偏 -85°  ->  转完法兰【朝下 45°】，离地掉到 1613，离轴 994
#         B  J5 零位偏 -91°  ->  转完法兰仍【朝上 19°】，离地 1751，离轴 1039
#         C  J5 零位偏 +151° ->  转完法兰仍【朝上 19°】，离地 1750，离轴 813
#
#       A 会让法兰明显翻下去，B/C 不会 —— 肉眼就能分。
#
# 安全：J4 是小臂自转，大臂小臂都不动；法兰中心只绕小臂轴画半径约 125mm 的圆。
#
# 跑法：~/miniforge3/envs/arm38/bin/python 29_单轴测试.py --go
#       ~/miniforge3/envs/arm38/bin/python 29_单轴测试.py --go --delta -85   # 转回去

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
JOINT = 3          # 0-based，J4


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
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.10:
            return True
        time.sleep(0.08)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--delta", type=float, default=85.0, help="J4 转多少度")
    ap.add_argument("--speed", type=int, default=10)
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
        time.sleep(0.6)
        q0 = J(fd)
        x0 = T(fd)
        print("[转动前] q = %s" % ["%.3f" % v for v in q0[:6]])
        print("         控制器报 TCP = (%.1f, %.1f, %.1f)" % (x0[0], x0[1], x0[2]))

        tgt = list(q0[:6]) + [0.0]
        tgt[JOINT] = q0[JOINT] + args.delta
        print("目标 J4: %.3f -> %.3f  （只动这一个轴）" % (q0[JOINT], tgt[JOINT]))
        if abs(tgt[JOINT]) > 130:
            raise SystemExit("J4 目标 %.1f 超过 ±130 限位" % tgt[JOINT])
        if not args.go:
            print("\n（预演，没动。确认后加 --go）")
            return

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)
        if not movej(fd, tgt, args.speed):
            raise SystemExit("J4 没转成")

        time.sleep(1.2)
        q1 = J(fd)
        x1 = T(fd)
        print("\n[转动后] q = %s" % ["%.3f" % v for v in q1[:6]])
        print("         J4 实际 %.3f（目标 %.3f，差 %.3f）"
              % (q1[JOINT], tgt[JOINT], q1[JOINT] - tgt[JOINT]))
        print("         控制器报 TCP = (%.1f, %.1f, %.1f)" % (x1[0], x1[1], x1[2]))

        print("\n" + "=" * 66)
        print("现在请看：法兰面朝哪？（这一步就能定案）")
        print("=" * 66)
        print("  A  J4 零位偏 -85°   -> 法兰【翻到朝下约 45°】，离地应约 1613")
        print("  B  J5 零位偏 -91°   -> 法兰【还是朝上约 19°】，离地约 1751")
        print("  C  J5 零位偏 +151°  -> 法兰【还是朝上约 19°】，离地约 1750")
        print()
        print("  然后量一下法兰中心离地，A 会掉 150mm，B/C 基本不动。")
    finally:
        print("\n保持上电中。")


if __name__ == "__main__":
    main()
