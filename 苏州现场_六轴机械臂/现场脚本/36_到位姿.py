# 36 走到指定位姿（关节插值，不经控制器正解）
#
# 跑法：
#   python 36_到位姿.py --q 0,-20,10,0,0,0 --go
#   ~/miniforge3/envs/arm38/bin/python ~/projects/07_机器人学/练习/机械臂现场/36_到位姿.py --q 0,-20,10,0,0,0 --go --speed 6

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
DWELL = 0.8


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
    for _ in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def movej(fd, target, speed, limit=120.0):
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
        if max(abs(a - b) for a, b in zip(target, js)) < 0.10:
            return True
        time.sleep(0.08)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q", required=True, help="6 个关节角（度），逗号分隔")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--speed", type=int, default=8)
    args = ap.parse_args()

    tgt = [float(v) for v in args.q.split(",")]
    if len(tgt) != 6:
        raise SystemExit("--q 需要 6 个数")
    tgt.append(0.0)

    fd = 0
    for attempt in range(8):
        fd = nrc.connect_robot("192.168.2.14", "6001")
        if fd > 0:
            break
        print("连接失败，等 12 秒重试（第 %d 次）..." % (attempt + 1))
        time.sleep(12)
    if fd <= 0:
        raise SystemExit("连接失败（重试 8 次）")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)

    if not power_on(fd):
        raise SystemExit("上电失败")
    time.sleep(0.6)
    q0 = J(fd)
    print("当前 q = %s" % ["%.2f" % v for v in q0[:6]])
    print("目标 q = %s" % ["%.2f" % v for v in tgt[:6]])
    dq = [tgt[i] - q0[i] for i in range(6)]
    print("各轴要转 %s  最大 %.1f 度"
          % (["%+.1f" % v for v in dq], max(abs(v) for v in dq)))
    if max(abs(v) for v in dq) > 100:
        raise SystemExit("单轴要转 %.0f 度，超过 100，太大，检查一下" % max(abs(v) for v in dq))
    if not args.go:
        print("（预演，没动。加 --go）")
        return

    nrc.set_speed(fd, args.speed)
    nrc.queue_motion_set_status(fd, False)
    time.sleep(0.4)
    nrc.queue_motion_set_status(fd, True)
    ok = False
    for _ in range(3):
        if movej(fd, tgt, args.speed):
            ok = True
            break
        power_on(fd)
        time.sleep(0.8)
    if not ok:
        raise SystemExit("没走到")
    time.sleep(DWELL)
    q1 = J(fd)
    x1 = nrc.VectorDouble()
    nrc.get_current_position(fd, 1, x1)
    print("\n到位  q = %s" % ["%.2f" % v for v in q1[:6]])
    print("偏差 %.3f 度" % max(abs(q1[i] - tgt[i]) for i in range(6)))
    print("控制器报 TCP = (%.1f, %.1f, %.1f)" % (x1[0], x1[1], x1[2]))
    print("保持上电中。")


if __name__ == "__main__":
    main()
