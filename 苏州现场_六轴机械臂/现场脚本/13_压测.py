# 13 压测 —— 六轴巡演重复 10 轮
#
# 一次上电里连续跑 10 轮（每轮 4 个姿态），统计成功率、耗时、回位残差。
# 用来判断"修好之后是否真的稳定"，而不是碰巧跑过一次。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 13_压测.py
#   ~/miniforge3/envs/arm38/bin/python 13_压测.py --rounds 5 --speed 40
#
# 幅度说明：J6 幅度取 ±15 而不是 ±25，因为它当前在 -68 度、软限位 ±90，
#          再大就会顶到限位。脚本每个航点都会先查软限位，超界直接拒绝。

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
MAX_DEV = 30.0

POSES = [
    ("P1", [10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P2", [10.0, -8.0, 8.0, -10.0, -8.0, -20.0, 0.0]),
    ("P3", [-10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P4", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


def J(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return list(p)


def fmt(v):
    return "[" + ", ".join("%7.2f" % x for x in v[:6]) + "]"


def power_on(fd):
    nrc.set_current_mode(fd, 0)
    time.sleep(0.3)
    nrc.clear_error(fd)
    st = S(fd)
    if st == 3:
        return True
    if st != 0:
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


def wait_stable(fd, tol=0.02, timeout=15.0):
    prev = J(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.6)
        cur = J(fd)
        if max(abs(a - b) for a, b in zip(prev, cur)) < tol:
            return cur
        prev = cur
    return prev


def soft_limits(fd):
    lim = []
    for i in range(6):
        p = nrc.RobotJointParam()
        if nrc.get_robot_joint_param(fd, i, p) == nrc.SUCCESS:
            lim.append((p.negSWLimit, p.posSWLimit))
        else:
            lim.append((-130.0, 130.0))
    return lim


def move(fd, home, target, speed, limit=60.0):
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        return False, "push失败"
    if nrc.queue_motion_send_to_controller(fd, 1) != nrc.SUCCESS:
        return False, "send失败"

    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < limit:
        st = S(fd)
        if st != 3:
            return False, "伺服变成%s" % SERVO.get(st, st)
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        gap = max(abs(a - b) for a, b in zip(target, js))
        dev = max(abs(a - b) for a, b in zip(home, js))
        if dev > MAX_DEV:
            nrc.queue_motion_stop(fd)
            return False, "偏离 %.1f 度" % dev
        if gap < 0.15:
            return True, ""
        time.sleep(0.08)
    nrc.queue_motion_stop(fd)
    return False, "超时"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=10)
    ap.add_argument("--speed", type=int, default=60)
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接  %s" % time.strftime("%H:%M:%S"))

    ok_rounds = 0
    worst = 0.0
    t_start = time.time()
    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        home = wait_stable(fd)
        print("原位 %s" % fmt(home))

        lim = soft_limits(fd)
        for name, delta in POSES:
            tgt = [a + b for a, b in zip(home, delta)]
            for i in range(6):
                lo, hi = lim[i]
                if not (lo <= tgt[i] <= hi):
                    raise SystemExit("%s 的 J%d 目标 %.1f 超出软限位 [%.0f, %.0f]，拒绝"
                                     % (name, i + 1, tgt[i], lo, hi))
        print("软限位检查通过，开始 %d 轮\n" % args.rounds)

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        fail_at = None
        for r in range(1, args.rounds + 1):
            tr = time.time()
            ok = True
            for name, delta in POSES:
                tgt = [a + b for a, b in zip(home, delta)]
                good, why = move(fd, home, tgt, args.speed)
                if not good:
                    print("第 %2d 轮 %s 失败：%s" % (r, name, why))
                    fail_at = (r, name, why)
                    ok = False
                    break
            if not ok:
                break
            res = max(abs(a - b) for a, b in zip(home, J(fd)))
            worst = max(worst, res)
            ok_rounds += 1
            print("第 %2d 轮 OK   用时 %5.2fs   残差 %.3f 度" % (r, time.time() - tr, res))

        print("\n========== 压测结果 ==========")
        print("成功 %d / %d 轮" % (ok_rounds, args.rounds))
        print("最大回位残差 %.3f 度" % worst)
        print("总耗时 %.1f 秒" % (time.time() - t_start))
        if fail_at:
            print("第一次失败：第 %d 轮 %s（%s）" % fail_at)
    finally:
        try:
            nrc.queue_motion_stop(fd)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, False)
        except Exception:
            pass
        st = S(fd)
        if st in (2, 3):
            nrc.set_servo_poweroff(fd)
            time.sleep(0.4)
            st = S(fd)
        if st != 0:
            nrc.set_servo_state(fd, 0)
        print("下电完成，状态 %s" % S(fd))
        nrc.disconnect_robot(fd)
        print("已断开  %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    main()
