# 14 压测 50 轮 —— 加大幅度，速度保持 60
#
# 与 09 的区别：
#   1. 幅度加大（J1/J4 到 15 度，J2/J3/J5 到 12 度，J6 保持 20 —— 它只剩 ±21 余量）
#   2. 连跑 50 轮，逐轮统计成功率
#   3. 某轮失败不终止，继续下一轮，这样 50 轮才是完整样本
#
# 跑法（建议放后台）：
#   python 14_压测50轮.py --rounds 50 --speed 60
#
# 关键：使能掉了就在同一会话里重跑上电阶梯（不断开重连），最多 3 次。
# 实测这是唯一有效的规避手段 —— 上位机侧加等待反而更差。

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
MAX_DEV = 35.0

POSES = [
    ("P1", [15.0, 12.0, -12.0, 15.0, 12.0, 20.0, 0.0]),
    ("P2", [15.0, -12.0, 12.0, -15.0, -12.0, -20.0, 0.0]),
    ("P3", [-15.0, 12.0, -12.0, 15.0, 12.0, 20.0, 0.0]),
    ("P4", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]

WHY = ""


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


def J(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return list(p)


def fmt(v):
    return "[" + ", ".join("%6.1f" % x for x in v[:6]) + "]"


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
    for _ in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def wait_stable(fd, tol=0.02, timeout=12.0):
    prev = J(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.5)
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
    global WHY
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        WHY = "push"
        return False
    if nrc.queue_motion_send_to_controller(fd, 1) != nrc.SUCCESS:
        WHY = "send"
        return False

    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < limit:
        st = S(fd)
        if st != 3:
            WHY = "servo"
            return False
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.15:
            return True
        if max(abs(a - b) for a, b in zip(home, js)) > MAX_DEV:
            WHY = "dev"
            nrc.queue_motion_stop(fd)
            return False
        time.sleep(0.08)
    WHY = "timeout"
    nrc.queue_motion_stop(fd)
    return False


def main():
    global WHY
    ap = argparse.ArgumentParser()
    ap.add_argument("--rounds", type=int, default=50)
    ap.add_argument("--speed", type=int, default=60)
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s   计划 %d 轮" % (time.strftime("%H:%M:%S"), args.rounds))

    ok = 0
    fails = []
    resids = []
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
                    raise SystemExit("%s J%d 目标 %.1f 超软限位 [%.0f,%.0f]，拒绝"
                                     % (name, i + 1, tgt[i], lo, hi))
        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)
        print("软限位通过，开始\n")

        for r in range(1, args.rounds + 1):
            tr = time.time()
            retries = 0
            r_ok = True
            for name, delta in POSES:
                tgt = [a + b for a, b in zip(home, delta)]
                got = False
                for _ in range(3):
                    WHY = ""
                    if move(fd, home, tgt, args.speed):
                        got = True
                        break
                    if WHY != "servo":
                        break
                    retries += 1
                    if not power_on(fd):
                        break
                    wait_stable(fd)
                    nrc.set_speed(fd, args.speed)
                    nrc.queue_motion_set_status(fd, False)
                    time.sleep(0.3)
                    nrc.queue_motion_set_status(fd, True)
                if not got:
                    r_ok = False
                    fails.append((r, name, WHY))
                    break
            if r_ok:
                res = max(abs(a - b) for a, b in zip(home, J(fd)))
                resids.append(res)
                ok += 1
                print("%3d/%d OK  %5.2fs  残差 %.3f  重试 %d"
                      % (r, args.rounds, time.time() - tr, res, retries))
            else:
                print("%3d/%d 失败 %s (%s)" % (r, args.rounds, fails[-1][1], fails[-1][2]))
            time.sleep(0.3)

        print("\n================ 压测结果 ================")
        print("成功 %d / %d  (%.0f%%)" % (ok, args.rounds, 100.0 * ok / args.rounds))
        if resids:
            print("残差 最大 %.3f  平均 %.3f 度" % (max(resids), sum(resids) / len(resids)))
        print("总耗时 %.1f 分钟" % ((time.time() - t_start) / 60.0))
        if fails:
            print("失败轮次 %d 次，前 5 条：" % len(fails))
            for f in fails[:5]:
                print("   第 %d 轮 %s (%s)" % f)
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
        print("下电完成 状态 %s" % S(fd))
        nrc.disconnect_robot(fd)
        print("已断开 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    main()
