# 12 完整演示 —— 纳博特 NexDroid（nrc_interface）
#
# 一次上电跑完全套动作，适合录视频：
#   第一段：六轴关节巡演（moveJ）  P1 抬起外展 → P2 下压内收 → P3 反向抬起 → P4 回原位
#   第二段：直角画方块（moveL）    四边形 + 回原点
#
# 中间不重新上下电、不重连 —— 今天实测掉站和上下电次数强相关，一次跑完最稳。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 12_完整演示.py
#   ~/miniforge3/envs/arm38/bin/python 12_完整演示.py --size 20 --vel 180
#
# 速度换算：末端速度 ≈ 0.36 × --vel（要 100 mm/s 就给 280）

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
MAX_DEV = 40.0

JOINT_POSES = [
    ("P1 抬起外展", [10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P2 下压内收", [10.0, -8.0, 8.0, -10.0, -8.0, -20.0, 0.0]),
    ("P3 反向抬起", [-10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P4 回原位", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


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


def fmt(v):
    return "[" + ", ".join("%7.2f" % x for x in v[:6]) + "]"


def power_on(fd):
    nrc.set_current_mode(fd, 0)
    time.sleep(0.3)
    nrc.clear_error(fd)
    st = S(fd)
    if st == 3:
        print("  已是状态 3")
        return True
    if st != 0:
        nrc.set_servo_state(fd, 0)
        time.sleep(0.4)
    nrc.set_servo_state(fd, 1)
    time.sleep(0.6)
    for i in range(6):
        r = nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                print("  上电成功（第 %d 次）" % (i + 1))
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
            return
        prev = cur


def run(fd, target, coord, vel, ref, label, limit=150.0):
    """发一条运动并等到位。coord=0 关节 / 1 直角。ref 用于偏离监视的基准。"""
    print("\n  → %s" % label)
    print("    %s" % fmt(target))

    cmd = nrc.MoveCmd()
    cmd.coord = coord
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = vel
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    push = (nrc.queue_motion_push_back_moveJ if coord == 0
            else nrc.queue_motion_push_back_moveL)
    r = push(fd, cmd)
    if r != nrc.SUCCESS:
        print("    push 失败 %s" % n(r))
        return False
    r = nrc.queue_motion_send_to_controller(fd, 1)
    if r != nrc.SUCCESS:
        print("    send 失败 %s" % n(r))
        return False

    t0 = time.time()
    prev = J(fd)
    last = 0.0
    while time.time() - t0 < limit:
        st = S(fd)
        if st != 3:
            print("    !! 伺服变成 %s，中止" % SERVO.get(st, st))
            return False
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        dev = max(abs(a - b) for a, b in zip(ref, js))
        gap = (max(abs(a - b) for a, b in zip(target, js)) if coord == 0
               else max(abs(a - b) for a, b in zip(target[:3], T(fd)[:3])))
        if time.time() - last > 1.0:
            print("    %5.1fs  与目标差 %6.2f   偏离基准 %5.2f 度" % (time.time() - t0, gap, dev))
            last = time.time()
        if dev > MAX_DEV:
            print("    !! 偏离基准 %.1f 度，停队列" % dev)
            nrc.queue_motion_stop(fd)
            return False
        thr = 0.15 if coord == 0 else 0.3
        if gap < thr:
            print("    到位（%.2fs）" % (time.time() - t0))
            return True
        time.sleep(0.1)
    print("    超时")
    nrc.queue_motion_stop(fd)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=int, default=60, help="关节运动速度百分比")
    ap.add_argument("--vel", type=float, default=120, help="直线速度（末端 ≈ 0.36×vel mm/s）")
    ap.add_argument("--size", type=float, default=15.0, help="方块半边长 mm")
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接  %s" % time.strftime("%H:%M:%S"))

    done = 0
    try:
        print("上电：")
        if not power_on(fd):
            raise SystemExit("上电失败")
        wait_stable(fd)
        j0 = J(fd)
        t0_home = T(fd)
        print("起始关节 %s" % fmt(j0))
        print("起始 TCP %s" % fmt(t0_home))

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        print("\n========== 第一段：六轴关节巡演 ==========")
        for name, delta in JOINT_POSES:
            tgt = [a + b for a, b in zip(j0, delta)]
            if run(fd, tgt, 0, args.speed, j0, name):
                done += 1
            else:
                break
            time.sleep(0.5)

        if done >= len(JOINT_POSES):
            print("\n========== 第二段：直角画方块 ==========")
            s = args.size
            base = T(fd)
            corners = [
                (base[0] + s, base[1] + s, base[2], base[3], base[4], base[5], 0.0),
                (base[0] + s, base[1] - s, base[2], base[3], base[4], base[5], 0.0),
                (base[0] - s, base[1] - s, base[2], base[3], base[4], base[5], 0.0),
                (base[0] - s, base[1] + s, base[2], base[3], base[4], base[5], 0.0),
                list(base),
            ]
            for i, c in enumerate(corners):
                label = "第 %d 条边" % (i + 1) if i < 4 else "回原点"
                if run(fd, c, 1, args.vel, j0, label):
                    done += 1
                else:
                    break

        print("\n========== 结果 ==========")
        print("完成 %d 段" % done)
        print("结束关节 %s" % fmt(J(fd)))
        print("结束 TCP %s" % fmt(T(fd)))
        print("回位差 %.2f mm" % (((T(fd)[0] - t0_home[0]) ** 2 + (T(fd)[1] - t0_home[1]) ** 2
                               + (T(fd)[2] - t0_home[2]) ** 2) ** 0.5))
    finally:
        try:
            nrc.queue_motion_stop(fd)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, False)
            print("\n已关闭运动队列")
        except Exception as e:
            print("\n关队列失败 %s" % e)
        st = S(fd)
        if st in (2, 3):
            print("下电 poweroff -> %s" % n(nrc.set_servo_poweroff(fd)))
            time.sleep(0.4)
            st = S(fd)
        if st != 0:
            print("下电 state(0) -> %s" % n(nrc.set_servo_state(fd, 0)))
        nrc.disconnect_robot(fd)
        print("已断开  %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    main()
