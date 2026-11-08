# 09 多轴巡演 —— 纳博特 NexDroid（nrc_interface）
#
# 在关节空间连续走 4 个姿态再回原位。全部走 moveJ（已验证可靠的通路）。
# 每一段都是 J1–J5 五轴同时运动，不是单轴依次摆。
#   P1  抬起外展：J1+10 J2+8  J3-8  J4+10 J5+8
#   P2  下压内收：J1+10 J2-8  J3+8  J4-10 J5-8
#   P3  反向抬起：J1-10 J2+8  J3-8  J4+10 J5+8
#   P4  回原位：  全部回 0
# J6 全程不动 —— 它当前 84 度，软限位 ±90，只剩 6 度余量。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 09_姿态巡演.py            # 默认速度 100
#   ~/miniforge3/envs/arm38/bin/python 09_姿态巡演.py --speed 60 # 慢一点
#
# 速度：默认 30 是实测出来的最优档，别以为越高越快 ——
#   速度 10  → 0.49 度/秒，4/4 成功
#   速度 30  → 1.71 度/秒，4/4 成功，全程 35 秒  ← 最快
#   速度 80  → 1.22 度/秒，成功 1 次 / 失败 1 次
#   速度 100 → 1.24 度/秒，2 次都在开头伺服掉站
# 百分比和速率不成正比：速度越高，驱动器电流变化越剧烈，总线越容易被干扰，
# 控制器反而被迫降速甚至掉站。想快就停在 30。
#
# 为什么不用 moveL：控制器对 moveL 报「指令(0x3B02)参数错误」直接拒收，
# SDK 侧却返回 0，错误只出现在示教器上。详见 臂接入日志.md 第 11 条。
#
# 保护：
#   - 每个航点先查软限位，超界直接拒绝
#   - 任一关节偏离原位超过 MAX_DEV 立刻停队列
#   - 伺服状态一旦不是 3 立刻中止（掉站时读回的坐标是垃圾，不可采信）
#   - 0.1 秒内关节跳变超过 JUMP_MAX 判为无效数据，重读而不误报

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
MAX_DEV = 25.0            # 度，任一关节偏离原位超过这个就停
JUMP_MAX = 5.0            # 度，0.1 秒内跳变超过这个判为无效读数据
LAST_WHY = ""             # move_to 最后一次失败的原因

POSES = [
    ("P1 抬起外展", [10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P2 下压内收", [10.0, -8.0, 8.0, -10.0, -8.0, -20.0, 0.0]),
    ("P3 反向抬起", [-10.0, 8.0, -8.0, 10.0, 8.0, 20.0, 0.0]),
    ("P4 回原位", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
]


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def read_state(fd):
    return nrc.get_servo_state(fd, 0)[1]


def read_joints(fd):
    p = nrc.VectorDouble()
    ret = nrc.get_current_position(fd, 0, p)
    if ret != nrc.SUCCESS:
        raise RuntimeError("读关节角失败 %s" % n(ret))
    return list(p)


def fmt(v):
    return "[" + ", ".join("%7.2f" % x for x in v[:6]) + "]"


def power_on(fd):
    nrc.set_current_mode(fd, 0)
    time.sleep(0.3)
    nrc.clear_error(fd)
    st = read_state(fd)
    if st == 3:
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
            if read_state(fd) == 3:
                print("  上电成功（第 %d 次）" % (i + 1))
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def wait_stable(fd, tol=0.02, timeout=15.0):
    prev = read_joints(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.6)
        cur = read_joints(fd)
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


def move_to(fd, home, target, speed, name):
    print("\n—— %s ——" % name)
    print("   目标 %s" % fmt(target))

    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    r = nrc.queue_motion_push_back_moveJ(fd, cmd)
    if r != nrc.SUCCESS:
        print("   push_moveJ 失败 %s" % n(r))
        return False
    r = nrc.queue_motion_send_to_controller(fd, 1)
    if r != nrc.SUCCESS:
        print("   send 失败 %s" % n(r))
        return False

    t0 = time.time()
    last = 0.0
    prev = read_joints(fd)
    while time.time() - t0 < 90.0:
        st = read_state(fd)
        if st != 3:
            print("   !! 伺服状态变成 %s，中止（后续读数不可信）" % SERVO.get(st, st))
            global LAST_WHY
            LAST_WHY = "servo"
            return False

        cur = read_joints(fd)
        jump = max(abs(a - b) for a, b in zip(prev, cur))
        if jump > JUMP_MAX:
            print("   !! 读到跳变 %.1f 度，判为无效数据，重读" % jump)
            time.sleep(0.2)
            continue
        prev = cur

        gap = max(abs(a - b) for a, b in zip(target, cur))
        dev = max(abs(a - b) for a, b in zip(home, cur))
        if time.time() - last > 1.0:
            print("   %5.1fs  与目标差 %6.2f 度  离原位 %6.2f 度  %s"
                  % (time.time() - t0, gap, dev, fmt(cur)))
            last = time.time()
        if dev > MAX_DEV:
            print("   !! 偏离原位 %.1f 度超过上限，停队列" % dev)
            nrc.queue_motion_stop(fd)
            return False
        if gap < 0.15:
            print("   到位（%.2fs）" % (time.time() - t0))
            return True
        time.sleep(0.1)
    print("   超时（90s）")
    nrc.queue_motion_stop(fd)
    return False


def main():
    global LAST_WHY
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=int, default=30,
                    help="速度百分比 1-100。默认 30 —— 实测最快且稳定，见文件头")
    ap.add_argument("--prewait", type=float, default=0.0,
                    help="上电后等多少秒再发第一条运动指令。实测 0 最好，别调大")
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接  本机时间 %s" % time.strftime("%H:%M:%S"))

    try:
        print("上电：")
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
                    raise SystemExit("%s：J%d 目标 %.1f 超出软限位 [%.0f, %.0f]，拒绝"
                                     % (name, i + 1, tgt[i], lo, hi))
        print("全部航点软限位检查通过")

        # 上电后多等一会儿再发第一条运动指令。
        # 压测显示失败全部是「第一条指令就掉使能」（0/4），从没出现过跑到一半掉，
        # 怀疑控制器上电后需要更长时间才真正接受运动。
        if args.prewait > 0:
            print("上电后等待 %.1f 秒再开始运动…" % args.prewait)
            time.sleep(args.prewait)

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, True)
        # 注意：这里【不要】调 queue_motion_clear_Data。
        # 08/09 早期版本调了它，示教器上就冒「指令(0x3B02)参数错误」；
        # 07 从来没调过，也从来没报过这个错。每段运动只 push 一条并等到位，
        # 队列自然会排空，不需要清。

        done = 0
        for name, delta in POSES:
            tgt = [a + b for a, b in zip(home, delta)]
            ok = False
            for attempt in range(3):
                LAST_WHY = ""
                ok = move_to(fd, home, tgt, args.speed, name)
                if ok or LAST_WHY != "servo":
                    break
                # 第一条指令就掉使能：在同一次连接里重新上电再试，不断开重连
                print("   启动失败，重新上电后重试（第 %d 次）" % (attempt + 1))
                if not power_on(fd):
                    print("   重新上电失败")
                    break
                wait_stable(fd)
                nrc.set_speed(fd, args.speed)
                nrc.queue_motion_set_status(fd, False)
                time.sleep(0.3)
                nrc.queue_motion_set_status(fd, True)
            if ok:
                done += 1
            else:
                print("   这一段没走成，后续跳过")
                break
            time.sleep(0.5)

        print("\n结果：完成 %d / %d 个姿态" % (done, len(POSES)))
        end = read_joints(fd)
        print("结束 %s" % fmt(end))
        print("回位残差 %.3f 度" % max(abs(a - b) for a, b in zip(home, end)))
        nrc.queue_motion_set_status(fd, False)
    finally:
        st = read_state(fd)
        if st in (2, 3):
            print("\n下电 poweroff -> %s" % n(nrc.set_servo_poweroff(fd)))
            time.sleep(0.4)
            st = read_state(fd)
        if st != 0:
            print("下电 state(0) -> %s" % n(nrc.set_servo_state(fd, 0)))
        else:
            print("\n已是停止状态，不再下发无谓的下电命令")
        nrc.disconnect_robot(fd)
        print("已断开  本机时间 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    main()
