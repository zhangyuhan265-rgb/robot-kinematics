# 07 往返演示 —— 纳博特 NexDroid（nrc_interface）
#
# 一次上电，做两次 moveJ：先把 J1 转出去，再转回原位，然后下电。
# 这是「让机械臂动起来」的可复现最小演示，也是现场验收用的脚本。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 07_往返demo.py --axis 0 --delta 5 --speed 5
#
# 关键坑（都在真机上踩过，别删）：
#   1. 上电前必须先 set_current_mode(fd, 0)。示教器一动模式就离开 0，
#      之后 set_servo_poweron 会一直返回 -1，而控制器只在示教器上提示
#      「指令 0x2002 参数错误」，SDK 侧完全看不到原因。
#   2. 上电瞬间腕部可能下沉几度（没有可靠抱闸）。取基准前必须等关节角稳定，
#      否则会把下沉量当成运动量。
#   3. 运动前查软限位。moveJ 的目标单位是【度】，与 get_current_position(fd,0) 一致。

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
MAX_OVERSHOOT = 2.0      # 度，超出目标这么多就停队列


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
    return "[" + ", ".join("%8.3f" % x for x in v[:6]) + "]"


def power_on(fd):
    nrc.set_current_mode(fd, 0)          # 必须！见文件头第 1 条
    time.sleep(0.3)
    nrc.clear_error(fd)
    st = read_state(fd)
    if st == 3:
        print("  已是状态 3，跳过上电")
        return True
    # 只在需要时归零：在已经停止的状态下发 set_servo_state(0) 会返回
    # -4(NOT_ALLOWED)，控制器会往示教器上记一条错误（疑似 0x3B02 的来源）
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
                print("  上电成功（第 %d 次，poweron=%s）" % (i + 1, n(r)))
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def wait_stable(fd, tol=0.02, window=0.6, timeout=15.0):
    t0 = time.time()
    prev = read_joints(fd)
    while time.time() - t0 < timeout:
        time.sleep(window)
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


def move_to(fd, target, speed, label):
    print("\n—— %s ——" % label)
    print("  目标 %s" % fmt(target))
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    
    print("  push_moveJ -> %s" % n(nrc.queue_motion_push_back_moveJ(fd, cmd)))
    print("  send(1)    -> %s" % n(nrc.queue_motion_send_to_controller(fd, 1)))

    t0 = time.time()
    last = 0.0
    goal = target
    while time.time() - t0 < 25.0:
        cur = read_joints(fd)
        gap = max(abs(a - b) for a, b in zip(goal, cur))
        if time.time() - last > 0.5:
            print("    %5.2fs  与目标差 %6.3f 度  %s" % (time.time() - t0, gap, fmt(cur)))
            last = time.time()
        if gap < 0.1:
            print("    已到位（%.2fs）" % (time.time() - t0))
            return True, cur
        if gap > MAX_OVERSHOOT + 20.0:
            print("    !! 偏离目标过大，停队列")
            nrc.queue_motion_stop(fd)
            return False, cur
        time.sleep(0.1)
    print("    超时未到位")
    return False, read_joints(fd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", type=int, default=0)
    ap.add_argument("--delta", type=float, default=5.0)
    ap.add_argument("--speed", type=int, default=5)
    args = ap.parse_args()

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接")

    try:
        print("上电：")
        if not power_on(fd):
            raise SystemExit("上电失败")
        home = wait_stable(fd)
        print("基准 %s" % fmt(home))

        lim = soft_limits(fd)
        out = list(home)
        out[args.axis] += args.delta
        lo, hi = lim[args.axis]
        print("J%d 软限位 [%.0f, %.0f]，目标 %.3f" % (args.axis + 1, lo, hi, out[args.axis]))
        if not (lo <= out[args.axis] <= hi):
            raise SystemExit("目标超出软限位，拒绝")

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, True)

        ok1, mid = move_to(fd, out, args.speed, "转出去 %+.1f 度" % args.delta)
        time.sleep(0.8)
        ok2, back = move_to(fd, home, args.speed, "转回原位")

        print("\n结果：")
        print("  基准   %s" % fmt(home))
        print("  出去后 %s" % fmt(mid))
        print("  回来后 %s" % fmt(back))
        res = max(abs(a - b) for a, b in zip(home, back))
        print("  回位残差 %.3f 度  %s" % (res, "OK" if res < 0.2 else "偏大"))

        nrc.queue_motion_set_status(fd, False)
    finally:
        # 同样只在需要时下命令，避免产生无谓的错误记录
        st = read_state(fd)
        if st in (2, 3):
            print("\n下电 poweroff -> %s" % n(nrc.set_servo_poweroff(fd)))
            time.sleep(0.4)
            st = read_state(fd)
        if st != 0:
            print("下电 state(0) -> %s  状态 %s" % (n(nrc.set_servo_state(fd, 0)), read_state(fd)))
        else:
            print("\n已是停止状态，不再下发无谓的下电命令")
        nrc.disconnect_robot(fd)
        print("已断开")


if __name__ == "__main__":
    main()
