# 04 关节运动 moveJ —— 纳博特 NexDroid（nrc_interface）
#
# 为什么不用点动：点动属于「手动操作」，按安全设计必须有人按住示教器上的使能开关
# 才允许出力。松开就停。所以上位机发点动指令会返回 OK 但不出力 —— 这不是故障。
# 程序运动要走运动队列（queue + moveJ），那条路不需要按住使能开关。
#
# 用法：
#   # 单位自检：目标 = 当前角度，理论上零运动。若单位搞错（当成弧度），
#   # 数值会远超软限位 ±130 而被控制器拒绝 —— 所以这一步是安全的判别手段。
#   python 04_moveJ.py --delta 0
#
#   # 真动 1 度
#   python 04_moveJ.py --delta 1.0 --axis 5 --speed 5
#
# 安全设计：
#   - 上电后先等关节角「稳定」（两次读数差 < 0.02 度），把重力下沉等掉再取基准
#   - 目标必须落在软限位内，且单次变化不超过 MAX_DELTA 度
#   - 运动过程中逐帧监视，任一关节偏离起点超过 MAX_OVERSHOOT 立刻 queue_motion_stop

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}
CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
MAX_DELTA = 3.0          # 度，单次允许的最大关节变化
MAX_OVERSHOOT = 1.0      # 度，超出目标这么多就急停


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def read_state(fd):
    ret, st = nrc.get_servo_state(fd, 0)
    if ret != nrc.SUCCESS:
        raise RuntimeError("读伺服状态失败 %s" % n(ret))
    return st


def read_joints(fd):
    p = nrc.VectorDouble()
    ret = nrc.get_current_position(fd, 0, p)
    if ret != nrc.SUCCESS:
        raise RuntimeError("读关节角失败 %s" % n(ret))
    return list(p)


def wait_stable(fd, tol=0.02, window=0.6, timeout=15.0):
    """等关节角不再变化。上电瞬间腕部会下沉几度，必须先等掉。"""
    t0 = time.time()
    prev = read_joints(fd)
    while time.time() - t0 < timeout:
        time.sleep(window)
        cur = read_joints(fd)
        dmax = max(abs(a - b) for a, b in zip(prev, cur))
        if dmax < tol:
            print("  关节角已稳定（%.1fs 内最大变化 %.3f 度）" % (time.time() - t0, dmax))
            return cur
        prev = cur
    print("  警告：%.0fs 内一直没稳定，继续用当前值" % timeout)
    return prev


def power_on(fd, rounds=4):
    """上电必须先把模式设成 0。

    ⚠️ 这条是这次踩坑的核心：示教器操作过之后，控制器模式会离开 0，
    此时 set_servo_poweron 会一直返回 RECEIVE_FAILED(-1)，而且控制器
    只在示教器上提示「指令 0x2002 参数错误」，SDK 侧完全看不到原因。
    先 set_current_mode(fd, 0) 就能一次上电成功。
    """
    for rd in range(rounds):
        nrc.set_current_mode(fd, 0)
        time.sleep(0.3)
        nrc.clear_error(fd)
        st = read_state(fd)
        if st == 3:
            print("  已是状态 3")
            return True
        # 关键：先归零。但在已经为 0 的状态上再发会返回 -4(NOT_ALLOWED)，
        # 控制器会往示教器上记一条错误（实测就是 0x3B02），所以先判状态。
        if st != 0:
            nrc.set_servo_state(fd, 0)
            time.sleep(0.4)
        nrc.set_servo_state(fd, 1)
        time.sleep(0.6)
        for k in range(3):
            r = nrc.set_servo_poweron(fd)
            t0 = time.time()
            while time.time() - t0 < 2.5:
                if read_state(fd) == 3:
                    print("  上电成功（第 %d 轮第 %d 次，poweron=%s）" % (rd + 1, k + 1, n(r)))
                    return True
                time.sleep(0.2)
            time.sleep(0.4)
        print("  第 %d 轮未成功，状态 %s" % (rd + 1, read_state(fd)))
    st = read_state(fd)
    print("  上电失败，最后状态 %s %s" % (st, SERVO.get(st, "?")))
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.2.14")
    ap.add_argument("--port", default="6001")
    ap.add_argument("--axis", type=int, default=5, help="改哪个关节 0-5")
    ap.add_argument("--delta", type=float, default=1.0, help="转多少度，0 = 单位自检")
    ap.add_argument("--speed", type=int, default=5, help="速度百分比 1-100")
    ap.add_argument("--pl", type=int, default=0, help="平滑等级，0=精确停止")
    ap.add_argument("--off", action="store_true", help="跑完下电")
    args = ap.parse_args()

    if abs(args.delta) > MAX_DELTA:
        raise SystemExit("--delta 超过 %.1f 度，拒绝" % MAX_DELTA)
    if not 0 <= args.axis <= 5:
        raise SystemExit("--axis 只能是 0-5")

    fd = nrc.connect_robot(args.ip, str(args.port))
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS:
        if time.time() > t:
            raise SystemExit("等待连接就绪超时")
        time.sleep(0.2)
    print("已连接 %s:%s" % (args.ip, args.port))

    try:
        print("上电：")
        if not power_on(fd):
            raise SystemExit("上电失败，退出")

        print("等关节角稳定：")
        start = wait_stable(fd)
        print("  基准 %s" % [round(v, 3) for v in start])

        # 软限位检查
        if args.delta != 0:
            p = nrc.RobotJointParam()
            r = nrc.get_robot_joint_param(fd, args.axis, p)
            if r == nrc.SUCCESS:
                target_axis = start[args.axis] + args.delta
                if not (p.negSWLimit <= target_axis <= p.posSWLimit):
                    raise SystemExit("目标 %.3f 度超出软限位 [%s, %s]，拒绝"
                                     % (target_axis, p.negSWLimit, p.posSWLimit))
                print("  软限位 [%s, %s] 度，目标 %.3f 度，通过"
                      % (p.negSWLimit, p.posSWLimit, target_axis))

        target = list(start)
        target[args.axis] += args.delta
        print("目标 %s" % [round(v, 3) for v in target])

        print("设速度 set_speed(%d) -> %s" % (args.speed, n(nrc.set_speed(fd, args.speed))))
        print("开队列 set_status(True) -> %s" % n(nrc.queue_motion_set_status(fd, True)))
        nrc.queue_motion_clear_Data(fd)

        cmd = nrc.MoveCmd()
        cmd.coord = 0                                   # 0 = 关节坐标
        cmd.targetPosType = nrc.PosType_data
        cmd.targetPosValue = nrc.VectorDouble(target)
        cmd.velocity = args.speed
        cmd.acc = 10
        cmd.dec = 10
        cmd.pl = args.pl
        print("push_moveJ -> %s" % n(nrc.queue_motion_push_back_moveJ(fd, cmd)))
        print("send_to_controller(1) -> %s" % n(nrc.queue_motion_send_to_controller(fd, 1)))
        print("指令已下发，开始监视…")

        # 监视：到位就退出，跑飞就停
        t0 = time.time()
        last = None
        while time.time() - t0 < 20.0:
            cur = read_joints(fd)
            over = max(abs(a - b) for a, b in zip(start, cur))
            reached = max(abs(a - b) for a, b in zip(target, cur))
            if time.time() - (last or 0) > 0.5:
                print("  %5.2fs  与目标差 %6.3f 度  离起点 %6.3f 度"
                      % (time.time() - t0, reached, over))
                last = time.time()
            if over > abs(args.delta) + MAX_OVERSHOOT:
                print("  !! 超出预期 %.1f 度，立刻停" % MAX_OVERSHOOT)
                nrc.queue_motion_stop(fd)
                break
            if reached < 0.1:
                print("  已到位")
                break
            time.sleep(0.1)

        end = read_joints(fd)
        print("结束 %s" % [round(v, 3) for v in end])
        print("实际变化 %s" % [round(b - a, 3) for a, b in zip(start, end)])

        nrc.queue_motion_set_status(fd, False)
        if args.off:
            st = read_state(fd)
            if st in (2, 3):
                print("下电 poweroff ->", n(nrc.set_servo_poweroff(fd)))
                time.sleep(0.3)
                st = read_state(fd)
            if st != 0:      # 已是 0 就别再发，否则示教器上多一条 0x3B02
                print("下电 state0   ->", n(nrc.set_servo_state(fd, 0)))
            else:
                print("已是停止状态，不再下发无谓的下电命令")
    except KeyboardInterrupt:
        print("\nCtrl+C，停队列")
        try:
            nrc.queue_motion_stop(fd)
        except Exception:
            pass
    finally:
        nrc.disconnect_robot(fd)
        print("已断开")


if __name__ == "__main__":
    main()
