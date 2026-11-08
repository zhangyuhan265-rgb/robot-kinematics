# 27 急停复现 —— 在运动中拍急停，测响应时间和停车距离
#
# 跟 05_急停验证.py 的区别：
#   05 只监视状态、不发运动指令，而且 0.2 秒轮询一次，测不出响应时间。
#   这个先用流式点动让臂真动起来，再 10 ms 采样，记录：
#     * 伺服状态 3 → ? 的时刻（响应时间）
#     * 状态掉下来之后关节还走了多少（停车距离）
#     * 停车用了多久（从状态掉下来到关节彻底不动）
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 27_急停复现.py                # 预演
#   ~/miniforge3/envs/arm38/bin/python 27_急停复现.py --go           # 默认转 J6（末端不动，最安全）
#   ... --go --axis 1 --sub 1.0     # 换 J2，动作更真实，但要确认周围有空间
#
# 流程：上电 → 开始连续点动 → 提示"现在拍急停" → 你拍 → 脚本记录并报告
#       → 提示你把急停旋出来 → 脚本尝试恢复上电（可选）
#
# 安全：默认只动 J6（TCP 在轴 6 上，末端不移动）；限流领先 1.5°；软限位保护。

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}


def S(fd):
    try:
        return nrc.get_servo_state(fd, 0)[1]
    except Exception:
        return -99


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
    for i in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def push_movej(fd, q6, speed, pl):
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(list(q6[:6]) + [0.0])
    cmd.velocity = speed
    cmd.acc = 30
    cmd.dec = 30
    cmd.pl = pl
    return nrc.queue_motion_push_back_moveJ(fd, cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--axis", type=int, default=5, help="点动哪根轴（0基，默认 5=J6）")
    ap.add_argument("--sub", type=float, default=1.0, help="子步长（度）")
    ap.add_argument("--speed", type=float, default=90.0)
    ap.add_argument("--maxdeg", type=float, default=25.0, help="最多走多少度就停（防止拍不急）")
    ap.add_argument("--watch", type=float, default=60.0, help="等你拍急停的最长秒数")
    args = ap.parse_args()

    print("计划：流式点动 J%d，子步 %.2f°，速度 %.0f，最多走 %.0f°"
          % (args.axis + 1, args.sub, args.speed, args.maxdeg))
    print("     臂动起来后提示你拍急停，10 ms 采样记录状态和关节")
    if not args.go:
        print("\n（预演，没连机器人。加 --go 真做）")
        return

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s" % time.strftime("%H:%M:%S"))

    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        nrc.set_speed(fd, 60)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, True)
        q0 = J(fd)
        print("上电成功，起点 q = %s" % ["%.2f" % v for v in q0[:6]])

        # 开始连续点动
        q_v = list(q0[:6])
        base = q_v[args.axis]
        direction = 1
        sent = 0
        samples = []          # (t, state, q[axis])
        t_start = time.time()
        print("\n臂开始动，准备拍急停 ...")
        print(">>> 现在就拍急停 <<<")

        triggered = None
        stop_q = None
        stall = 0
        prev_q = None
        while time.time() - t_start < args.watch:
            st = S(fd)
            try:
                qa = J(fd)[args.axis]
            except Exception:
                qa = prev_q if prev_q is not None else base
            samples.append((time.time() - t_start, st, qa))

            if triggered is None:
                # 还没拍：继续喂运动
                if st != 3:
                    triggered = time.time() - t_start
                    stop_q = qa
                    print("\n[%6.3fs] 状态 3 → %s，检测到急停" % (triggered, st))
                else:
                    lead = (q_v[args.axis] - qa) * direction
                    # 到边界就掉头，让它一直来回动
                    if (q_v[args.axis] - base) * direction > args.maxdeg:
                        direction = -direction
                        lead = 0.0
                    if lead < 1.5:
                        n = 0
                        for _ in range(3):
                            q_v[args.axis] += args.sub * direction
                            if push_movej(fd, q_v, args.speed, 5) != nrc.SUCCESS:
                                q_v[args.axis] -= args.sub * direction
                                break
                            n += 1
                        if n:
                            if nrc.queue_motion_send_to_controller(fd, n, False) != nrc.SUCCESS:
                                for _ in range(n):
                                    q_v[args.axis] -= args.sub * direction
                    else:
                        time.sleep(0.005)
            else:
                # 已拍：看它什么时候彻底停下
                if prev_q is not None and abs(qa - prev_q) < 0.005:
                    stall += 1
                else:
                    stall = 0
                prev_q = qa
                if stall >= 15:      # 连续 15 次(约0.15s)不动
                    stop_q = qa
                    break
            prev_q = qa
            time.sleep(0.008)

        # ---------- 报告 ----------
        print("\n" + "=" * 60)
        if triggered is None:
            print("没检测到急停（%.0f 秒内状态一直是 3）" % args.watch)
            print("最终状态 %s，q%d = %.3f" % (S(fd), args.axis + 1, J(fd)[args.axis]))
        else:
            t_stop = samples[-1][0]
            # 找状态第一次到 0/1/2 的时刻
            t_zero = None
            for (tt, ss, qq) in samples:
                if tt >= triggered and ss in (0, 1, 2):
                    t_zero = tt
                    break
            q_at_trigger = None
            for (tt, ss, qq) in samples:
                if tt >= triggered:
                    q_at_trigger = qq
                    break
            dq = abs(stop_q - q_at_trigger) if (q_at_trigger is not None and stop_q is not None) else 0.0
            print("检测到状态变化        : t = %6.3f s" % triggered)
            if t_zero is not None:
                print("状态变成 停止/就绪    : t = %6.3f s（距检测 %.3f s）" % (t_zero, t_zero - triggered))
            print("关节彻底停下          : t = %6.3f s（距检测 %.3f s）" % (t_stop, t_stop - triggered))
            print("停车期间关节又走了    : %.3f 度" % dq)
            # 从前 0.4 秒的采样估算拍之前的关节速度
            pre = [(tt, qq) for (tt, ss, qq) in samples if triggered - 0.4 <= tt < triggered]
            vpre = 0.0
            if len(pre) >= 2:
                span = pre[-1][0] - pre[0][0]
                if span > 1e-3:
                    vpre = abs(pre[-1][1] - pre[0][1]) / span
            print("拍之前关节速度        : %.2f 度/秒（实测算出来的）" % vpre)
            print("\n注意：检测时刻受轮询周期（约 8~10 ms）影响，真实响应只会更快一点。")
            print("末态 q%s = %.3f（起点 %.3f，本次共走 %+.3f 度）"
                  % (args.axis + 1, J(fd)[args.axis], base, J(fd)[args.axis] - base))

            with open("/tmp/estop_trace.csv", "w") as f:
                f.write("t_s,servo_state,axis_deg\n")
                for (tt, ss, qq) in samples:
                    f.write("%.4f,%d,%.4f\n" % (tt, ss, qq))
            print("采样轨迹已写 /tmp/estop_trace.csv（%d 点）" % len(samples))

        print("=" * 60)
        print("\n恢复步骤：")
        print("  1. 把急停按钮【旋出来】复位")
        print("  2. 本脚本会尝试 clear_error + 重新上电")
        time.sleep(1.0)
        print("\n清报警 ->", nrc.clear_error(fd))
        st = S(fd)
        if st != 0:
            nrc.set_servo_state(fd, 0)
            time.sleep(0.4)
        print("复位后状态 %s" % SERVO.get(S(fd), S(fd)))
    finally:
        try:
            nrc.queue_motion_stop(fd)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, False)
        except Exception:
            pass
        if S(fd) in (2, 3):
            nrc.set_servo_poweroff(fd)
            time.sleep(0.4)
        if S(fd) != 0:
            nrc.set_servo_state(fd, 0)
        nrc.disconnect_robot(fd)
        print("\n已下电断开 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    main()
