# 03 单轴点动 —— 纳博特 NexDroid（nrc_interface）
#
# ⚠️ 2026-11-06 真机实测结论：点动在这台控制器上【不出力】。
#     robot_start_jogging 返回 0，但臂一动不动（连 30% 速度都不动）。
#     推测原因：点动属手动操作，按安全设计必须按住示教器上的使能开关才允许出力。
#     所以「第一次让臂动起来」请用 04_moveJ.py / 07_往返demo.py，不要用这个脚本。
#     本脚本保留，仅用于后续排查「点动到底需要什么前置条件」。
#
# 保留的观察：点动是保持式 —— 200ms 内不重发，控制器自己停。
# 脚本用「关节角变化量的绝对值」判停，方向猜反也不会跑飞。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 03_点动.py --axis 0 --dir + --speed 5 --deg 3
#   下电：加 --off
#
# 安全：
#   - 手放急停上。臂展 2 m，转 J1 时末端扫过的弧长 = 臂长 × 角度，
#     3 度 ≈ 10 cm，别小看。
#   - 速度默认 5%（控制器速度百分比），别一上来就 30。

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}
JOG_AXIS_NAME = {0: "J1 底座回转", 1: "J2 大臂", 2: "J3 小臂",
                 3: "J4 腕部回转", 4: "J5 腕部摆动", 5: "J6 法兰自转"}


def read_state(fd):
    ret, st = nrc.get_servo_state(fd, 0)
    if ret != nrc.SUCCESS:
        raise RuntimeError(f"读伺服状态失败 {ret}")
    return st


def read_joints(fd):
    pos = nrc.VectorDouble()
    ret = nrc.get_current_position(fd, 0, pos)
    if ret != nrc.SUCCESS:
        raise RuntimeError(f"读关节角失败 {ret}")
    return list(pos)


def power_on(fd, timeout=90.0):
    """官方顺序：设模式 → clear_error →（若3则下电）→（若非0则置0）→ 置1 → 上电 → 轮询到 3。

    ⚠️ 2026-11-06 真机实测：少了「设模式」这一步，示教器操作过之后
    set_servo_poweron 会稳定返回 RECEIVE_FAILED(-1)，控制器只在示教器上
    提示「指令 0x2002 参数错误」，SDK 侧看不到原因。
    """
    print("设模式 set_current_mode(0) ->", nrc.set_current_mode(fd, 0))
    time.sleep(0.3)
    print("清报警 clear_error        ->", nrc.clear_error(fd))
    st = read_state(fd)
    if st == 3:
        print("先下电 set_servo_poweroff ->", nrc.set_servo_poweroff(fd))
        st = read_state(fd)
    if st != 0:
        print(f"归零 set_servo_state(0)   -> {nrc.set_servo_state(fd, 0)}  (当前{st})")
        time.sleep(0.5)
        st = read_state(fd)
    print("使能 set_servo_state(1)   ->", nrc.set_servo_state(fd, 1))
    time.sleep(0.5)
    ret = nrc.set_servo_poweron(fd)
    if ret != nrc.SUCCESS:
        time.sleep(0.5)
        ret = nrc.set_servo_poweron(fd)
    print("上电 set_servo_poweron    ->", ret)

    t0 = time.time()
    while time.time() - t0 < timeout:
        st = read_state(fd)
        if st == 3:
            print(f"伺服已使能（状态 3），耗时 {time.time()-t0:.1f}s")
            return
        time.sleep(0.3)
    raise RuntimeError(f"上电超时，最后状态 {st} {SERVO.get(st)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.2.14")
    ap.add_argument("--port", default="6001")
    ap.add_argument("--axis", type=int, default=0, help="轴号 0-5（0 起）")
    ap.add_argument("--dir", default="+", choices=["+", "-"], help="转动方向")
    ap.add_argument("--speed", type=int, default=5, help="速度百分比 1-100")
    ap.add_argument("--deg", type=float, default=3.0, help="该关节最多转过多少度")
    ap.add_argument("--off", action="store_true", help="跑完下电")
    args = ap.parse_args()

    if not 0 <= args.axis <= 5:
        raise SystemExit("--axis 只能是 0-5")
    if not 1 <= args.speed <= 100:
        raise SystemExit("--speed 只能是 1-100")

    fd = nrc.connect_robot(args.ip, str(args.port))
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS:
        if time.time() > t:
            raise SystemExit("等待连接就绪超时")
        time.sleep(0.2)
    print(f"已连接 {args.ip}:{args.port}  SDK {nrc.get_library_version()}")

    jogging = False
    try:
        st = read_state(fd)
        if st != 3:
            power_on(fd)
        else:
            print("伺服已是状态 3，跳过上电")

        print(f"设速度 set_speed({args.speed}) ->", nrc.set_speed(fd, args.speed))

        before = read_joints(fd)
        print(f"点动前 {args.axis} 号轴 {before[args.axis]:9.3f} 度")
        print(f"开始点动：{JOG_AXIS_NAME[args.axis]}  方向 {args.dir}  "
              f"速度 {args.speed}%  上限 {args.deg} 度")
        print("（点动是保持式：每 50ms 重发一次，200ms 不重发控制器自动停）")

        # 注意：dir 的 True 到底对应正转还是反转，文档没写，实测才知道。
        # 所以这里一律用「角度变化量的绝对值」判停 —— 方向猜反也不会跑飞。
        sign = 1.0 if args.dir == "+" else -1.0
        start = before[args.axis]
        t0 = time.time()
        moved = 0.0
        last_log = 0.0
        while time.time() - t0 < 10.0:            # 最多 10 s，兜底
            nrc.robot_start_jogging(fd, args.axis, sign > 0)
            jogging = True
            time.sleep(0.05)
            moved = abs(read_joints(fd)[args.axis] - start)
            if time.time() - last_log > 0.5:
                print(f"  {time.time()-t0:5.2f}s  已转 {moved:6.3f} 度")
                last_log = time.time()
            if moved >= args.deg:
                break
            if time.time() - t0 > 2.0 and moved < 0.01:
                print("  警告：指令被接受但关节 2 秒没动 —— 检查伺服状态/模式/是否被示教器占用")
                break
        nrc.robot_stop_jogging(fd, args.axis)
        jogging = False

        time.sleep(0.5)                            # 等它减速停稳
        after = read_joints(fd)
        delta = after[args.axis] - start
        print(f"停止 robot_stop_jogging -> 0")
        print(f"点动后 {args.axis} 号轴 {after[args.axis]:9.3f} 度"
              f"   实际转过 {delta:+.3f} 度")
        print(f"全部关节 {[round(v, 3) for v in after]}")

        if args.off:
            nrc.robot_stop_jogging(fd, args.axis)
            print("下电 set_servo_state(0) ->", nrc.set_servo_state(fd, 0))
            print("下电 set_servo_poweroff ->", nrc.set_servo_poweroff(fd))
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，立刻停")
    finally:
        try:
            if jogging:
                nrc.robot_stop_jogging(fd, args.axis)
                print("已发停点动")
        except Exception:
            pass
        nrc.disconnect_robot(fd)
        print("已断开连接")


if __name__ == "__main__":
    main()
