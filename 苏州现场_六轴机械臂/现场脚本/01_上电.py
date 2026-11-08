# 01 机械臂上电 / 下电 —— 纳博特 NexDroid（nrc_interface）
#
# 只做一件事：把伺服从「停止」推到「运行」，或者反过来。不发任何运动指令。
#
# 跑法：
#   上电：~/miniforge3/envs/arm38/bin/python 01_上电.py --ip 192.168.2.14
#   下电：~/miniforge3/envs/arm38/bin/python 01_上电.py --ip 192.168.2.14 --off
#
# 安全约定（别改）：
#   - 上电前手放在急停上。上电瞬间抱闸松开，臂可能轻微抖一下，属正常。
#   - 命令返回成功 ≠ 已经上电，必须轮询到状态 3。
#   - 本脚本永不自动下电（finally 里只断开连接）。
#     上电失败要反复试的时候，自动下电会让你分不清"是上不去"还是"被下了"。

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}


def read_state(fd):
    """返回伺服状态 0/1/2/3。注意 SDK 返回的是 [返回码, 状态] 两元组。"""
    ret, st = nrc.get_servo_state(fd, 0)
    if ret != nrc.SUCCESS:
        raise RuntimeError(f"读伺服状态失败，返回码 {ret}")
    return st


def read_joints(fd):
    """返回 7 个关节角（度）。第 7 个是外部轴，通常恒为 0。"""
    pos = nrc.VectorDouble()
    ret = nrc.get_current_position(fd, 0, pos)      # 0 = 关节坐标
    if ret != nrc.SUCCESS:
        raise RuntimeError(f"读关节角失败，返回码 {ret}")
    return list(pos)


def fmt(vals):
    return " ".join(f"{v:9.3f}" for v in vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.2.14", help="控制器 IP")
    ap.add_argument("--port", default="6001", help="上位机 SDK 端口")
    ap.add_argument("--off", action="store_true", help="下电，而不是上电")
    ap.add_argument("--timeout", type=float, default=90.0, help="等待上电的秒数")
    args = ap.parse_args()

    fd = nrc.connect_robot(args.ip, str(args.port))
    if fd <= 0:
        raise SystemExit(f"连接失败 {args.ip}:{args.port}")
    deadline = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS:
        if time.time() > deadline:
            raise SystemExit("连接后等待就绪超时")
        time.sleep(0.2)
    print(f"已连接 {args.ip}:{args.port}  SDK {nrc.get_library_version()}")

    try:
        st = read_state(fd)
        before = read_joints(fd)
        print(f"当前伺服状态 {st}  ({SERVO.get(st, '?')})")
        print(f"当前关节角   {fmt(before)}")

        if args.off:
            print("下电：set_servo_state(0)")
            print("  ->", nrc.set_servo_state(fd, 0))
            print("下电：set_servo_poweroff")
            print("  ->", nrc.set_servo_poweroff(fd))
        else:
            if st == 3:
                print("已经是运行状态，跳过上电。")
            else:
                # 官方顺序：设模式 → clear_error →（若3则下电）→（若非0则置0）→ 置1 → 上电。
                # 少"设模式"这一步：示教器动过之后 poweron 会稳定返回 RECEIVE_FAILED(-1)。
                # 少"归零"这一步：状态不为 0 时直接置 1 再上电也可能失败。
                print("设模式：set_current_mode(0) ->", nrc.set_current_mode(fd, 0))
                time.sleep(0.3)
                print("清报警：clear_error        ->", nrc.clear_error(fd))
                st = read_state(fd)
                if st == 3:
                    print("先下电：set_servo_poweroff ->", nrc.set_servo_poweroff(fd))
                    st = read_state(fd)
                if st != 0:
                    print(f"归零：set_servo_state(0) (当前{st}) ->", nrc.set_servo_state(fd, 0))
                    time.sleep(0.5)
                    st = read_state(fd)
                print("使能：set_servo_state(1)   ->", nrc.set_servo_state(fd, 1))
                time.sleep(0.5)
                ret = nrc.set_servo_poweron(fd)
                if ret != nrc.SUCCESS:          # RECEIVE_FAILED 偶发，重试一次
                    time.sleep(0.5)
                    print(f"上电：set_servo_poweron    -> {ret}，重试一次")
                    ret = nrc.set_servo_poweron(fd)
                print(f"上电：set_servo_poweron    -> {ret}")
                print("命令已下发，开始轮询（命令成功 ≠ 已上电）…")
                t0 = time.time()
                last = None
                while time.time() - t0 < args.timeout:
                    st = read_state(fd)
                    if st != last:
                        print(f"  {time.time() - t0:6.1f}s  状态 {st}  ({SERVO.get(st, '?')})")
                        last = st
                    if st == 3:
                        break
                    time.sleep(0.5)
                if st != 3:
                    raise SystemExit(f"上电超时 {args.timeout:.0f}s，最后状态 {st} "
                                     f"({SERVO.get(st, '?')})，去看示教器有没有报警")

        after = read_joints(fd)
        print(f"最终伺服状态 {read_state(fd)}  ({SERVO.get(read_state(fd), '?')})")
        print(f"最终关节角   {fmt(after)}")
        delta = [b - a for a, b in zip(before, after)]
        print(f"关节变化     {fmt(delta)}   最大 {max(abs(x) for x in delta):.3f} 度")
    finally:
        nrc.disconnect_robot(fd)
        print("已断开连接（伺服状态保持不变）")


if __name__ == "__main__":
    main()
