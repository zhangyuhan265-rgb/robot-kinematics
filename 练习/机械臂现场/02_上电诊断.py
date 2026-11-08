# 02 上电失败诊断 —— 纳博特 NexDroid
#
# 依据：官方《基础连接与系统接口》§上下电失败排查
#   set_servo_poweron 返回 RECEIVE_FAILED(= -1) 时，按顺序排除：
#     ① 是否被占用 → 按官方顺序重跑 clear_error →（状态3则下电）→（非0则置0）→ 置1 → 上电
#     ② 模式：示教 / 远程 / 运行 各试一次下电（三种都失败 = 与模式无关）
#     ③ 读 JSON 命令字 0x3034 看使能方式（mode=0 即软件触发）
#     ④ 用不存在的机器人编号（robot=2）探测命令字是否已实现
#        → 返回"参数错误"(-3) 说明已实现；返回 -1 说明固件不答这条命令
#
# 全程只有可逆操作：清报警 / 置状态 / 试上电 / 试下电。没有任何运动指令。

import time

import nrc_interface as nrc

CODE = {0: "SUCCESS", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "OPERATION_NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}


def name(code):
    return f"{code} ({CODE.get(code, '?' )})"


def state(fd):
    ret, st = nrc.get_servo_state(fd, 0)
    return st if ret == nrc.SUCCESS else f"?({ret})"


def step(label, fn, *a):
    r = fn(*a)
    print(f"  {label:38s} -> {name(r)}")
    return r


def main():
    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS:
        if time.time() > t:
            raise SystemExit("等待连接就绪超时")
        time.sleep(0.2)
    print(f"已连接  SDK {nrc.get_library_version()}\n")

    try:
        print("【① 按官方顺序重跑】")
        print(f"  起始状态 {state(fd)} {SERVO.get(state(fd), '')}")
        step("clear_error", nrc.clear_error, fd)
        st = state(fd)
        print(f"  清报警后状态 {st} {SERVO.get(st, '')}")
        if st == 3:
            step("set_servo_poweroff (状态3先下电)", nrc.set_servo_poweroff, fd)
            st = state(fd)
        if st != 0:
            step(f"set_servo_state(0) (当前{st})", nrc.set_servo_state, fd, 0)
            time.sleep(0.5)
            st = state(fd)
            print(f"  归零后状态 {st} {SERVO.get(st, '')}")
        step("set_servo_state(1)", nrc.set_servo_state, fd, 1)
        time.sleep(0.5)
        st = state(fd)
        print(f"  置1后状态 {st} {SERVO.get(st, '')}")

        print("\n【① 上电】")
        r_on = step("set_servo_poweron", nrc.set_servo_poweron, fd)
        for _ in range(6):
            time.sleep(0.5)
            print(f"    状态 {state(fd)} {SERVO.get(state(fd), '')}")

        print("\n【④ 探测已实现的命令字（不存在的机器人 2）】")
        r_po2 = step("set_servo_poweron_robot(fd, 2)", nrc.set_servo_poweron_robot, fd, 2)
        r_pf2 = step("set_servo_poweroff_robot(fd, 2)", nrc.set_servo_poweroff_robot, fd, 2)

        print("\n【对照】")
        step("set_servo_poweroff", nrc.set_servo_poweroff, fd)
        time.sleep(0.5)
        print(f"  下电后状态 {state(fd)} {SERVO.get(state(fd), '')}")

        print("\n【结论】")
        if r_on == nrc.SUCCESS:
            print("  上电命令被接受 → 之前失败是【①被占用/状态顺序】问题，按官方顺序重跑已修复")
        elif r_po2 == nrc.PARAM_ERR:
            print("  robot=2 返回 PARAM_ERR → 该命令字【已实现】，固件认得它")
            print("  ⇒ 上电失败不是固件不支持，而是被拒绝（模式 / 安全回路 / 使能方式 / 示教器占用）")
        elif r_po2 == nrc.RECEIVE_FAILED:
            print("  robot=2 也返回 RECEIVE_FAILED → 该命令字【本机固件不答】")
            print("  ⇒ 与模式无关，是固件未实现这条上下电命令，需走上电方式确认（0x3034）")
        else:
            print(f"  robot=2 返回 {name(r_po2)}，需人工判断")
    finally:
        nrc.disconnect_robot(fd)
        print("\n已断开（伺服状态保持不变）")


if __name__ == "__main__":
    main()
