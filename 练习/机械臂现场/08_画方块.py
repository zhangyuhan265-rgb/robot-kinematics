# 08 画方块 —— 笛卡尔直线运动 moveL 演示
#
# 用 4 段 moveL 让 TCP 在一个平面上画出一个方块，最后回到原点。
# 这是「墙面打印」的原型：末端按直线走，不是各关节各转各的。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 08_画方块.py --size 15 --speed 5
#
# ⚠️ 第一次发 moveL，注意三件事：
#   1. coord=1 是直角坐标，单位按「读取时的 mm」推断，未独立验证。
#      万一不对，目标会远超工作空间，控制器应当拒绝（而不是乱走）——
#      「应当」不等于「保证」，所以尺寸给得很小。
#   2. moveL 会协调多个关节，可能出现腕部翻转或奇异点。所以全程监视
#      每个关节相对起始姿态的偏离，超过 MAX_JOINT_DEV 立刻停队列。
#   3. 姿态（Rx/Ry/Rz）全程保持不变，只改 X/Y。

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
MAX_JOINT_DEV = 15.0      # 度，任一关节偏离起始姿态超过这个就停
JUMP_MAX = 5.0            # 度，0.1 秒内单个关节跳变超过这个就判为无效读数据


def n(c):
    return "%d(%s)" % (c, CODE.get(c, "?"))


def read_state(fd):
    return nrc.get_servo_state(fd, 0)[1]


def read_joints(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    return list(p)


def read_tcp(fd):
    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 1, p)
    return list(p)


def fmt_tcp(t):
    return "X %8.2f  Y %8.2f  Z %8.2f  Rx %6.2f  Ry %6.2f  Rz %6.2f" % tuple(t[:6])


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


def line_to(fd, target, vel, joints0, label):
    """走一段 moveL（coord=1，mm）。返回是否到位。

    ⚠️ 速度模型（2026-11-06 实测推断）：
      cmd.velocity 是物理速度（直角坐标下为 mm/s），
      set_speed 是全局百分比，两者是【相乘】关系。
      所以这里全局速度给满 100，速度只由 vel 一个地方控制，
      否则 5% × 5 mm/s = 0.25 mm/s，慢到会超时。
    """
    print("\n  → %s" % label)
    print("    %s" % fmt_tcp(target))

    cmd = nrc.MoveCmd()
    cmd.coord = 1                                    # 1 = 直角坐标
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = vel
    cmd.acc = 20
    cmd.dec = 20
    cmd.pl = 0
    r = nrc.queue_motion_push_back_moveL(fd, cmd)
    if r != nrc.SUCCESS:
        print("    push_moveL 失败 %s" % n(r))
        return False
    r = nrc.queue_motion_send_to_controller(fd, 1)
    if r != nrc.SUCCESS:
        print("    send 失败 %s" % n(r))
        return False

    t0 = time.time()
    last = 0.0
    prev_joints = read_joints(fd)
    while time.time() - t0 < 40.0:
        # 先查伺服状态：掉电之后控制器会返回垃圾坐标，绝不能采信
        st = read_state(fd)
        if st != 3:
            print("     !! 伺服状态变成 %s，运动中止（后面的读数不可信，不再打印）"
                  % SERVO.get(st, st))
            return False

        js = read_joints(fd)
        # 合理性检查：0.1 秒内单个关节跳变超过 JUMP_MAX 就是无效数据
        jump = max(abs(a - b) for a, b in zip(prev_joints, js))
        if jump > JUMP_MAX:
            print("     !! 读到关节跳变 %.1f 度（> %.0f），判为无效数据，重读"
                  % (jump, JUMP_MAX))
            time.sleep(0.2)
            continue
        prev_joints = js

        cur = read_tcp(fd)
        dev = max(abs(a - b) for a, b in zip(joints0, js))
        d = ((cur[0] - target[0]) ** 2 + (cur[1] - target[1]) ** 2
             + (cur[2] - target[2]) ** 2) ** 0.5
        if time.time() - last > 1.0:
            print("      %5.2fs  距目标 %7.2f mm  关节最大偏离 %5.2f 度  TCP %s"
                  % (time.time() - t0, d, dev, fmt_tcp(cur)))
            last = time.time()
        if dev > MAX_JOINT_DEV:
            print("     !! 关节偏离 %.1f 度，超过上限，停队列" % dev)
            nrc.queue_motion_stop(fd)
            return False
        if d < 0.3:
            print("      到位（%.2fs）" % (time.time() - t0))
            return True
        time.sleep(0.1)
    print("      超时")
    nrc.queue_motion_stop(fd)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=float, default=10.0, help="方块半边长 mm")
    ap.add_argument("--vel", type=float, default=60.0,
                    help="直线速度 cmd.velocity。实测 60→约6mm/s、120→10mm/s、180→12.5mm/s，250 会报警")
    ap.add_argument("--speed", type=int, default=30, help="全局速度百分比，实测 30 最稳最快")
    args = ap.parse_args()

    if args.size > 40:
        raise SystemExit("--size 太大，拒绝（先确认 moveL 单位再放大）")

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
        joints0 = wait_stable(fd)
        home = read_tcp(fd)
        print("起始 TCP %s" % fmt_tcp(home))
        print("起始关节 %s" % [round(v, 3) for v in joints0[:6]])

        s = args.size
        corners = [
            (home[0] + s, home[1] + s, home[2], home[3], home[4], home[5], 0.0),
            (home[0] + s, home[1] - s, home[2], home[3], home[4], home[5], 0.0),
            (home[0] - s, home[1] - s, home[2], home[3], home[4], home[5], 0.0),
            (home[0] - s, home[1] + s, home[2], home[3], home[4], home[5], 0.0),
            list(home),
        ]

        nrc.set_speed(fd, args.speed)
        # 上一轮如果异常退出，队列状态可能是开着的、里面还挂着老指令。
        # 先关再开，把状态摆正（不要用 clear_Data，那会触发 0x3B02）。
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)
        # 注意：不要调 queue_motion_clear_Data —— 早期版本调了它之后
        # 示教器上冒出「指令(0x3B02)参数错误」，是本脚本的头号嫌疑。

        ok = 0
        for i, c in enumerate(corners):
            label = "第 %d 边" % (i + 1) if i < 4 else "回原点"
            if line_to(fd, c, args.vel, joints0, label):
                ok += 1
            else:
                print("  这一段没走成，后面跳过")
                break

        print("\n结果：完成 %d / %d 段" % (ok, len(corners)))
        end = read_tcp(fd)
        print("结束 TCP %s" % fmt_tcp(end))
        print("回位偏差 %.2f mm" % (((end[0] - home[0]) ** 2 + (end[1] - home[1]) ** 2
                                   + (end[2] - home[2]) ** 2) ** 0.5))
        nrc.queue_motion_set_status(fd, False)
    finally:
        # 关键：无论怎么退出，都要先关掉运动队列再下电。
        # 否则残留指令会留在控制器里，下一轮一开队列就一起执行，
        # 臂会在几个老目标之间乱跑（2026-11-06 实测踩过）。
        try:
            nrc.queue_motion_stop(fd)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, False)
            print("\n已关闭运动队列")
        except Exception as e:
            print("\n关闭队列失败：%s" % e)

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
        print("已断开")


if __name__ == "__main__":
    main()
