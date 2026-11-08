# 10 标定 moveL 速度 —— 纳博特 NexDroid（nrc_interface）
#
# moveL 的 cmd.velocity 实际含义不明：实测 --vel 5 时末端只有 0.34 mm/s。
# 本脚本在同一次上电里，用同样的 4 mm 直线、不同 velocity 各走一次，
# 量出「velocity -> 实际 mm/s」的对应关系。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 10_标定moveL.py
#
# 安全：沿 X 轴走 4 mm 再返回，Z 和姿态不动。全程监视伺服状态和读数合理性。

import argparse
import time

import nrc_interface as nrc

CODE = {0: "OK", -1: "RECEIVE_FAILED", -2: "DISCONNECT", -3: "PARAM_ERR",
        -4: "NOT_ALLOWED", -5: "EXCEPTION", -6: "TIMEOUT"}
SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
GLOBAL_SPEED = 30
JUMP_MAX = 5.0

ap = argparse.ArgumentParser()
ap.add_argument("--vels", default="10,60,250", help="逗号分隔的 cmd.velocity 列表")
ap.add_argument("--dist", type=float, default=4.0, help="直线距离 mm")
args = ap.parse_args()

DIST = args.dist
VELS = [float(v) for v in args.vels.split(",")]


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


def dist(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5


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


def line(fd, target, vel, limit=150.0):
    """走一段 moveL，返回 (是否到位, 耗时)。"""
    cmd = nrc.MoveCmd()
    cmd.coord = 1
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = vel
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveL(fd, cmd) != nrc.SUCCESS:
        return False, 0.0
    if nrc.queue_motion_send_to_controller(fd, 1) != nrc.SUCCESS:
        return False, 0.0

    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < limit:
        st = S(fd)
        if st != 3:
            print("    伺服掉到 %s，中止" % SERVO.get(st, st))
            return False, time.time() - t0
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        if dist(T(fd), target) < 0.2:
            return True, time.time() - t0
        time.sleep(0.05)
    print("    超时 %.0fs" % limit)
    nrc.queue_motion_stop(fd)
    return False, limit


fd = nrc.connect_robot("192.168.2.14", "6001")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)
print("已连接  %s" % time.strftime("%H:%M:%S"))

try:
    print("上电：")
    if not power_on(fd):
        raise SystemExit("上电失败")
    wait_stable(fd)
    home = T(fd)
    # 沿 X 走，目标 = home + DIST
    out = list(home)
    out[0] = home[0] + DIST
    print("起点 TCP %s" % [round(v, 2) for v in home[:6]])
    print("目标    %s" % [round(v, 2) for v in out[:6]])

    nrc.set_speed(fd, GLOBAL_SPEED)
    nrc.queue_motion_set_status(fd, False)
    time.sleep(0.4)
    nrc.queue_motion_set_status(fd, True)
    print("全局速度 %d%%\n" % GLOBAL_SPEED)

    results = []
    for vel in VELS:
        print("--- cmd.velocity = %d ---" % vel)
        ok, dt = line(fd, out, vel)
        if ok:
            rate = DIST / dt if dt > 0 else 0.0
            print("    出去 %.2fs  -> %.2f mm/s" % (dt, rate))
        else:
            rate = None
            print("    出去没走成")
        time.sleep(0.5)
        ok2, dt2 = line(fd, home, vel)
        if ok2:
            print("    返回 %.2fs" % dt2)
        else:
            print("    返回没走成")
            break
        results.append((vel, rate))
        time.sleep(0.5)

    print("\n=== 标定结果 ===")
    print(" velocity | 实测 mm/s")
    for vel, rate in results:
        print(" %8d | %s" % (vel, "%.3f" % rate if rate else "失败"))
    print("\n结束 TCP %s" % [round(v, 2) for v in T(fd)[:6]])
finally:
    try:
        nrc.queue_motion_stop(fd)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, False)
        print("已关闭运动队列")
    except Exception as e:
        print("关闭队列失败：%s" % e)
    st = S(fd)
    if st in (2, 3):
        print("下电 poweroff -> %s" % n(nrc.set_servo_poweroff(fd)))
        time.sleep(0.4)
        st = S(fd)
    if st != 0:
        print("下电 state(0) -> %s" % n(nrc.set_servo_state(fd, 0)))
    nrc.disconnect_robot(fd)
    print("已断开  %s" % time.strftime("%H:%M:%S"))
