# 20 轨迹秀 —— 圆 / 五角星 / 锥形螺旋 / 外扩螺旋
#
# 轨迹特点：
#   1. 圆      ：24 段连成一条，工具同时自转 360°
#   2. 五角星  ：5 段长直线，最能冲速度
#   3. 锥形螺旋：绕圈 2 圈 + Z 起伏 150mm + 工具绕 30° 圆锥摆
#               → J1 J2 J3 J4 J5 J6 全部参与（前两个形状里 J3/J4 几乎不动）
#   4. 外扩螺旋：R60→180
#
# 关键实现：把整条曲线的点一次性 push 进运动队列，每段 pl=5（平滑等级 0~5），
#          再 queue_motion_send_to_controller(fd, 0, False) 一次下发，
#          控制器会连成一条不断点的轨迹。队列上限 31，所以每批 ≤30 点。
#
# 判到位：不看 queue_motion_get_queuelen（那个接口 Python 签名对不上，v1 就是卡在这），
#        改成「TCP 到达末点 1mm 内 + 连续 5 次关节角几乎不动」。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 20_轨迹秀.py                     # 预演
#   ~/miniforge3/envs/arm38/bin/python ~/projects/07_机器人学/练习/机械臂现场/20_轨迹秀.py --go --vel 800 --acc 60 --speed 90

import argparse
import math
import sys
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 8.0
CX, CY, CZ = 1000.0, 0.0, 1250.0
PI = math.pi
RZ_DOWN = -PI / 2


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
    return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5


def connect_retry(tries=6, gap=8.0):
    for i in range(tries):
        fd = nrc.connect_robot("192.168.2.14", "6001")
        if fd > 0:
            t = time.time() + 10
            while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
                time.sleep(0.2)
            if nrc.get_connection_status(fd) == nrc.SUCCESS:
                return fd
            nrc.disconnect_robot(fd)
        print("  连接失败（第 %d 次），%.0f 秒后重试" % (i + 1, gap))
        time.sleep(gap)
    return -1


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
                print("  上电成功（第 %d 次）" % (i + 1))
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def queue_on(fd, speed):
    nrc.set_speed(fd, speed)
    nrc.queue_motion_set_status(fd, False)
    time.sleep(0.4)
    nrc.queue_motion_set_status(fd, True)


def make_cmd(p, vel, acc, pl):
    cmd = nrc.MoveCmd()
    cmd.coord = 1
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble([p[0], p[1], p[2], p[3], p[4], p[5], 0.0])
    cmd.velocity = vel
    cmd.acc = acc
    cmd.dec = acc
    cmd.pl = pl
    return cmd


def wait_arrive(fd, tgt, timeout):
    t0 = time.time()
    prev = J(fd)
    stable = 0
    last = 0.0
    while time.time() - t0 < timeout:
        if S(fd) != 3:
            print("    伺服掉到 %s" % SERVO.get(S(fd), S(fd)))
            return False, time.time() - t0
        js = J(fd)
        moved = max(abs(a - b) for a, b in zip(prev, js))
        prev = js
        d = dist(T(fd), tgt)
        if moved > JUMP_MAX:
            stable = 0
            time.sleep(0.2)
            continue
        if d > 1.0:
            stable = 0
        elif moved < 0.02:
            stable += 1
        else:
            stable = 0
        if stable >= 5:
            return True, time.time() - t0
        now = time.time() - t0
        if now - last > 2.0:
            last = now
            print("      距末点 %7.1f mm，已用 %4.1f s" % (d, now))
        time.sleep(0.12)
    print("    超时 %.0fs" % timeout)
    return False, time.time() - t0


def single_moveL(fd, p, vel=250.0):
    if nrc.queue_motion_push_back_moveL(fd, make_cmd(p, vel, 30, 0)) != nrc.SUCCESS:
        return False
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        return False
    ok, _ = wait_arrive(fd, p[:3], 90.0)
    return ok


def run_batch(fd, pts, vel, acc, pl, label):
    L = sum(dist(pts[i - 1][:3], pts[i][:3]) for i in range(1, len(pts)))
    if len(pts) > 30:
        raise SystemExit("一批 %d 段，超过队列上限 31" % len(pts))
    for p in pts:
        if nrc.queue_motion_push_back_moveL(fd, make_cmd(p, vel, acc, pl)) != nrc.SUCCESS:
            print("    push 失败")
            return
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        print("    send 失败")
        return
    ok, dt = wait_arrive(fd, pts[-1][:3], 240.0)
    if ok:
        print("    %-26s %6.0f mm / %5.2f s = %5.1f mm/s"
              % (label, L, dt, L / dt if dt > 0 else 0))


def P6(x, y, z, spin_deg=0.0):
    return (x, y, z, PI, 0.0, RZ_DOWN + math.radians(spin_deg))


def make_shapes(n_circle=20, n_cone=24, dz=150.0, tilt_deg=30.0):
    out = []
    # 1 圆 + 自转
    pts = []
    for k in range(n_circle + 1):
        a = 2 * PI * k / n_circle
        pts.append(P6(CX + 170 * math.cos(a), CY + 170 * math.sin(a), CZ,
                      360.0 * k / n_circle))
    out.append(("圆 R170 + 自转360°", pts))

    # 2 五角星
    pts = []
    for k in range(6):
        a = math.radians(-90 + 144 * k)
        pts.append(P6(CX + 170 * math.cos(a), CY + 170 * math.sin(a), CZ))
    out.append(("五角星 R170", pts))

    # 3 锥形螺旋：绕 2 圈 + Z 起伏 + 工具绕 30° 圆锥摆
    pts = []
    for k in range(n_cone + 1):
        t = k / float(n_cone)
        phi = 2 * PI * 2.0 * t
        x = CX + 170 * math.cos(phi)
        y = CY + 170 * math.sin(phi)
        z = CZ + dz * math.sin(2 * PI * t)
        b = math.radians(tilt_deg) * math.sin(2 * PI * t)
        rx = PI + b * math.sin(phi)
        ry = b * math.cos(phi)
        pts.append((x, y, z, rx, ry, RZ_DOWN))
    out.append(("锥形螺旋(全轴)", pts))

    # 4 外扩螺旋
    pts = []
    n = 28
    for k in range(n + 1):
        t = k / float(n)
        r = 60 + 120 * t
        a = 2 * PI * 1.5 * t
        pts.append(P6(CX + r * math.cos(a), CY + r * math.sin(a), CZ))
    out.append(("外扩螺旋 R60→180", pts))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--vel", type=float, default=800.0)
    ap.add_argument("--acc", type=float, default=60.0)
    ap.add_argument("--pl", type=int, default=5)
    ap.add_argument("--speed", type=int, default=90)
    ap.add_argument("--segs", type=int, default=20)
    ap.add_argument("--dz", type=float, default=150.0)
    ap.add_argument("--tilt", type=float, default=30.0)
    args = ap.parse_args()

    sh = make_shapes(args.segs, 24, args.dz, args.tilt)
    total = 0.0
    print("轨迹规划：")
    for name, pts in sh:
        L = sum(dist(pts[i - 1][:3], pts[i][:3]) for i in range(1, len(pts)))
        total += L
        print("  %-26s %2d 段  路径 %6.0f mm" % (name, len(pts) - 1, L))
    print("  总路径 %.0f mm  vel=%.0f acc=%.0f pl=%d 全局 %d%%"
          % (total, args.vel, args.acc, args.pl, args.speed))
    if not args.go:
        print("\n（预演，没连机器人。加 --go 真跑）")
        return

    fd = connect_retry()
    if fd <= 0:
        raise SystemExit("连不上")
    print("已连接 %s" % time.strftime("%H:%M:%S"))
    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        queue_on(fd, args.speed)
        print("当前 TCP = %s" % [round(v, 1) for v in T(fd)[:3]])

        print("\n回圆心 ...")
        if not single_moveL(fd, P6(CX, CY, CZ)):
            raise SystemExit("回不到圆心")

        print("\n=== 开始 ===")
        t_all = time.time()
        for name, pts in sh:
            print("\n[%s]" % name)
            single_moveL(fd, pts[0])
            run_batch(fd, pts, args.vel, args.acc, args.pl, name)
        print("\n全部完成，总耗时 %.1f s" % (time.time() - t_all))

        print("\n回圆心 ...")
        single_moveL(fd, P6(CX, CY, CZ))
        print("结束 TCP = %s" % [round(v, 1) for v in T(fd)[:3]])
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
    sys.exit(main())
