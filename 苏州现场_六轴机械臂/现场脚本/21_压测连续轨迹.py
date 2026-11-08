# 21 连续轨迹压测 —— 50 轮，测队列下发链路的稳定性和重复定位精度
#
# 跟 13/14 号那两个关节巡演压测的区别：
#   那两个压的是 moveJ 逐点下发（关节空间），一次一条指令，等它到位再发下一条。
#   这个压的是「把整条曲线一次性塞进队列再一起下发 + pl 平滑」这条链路，
#   也就是 20_轨迹秀.py 用的方式 —— 今晚这条链路把控制器 SDK 服务打挂过一次，
#   所以必须反复压，看它扛不扛得住。
#
# 每轮：圆(20段) → 锥形螺旋(24段) → 回圆心
# 测量：
#   * 每轮耗时
#   * 回圆心后 TCP 与基准的偏差（mm）
#   * 回圆心后各关节角与基准的偏差（度，取绝对值最大）
#   * 掉使能/重试次数
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 21_压测连续轨迹.py                 # 预演
#   ~/miniforge3/envs/arm38/bin/python 21_压测连续轨迹.py --go --rounds 50
# 结果同时打印并写入 /tmp/stress_cont.csv

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


def connect_retry(tries=8, gap=10.0):
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
    while time.time() - t0 < timeout:
        if S(fd) != 3:
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
        time.sleep(0.1)
    return False, time.time() - t0


def single_moveL(fd, p, vel=300.0):
    if nrc.queue_motion_push_back_moveL(fd, make_cmd(p, vel, 40, 0)) != nrc.SUCCESS:
        return False
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        return False
    ok, _ = wait_arrive(fd, p[:3], 60.0)
    return ok


def run_batch(fd, pts, vel, acc, pl):
    for p in pts:
        if nrc.queue_motion_push_back_moveL(fd, make_cmd(p, vel, acc, pl)) != nrc.SUCCESS:
            return False
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        return False
    ok, _ = wait_arrive(fd, pts[-1][:3], 120.0)
    return ok


def P6(x, y, z, spin_deg=0.0):
    return (x, y, z, PI, 0.0, RZ_DOWN + math.radians(spin_deg))


def build_path():
    """一条固定的、固定点数的复合轨迹"""
    circle = []
    n = 20
    for k in range(n + 1):
        a = 2 * PI * k / n
        circle.append(P6(CX + 170 * math.cos(a), CY + 170 * math.sin(a), CZ,
                         360.0 * k / n))
    cone = []
    n = 24
    for k in range(n + 1):
        t = k / float(n)
        phi = 2 * PI * 2.0 * t
        b = math.radians(30.0) * math.sin(2 * PI * t)
        cone.append((CX + 170 * math.cos(phi), CY + 170 * math.sin(phi),
                     CZ + 150.0 * math.sin(2 * PI * t),
                     PI + b * math.sin(phi), b * math.cos(phi), RZ_DOWN))
    return [("圆", circle), ("锥形螺旋", cone)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--rounds", type=int, default=50)
    ap.add_argument("--vel", type=float, default=800.0)
    ap.add_argument("--acc", type=float, default=60.0)
    ap.add_argument("--pl", type=int, default=5)
    ap.add_argument("--speed", type=int, default=90)
    args = ap.parse_args()

    path = build_path()
    npts = sum(len(p) for _, p in path)
    print("压测内容：每轮 %s，共 %d 个点，%d 轮"
          % (" + ".join(n for n, _ in path), npts, args.rounds))
    print("参数：vel=%.0f acc=%.0f pl=%d 全局速度 %d%%"
          % (args.vel, args.acc, args.pl, args.speed))
    if not args.go:
        print("\n（预演，没连机器人。加 --go 真跑）")
        return

    fd = connect_retry()
    if fd <= 0:
        raise SystemExit("连不上控制器")
    print("已连接 %s\n" % time.strftime("%H:%M:%S"))

    rows = []
    n_rearm = 0
    n_fail = 0
    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        queue_on(fd, args.speed)

        # 建立基准：先回圆心两次，取第二次
        print("建立基准位姿 ...")
        for _ in range(2):
            if not single_moveL(fd, P6(CX, CY, CZ)):
                raise SystemExit("回不到圆心")
        q_ref = J(fd)
        p_ref = T(fd)
        print("基准 q = %s" % ["%.3f" % v for v in q_ref[:6]])
        print("基准 TCP = %s\n" % ["%.3f" % v for v in p_ref[:3]])

        t_start = time.time()
        for r in range(1, args.rounds + 1):
            t0 = time.time()
            ok = True
            # 掉使能就重试一次
            for name, pts in path:
                single_moveL(fd, pts[0])
                if not run_batch(fd, pts, args.vel, args.acc, args.pl):
                    if S(fd) != 3:
                        n_rearm += 1
                        print("  [第 %d 轮] 掉使能，重新上电" % r)
                        if not power_on(fd):
                            raise SystemExit("无法恢复上电")
                        queue_on(fd, args.speed)
                        ok = False
                        break
                    ok = False
                    break
            if not ok:
                n_fail += 1
                print("  [第 %d 轮] 未完成" % r)
                continue

            if not single_moveL(fd, P6(CX, CY, CZ)):
                n_fail += 1
                print("  [第 %d 轮] 回圆心失败" % r)
                continue

            q = J(fd)
            p = T(fd)
            dtcp = dist(p, p_ref)
            dq = max(abs(q[i] - q_ref[i]) for i in range(6))
            dt = time.time() - t0
            rows.append((r, dt, dtcp, dq, n_rearm))
            if r % 5 == 0 or r == 1 or r == args.rounds:
                print("  第 %3d 轮  %5.1fs  TCP偏差 %6.3f mm  关节偏差 %6.4f°  累计重试 %d"
                      % (r, dt, dtcp, dq, n_rearm))

        total = time.time() - t_start
        print("\n=== 压测结果 ===")
        print("轮数        : %d / %d" % (len(rows), args.rounds))
        print("总耗时      : %.1f 分钟" % (total / 60.0))
        if rows:
            print("平均每轮    : %.1f 秒" % (sum(r[1] for r in rows) / len(rows)))
            dtcps = [r[2] for r in rows]
            dqs = [r[3] for r in rows]
            print("TCP 回位偏差: 平均 %.4f mm  最大 %.4f mm" % (sum(dtcps) / len(dtcps), max(dtcps)))
            print("关节 回位偏差: 平均 %.5f°  最大 %.5f°" % (sum(dqs) / len(dqs), max(dqs)))
        print("掉使能重试  : %d 次" % n_rearm)
        print("未完成轮数  : %d" % n_fail)

        with open("/tmp/stress_cont.csv", "w") as f:
            f.write("round,seconds,tcp_err_mm,joint_err_deg,rearmer_count\n")
            for r in rows:
                f.write("%d,%.2f,%.4f,%.5f,%d\n" % r)
        print("\n已写 /tmp/stress_cont.csv")

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
        print("已下电断开 %s" % time.strftime("%H:%M:%S"))


if __name__ == "__main__":
    sys.exit(main())
