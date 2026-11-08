# 35 仿真 vs 真机 对齐度测试
#
# 自动依次走到 8 个位姿，每个位姿停 40 秒让你量，最后回零位。
# 你只需要：每个位姿量一次法兰离地，记下来。
#
# 跑法：~/miniforge3/envs/arm38/bin/python 35_对齐测试.py --go
#       ~/miniforge3/envs/arm38/bin/python 35_对齐测试.py --go --dwell 60

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
D = 0.001

# (关节角 deg, 仿真预测法兰中心离地 mm, 水平半径 mm, 盘面倾角度)
# 量法：法兰盘【上沿】和【下沿】各量一次，取平均 = 中心高度（对任何倾角都成立）
POSES = [
    ((0,    0,   0,  0,   0, 0), 1581,  920,  0),
    ((0,  -20,  10,  0,   0, 0), 1403, 1183, 10),
    ((0,  -30,  30,  0,   0, 0), 1475, 1315,  0),
    ((0,   15, -15,  0,   0, 0), 1554,  716,  0),
    ((0,    0,  40,  0,   0, 0), 2058,  729, 40),
    ((0,  -10,  20,  0,  45, 0), 1721,  943, 35),
    ((30, -15,  15,  0,   0, 0), 1554, 1124,  0),
    ((0,  -30,  50,  0, -30, 0), 1770, 1318, 50),
]

# 腕部（J4~J6）和基座都不动，只转 q2/q3/q5，走的路径短
JOINTS = [1, 2, 4]


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


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
    for _ in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def movej(fd, target, speed, limit=90.0):
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(target)
    cmd.velocity = speed
    cmd.acc = 10
    cmd.dec = 10
    cmd.pl = 0
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        return False
    if nrc.queue_motion_send_to_controller(fd, 1) != nrc.SUCCESS:
        return False
    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < limit:
        if S(fd) != 3:
            return False
        js = J(fd)
        if max(abs(a - b) for a, b in zip(prev, js)) > 5.0:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.10:
            return True
        time.sleep(0.08)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--dwell", type=float, default=40.0, help="每个位姿停留几秒")
    ap.add_argument("--speed", type=int, default=8)
    args = ap.parse_args()

    print("=" * 70)
    print("仿真 vs 真机 对齐度测试   共 %d 个位姿，每个停 %.0f 秒" % (len(POSES), args.dwell))
    print("=" * 70)
    for i, (q, zc, r, tilt) in enumerate(POSES, 1):
        print("  位姿 %d: q=%-28s 法兰中心应 %.0f mm  (盘面倾角 %d 度)"
              % (i, " ".join("%4d" % v for v in q), zc, tilt))
    if not args.go:
        print("\n（预演，没动。加 --go 才走）")
        return

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("\n已连接 %s" % time.strftime("%H:%M:%S"))
    if not power_on(fd):
        raise SystemExit("上电失败")
    time.sleep(0.6)
    nrc.set_speed(fd, args.speed)
    nrc.queue_motion_set_status(fd, False)
    time.sleep(0.4)
    nrc.queue_motion_set_status(fd, True)

    try:
        for i, (q, zc, r, tilt) in enumerate(POSES, 1):
            tgt = [float(v) for v in q] + [0.0]
            ok = False
            for _ in range(3):
                if movej(fd, tgt, args.speed):
                    ok = True
                    break
                power_on(fd)
                time.sleep(0.8)
            if not ok:
                print("!! 位姿 %d 没走到，跳过" % i)
                continue
            time.sleep(0.8)
            cur = J(fd)
            print("\n" + "-" * 70)
            print("【位姿 %d/%d】 实际 q = %s" % (i, len(POSES), ["%.1f" % v for v in cur[:6]]))
            print("   仿真预测：法兰中心 %.0f mm，水平半径 %.0f mm，盘面倾角 %d 度"
                  % (zc, r, tilt))
            print("   >>> 量法兰盘【上沿】和【下沿】，取平均 = 中心，看是否 ≈ %.0f mm" % zc)
            print("       量完等下一个位姿（%.0f 秒）" % args.dwell)
            print("-" * 70)
            time.sleep(args.dwell)

        print("\n走完 8 个位姿，回零位 ...")
        movej(fd, [0.0] * 7, args.speed)
        time.sleep(1.0)
        print("已回零位。q = %s" % ["%.2f" % v for v in J(fd)[:6]])
    finally:
        print("保持上电中。")


if __name__ == "__main__":
    main()
