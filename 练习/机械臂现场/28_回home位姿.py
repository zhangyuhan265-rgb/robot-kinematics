# 28 回 home 位姿 —— 用关节插值回到 12 组标定样本里的那个 home
#
# 为什么用 moveJ 而不是 moveL：
#   控制器里的 DH 是错的（a2 900 vs 真 790 等），任何 Cartesian 目标都会被它算歪，
#   moveL 还会走出一条离谱的曲线。关节角是编码器直接量出来的，不经正解，最安全。
#
# home 位姿（= fk_samples.csv 的 home 行，也是新 URDF 校验用的那个）：
#   q = [-11.382, -7.617, 7.674, -34.306, -30.333, -68.221]
#
# 到了之后按【修正后的模型】法兰面中心应该在：
#   离地 1612.8 mm，离 J1 轴水平 993.7 mm
# 而控制器自己会报 (1305.8, -339.3, 1290.3) —— 水平差了 355 mm，这正是要量的东西。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 28_回home位姿.py           # 只看状态，不动
#   ~/miniforge3/envs/arm38/bin/python 28_回home位姿.py --go      # 真动
# 可选 --release：动完下电（默认保持上电，方便你再来回量）

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0

HOME = [-11.382, -7.617, 7.674, -34.306, -30.333, -68.221]

# 修正模型（彩页长度 + 336.5 垫高）算出来的期望值
EXPECT = {
    "flange_z_mm": 1612.8,      # 法兰面中心离地
    "flange_r_mm": 993.7,       # 法兰面中心离 J1 轴水平距
    "j2_z_mm": 767.5,           # J2 轴离地
}
CTRL_REPORT = (1305.776559, -339.270426, 1290.261209)   # 控制器自己的 FK


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


def wait_stable(fd, tol=0.02, timeout=12.0):
    prev = J(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.4)
        cur = J(fd)
        if max(abs(a - b) for a, b in zip(prev, cur)) < tol:
            return cur
        prev = cur
    return J(fd)


def movej(fd, target, speed, limit=120.0):
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
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.10:
            return True
        time.sleep(0.08)
    return False


def report(fd, tag):
    q = J(fd)
    x = T(fd)
    r = (x[0] ** 2 + x[1] ** 2) ** 0.5
    print("%s" % tag)
    print("   关节 q = %s" % ["%.3f" % v for v in q[:6]])
    print("   控制器报 TCP = (%.1f, %.1f, %.1f)   水平 %.1f mm"
          % (x[0], x[1], x[2], r))
    return q, x


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--release", action="store_true", help="动完下电（默认保持上电）")
    ap.add_argument("--speed", type=int, default=15)
    args = ap.parse_args()

    tgt = HOME + [0.0]
    print("目标 home q = %s" % ["%.2f" % v for v in HOME])

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s" % time.strftime("%H:%M:%S"))

    try:
        print("伺服 = %s" % SERVO.get(S(fd), S(fd)))
        if not power_on(fd):
            raise SystemExit("上电失败")
        wait_stable(fd)

        q0, x0 = report(fd, "\n[当前]")

        # J6 行程 ±360，同一个位姿有多个等价角。取离当前位置最近的那个，
        # 否则会为了 -68 而反拧 338 度，缆线受不了。
        tgt = list(HOME) + [0.0]
        k = round((q0[5] - HOME[5]) / 360.0)
        tgt[5] = HOME[5] + 360.0 * k
        if k:
            print("   J6 取等价角 %.3f（而不是 %.3f），少拧 %d 度"
                  % (tgt[5], HOME[5], abs(int(360 * k))))

        dq = [tgt[i] - q0[i] for i in range(6)]
        print("   各轴要转 %s   最大 %.1f 度"
              % (["%+.1f" % v for v in dq], max(abs(v) for v in dq)))
        if max(abs(v) for v in dq) > 160:
            raise SystemExit("单轴要转 %.0f 度，超出预期，停下检查" % max(abs(v) for v in dq))

        if not args.go:
            print("\n（只看状态，没动。确认后加 --go）")
            return

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        ok = False
        for attempt in range(3):
            if movej(fd, tgt, args.speed):
                ok = True
                break
            print("   第 %d 次没走成，重试" % (attempt + 1))
            if not power_on(fd):
                break
        if not ok:
            raise SystemExit("没能到 home")

        wait_stable(fd)
        q1, x1 = report(fd, "\n[到位]")
        err = max(abs(q1[i] - tgt[i]) for i in range(6))
        print("   与目标最大偏差 %.3f 度" % err)
        print("   J6 当前 %.3f 度，等价于 %.3f 度"
              % (q1[5], ((q1[5] + 180.0) % 360.0) - 180.0))

        print("\n" + "=" * 62)
        print("现在可以量了（修正模型算出来的期望值）")
        print("=" * 62)
        print("  法兰面中心 离地          %.1f mm" % EXPECT["flange_z_mm"])
        print("  法兰面中心 离 J1 轴水平  %.1f mm" % EXPECT["flange_r_mm"])
        print("  J2 轴 离地               %.1f mm" % EXPECT["j2_z_mm"])
        print()
        print("  控制器此刻自报          (%.1f, %.1f, %.1f)"
              % (x1[0], x1[1], x1[2]))
        print("  它以为的水平距          %.1f mm"
              % ((x1[0] ** 2 + x1[1] ** 2) ** 0.5))
        print("  -> 水平差 %.0f mm，高度差 %.0f mm"
              % (((x1[0] ** 2 + x1[1] ** 2) ** 0.5) - EXPECT["flange_r_mm"],
                 x1[2] - (EXPECT["flange_z_mm"] - 336.5)))
        print()
        print("  量法：吊根线从法兰中心落到地面，量 (a) 地面->法兰中心 的竖直高度")
        print("        (b) 落点到 J1 轴（底座中心）的水平距。量到 ±5mm 就够判断了。")
    finally:
        try:
            if args.release:
                nrc.set_servo_state(fd, 0)
                print("\n已下电（抱闸保持）")
            else:
                print("\n保持上电中。要下电：再跑一次加 --release，或直接在示教器上停。")
        except Exception as e:
            print("收尾异常：%s" % e)


if __name__ == "__main__":
    main()
