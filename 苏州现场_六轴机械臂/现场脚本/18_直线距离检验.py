# 18 直线距离检验 —— 命令走 N mm，物理上到底走了多少？
#
# 为什么要做这个：
#   现场尺子量出 J2→J3=805（控制器参数 900）、J3→法兰=900（参数 1014.7），
#   两段比值都约 0.89，怀疑控制器参数整体大了一号。
#   但"盖心到盖心"可能不在关节轴线上，所以需要一个不依赖端点识别的判据：
#   命令控制器按【它自己的模型】走 300 mm，物理上走了多少？
#     参数对  -> 物理也走 300
#     参数大 11% -> 物理只走约 267
#
# 流程：
#   1. 先不加 --go 跑一遍，看清楚方向和目标点，确认空间够、不会撞
#   2. 在法兰旁边贴一张纸/立一块板，把臂摆到方便做记号的位置
#   3. 加 --go 跑：走到目标点后会【停住等你】
#      这时在纸上一头一尾各画一个记号（对准法兰中心或杆件边缘）
#      按回车，臂自动回原点
#   4. 拿尺子量两个记号之间的距离
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 18_直线距离检验.py                 # 只看计划
#   ~/miniforge3/envs/arm38/bin/python 18_直线距离检验.py --go            # 真走
#   ~/miniforge3/envs/arm38/bin/python 18_直线距离检验.py --go --d 300,0,0
#
# 安全：单次直线 ≤300mm、速度按 0.36×velocity 折算（默认约 36 mm/s）、
#       全程监视伺服状态和单步跳变，跑完一定回原点。

import argparse
import sys
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
GLOBAL_SPEED = 30


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


def wait_stable(fd, tol=0.02, timeout=15.0):
    prev = J(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.5)
        cur = J(fd)
        if max(abs(a - b) for a, b in zip(prev, cur)) < tol:
            return cur
        prev = cur
    return J(fd)


def line(fd, target, vel, limit=180.0):
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--go", action="store_true", help="不加就只打印计划，不动")
    ap.add_argument("--d", default="300,0,0", help="位移向量 dx,dy,dz（mm）")
    ap.add_argument("--vel", type=float, default=100.0, help="cmd.velocity（约 0.36×=mm/s）")
    ap.add_argument("--pause", type=float, default=90.0,
                    help="到终点后停多久等你做记号（秒，无交互终端时用）")
    ap.add_argument("--prepause", type=float, default=0.0,
                    help="开始走之前停多久（秒），用来先把尺子对好、记下起点读数")
    ap.add_argument("--no-return", action="store_true",
                    help="走完停在终点不回原点（做记号一定要用这个）")
    args = ap.parse_args()

    dv = [float(v) for v in args.d.split(",")]
    step = (dv[0] ** 2 + dv[1] ** 2 + dv[2] ** 2) ** 0.5
    print("计划：沿 (%g, %g, %g) 走 %.1f mm，速度约 %.0f mm/s"
          % (dv[0], dv[1], dv[2], step, args.vel * 0.36))
    print("判定：参数对 → 物理也走 %.1f；参数大 11%% → 物理只走 %.1f"
          % (step, step * 0.889))
    if not args.go:
        print("\n（这是预演，没有连接机器人。确认空间够、方向对，再加 --go）")
        return

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("\n已连接 %s" % time.strftime("%H:%M:%S"))

    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        wait_stable(fd)
        nrc.set_speed(fd, GLOBAL_SPEED)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        p0 = T(fd)
        q0 = J(fd)
        tgt = list(p0)
        for i in range(3):
            tgt[i] = p0[i] + dv[i]
        print("\n起点 TCP = %s" % [round(v, 3) for v in p0[:6]])
        print("起点 关节 = %s" % [round(v, 3) for v in q0[:6]])
        print("目标 TCP = %s" % [round(v, 3) for v in tgt[:6]])

        print("\n开始走 ...")
        if args.prepause > 0:
            print("!!! 先别走：给你 %.0f 秒把尺子对好、记下法兰在尺子上的起点读数 !!!" % args.prepause)
            time.sleep(args.prepause)
        ok, dt = line(fd, tgt, args.vel)
        if not ok:
            raise SystemExit("没走到目标，已中止")
        time.sleep(0.8)
        p1 = T(fd)
        q1 = J(fd)
        print("到位，用时 %.1fs" % dt)
        print("终点 TCP = %s" % [round(v, 3) for v in p1[:6]])
        print("终点 关节 = %s" % [round(v, 3) for v in q1[:6]])
        print("控制器认为走了 %.3f mm" % dist(p0, p1))

        print("\n" + "=" * 58)
        print("臂现在停在终点，不要动它。")
        print("请在纸/板上对准法兰中心（或杆件同一边缘）画第一个记号。")
        print("画好后按回车，臂会自动回起点，再在同一张纸上画第二个记号。")
        print("=" * 58)
        if args.no_return:
            print("\n!!! --no-return：臂就停在这里，不回原点 !!!")
            print("现在给【终点】做记号。做完告诉我，我再让它回去。")
            print("（脚本到此结束，臂保持位置后下电，抱闸会抱住）")
            return
        try:
            input("按回车继续 ...")
        except EOFError:
            print("（没有交互终端，停 %.0f 秒后自动继续，趁现在做记号）" % args.pause)
            time.sleep(args.pause)

        ok2, dt2 = line(fd, p0, args.vel)
        print("回原点：%s，用时 %.1fs" % ("成功" if ok2 else "失败", dt2))
        time.sleep(0.8)
        p2 = T(fd)
        print("回到 TCP = %s" % [round(v, 3) for v in p2[:6]])
        print("回位残差 %.3f mm" % dist(p0, p2))
        print("\n现在在同一张纸上给起点也画个记号，然后用尺子量两个记号的距离。")
        print("命令 %.1f mm  →  物理应当 %.1f mm（模型正确时）" % (step, step))
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
