# 19 摆正法兰 —— 保持 TCP 基本不动，把法兰面转到指定朝向
#
# 背景：要用法兰做「命令 300mm，物理走多少」的直线检验，需要先把法兰转到一个规整朝向。
#       目标关节角已在 robot 环境用 IK 算好（arm38 没有 numpy），这里直接写死。
#
# 各朝向的解（TCP 位置基本不动，误差见括号）：
#   plusX  工具轴水平朝 +X  —— 本脚本默认。腕部离奇异仅 17.8°，静态位姿没问题，
#                              但**不要从这个位姿做 Cartesian 直线**
#   minusY 工具轴水平朝 −Y  —— 离奇异 82°
#   down   工具轴竖直向下   —— 离奇异 86°
#
# moveJ 做关节直线插值，TCP 会扫过一小段（plusX 实测模型预测最大 41.6mm，全程在 1.7m 高度），
# 比 moveL 安全：不经过控制器逆解，不会因为奇异点甩关节。
#
# 跑法：
#   ~/miniforge3/envs/arm38/bin/python 19_摆正法兰.py --pose plusX          # 预演
#   ~/miniforge3/envs/arm38/bin/python 19_摆正法兰.py --pose plusX --go     # 真动
# 可选 --keep：动完保持上电（默认下电，靠抱闸保持）

import argparse
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0
GLOBAL_SPEED = 25

# 关节角（度），来自 probe_dirs.py / probe_path.py
TARGETS = {
    "plusX": [-16.41, -2.66, -4.40, -67.35, -107.83, -23.67],
    "minusY": [-8.08, -10.26, 4.01, -90.88, -8.03, 6.31],
    "down": [-14.56, -13.49, 17.32, -0.00, 3.83, -104.56],
}
NOTE = {
    "plusX": "工具轴朝 +X（离腕奇异 17.8°，静态位姿可用，别从这儿走直线）",
    "minusY": "工具轴朝 −Y（离腕奇异 82°）",
    "down": "工具轴竖直向下（离腕奇异 86°）",
}


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
        if max(abs(a - b) for a, b in zip(prev, js)) > JUMP_MAX:
            time.sleep(0.2)
            continue
        prev = js
        if max(abs(a - b) for a, b in zip(target, js)) < 0.15:
            return True
        time.sleep(0.08)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", default="plusX", choices=list(TARGETS))
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--keep", action="store_true", help="动完保持上电")
    ap.add_argument("--speed", type=int, default=GLOBAL_SPEED)
    args = ap.parse_args()

    tgt = TARGETS[args.pose] + [0.0]
    print("目标：%s" % NOTE[args.pose])
    print("目标 q = %s" % ["%.2f" % v for v in TARGETS[args.pose]])
    if not args.go:
        print("\n（预演，没连机器人。确认后加 --go）")
        return

    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s" % time.strftime("%H:%M:%S"))

    ok_all = False
    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        wait_stable(fd)
        q0 = J(fd)
        x0 = T(fd)
        print("当前 q = %s" % ["%.3f" % v for v in q0[:6]])
        print("当前 TCP = %s" % ["%.3f" % v for v in x0[:6]])
        dq = [abs(tgt[i] - q0[i]) for i in range(6)]
        print("各轴要转：%s   最大 %.1f 度"
              % (["%+.1f" % (tgt[i] - q0[i]) for i in range(6)], max(dq)))
        if max(dq) > 160:
            raise SystemExit("单轴要转 %.0f 度，超过脚本预期，停下检查" % max(dq))

        nrc.set_speed(fd, args.speed)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        for attempt in range(3):
            if movej(fd, tgt, args.speed):
                ok_all = True
                break
            print("  第 %d 次没走成，重新上电重试" % (attempt + 1))
            if not power_on(fd):
                break
            wait_stable(fd)
            nrc.set_speed(fd, args.speed)
            nrc.queue_motion_set_status(fd, False)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, True)

        if not ok_all:
            raise SystemExit("三次都没走到，放弃")

        time.sleep(1.0)
        q1 = J(fd)
        x1 = T(fd)
        print("\n到位")
        print("  q   = %s" % ["%.3f" % v for v in q1[:6]])
        print("  TCP = %s" % ["%.3f" % v for v in x1[:6]])
        print("  关节误差 %s" % ["%+.3f" % (q1[i] - tgt[i]) for i in range(6)])
        print("  TCP 相对移动 %.3f mm" % (
            sum((x1[i] - x0[i]) ** 2 for i in range(3)) ** 0.5))
        import math
        R = x1[3:6]
        # 姿态是弧度 XYZ 欧拉角：R = Rx·Ry·Rz，工具轴 = R·ez
        cx, sx = math.cos(R[0]), math.sin(R[0])
        cy, sy = math.cos(R[1]), math.sin(R[1])
        cz, sz = math.cos(R[2]), math.sin(R[2])
        # Rx·Ry·Rz 的第三列
        col3 = [cy * sz, -sy, cy * cz]
        # 修正：Rz 的第三列是 (0,0,1)，所以 R·ez = Rx·Ry·(0,0,1) = (sy, -sx*cy, cx*cy)
        col3 = [sy, -sx * cy, cx * cy]
        print("  工具轴方向（控制器自报）= (%+.3f %+.3f %+.3f)" % tuple(col3))
    finally:
        try:
            nrc.queue_motion_stop(fd)
            time.sleep(0.3)
            nrc.queue_motion_set_status(fd, False)
        except Exception:
            pass
        if not args.keep:
            if S(fd) in (2, 3):
                nrc.set_servo_poweroff(fd)
                time.sleep(0.4)
            if S(fd) != 0:
                nrc.set_servo_state(fd, 0)
        nrc.disconnect_robot(fd)
        print("\n%s %s" % ("保持上电" if args.keep else "已下电（抱闸保持）",
                           time.strftime("%H:%M:%S")))


if __name__ == "__main__":
    main()
