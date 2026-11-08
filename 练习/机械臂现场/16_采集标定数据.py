# 16 采集 DH 标定数据 —— 走 12 个姿态，每组记录 (关节角, TCP)
#
# 目的：用真机数据反推 DH 参数和关节零位偏置，之后生成 URDF。
# 单关节动作最能隔离每个轴的影响，所以姿态设计以单轴为主。
#
# 跑法：python 16_采集标定数据.py
# 输出：/tmp/calib_samples.txt

import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}
JUMP_MAX = 5.0

POSES = [
    ("home", [0, 0, 0, 0, 0, 0]),
    ("J1+15", [15, 0, 0, 0, 0, 0]),
    ("J2+15", [0, 15, 0, 0, 0, 0]),
    ("J3+15", [0, 0, 15, 0, 0, 0]),
    ("J4+15", [0, 0, 0, 15, 0, 0]),
    ("J5+15", [0, 0, 0, 0, 15, 0]),
    ("J6+20", [0, 0, 0, 0, 0, 20]),
    ("J1-15 J2-15", [-15, -15, 0, 0, 0, 0]),
    ("J2+10 J3-15", [0, 10, -15, 0, 0, 0]),
    ("J3+15 J4+15", [0, 0, 15, 15, 0, 0]),
    ("J4-15 J5+15 J6-20", [0, 0, 0, -15, 15, -20]),
    ("home2", [0, 0, 0, 0, 0, 0]),
]

SAMPLES = []


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
    for _ in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def wait_stable(fd, tol=0.02, timeout=10.0):
    prev = J(fd)
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.5)
        cur = J(fd)
        if max(abs(a - b) for a, b in zip(prev, cur)) < tol:
            return cur
        prev = cur
    return prev


def move(fd, target, speed, limit=60.0):
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
    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    print("已连接 %s" % time.strftime("%H:%M:%S"))

    try:
        if not power_on(fd):
            raise SystemExit("上电失败")
        wait_stable(fd)
        home = J(fd)
        print("基准 %s" % ["%.3f" % v for v in home[:6]])
        nrc.set_speed(fd, 60)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.4)
        nrc.queue_motion_set_status(fd, True)

        for name, delta in POSES:
            tgt = [home[i] + delta[i] for i in range(6)] + [0.0]
            ok = False
            for attempt in range(3):
                if move(fd, tgt, 60):
                    ok = True
                    break
                print("  %s 第 %d 次失败，重新上电" % (name, attempt + 1))
                if not power_on(fd):
                    break
                wait_stable(fd)
                nrc.set_speed(fd, 60)
                nrc.queue_motion_set_status(fd, False)
                time.sleep(0.3)
                nrc.queue_motion_set_status(fd, True)
            if not ok:
                print("  %s 放弃" % name)
                continue
            time.sleep(1.2)          # 等停稳
            q = J(fd)
            x = T(fd)
            SAMPLES.append((name, q, x))
            print("  %-22s q=%s" % (name, ["%.3f" % v for v in q[:6]]))
            print("  %-22s x=%s" % ("", ["%.3f" % v for v in x[:6]]))

        # 回原位
        move(fd, [home[i] for i in range(6)] + [0.0], 60)
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

    with open("/tmp/calib_samples.txt", "w", encoding="utf-8") as f:
        f.write("# DH 标定样本  %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
        f.write("# name | q1..q6 (deg) | x y z rx ry rz (mm, deg)\n")
        for name, q, x in SAMPLES:
            f.write("S %s | %s | %s\n"
                    % (name,
                       " ".join("%.6f" % v for v in q[:6]),
                       " ".join("%.6f" % v for v in x[:6])))
    print("\n采到 %d 组，写入 /tmp/calib_samples.txt" % len(SAMPLES))


if __name__ == "__main__":
    main()
