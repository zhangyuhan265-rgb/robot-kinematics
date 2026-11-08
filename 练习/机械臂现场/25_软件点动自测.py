# -*- coding: utf-8 -*-
"""软件点动自测：不读键盘，脚本化地走几步，验证机制、响应、精度。

测什么：
  1. 每步耗时（响应速度）
  2. 每步实际走了多少（步长准不准）
  3. 走过去再走回来，能不能回到原点（可逆性）

安全：J6 先试（TCP 在轴 6 上，末端不动），步长 0.5°，慢速。
"""
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}


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
    for i in range(6):
        nrc.set_servo_poweron(fd)
        t0 = time.time()
        while time.time() - t0 < 2.5:
            if S(fd) == 3:
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def step(fd, axis, d, speed=20, timeout=6.0):
    q = J(fd)
    q6 = list(q[:6])
    q6[axis] += d
    tgt = q6 + [0.0]
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(tgt)
    cmd.velocity = speed
    cmd.acc = 20
    cmd.dec = 20
    cmd.pl = 0
    t0 = time.time()
    if nrc.queue_motion_push_back_moveJ(fd, cmd) != nrc.SUCCESS:
        return None, None
    if nrc.queue_motion_send_to_controller(fd, 0, False) != nrc.SUCCESS:
        return None, None
    while time.time() - t0 < timeout:
        if S(fd) != 3:
            return None, None
        cur = J(fd)
        if max(abs(cur[i] - tgt[i]) for i in range(6)) < 0.02:
            dt = time.time() - t0
            return dt, cur[axis] - q[axis]
        time.sleep(0.02)
    return None, None


def run(fd, axis, d, n, label):
    print("\n%s：J%d %+0.2f° × %d" % (label, axis + 1, d, n))
    q0 = J(fd)[axis]
    times = []
    deltas = []
    for k in range(n):
        dt, dq = step(fd, axis, d)
        if dt is None:
            print("  第 %d 步失败" % (k + 1))
            return None
        times.append(dt)
        deltas.append(dq)
        print("    第 %d 步  %5.0f ms   实际 %+0.4f°" % (k + 1, dt * 1000, dq))
    total = J(fd)[axis] - q0
    print("  合计：目标 %+.2f°，实际 %+.3f°，误差 %+.3f°"
          % (d * n, total, total - d * n))
    print("  平均每步 %.0f ms，最快 %.0f ms"
          % (sum(times) / len(times) * 1000, min(times) * 1000))
    return total


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
        nrc.set_speed(fd, 60)
        nrc.queue_motion_set_status(fd, False)
        time.sleep(0.3)
        nrc.queue_motion_set_status(fd, True)
        q_start = J(fd)
        print("上电成功，起点 q = %s" % ["%.3f" % v for v in q_start[:6]])

        # 1) J6 正向 5 步
        run(fd, 5, +0.5, 5, "第一段")
        # 2) J6 返回 5 步
        run(fd, 5, -0.5, 5, "返回段")
        print("\nJ6 往返总误差 %+0.4f°" % (J(fd)[5] - q_start[5]))

        # 3) J2 小幅 3 步往返
        run(fd, 1, +0.3, 3, "J2 正向")
        run(fd, 1, -0.3, 3, "J2 返回")
        print("\nJ2 往返总误差 %+0.4f°" % (J(fd)[1] - q_start[1]))

        print("\n最终 q = %s" % ["%.3f" % v for v in J(fd)[:6]])
        print("全部轴回位误差 %s"
              % ["%+.4f" % (J(fd)[i] - q_start[i]) for i in range(6)])
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
    main()
