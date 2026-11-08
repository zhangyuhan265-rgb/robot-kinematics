# -*- coding: utf-8 -*-
"""流式点动核心：一边走一边往队列里补点，用 pl 平滑连起来。

思路：
  普通的点动是「发一步 → 等它到位 → 再发一步」，每步都要吃一次完整的
  「命令下发 → 规划 → 运动 → 到位」周期，所以一顿一顿。
  （注：这个周期具体多长**没有留存测量日志**。早期笔记里的 ~670 ms 是回忆值，
    不可当依据；要用就先跑 25_软件点动自测.py 并把输出 tee 到文件。）
  流式版改成：**提前把后面几步塞进队列**，控制器就永远有活干，
  再配合每段 pl=5（轨迹间平滑），动作就连续了。

  关键：命令要按「已下达的目标位置」累加，不能读当前实际位置
  （实际位置落后于队列），否则会越走越短。

自测模式会记录运动过程中的关节角，算出速度曲线，
看中间有没有掉到零速的"谷"——有谷就是一顿一顿，没谷就是连续。

用法：
  python3 26_流式点动.py --selftest                 # 脚本化跑 3 秒并出速度曲线
  python3 26_流式点动.py --go                       # 交互式（真终端）
"""
import argparse
import math
import select
import sys
import time

import nrc_interface as nrc

SERVO = {0: "停止", 1: "就绪", 2: "报警", 3: "运行"}


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
                return True
            time.sleep(0.2)
        time.sleep(0.4)
    return False


def push_movej(fd, q6, speed, pl):
    cmd = nrc.MoveCmd()
    cmd.coord = 0
    cmd.targetPosType = nrc.PosType_data
    cmd.targetPosValue = nrc.VectorDouble(list(q6[:6]) + [0.0])
    cmd.velocity = speed
    cmd.acc = 30
    cmd.dec = 30
    cmd.pl = pl
    return nrc.queue_motion_push_back_moveJ(fd, cmd)


def stream_jog(fd, axis, sign, duration, q0=None, sub=0.25, batch=3,
               speed=40, pl=5, trace=None, limit_deg=20.0, max_lead=2.0):
    """连续点动 axis 轴，持续 duration 秒；或直到走到目标位置。

    限流很关键：不能按固定频率猛塞，否则控制器队列塞满、send 失败。
    这里按「已下达位置 与 实际位置 的差」限流——领先量超过 max_lead 就先等等，
    等于喂得和跑得一样快，队列永远不会空也不会满。
    """
    q_v = list((q0 if q0 is not None else J(fd))[:6])
    q_start = q_v[axis]
    t0 = time.time()
    sent = 0
    while time.time() - t0 < duration:
        if S(fd) != 3:
            print("    伺服掉了")
            break
        q_act = J(fd)[axis]
        lead = (q_v[axis] - q_act) * sign
        if lead < max_lead and abs(q_v[axis] - q_start) < limit_deg:
            n = 0
            for _ in range(batch):
                q_v[axis] += sign * sub
                if abs(q_v[axis] - q_start) > limit_deg:
                    q_v[axis] -= sign * sub
                    break
                if push_movej(fd, q_v, speed, pl) != nrc.SUCCESS:
                    q_v[axis] -= sign * sub
                    break
                n += 1
            if n == 0:
                break
            r = nrc.queue_motion_send_to_controller(fd, n, False)
            if r != nrc.SUCCESS:
                # 控制器队列满了，退回去，等它消耗
                for _ in range(n):
                    q_v[axis] -= sign * sub
                time.sleep(0.01)
                continue
            sent += n
        else:
            time.sleep(0.008)
        if trace is not None:
            trace.append((time.time() - t0, J(fd)[axis]))
    return q_v[axis] - q_start, sent


def stream_to(fd, axis, target, sub=0.25, batch=3, speed=40, pl=5,
              tol=0.06, timeout=20.0, trace=None):
    """流式走到指定关节角（用于回位）"""
    t0 = time.time()
    q_v = list(J(fd)[:6])
    sent = 0
    while time.time() - t0 < timeout:
        if S(fd) != 3:
            break
        q_act = J(fd)[axis]
        if abs(q_act - target) < tol and abs(q_v[axis] - target) < sub:
            break
        sign = 1.0 if target > q_v[axis] else -1.0
        if abs(q_v[axis] - target) <= sub * 0.5:
            break
        lead = abs(q_v[axis] - q_act)
        if lead < 2.0:
            n = 0
            for _ in range(batch):
                nxt = q_v[axis] + sign * sub
                if (target - nxt) * sign < 0:
                    break
                q_v[axis] = nxt
                if push_movej(fd, q_v, speed, pl) != nrc.SUCCESS:
                    break
                n += 1
            if n:
                if nrc.queue_motion_send_to_controller(fd, n, False) != nrc.SUCCESS:
                    for _ in range(n):
                        q_v[axis] -= sign * sub
                    time.sleep(0.01)
                else:
                    sent += n
        else:
            time.sleep(0.008)
        if trace is not None:
            trace.append((time.time() - t0, J(fd)[axis]))
    # 补最后一点点
    wait_settle(fd)
    return sent


def wait_settle(fd, timeout=10.0):
    t0 = time.time()
    prev = J(fd)
    while time.time() - t0 < timeout:
        time.sleep(0.15)
        cur = J(fd)
        if max(abs(a - b) for a, b in zip(prev, cur)) < 0.01:
            return
        prev = cur


def analyse(trace, label):
    """从采样曲线算速度，看中间有没有零速谷"""
    if len(trace) < 4:
        print("    %s: 采样太少" % label)
        return
    ts = [p[0] for p in trace]
    qs = [p[1] for p in trace]
    v = []
    for k in range(1, len(ts)):
        dt = ts[k] - ts[k - 1]
        if dt > 1e-4:
            v.append(abs(qs[k] - qs[k - 1]) / dt)
    if not v:
        return
    v_sorted = sorted(v)
    mid = v[len(v) // 4: 3 * len(v) // 4]          # 掐掉头尾
    mid = mid or v
    print("    %s: 采样 %d 点  速度 中位 %.1f  均值 %.1f  最小 %.2f  最大 %.1f 度/秒"
          % (label, len(v), v_sorted[len(v_sorted) // 2],
             sum(v) / len(v), min(mid), max(v)))
    zeros = sum(1 for x in mid if x < 0.5)
    print("       中段零速(<0.5°/s)采样 %d/%d  %s"
          % (zeros, len(mid), "→ 一顿一顿" if zeros > len(mid) * 0.15 else "→ 基本连续"))


def selftest(args):
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
        q0 = J(fd)
        print("起点 q6 = %.3f\n" % q0[5])

        # A: 普通点动（一步一停）—— 对照组
        print("[A] 普通步进 J6（一步等一步）")
        tr_a = []
        qa = list(q0[:6])
        ta = time.time()
        for k in range(8):
            qa[5] += 0.25
            push_movej(fd, qa, 40, 0)
            nrc.queue_motion_send_to_controller(fd, 1, False)
            t1 = time.time()
            while time.time() - t1 < 3:
                cur = J(fd)
                tr_a.append((time.time() - ta, cur[5]))
                if abs(cur[5] - qa[5]) < 0.02:
                    break
                time.sleep(0.03)
        print("    走了 %+.3f 度，用时 %.2f s" % (J(fd)[5] - q0[5], time.time() - ta))
        analyse(tr_a, "A")
        wait_settle(fd)
        stream_to(fd, 5, q0[5], tol=0.05)
        print("    回位到 q6 = %.3f（差 %+.3f）" % (J(fd)[5], J(fd)[5] - q0[5]))

        # B: 流式点动
        print("\n[B] 流式 J6（限流 2° 领先，pl=5）")
        tr_b = []
        qb0 = J(fd)[5]
        t0 = time.time()
        moved, sent = stream_jog(fd, 5, +1, 3.0, q0=J(fd), sub=0.25, batch=3,
                                 speed=40, pl=5, trace=tr_b)
        print("    发送 %d 条，实际走了 %+.3f 度，用时 %.2f s，等效 %.2f 度/秒"
              % (sent, J(fd)[5] - qb0, time.time() - t0,
                 (J(fd)[5] - qb0) / max(time.time() - t0, 1e-6)))
        analyse(tr_b, "B")
        wait_settle(fd)

        # C: 流式走回去
        print("\n[C] 流式回位")
        t1 = time.time()
        stream_to(fd, 5, q0[5], tol=0.05)
        print("    回到 q6 = %.3f（起点 %.3f，差 %+.3f），用时 %.1f s"
              % (J(fd)[5], q0[5], J(fd)[5] - q0[5], time.time() - t1))
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


class Jogger:
    """流式点动状态机：start 之后每 tick 一次就往后补一点"""

    def __init__(self, fd, sub=0.5, batch=3, speed=60, pl=5, max_lead=1.5):
        self.fd = fd
        self.axis = 0
        self.sign = 1
        self.sub = sub
        self.batch = batch
        self.speed = speed
        self.pl = pl
        self.max_lead = max_lead
        self.active = False
        self.q_v = list(J(fd)[:6])
        self.sent = 0
        self.travel = 0.0
        self.base = self.q_v[0]

    def start(self, sign):
        self.sign = sign
        self.q_v = list(J(self.fd)[:6])
        self.base = self.q_v[self.axis]
        self.travel = 0.0
        self.active = True

    def stop(self):
        self.active = False

    def tick(self):
        if not self.active:
            return
        if S(self.fd) != 3:
            self.active = False
            return
        q_act = J(self.fd)[self.axis]
        lead = (self.q_v[self.axis] - q_act) * self.sign
        if lead >= self.max_lead:
            time.sleep(0.005)
            return
        n = 0
        for _ in range(self.batch):
            nxt = self.q_v[self.axis] + self.sign * self.sub
            # 软限位保护
            lim = 355 if self.axis == 5 else 125
            if abs(nxt) > lim:
                self.active = False
                break
            self.q_v[self.axis] = nxt
            if push_movej(self.fd, self.q_v, self.speed, self.pl) != nrc.SUCCESS:
                self.q_v[self.axis] -= self.sign * self.sub
                break
            n += 1
        if n == 0:
            return
        if nrc.queue_motion_send_to_controller(self.fd, n, False) != nrc.SUCCESS:
            for _ in range(n):
                self.q_v[self.axis] -= self.sign * self.sub
            time.sleep(0.005)
        else:
            self.sent += n
            self.travel += n * self.sub * self.sign


def interactive(fd):
    import termios
    import tty
    jog = Jogger(fd)
    print("\n" + "=" * 62)
    print("流式点动  1-6 选轴 | + / - 开始连续动 | 任意其它键 停 | p 打印 | q 退出")
    print("步长 %.2f° 每次补 %d 条，限流领先 %.1f°" % (jog.sub, jog.batch, jog.max_lead))
    print("=" * 62)
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            sys.stdout.write("\r轴 J%d  步长%.2f°  %s  %-30s"
                             % (jog.axis + 1, jog.sub,
                                "▶ 运行中" if jog.active else "■ 停止",
                                "已走 %+.1f°" % jog.travel if jog.active else ""))
            sys.stdout.flush()
            jog.tick()
            k = select.select([sys.stdin], [], [], 0.01)[0]
            if not k:
                continue
            ch = sys.stdin.read(1)
            if ch == "q":
                jog.stop()
                print("\n退出")
                break
            if ch in "123456":
                jog.stop()
                jog.axis = int(ch) - 1
                continue
            if ch == "p":
                print("\n  q   = %s" % ["%.3f" % v for v in J(fd)[:6]])
                print("  TCP = %s" % ["%.3f" % v for v in T(fd)[:6]])
                continue
            if ch == "[":
                jog.sub = max(0.05, jog.sub / 2)
                continue
            if ch == "]":
                jog.sub = min(2.0, jog.sub * 2)
                continue
            if ch in "+=":
                jog.start(+1)
                continue
            if ch in "-_":
                jog.start(-1)
                continue
            jog.stop()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--sub", type=float, default=0.5, help="子步长（度）")
    ap.add_argument("--speed", type=float, default=60.0)
    args = ap.parse_args()
    if args.selftest:
        selftest(args)
        return
    if not args.go:
        print(__doc__)
        print("（预演。--selftest 跑自动测试；--go 进交互模式，需真终端）")
        return
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
        print("起点 q = %s" % ["%.2f" % v for v in J(fd)[:6]])
        interactive(fd)
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
