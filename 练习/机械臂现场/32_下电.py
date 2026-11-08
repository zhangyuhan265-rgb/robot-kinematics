# 32 下电 —— 把伺服断掉，交给示教器
#
# 跑法：~/miniforge3/envs/arm38/bin/python ~/projects/07_机器人学/练习/机械臂现场/32_下电.py

import time

import nrc_interface as nrc

SERVO = {0: "停止(未上电)", 1: "就绪(上电中)", 2: "报警", 3: "运行(已使能)"}


def S(fd):
    return nrc.get_servo_state(fd, 0)[1]


def main():
    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)

    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    print("下电前 q = %s" % ["%.3f" % v for v in p[:6]])
    print("下电前伺服 = %s" % SERVO.get(S(fd), S(fd)))

    r1 = nrc.set_current_mode(fd, 0)
    time.sleep(0.4)
    r2 = nrc.set_servo_poweroff(fd)
    time.sleep(1.5)
    print("set_current_mode(0) ret=%s   set_servo_poweroff ret=%s" % (r1, r2))
    if S(fd) != 0:
        r3 = nrc.set_servo_state(fd, 0)
        time.sleep(1.2)
        print("set_servo_state(0) ret=%s" % r3)

    st = S(fd)
    print("下电后伺服 = %s" % SERVO.get(st, st))
    if st == 0:
        print("已下电，抱闸保持。示教器可以接管了。")
    else:
        print("!! 没下电成功（状态 %s），请在示教器上确认" % st)


if __name__ == "__main__":
    main()
