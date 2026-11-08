# 反复尝试连接控制器，看是不是上一次的半开连接占着槽位
import time

import nrc_interface as nrc

IP, PORT = "192.168.2.14", "6001"

for i in range(1, 13):
    fd = nrc.connect_robot(IP, PORT)
    if fd > 0:
        t = time.time() + 8
        while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
            time.sleep(0.2)
        p = nrc.VectorDouble()
        r = nrc.get_current_position(fd, 0, p)
        s = nrc.get_servo_state(fd, 0)
        print("第 %2d 次：连上了 fd=%d，读关节 ret=%s" % (i, fd, r))
        print("       伺服状态 = %s" % (s,))
        if r == 0:
            print("       q = %s" % ["%.2f" % v for v in p[:6]])
            print("OK")
            break
        nrc.disconnect_robot(fd)
    else:
        print("第 %2d 次：connect 返回 %s" % (i, fd))
    time.sleep(8)
else:
    print("12 次都不行")
