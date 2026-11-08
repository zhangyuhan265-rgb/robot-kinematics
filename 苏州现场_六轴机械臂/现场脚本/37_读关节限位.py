# 读 J1~J6 的完整关节参数（限位、方向、减速比），确认 J5 的软限位到底是多少
# 注意：get_robot_joint_param 的 id 是 0-based，所以 id=4 才是 J5
import time

import nrc_interface as nrc

fd = 0
for _ in range(8):
    fd = nrc.connect_robot("192.168.2.14", "6001")
    if fd > 0:
        break
    time.sleep(10)
if fd <= 0:
    raise SystemExit("连接失败")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)

p = nrc.VectorDouble()
nrc.get_current_position(fd, 0, p)
print("当前 q(deg) = %s" % ["%.2f" % v for v in p[:6]])
print()
print("%-6s %8s %8s %8s %8s %8s" % ("关节", "negSWLimit", "posSWLimit", "direction", "reducRatio", "ratedVel"))
for jid in range(6):
    jp = nrc.RobotJointParam()
    r = nrc.get_robot_joint_param(fd, jid, jp)
    print("J%-5d %8.1f %8.1f %8.0f %8.1f %8.1f"
          % (jid + 1, jp.negSWLimit, jp.posSWLimit, jp.direction,
             jp.reducRatio, jp.ratedVel))
print()
print("说明：id 是 0-based，所以上面第 5 行（J5）对应 id=4。")
