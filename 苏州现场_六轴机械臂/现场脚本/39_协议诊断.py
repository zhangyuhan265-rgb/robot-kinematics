import sys
import time

sys.path.insert(0, "/home/yohan/projects/07_机器人学/nrc_sdk")
import nrc_interface as nrc

names = [n for n in dir(nrc) if not n.startswith("_")]
print("接口总数 =", len(names))
print()
key = [n for n in names if any(k in n.lower() for k in
       ("version", "connect", "status", "alarm", "error", "mode", "warn"))]
print("--- 版本/连接/报警相关 ---")
for n in key:
    print("  ", n)
print()
fd = nrc.connect_robot("192.168.2.14", "6001")
print("connect fd =", fd)
t = time.time() + 8
st = None
while time.time() < t:
    st = nrc.get_connection_status(fd)
    if st == nrc.SUCCESS:
        break
    time.sleep(0.2)
print("get_connection_status =", st, " SUCCESS =", nrc.SUCCESS)
for fn in ("get_robot_version", "get_software_version", "get_version",
           "get_sdk_version", "get_controller_version"):
    if hasattr(nrc, fn):
        try:
            print(fn, "->", getattr(nrc, fn)(fd))
        except Exception as e:
            print(fn, "异常:", e)
print("servo_state ->", nrc.get_servo_state(fd, 0))
print("current_mode ->", nrc.get_current_mode(fd))
print("running_state ->", nrc.get_robot_running_state(fd))
