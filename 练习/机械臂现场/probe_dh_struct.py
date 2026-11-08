# 探测 RobotDHParam 结构怎么用（只读）
import nrc_interface as nrc

print("含 DH / Param 的名字：")
print([n for n in dir(nrc) if 'DH' in n or 'Param' in n])
print()

P = getattr(nrc, 'RobotDHParam', None)
print("RobotDHParam =", P)
if P is not None:
    o = P()
    print("成员:", [a for a in dir(o) if not a.startswith('_')])
    for a in [a for a in dir(o) if not a.startswith('_')]:
        try:
            print("   %-24s = %r" % (a, getattr(o, a)))
        except Exception as e:
            print("   %-24s ERR %s" % (a, e))

print()
fd = nrc.connect_robot("192.168.2.14", "6001")
if fd > 0:
    import time
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)
    try:
        obj = nrc.RobotDHParam()
        r = nrc.get_robot_dh_param(fd, obj)
        print("get_robot_dh_param ret =", r)
        for a in [a for a in dir(obj) if not a.startswith('_')]:
            try:
                print("   %-24s = %r" % (a, getattr(obj, a)))
            except Exception as e:
                print("   %-24s ERR %s" % (a, e))
    except Exception as e:
        print("调用失败:", e)
