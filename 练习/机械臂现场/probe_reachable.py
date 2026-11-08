# 探测 get_pos_reachable 怎么调 + 问几个 600mm 候选方向（只读，不动机器人）
import time

import nrc_interface as nrc

fd = nrc.connect_robot("192.168.2.14", "6001")
if fd <= 0:
    raise SystemExit("连接失败")
t = time.time() + 10
while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
    time.sleep(0.2)

p = nrc.VectorDouble()
nrc.get_current_position(fd, 1, p)
cur = list(p)
print("当前 TCP = [%s]" % ", ".join("%.3f" % v for v in cur[:6]))
print()

cands = {
    "+X 600": (600, 0, 0), "-X 600": (-600, 0, 0),
    "+Y 600": (0, 600, 0), "-Y 600": (0, -600, 0),
    "+Z 600": (0, 0, 600), "-Z 600": (0, 0, -600),
}

for name, d in cands.items():
    tgt = list(cur[:6]) + [0.0]
    tgt[0] += d[0]; tgt[1] += d[1]; tgt[2] += d[2]
    pos = nrc.VectorDouble(list(tgt) + [0.0] * (14 - len(tgt))) if False else nrc.VectorDouble(tgt)
    ok = None
    err = ""
    for tag, mk in (("VectorInt", lambda: nrc.VectorInt()),
                    ("list", lambda: [0]),
                    ("int", lambda: 0)):
        try:
            res = mk()
            r = nrc.get_pos_reachable(fd, pos, "MOVL", res)
            ok = (tag, r, res)
            break
        except Exception as e:
            err = "%s: %s" % (tag, str(e)[:70])
    print("%-8s 目标 (%.0f, %.0f, %.0f)  ->  %s   %s"
          % (name, tgt[0], tgt[1], tgt[2], ok, err if ok is None else ""))
