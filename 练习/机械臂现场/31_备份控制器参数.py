# 31 备份控制器参数（全程只读，不写任何东西）
#
# 在任何人去改 DH / 零位之前，把控制器现在的参数原样存一份，改坏了能照着恢复。
#
# 跑法：~/miniforge3/envs/arm38/bin/python 31_备份控制器参数.py

import io
import time

import nrc_interface as nrc

IP, PORT = "192.168.2.14", "6001"
OUT = "控制器参数备份_%s.txt" % time.strftime("%Y%m%d_%H%M%S")
lines = []


def w(s=""):
    lines.append(s)
    print(s)


def dump_obj(tag, obj):
    w("\n[%s]" % tag)
    for a in sorted(a for a in dir(obj) if not a.startswith('_')
                    and a not in ('this', 'thisown')):
        try:
            v = getattr(obj, a)
        except Exception as e:
            v = "ERR(%s)" % e
        if isinstance(v, float) and v == 0.0:
            continue
        if isinstance(v, str) and v.strip() in ("0.000000", "0", ""):
            continue
        if isinstance(v, int) and v == 0:
            continue
        w("  %-26s = %r" % (a, v))


def main():
    fd = nrc.connect_robot(IP, PORT)
    if fd <= 0:
        raise SystemExit("连接失败")
    t = time.time() + 10
    while nrc.get_connection_status(fd) != nrc.SUCCESS and time.time() < t:
        time.sleep(0.2)

    w("=" * 70)
    w("纳博特控制器参数备份（只读）")
    w("时间   : %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    w("控制器 : %s:%s   SDK 2.0.4-20260202" % (IP, PORT))
    w("说明   : 空值/零值已省略，只列出非零项。恢复时零值按 0 处理。")
    w("=" * 70)

    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 0, p)
    w("\n[当前关节角 q（度）]")
    w("  [%s]" % ", ".join("%.6f" % v for v in p))

    p = nrc.VectorDouble()
    nrc.get_current_position(fd, 1, p)
    w("\n[当前 TCP（控制器自算，mm + rad）]")
    w("  [%s]" % ", ".join("%.6f" % v for v in p))

    # DH
    try:
        dh = nrc.RobotDHParam()
        r = nrc.get_robot_dh_param(fd, dh)
        w("\n[DH 参数  get_robot_dh_param  ret=%s]" % r)
        for i in range(1, 21):
            w("  L%-3d = %s" % (i, getattr(dh, "L%d" % i)))
        for a in sorted(a for a in dir(dh) if not a.startswith('_')
                        and a not in ('this', 'thisown')):
            if a.startswith('L') and a[1:].isdigit():
                continue
            try:
                v = getattr(dh, a)
            except Exception:
                continue
            w("  %-26s = %r" % (a, v))
    except Exception as e:
        w("\n[DH 读取失败] %s" % e)

    # 关节参数
    for jid in range(6):
        try:
            jp = nrc.RobotJointParam()
            r = nrc.get_robot_joint_param(fd, jid, jp)
            dump_obj("J%d 关节参数  get_robot_joint_param  ret=%s" % (jid + 1, r), jp)
        except Exception as e:
            w("\n[J%d 读取失败] %s" % (jid + 1, e))

    # 工具
    for tnum in range(3):
        try:
            tp = nrc.ToolParam()
            r = nrc.get_tool_hand_param(fd, tnum, tp)
            dump_obj("工具 %d  get_tool_hand_param  ret=%s" % (tnum, r), tp)
        except Exception as e:
            w("\n[工具 %d 读取失败] %s" % (tnum, e))

    io.open(OUT, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n已写入 %s（%d 行）" % (OUT, len(lines)))


if __name__ == "__main__":
    main()
