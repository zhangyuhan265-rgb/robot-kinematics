# 34 下载控制器日志并找报错（只读）
#
# 跑法：~/miniforge3/envs/arm38/bin/python 34_下载日志.py

import io
import os
import time

import nrc_interface as nrc

OUTDIR = "/tmp/nrc_log"


def main():
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

    if not os.path.isdir(OUTDIR):
        os.makedirs(OUTDIR)

    print("下载日志到 %s ..." % OUTDIR)
    r = nrc.log_download_by_quantity(fd, 20, OUTDIR)
    print("log_download_by_quantity ret = %s" % r)
    time.sleep(3)

    files = sorted(os.listdir(OUTDIR))
    print("目录里 %d 个文件：" % len(files))
    for f in files[:40]:
        p = os.path.join(OUTDIR, f)
        print("  %-50s %8d 字节" % (f, os.path.getsize(p)))

    # 找含报错关键字的行
    KEY = ["超限", "超差", "limit", "Limit", "error", "Error", "alarm", "Alarm",
           "fault", "Fault", "报警", "错误", "异常"]
    print("\n" + "=" * 70)
    print("含报错关键字的行")
    print("=" * 70)
    hits = 0
    for f in files:
        p = os.path.join(OUTDIR, f)
        if os.path.getsize(p) > 8 * 1024 * 1024:
            continue
        try:
            txt = io.open(p, encoding="utf-8", errors="ignore").read()
        except Exception:
            continue
        for ln in txt.splitlines():
            if any(k in ln for k in KEY):
                print("  [%s] %s" % (f, ln.strip()[:200]))
                hits += 1
                if hits > 80:
                    print("  ...（截断）")
                    return
    if hits == 0:
        print("  没找到（可能日志不是文本，或者关键字不对）")


if __name__ == "__main__":
    main()
