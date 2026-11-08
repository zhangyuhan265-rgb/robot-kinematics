# -*- coding: utf-8 -*-
import nrc_interface as n

print("queue 相关接口：")
for x in sorted(dir(n)):
    if "queue" in x.lower():
        print("  ", x)
print()
print("MoveCmd 字段：")
c = n.MoveCmd()
for a in sorted(dir(c)):
    if not a.startswith("_"):
        print("  ", a, "=", getattr(c, a, None))
