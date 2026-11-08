# -*- coding: utf-8 -*-
"""用 MuJoCo 加载我们的 URDF，和自研 FK 对账。
如果两边一致，就说明模型在第三个独立实现里也对。
"""
import numpy as np
import mujoco

URDF = "nbt_arm6_on_riser.urdf"

print("MuJoCo", mujoco.__version__)
m = mujoco.MjModel.from_xml_path(URDF)
d = mujoco.MjData(m)
print("nq=%d  nu=%d  nbody=%d  njnt=%d" % (m.nq, m.nu, m.nbody, m.njnt))
print()

print("关节：")
for j in range(m.njnt):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    print("  %-3d %-12s type=%d  qposadr=%d  range=[%.3f, %.3f]"
          % (j, name, m.jnt_type[j], m.jnt_qposadr[j],
             m.jnt_range[j][0], m.jnt_range[j][1]))

print()
print("body：")
for b in range(m.nbody):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
    print("  %-3d %s" % (b, name))

# 零位下 tcp 的位置
d.qpos[:] = 0.0
mujoco.mj_forward(m, d)
print()
for nm in ("tcp", "link5", "link3"):
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, nm)
    if bid >= 0:
        print("  %-7s 世界位置 = (%.4f, %.4f, %.4f) m" % (nm, *d.xpos[bid]))
