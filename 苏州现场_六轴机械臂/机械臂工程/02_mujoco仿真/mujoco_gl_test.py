import time

import mujoco
import mujoco.viewer

XML = """
<mujoco>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="2 2 .1"/>
    <body pos="0 0 1">
      <joint type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 -0.3 0 0 0.3" size="0.06" rgba=".7 .7 .8 1"/>
    </body>
  </worldbody>
</mujoco>
"""

m = mujoco.MjModel.from_xml_string(XML)
d = mujoco.MjData(m)
print("模型建好，开始 launch_passive ...")
try:
    with mujoco.viewer.launch_passive(m, d) as v:
        print("  launched, is_running =", v.is_running())
        for i in range(6):
            time.sleep(0.5)
            mujoco.mj_step(m, d)
            v.sync()
            print("  %.1fs  is_running = %s" % (0.5 * (i + 1), v.is_running()))
except Exception as e:
    print("  抛异常:", type(e).__name__, e)
print("结束")
