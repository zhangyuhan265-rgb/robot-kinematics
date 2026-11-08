# 06 机械臂冒烟测试 —— 纳博特 NexDroid（nrc_interface）
#
# 依据官方文档：open.inexbot.com
#   默认端口 6001；Linux 扩展库要求 Python 3.8，Windows 要求 Python 3.10 x64。
#
# 跑法：
#   假模式（不需要 SDK，先验证流程）：
#     ~/miniforge3/envs/dl/bin/python ~/projects/07_机器人学/练习/06_机械臂冒烟.py --dry-run
#   真机（需要 nrc_interface.py + _nrc_host.so 已就位）：
#     ~/miniforge3/envs/arm38/bin/python ~/projects/07_机器人学/练习/06_机械臂冒烟.py --ip 192.168.1.15
#
# 安全约定（别改）：
#   - 默认速度 20（控制器速度百分比，范围 0<speed<=100）
#   - 单次关节变化超过 MAX_DELTA 直接拒绝，防手滑
#   - 到位判断用"与目标比对"，不用"是否停止变化"
#   - Ctrl+C / 任何异常都走 finally：先停、再下电、再断开

import argparse
import logging
import sys
import time

log = logging.getLogger("arm")

MAX_DELTA = 10.0         # 度，单次运动允许的最大关节变化
SETTLE_TOL = 0.1         # 度，判断"到了"的阈值
SETTLE_TIMEOUT = 20.0    # s，等待到位的总时长
DEFAULT_PORT = "6001"    # 纳博特上位机 SDK 默认端口


class ArmError(RuntimeError):
    """机械臂相关错误统一从这儿抛。"""


class Arm:
    """纳博特 NexDroid 上位机 SDK 包一层。"""

    def __init__(self, ip, port=DEFAULT_PORT, dry_run=False):
        self.ip = ip
        self.port = port
        self.dry_run = dry_run
        self._fd = None
        self._nrc = None
        self._fake_joints = [0.0] * 7

    # ---------- 内部 ----------

    def _sdk(self):
        """延迟导入：假模式下不需要装 SDK。"""
        if self._nrc is None:
            try:
                import nrc_interface as nrc
            except ImportError as exc:
                raise ArmError(
                    "导入 nrc_interface 失败。确认 nrc_interface.py 和 _nrc_host.so "
                    "已经放到 Python 能找到的目录，且 Python 是 3.8（Linux）"
                ) from exc
            self._nrc = nrc
        return self._nrc

    def _check(self, result, what):
        nrc = self._sdk()
        if result != nrc.SUCCESS:
            raise ArmError(f"{what} 失败，错误码 {result}")

    def _read_servo_state(self):
        nrc = self._sdk()
        result, state = nrc.get_servo_state(self._fd, 0)
        if result != nrc.SUCCESS:
            raise ArmError(f"读取伺服状态失败，错误码 {result}")
        return state

    # ---------- 生命周期 ----------

    def connect(self):
        log.info("连接控制器 %s:%s", self.ip, self.port)
        if self.dry_run:
            log.info("[dry-run] 假装连接成功")
            return
        nrc = self._sdk()
        fd = nrc.connect_robot(self.ip, str(self.port))
        if fd <= 0:
            raise ArmError(f"连接失败：{self.ip}:{self.port}")
        self._fd = fd

        deadline = time.time() + 10
        while nrc.get_connection_status(fd) != nrc.SUCCESS:
            if time.time() > deadline:
                raise ArmError("连接后等待就绪超时")
            time.sleep(0.2)
        log.info("连接成功，SDK 版本 %s", nrc.get_library_version())

    def power_on(self):
        log.info("上电")
        if self.dry_run:
            log.info("[dry-run] 假装上电完成")
            return
        nrc = self._sdk()
        state = self._read_servo_state()
        log.info("当前伺服状态：%s（0=停止 1=就绪 2=报警 3=运行）", state)

        if state == 3:
            return
        if state == 2:
            self._check(nrc.clear_error(self._fd), "clear_error")
            state = 1
        if state == 0:
            self._check(nrc.set_servo_state(self._fd, 1), "set_servo_state(1)")
        self._check(nrc.set_servo_poweron(self._fd), "set_servo_poweron")

        # 命令返回成功 ≠ 已经上电，必须轮询到状态 3
        deadline = time.time() + 90
        while time.time() < deadline:
            state = self._read_servo_state()
            if state == 3:
                log.info("伺服已进入运行状态（3）")
                return
            time.sleep(0.5)
        raise ArmError(f"等待伺服上电超时，最后状态 {state}")

    def enable(self):
        """纳博特里"上电"即包含使能；这一步只是设速度、切模式。"""
        log.info("设置速度与模式")
        if self.dry_run:
            log.info("[dry-run] 假装已设置")
            return
        nrc = self._sdk()
        self._check(nrc.set_current_mode(self._fd, 0), "set_current_mode(示教)")
        self._check(nrc.set_speed(self._fd, 20), "set_speed(20)")
        # 队列运动必须先打开队列状态，否则 push 的 moveJ 不会被执行
        self._check(nrc.queue_motion_set_status(self._fd, True), "queue_motion_set_status(True)")

    def get_state(self) -> dict:
        if self.dry_run:
            return {"joints": list(self._fake_joints), "tcp": [0.0] * 7, "status": "ready"}
        nrc = self._sdk()
        pos = nrc.VectorDouble(7)
        self._check(nrc.get_current_position(self._fd, 0, pos), "get_current_position")
        return {"joints": [pos[i] for i in range(7)], "tcp": [pos[i] for i in range(7)], "status": "running"}

    def move_joints(self, joints, speed_pct=20, wait=True):
        log.info("关节运动 → %s  速度 %s%%  （单位：度）", [round(v, 3) for v in joints], speed_pct)
        if self.dry_run:
            self._fake_joints = list(joints)
            return
        nrc = self._sdk()
        cmd = nrc.MoveCmd()
        cmd.coord = 0
        cmd.targetPosType = nrc.PosType_data
        cmd.targetPosValue = nrc.VectorDouble(list(joints))
        cmd.velocity = speed_pct
        cmd.acc = 20
        cmd.dec = 20
        cmd.pl = 5
        self._check(nrc.queue_motion_push_back_moveJ(self._fd, cmd), "push_back_moveJ")
        self._check(nrc.queue_motion_send_to_controller(self._fd, 1), "send_to_controller")

    def set_gripper(self, closed: bool):
        """夹爪走控制器数字输出。端口号要按现场接线改。"""
        log.info("夹爪 %s", "闭合" if closed else "张开")
        if self.dry_run:
            return
        nrc = self._sdk()
        do_port = 1                       # ← 现场核对！从 1 开始
        self._check(nrc.set_digital_output(self._fd, do_port, 1 if closed else 0),
                    "set_digital_output")

    def wait_until_reached(self, target) -> bool:
        """轮询关节角，直到与目标差小于阈值。判断"到没到"，不是"停没停"。"""
        if self.dry_run:
            log.info("[dry-run] 假装已到位")
            return True
        deadline = time.time() + SETTLE_TIMEOUT
        gap = float("inf")
        while time.time() < deadline:
            now = self.get_state()["joints"]
            gap = max(abs(a - b) for a, b in zip(now[:6], target[:6]))
            if gap < SETTLE_TOL:
                log.info("已到位，最大残差 %.4f 度", gap)
                return True
            time.sleep(0.05)
        log.error("等待到位超时（%.1f s），仍差 %.4f 度", SETTLE_TIMEOUT, gap)
        return False

    def stop(self):
        log.warning("停止")
        if self.dry_run:
            return
        nrc = self._sdk()
        self._check(nrc.set_servo_state(self._fd, 0), "set_servo_state(0=停止)")

    def disconnect(self):
        if self.dry_run or self._fd is None:
            return
        nrc = self._sdk()
        try:
            nrc.set_servo_poweroff(self._fd)
        except Exception:
            log.warning("下电失败，但继续断开")
        finally:
            nrc.disconnect_robot(self._fd)
            self._fd = None
            log.info("已断开连接")


def check_delta(current, target):
    """防手滑：单次关节变化太大就拒绝。"""
    worst = max(abs(a - b) for a, b in zip(current[:6], target[:6]))
    if worst > MAX_DELTA:
        raise ArmError(f"目标与当前偏差 {worst:.3f} 度，超过上限 {MAX_DELTA}，拒绝执行")
    return worst


def main():
    ap = argparse.ArgumentParser(description="机械臂冒烟测试（纳博特 NexDroid）")
    ap.add_argument("--ip", default="192.168.1.15", help="控制器 IP")
    ap.add_argument("--port", default=DEFAULT_PORT, help="SDK 端口，默认 6001")
    ap.add_argument("--speed", type=int, default=20, help="速度百分比 1-100")
    ap.add_argument("--jog-deg", type=float, default=5.0, help="单次摆动角度（度）")
    ap.add_argument("--dry-run", action="store_true", help="不用真机，只验证流程")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(message)s",
                        datefmt="%H:%M:%S")

    arm = Arm(args.ip, args.port, dry_run=args.dry_run)
    try:
        arm.connect()
        arm.power_on()
        arm.enable()

        state = arm.get_state()
        home = list(state["joints"])
        log.info("初始关节角(度)：%s", [round(v, 3) for v in home])

        target = list(home)
        target[0] += args.jog_deg                          # 只动 J1，单位：度
        check_delta(home, target)

        log.info("---- 开始动作 ----")
        arm.move_joints(target, args.speed)
        if not arm.wait_until_reached(target):
            raise ArmError("运动未在超时内到达目标")

        arm.set_gripper(True)
        time.sleep(0.5)
        arm.set_gripper(False)

        arm.move_joints(home, args.speed)
        if not arm.wait_until_reached(home):
            raise ArmError("回原位超时")

        log.info("---- 往返完成 ----")

    except KeyboardInterrupt:
        log.warning("用户中断")
    except Exception as exc:
        log.error("失败：%s", exc)
        raise
    finally:
        try:
            arm.stop()
        except Exception:
            log.warning("停止失败，现场请手动确认臂已停")
        finally:
            arm.disconnect()


if __name__ == "__main__":
    main()
