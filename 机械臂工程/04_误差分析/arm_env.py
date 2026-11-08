# -*- coding: utf-8 -*-
"""壁面打印机械臂的训练环境骨架（纯运动学，不依赖物理引擎）。

为什么不用物理引擎：
    壁面打印的核心是"末端走对轨迹、不撞墙"，属于运动学 + 碰撞问题。
    真需要接触/柔顺再换 MuJoCo / Isaac，本文件的分层可以整体替换。

结构：
    UrdfArm     —— 只做 FK / Jacobian / 简单 IK，与任务无关，可复用
    ReachEnv    —— 一个最小可跑的任务：把末端挪到目标点（占位任务，供后续替换）

用法：
    python arm_env.py            # 跑 5 个 episode 看形状

⚠️ 这是**骨架**，不是能直接训的策略环境。要替换的有三处，文件里用 TODO 标了：
    1. 观测/动作的定义（任务相关）
    2. 奖励（壁面打印该奖励"沿墙走直线 + 贴合墙面"，不是"到点"）
    3. 碰撞检测（目前只有关节限位，没有和墙/自碰撞）
"""
import os
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------- 运动学
def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]; return M


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]; return M


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    M = np.eye(4); M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]; return M


def _T(xyz, rpy):
    """URDF 的 <origin>：rpy 是**弧度**，顺序 Rz(yaw)·Ry(pitch)·Rx(roll)"""
    roll, pitch, yaw = rpy
    M = np.eye(4)
    M[:3, :3] = (_Rz(yaw) @ _Ry(pitch) @ _Rx(roll))[:3, :3]
    M[:3, 3] = xyz
    return M


def _axis_rot(axis, ang):
    x, y, z = axis
    n = (x * x + y * y + z * z) ** 0.5
    x, y, z = x / n, y / n, z / n
    c, s = np.cos(ang), np.sin(ang); C = 1 - c
    M = np.eye(4)
    M[:3, :3] = [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                 [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                 [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]
    return M


def _rotvec(R):
    """旋转矩阵 → 旋转向量（轴×角）。位姿 IK 的姿态误差用它。"""
    ang = np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))
    if ang < 1e-12:
        return np.zeros(3)
    return ang / (2 * np.sin(ang)) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class UrdfArm:
    """从一个 URDF 里抽出串联链，做 FK / 数值 Jacobian / IK。单位：米、弧度。"""

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        links, joints = {}, {}
        for l in root.findall("link"):
            links[l.get("name")] = True
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(t) for t in (o.get("xyz") if o is not None else "0 0 0").split()]
            rpy = [float(t) for t in (o.get("rpy") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(t) for t in (ax.get("xyz") if ax is not None else "0 0 1").split()]
            lim = j.find("limit")
            lo = float(lim.get("lower")) if lim is not None and lim.get("lower") else -np.pi
            hi = float(lim.get("upper")) if lim is not None and lim.get("upper") else np.pi
            joints[j.get("name")] = dict(
                name=j.get("name"), type=j.get("type"),
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                T=_T(xyz, rpy), axis=axis, lo=lo, hi=hi)

        childs = {v["child"] for v in joints.values()}
        self.root_link = [n for n in links if n not in childs][0]
        by_parent = {v["parent"]: v for v in joints.values()}
        chain, cur = [], self.root_link
        while cur in by_parent:
            j = by_parent[cur]
            chain.append(j)
            cur = j["child"]
        self.chain = chain

        self.rev = [i for i, j in enumerate(chain) if j["type"] == "revolute"]
        self.nq = len(self.rev)
        self.lo = np.array([chain[k]["lo"] for k in self.rev])
        self.hi = np.array([chain[k]["hi"] for k in self.rev])
        # 末端 link 名（链的最后一节）
        self.tip = chain[-1]["child"] if chain else self.root_link

    # ---- FK ----
    def fk(self, q):
        """q: (nq,) 弧度 → {link: 4x4}"""
        out = {self.root_link: np.eye(4)}
        M = np.eye(4)
        k = 0
        for j in self.chain:
            M = M @ j["T"]
            if j["type"] == "revolute":
                M = M @ _axis_rot(j["axis"], q[k]); k += 1
            out[j["child"]] = M.copy()
        return out

    def tcp(self, q):
        return self.fk(q)[self.tip][:3, 3].copy()

    # ---- Jacobian（位置部分，数值差分，简单但够用）----
    def jacobian(self, q, h=1e-6):
        p0 = self.tcp(q)
        J = np.zeros((3, self.nq))
        for i in range(self.nq):
            qq = q.copy(); qq[i] += h
            J[:, i] = (self.tcp(qq) - p0) / h
        return J

    # ---- IK：Levenberg-Marquardt + 多起点 + 残差校验 ----
    #
    # 这里原来是「固定阻尼 DLS + 单起点 + 每步直接 clip 到限位」的写法。
    # 实测（见 ik_verify.py，60 个随机可达位姿）：
    #   单起点成功率 50%，失败时**静默**返回差 1.5 m 的解，不抛错；
    #   即使目标可达，40 个随机起点里也只有 16~22 个能收敛（局部极小是结构性的）。
    # 现在：LM 阻尼自适应 + 多起点 + 显式返回残差，往返成功率 98%。

    def _err_pos(self, Pt):
        return lambda q: Pt - self.tcp(q)

    def _err_pose(self, Pt, Rt):
        def f(q):
            T = self.fk(q)[self.tip]
            return np.r_[Pt - T[:3, 3], _rotvec(Rt @ T[:3, :3].T)]
        return f

    def _num_jac(self, q, err, e0, h=1e-7):
        J = np.zeros((len(e0), self.nq))
        for i in range(self.nq):
            qq = q.copy()
            qq[i] += h
            J[:, i] = (err(qq) - e0) / h
        return J

    def _joint_frames(self, q):
        """走一遍链，返回每个转动关节的 (轴原点, 轴方向) 世界坐标 + 末端位置。"""
        frames = []
        M = np.eye(4)
        k = 0
        for j in self.chain:
            M = M @ j["T"]
            if j["type"] == "revolute":
                p = M[:3, 3].copy()
                z = M[:3, :3] @ np.asarray(j["axis"], dtype=float)
                frames.append((p, z / np.linalg.norm(z)))
                M = M @ _axis_rot(j["axis"], q[k])
                k += 1
        return frames, M[:3, 3].copy()

    def jacobian_geo(self, q):
        """几何 Jacobian（6×nq）：[Jv; Jw]，一次 FK 走完。

        比数值差分快约 7 倍（后者每迭代要 7 次 FK）。位置 IK 取前 3 行，
        位姿 IK 取全部 6 行。
        """
        frames, p_tip = self._joint_frames(q)
        J = np.zeros((6, self.nq))
        for i, (p, z) in enumerate(frames):
            J[:3, i] = np.cross(z, p_tip - p)
            J[3:, i] = z
        return J

    def _lm(self, seed, err, dim=3, iters=150, step=0.2, tol=1e-8, stall=40,
            trace=None):
        """单起点 LM。返回 (q, |e|)。

        dim=3 位置约束，dim=6 位姿约束。阻尼随成败自适应，步长限幅，
        每步裁剪到限位；连续 stall 次没有改善就放弃这个起点。
        trace 传一个 list 进来可以记录每步的 (q, |e|, λ)，用于可视化/调试。
        """
        q = np.clip(np.array(seed, dtype=float), self.lo, self.hi)
        e = err(q)
        cost = float(e @ e)
        lam, best, since = 1e-3, cost, 0
        if trace is not None:
            trace.append((q.copy(), float(np.linalg.norm(e)), lam))
        for _ in range(iters):
            if np.linalg.norm(e) < tol:
                break
            if since > stall:
                break
            J = self.jacobian_geo(q)[:dim]
            try:
                # 注意符号：J 是几何 Jacobian（p 对 q 的导数），而误差是
                # e = 目标 − 当前，所以 de/dq = −J，这里必须用 +J⁺e。
                # （早期的数值版本算的是 de/dq 本身，才配 −J⁺e。）
                dq = np.linalg.solve(J.T @ J + lam * np.eye(self.nq), J.T @ e)
            except np.linalg.LinAlgError:
                break
            n = np.linalg.norm(dq)
            if n > step:
                dq *= step / n
            qn = np.clip(q + dq, self.lo, self.hi)
            en = err(qn)
            cn = float(en @ en)
            if trace is not None:
                trace.append((qn.copy(), float(np.linalg.norm(en)), lam))
            if cn < cost:                 # 接受：放松阻尼
                q, e, cost = qn, en, cn
                lam = max(lam * 0.3, 1e-9)
                if cn < best - 1e-14:
                    best, since = cn, 0
                else:
                    since += 1
            else:                         # 拒绝：加大阻尼
                lam *= 10.0
                since += 1
                if lam > 1e7:
                    break
        return q, float(np.linalg.norm(e))

    def _seed_lib(self):
        """冷启动种子库：零位 + 每个中段关节单独取正/负半行程。

        实测（ik_seeds.py）：随机起点即使给 6 个也只有 52%；
        零位 1 个就有 97%、6 ms；零位 + 种子库 6 个到 100%。
        随机起点差是因为它均匀撒在关节盒里，大量落在折叠姿态上。
        """
        if not hasattr(self, "_seed_lib_cache"):
            lib = [np.zeros(self.nq)]
            for i in range(1, self.nq - 1):
                for v in (0.5 * self.hi[i], 0.5 * self.lo[i]):
                    s = np.zeros(self.nq)
                    s[i] = v
                    lib.append(s)
            self._seed_lib_cache = lib
        return self._seed_lib_cache

    def _multi(self, q0, err, dim=3, tries=8, iters=150, rng=None, tol=1e-4):
        """多起点求解，返回 (q, |e|, 是否收敛)。

        顺序：q0 → 种子库 → 随机。
        **收敛的解里优先取离 q0 最近的**——真机从当前位姿出发，不需要关节大行程；
        随机起点容易收敛到远处的另一个解，那会平白多转几百毫米。
        q0 本身收敛就直接返回（热启动，通常 1 次就够）。
        """
        q0 = np.clip(np.asarray(q0, dtype=float), self.lo, self.hi)
        if rng is None:
            if not hasattr(self, "_ik_rng"):
                self._ik_rng = np.random.default_rng(0)
            rng = self._ik_rng
        cands = [q0] + self._seed_lib()
        sols, best_q, best_e = [], None, np.inf
        for k in range(max(1, int(tries))):
            s = cands[k] if k < len(cands) else rng.uniform(self.lo, self.hi)
            q, e = self._lm(s, err, dim=dim, iters=iters)
            if e < best_e:
                best_q, best_e = q, e
            if e < tol:
                if k == 0:                      # 热启动成功，直接用它
                    return q, e, True
                d = float(np.linalg.norm(q - q0))
                if d < 0.5:                     # 已经够近了，不必再找
                    return q, e, True
                sols.append((d, q, e))
        if sols:
            _, q, e = min(sols, key=lambda t: t[0])
            return q, e, True
        return best_q, best_e, False

    def ik(self, q0, target, iters=150, tries=8, tol=1e-4, rng=None):
        """位置 IK。返回 (q, 残差米)——残差就是校验值，> tol 说明没解到。"""
        q, e, _ = self.ik_ex(q0, target, iters=iters, tries=tries, tol=tol, rng=rng)
        return q, e

    def ik_ex(self, q0, target, iters=150, tries=8, tol=1e-4, rng=None):
        """位置 IK，返回 (q, 残差, 是否收敛)。**做可达性判断时用这个。**"""
        Pt = np.asarray(target, dtype=float)
        q, e, _ = self._multi(q0, self._err_pos(Pt), dim=3, tries=tries,
                              iters=iters, rng=rng, tol=tol)
        return q, e, bool(e <= tol)

    def ik_pose(self, q0, Pt, Rt, iters=200, tries=8, tol=1e-6, rng=None):
        """位姿 IK（位置 + 姿态都约束）。返回 (q, 位置残差米, 姿态残差度)。"""
        Pt = np.asarray(Pt, dtype=float)
        Rt = np.asarray(Rt, dtype=float)
        q, e, _ = self._multi(q0, self._err_pose(Pt, Rt), dim=6, tries=tries,
                              iters=iters, rng=rng)
        T = self.fk(q)[self.tip]
        return (q, float(np.linalg.norm(Pt - T[:3, 3])),
                float(np.degrees(np.linalg.norm(_rotvec(Rt @ T[:3, :3].T)))))


# ---------------------------------------------------------------- 环境
class ReachEnv:
    """最小任务：把末端挪到目标点。**占位任务**，真任务是壁面打印轨迹跟随。

    观测  obs  = [q(nq), tcp(3), target(3), clearance(1)]   全 SI 单位
    动作  act  = Δq(nq)，每步限幅 ±act_limit（rad）
    奖励  r    = 上一帧距离 − 当前距离 − 碰撞惩罚

    wall: (n, d) —— 半空间 {p : n·p ≤ d}，臂在里侧。None 表示不放墙。
    """

    def __init__(self, urdf_path, seed=0, act_limit=0.05, max_steps=200,
                 wall=None, safe_margin=0.05):
        self.arm = UrdfArm(urdf_path)
        try:
            from arm_collision import CollisionModel      # 同目录
        except Exception:
            import os as _os
            import sys as _sys
            _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
            from arm_collision import CollisionModel
        self.col = CollisionModel(urdf_path)
        self.wall = wall                    # ((nx,ny,nz), d) 或 None
        self.safe_margin = safe_margin
        self.rng = np.random.default_rng(seed)
        self.act_limit = act_limit
        self.max_steps = max_steps
        self.q = np.zeros(self.arm.nq)
        self.target = np.zeros(3)
        self.steps = 0

    def clearance(self):
        """到墙的最小间隙（米）；没放墙就返回一个很大的数"""
        if self.wall is None:
            return 1e9
        n, d = self.wall
        dist, _ = self.col.distance_to_plane(self.q, n, d)
        return dist

    # TODO(1/3): 观测/动作是任务相关的最先要改的地方
    def _obs(self):
        return np.concatenate([self.q, self.arm.tcp(self.q), self.target,
                               [self.clearance()]])

    def reset(self):
        # 起始位姿：在零位附近抖动，且必须离墙有安全余量（撞墙的起点直接重抽）
        for _ in range(100):
            self.q = np.clip(self.rng.normal(0.0, 0.15, self.arm.nq),
                             self.arm.lo, self.arm.hi)
            if self.clearance() > self.safe_margin:
                break
        # 目标点：可达工作空间内的随机点（用 IK 反求确认可达；且不能落在墙外）
        for _ in range(80):
            cand = np.array([self.rng.uniform(0.3, 1.4),
                             self.rng.uniform(-0.6, 0.6),
                             self.rng.uniform(0.4, 1.9)])
            if self.wall is not None:
                n, dd = self.wall
                if dd - float(np.dot(np.asarray(n, float) / np.linalg.norm(n), cand)) < self.safe_margin + 0.05:
                    continue
            _, err = self.arm.ik(self.q, cand, iters=80, tries=3)
            if err < 0.01:
                self.target = cand
                break
        else:
            self.target = self.arm.tcp(self.q)
        self.steps = 0
        if hasattr(self, "_last_d"):
            del self._last_d
        return self._obs()

    def step(self, action):
        action = np.clip(np.asarray(action, dtype=float), -self.act_limit, self.act_limit)
        self.q = np.clip(self.q + action, self.arm.lo, self.arm.hi)
        self.steps += 1

        d_prev = self._last_d if hasattr(self, "_last_d") else None
        d = float(np.linalg.norm(self.target - self.arm.tcp(self.q)))
        # TODO(2/3): 奖励要换成"沿墙直线 + 贴合墙面 + 不超限"，不是单纯接近
        reward = (d_prev - d) if d_prev is not None else 0.0
        clr = self.clearance()
        collided = clr < self.safe_margin
        if collided:
            reward -= 1.0 + 20.0 * (self.safe_margin - clr)
        done = d < 0.02 or collided or self.steps >= self.max_steps
        if d < 0.02:
            reward += 10.0
        self._last_d = d
        # TODO(3/3): 自碰撞也要惩罚；目前只查了墙
        info = {"dist": d, "success": d < 0.02, "clearance": clr, "collision": collided}
        return self._obs(), reward, done, info


# ---------------------------------------------------------------- demo
def main():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        os.path.join(here, "..", "models", "nbt_arm6", "urdf", "nbt_arm6_on_riser.urdf"),
        os.path.join(here, "nbt_arm6_on_riser.urdf"),
        os.path.join(here, "..", "nbt_arm6_on_riser.urdf"),
    ]
    urdf = next((os.path.abspath(p) for p in cands if os.path.exists(p)), None)
    if urdf is None:
        raise SystemExit("找不到 URDF，试过：\n  " + "\n  ".join(cands))

    arm = UrdfArm(urdf)
    print("关节数 %d   末端 link = %s" % (arm.nq, arm.tip))
    print("限位(度) = %s" % np.round(np.degrees(arm.lo), 0))
    q0 = np.zeros(arm.nq)
    print("零位 末端位置 = %s m" % np.round(arm.tcp(q0), 4))

    env = ReachEnv(urdf, seed=0, wall=((1.0, 0.0, 0.0), 1.10))
    print("放了一面墙：x = 1.10 m（法向 +X），安全余量 50 mm")
    for ep in range(5):
        obs = env.reset()
        total, done, info = 0.0, False, {}
        while not done:
            # 占位策略：直接用 IK 求一步方向（不是学习出来的，只验证环境能跑）
            q_tgt, _ = env.arm.ik(env.q, env.target, iters=30)
            act = np.clip(q_tgt - env.q, -env.act_limit, env.act_limit)
            obs, r, done, info = env.step(act)
            total += r
        print("  episode %d: 步数 %3d  最终距离 %.4f m  间隙 %+7.1f mm  累计奖励 %+7.3f  %s"
              % (ep, env.steps, info["dist"], info["clearance"] * 1000, total,
                 "撞了" if info["collision"] else ("成功" if info["success"] else "超时")))


if __name__ == "__main__":
    main()
