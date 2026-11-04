"""
teleop/arm_kinematics.py
========================

双臂 7-DoF 运动学 —— **用 pinocchio 做逆解把遥操位姿变成关节目标**。

为什么需要这一层
----------------
本方案走**客户端自己逆解**的路子：实时会话以 ``cartesianIkPath: "none"``
入控，只接受**关节包**（``controlSpace: 0`` / ``commandMode: 0``）。于是
VR 增量 -> 影子目标位姿 -> **本模块的 IK** -> 绝对关节角 -> 控制器。

算法结构对齐 ``tele_ik``
------------------------
本模块刻意**只移植 tele_ik 的算法结构**，模型换成真机 URDF
（左臂 ``urdf_dualarm/left_arm.urdf``、右臂 ``urdf_dualarm/right_arm.urdf``，
两条臂各读各的，互不依赖）：

===================  ==========================================================
tele_ik 的做法      本方案的对应
===================  ==========================================================
``self.init_data``   ``q_seed`` —— 上一拍解（首次 = 实测关节角）作 warm start
作 warm start
``0.1 * smooth_cost`` 零空间姿态回拉 ``posture_gain``，防七轴冗余跳解支
（拉向上一拍解）      （tele_ik 用全局二次正则，这里用零空间投影 —— 更精准，
                      不与主任务竞争）
``opti.bounded``     每步裁剪到 ``model.lower/upperPositionLimit``
(lower, var_q, upper)
``translational_cost`` 位置残差 ``tol_pos`` 与姿态残差 ``tol_rot`` **分开**判阈值
+ ``rotational_cost``
``solve_fk(q)``      ``fk_world`` / ``fk_base`` 对账
出 ``L_ee``/``R_ee``
目标不可达 ->        解不出 ⇒ 返回 ``None``，**本包丢弃且影子不推进**
``opti.debug.value``
===================  ==========================================================

⚠️ tele_ik 用的是 pinocchio 3.x 的 ``RobotWrapper`` API，在 pin 4.1 下
**不可直接跑**；本模块是按同一算法结构重写的 pin 4.x 实现。

实测性能
--------
pinocchio DLS（8 次迭代、7-DOF）单臂中位 **0.10 ms**、p95 0.11 ms
⇒ 双臂 90 Hz（11.1 ms 预算）余量充足。

坐标系（关键，别搞混）
----------------------
URDF 里的 ``world`` 与控制器上报的**世界系不是同一个系**：URDF 的两条臂
基座是 ``origin rpy="±1.57 1.57 0"``，而真机 ``baseFrame``
实测为 ``pos=(0, ±0.2, 0)``（见 :mod:`teleop_map`）。两者的旋转还差一个
转置关系 —— 直接混用会让 IK 目标整体转到某个奇怪的位置。

因此本模块的**唯一口径**：

* IK / FK 都在**该臂基座系（base_link）**里做。
* 世界系位姿与基座系位姿之间靠 ``T_world_base`` 换算，它的来源二选一：
  1. URDF 里固定的 ``arm_left_base`` / ``arm_right_base``（缺省，仅离线用）；
  2. 控制器的 ``robot/getKinematicGeometry -> baseFrame``（**真机请用这个**，
     见 :func:`DualArmKinematics.sync_base_from_client`）。

⚠️ 关节序号 / 转向也是**约定**：URDF 的 ``j1..j7`` 与控制器上报的
``jointActualPosition[0..6]`` 是否同序同向**没有文档保证**。本模块提供
:func:`detect_joint_convention`：把实测关节角代入 FK，与控制器上报的
``tcpPose`` 对比，自动探测「同序 / 反序 / 取反」里哪个口径对得上。
对不上就说明约定不同，别急着下发关节包。

末端 frame
----------
缺省取 ``{prefix}_j7_Link``（URDF 的最后一节，等价于法兰）。工具偏置暂不支持
（本项目 ``flange`` 零偏置，TCP 即法兰）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence

import numpy as np
import pinocchio as pin

__all__ = [
    "PREFIX_OF",
    "ARM_OF_PREFIX",
    "DEFAULT_URDF_DIR",
    "DEFAULT_URDFS",
    "ArmKinematics",
    "DualArmKinematics",
    "JointTracker",
    "IkResult",
    "detect_joint_convention",
    "convention_matches",
    "JOINT_CONVENTIONS",
    "quat_xyzw_to_matrix",
    "matrix_to_quat_xyzw",
    "rpy_to_matrix",
    "matrix_to_rpy",
    "pose_error",
    "pose_error_world",
]

# 控制器臂名 -> URDF 前缀
PREFIX_OF = {"LeftArm": "left", "RightArm": "right"}
ARM_OF_PREFIX = {v: k for k, v in PREFIX_OF.items()}

#: 每条臂各用自己的 URDF：左臂读 left_arm.urdf，右臂读 right_arm.urdf
DEFAULT_URDF_DIR = Path(__file__).resolve().parent.parent / "urdf_dualarm"
DEFAULT_URDFS: dict[str, Path] = {
    "LeftArm": DEFAULT_URDF_DIR / "left_arm.urdf",
    "RightArm": DEFAULT_URDF_DIR / "right_arm.urdf",
}

#: 每条臂的关节数
NDOF = 7


# ===========================================================================
# 姿态工具（内部统一 xyzw 四元数 —— 与 teleop_map / robot_client 一致）
# ===========================================================================
def quat_xyzw_to_matrix(q: Sequence[float]) -> np.ndarray:
    """``(qx,qy,qz,qw)`` -> 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in q)
    return np.asarray(pin.Quaternion(w, x, y, z).matrix(), dtype=float)


def matrix_to_quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 旋转矩阵 -> ``(qx,qy,qz,qw)``（取 w>=0 的那个表示，保证连续比较）。"""
    q = pin.Quaternion(np.asarray(R, dtype=float))
    q.normalize()
    if q.w < 0.0:
        q = pin.Quaternion(-q.w, -q.x, -q.y, -q.z)
    return (float(q.x), float(q.y), float(q.z), float(q.w))


def rpy_to_matrix(rpy: Sequence[float]) -> np.ndarray:
    """``(roll, pitch, yaw)`` -> R = Rz(yaw)·Ry(pitch)·Rx(roll)。

    与 :func:`teleop_map.quat_from_rpy` / 控制器 ``a,b,c`` 口径一致。
    """
    return np.asarray(pin.rpy.rpyToMatrix(np.asarray(rpy, dtype=float)), dtype=float)


def matrix_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """3x3 -> ``(roll, pitch, yaw)``（pinocchio 的 ZYX 约定，与上面互逆）。"""
    rpy = pin.rpy.matrixToRpy(np.asarray(R, dtype=float))
    return (float(rpy[0]), float(rpy[1]), float(rpy[2]))


def pose_error(T_a: pin.SE3, T_b: pin.SE3) -> tuple[float, float]:
    """两个位姿的 ``(位置误差 m, 姿态误差 rad)``，姿态取测地角。"""
    e_pos = float(np.linalg.norm(T_a.translation - T_b.translation))
    dR = T_a.rotation.T @ T_b.rotation
    e_rot = float(np.linalg.norm(pin.log3(dR)))
    return e_pos, e_rot


def pose_error_world(T_world: pin.SE3, pos: Sequence[float],
                     rpy: Sequence[float]) -> tuple[float, float]:
    """``T_world`` 与「世界系位置 + RPY」的位姿误差 ``(位置 m, 姿态 rad)``。

    控制器上报的 ``tcpPose`` 用的就是世界系 RPY（``a,b,c``，ZYX），所以
    这个函数是「拿实测关节角做 FK，再和控制器说的末端位姿对账」的入口 ——
    见 :func:`detect_joint_convention`。
    """
    T_ref = pin.SE3(rpy_to_matrix(rpy), np.asarray(pos, dtype=float))
    return pose_error(T_world, T_ref)


# ===========================================================================
# 逆解结果
# ===========================================================================
@dataclass
class IkResult:
    """一次逆解的结果。

    ``ok`` 表示「收敛且满足容差」——调用方**必须**看它再决定下发，
    不然 IK 失败时会把一个乱掉的关节角当目标发出去。
    """

    q: np.ndarray                  # 关节角（rad），7 维
    err_pos: float                 # 位置残差（m）
    err_rot: float                 # 姿态残差（rad）
    ok: bool
    iters: int
    at_limit: bool = False         # 是否有关节顶在限位上

    def summary(self) -> str:
        return (f"{'OK ' if self.ok else 'FAIL'} "
                f"err={self.err_pos * 1000:.3f}mm/{math.degrees(self.err_rot):.3f}° "
                f"iters={self.iters}"
                + ("  [顶限位]" if self.at_limit else ""))


# ===========================================================================
# 单臂运动学
# ===========================================================================
class ArmKinematics:
    """一条 7-DoF 臂的 FK / IK（pinocchio 实现）。

    每条臂加载**自己的**单臂 URDF（缺省 ``DEFAULT_URDFS[arm]``），模型天然
    是 7 维 —— :meth:`fk_base` / :meth:`ik_base` 的 ``q`` 就是干净的长度 7
    向量，与控制器 ``jointActualPosition`` 一一对应（顺序见 :attr:`joint_names`）。
    若传入的是含两条臂的合并 URDF，则会把另一条臂的关节锁死后降维。
    """

    def __init__(self, arm: str, urdf_path: Path | str | None = None,
                 tcp_frame: str | None = None,
                 base_pos: Sequence[float] = (0.0, 0.0, 0.0),
                 base_rpy: Optional[Sequence[float]] = None,
                 base_matrix: Optional[np.ndarray] = None):
        if arm not in PREFIX_OF:
            raise ValueError(f"未知臂名 {arm!r}（应为 {list(PREFIX_OF)}）")
        self.arm = arm
        self.prefix = PREFIX_OF[arm]
        self.urdf_path = Path(urdf_path) if urdf_path else DEFAULT_URDFS[arm]
        if not self.urdf_path.exists():
            raise FileNotFoundError(f"URDF 不存在: {self.urdf_path}")

        full = pin.buildModelFromUrdf(str(self.urdf_path))
        # 本臂的 7 个关节在完整模型里的 joint index
        self._full_joint_ids = [
            i for i in range(1, full.njoints)
            if full.names[i].startswith(f"{self.prefix}_j")
        ]
        if len(self._full_joint_ids) != NDOF:
            raise RuntimeError(
                f"{arm}: URDF 里找到 {len(self._full_joint_ids)} 个 "
                f"{self.prefix}_j* 关节，期望 {NDOF} 个（{full.names[1:]}）")

        # 单臂 URDF 无需降维；合并 URDF 才锁死另一条臂的关节
        lock_ids = [i for i in range(1, full.njoints)
                    if i not in self._full_joint_ids]
        if lock_ids:
            model = pin.buildReducedModel(full, lock_ids, pin.neutral(full))
        else:
            model = full
        if model.nq != NDOF:
            raise RuntimeError(f"{arm}: 降维后 nq={model.nq}，期望 {NDOF}")

        self.model = model
        self.data = model.createData()
        self.joint_names = [model.names[i] for i in range(1, model.njoints)]

        self.tcp_frame = tcp_frame or f"{self.prefix}_j7_Link"
        self.frame_id = model.getFrameId(self.tcp_frame)
        if self.frame_id >= len(model.frames):
            raise RuntimeError(
                f"{arm}: URDF 里找不到末端 frame {self.tcp_frame!r}")

        self.lower = np.asarray(model.lowerPositionLimit, dtype=float).copy()
        self.upper = np.asarray(model.upperPositionLimit, dtype=float).copy()
        # URDF 里 j4 的限位是 [-2.53, 1.05]（非对称）——保留原样，不要对称化

        # URDF 的 world -> base_link（固定关节，静态）
        self.T_urdf_world_base = self._frame_pose_at_neutral(f"{self.prefix}_base_link")
        # 世界系（控制器口径）-> base_link。缺省用 URDF 自带的装法，
        # 真机请用 set_base_from_controller() 覆盖成实测 baseFrame。
        self.T_world_base = pin.SE3(
            rpy_to_matrix(base_rpy if base_rpy is not None
                          else matrix_to_rpy(self.T_urdf_world_base.rotation)),
            np.asarray(base_pos, dtype=float))
        if base_matrix is not None:
            self.T_world_base = pin.SE3(np.asarray(base_matrix, dtype=float),
                                        np.asarray(base_pos, dtype=float))

    # ---------------- 基础 ----------------
    def _frame_pose_at_neutral(self, frame_name: str) -> pin.SE3:
        """在关节全零位时取某 frame 在 URDF world 下的位姿。"""
        fid = self.model.getFrameId(frame_name)
        d = self.model.createData()
        pin.forwardKinematics(self.model, d, pin.neutral(self.model))
        pin.updateFramePlacements(self.model, d)
        return pin.SE3(np.array(d.oMf[fid].rotation, dtype=float),
                       np.array(d.oMf[fid].translation, dtype=float))

    def set_base_from_controller(self, pos: Sequence[float],
                                 rpy: Sequence[float]) -> None:
        """用控制器 ``baseFrame``（``{x,y,z,a,b,c}``，世界系）设定基座位姿。

        这是**真机唯一正确的用法** —— URDF 的装法与实测 ``baseFrame``
        不是同一个数（见模块 docstring）。
        """
        self.T_world_base = pin.SE3(rpy_to_matrix(rpy),
                                    np.asarray(pos, dtype=float))

    def set_base_from_arm_frame_dict(self, bf: dict) -> bool:
        """``robot/getKinematicGeometry -> baseFrame`` 字典 -> 基座位姿。"""
        if not bf or not all(k in bf for k in ("x", "y", "z", "a", "b", "c")):
            return False
        self.set_base_from_controller((bf["x"], bf["y"], bf["z"]),
                                      (bf["a"], bf["b"], bf["c"]))
        return True

    @property
    def base_pos(self) -> np.ndarray:
        return np.array(self.T_world_base.translation, dtype=float)

    @property
    def base_rpy(self) -> tuple[float, float, float]:
        return matrix_to_rpy(self.T_world_base.rotation)

    def clamp(self, q: Iterable[float]) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.lower, self.upper)

    def wrist_offset_in_tcp(self) -> tuple[float, float, float]:
        """腕心（j6 关节原点）在 **TCP 体坐标系**下的位置（m）。

        j5/j6 轴交于腕心，j7 轴也过腕心，所以这个偏移是常量（与关节角无关）。
        遥操时让旋转绕腕心转而不是绕 TCP 转：只有腕部三个关节需要动，
        大关节不用跟着摆。
        """
        j6 = self.model.getFrameId(f"{self.prefix}_j6_Link")
        if j6 >= len(self.model.frames):
            raise RuntimeError(f"{self.arm}: URDF 里找不到腕心 frame "
                               f"{self.prefix}_j6_Link")
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        T_tcp = self.data.oMf[self.frame_id]
        w = np.asarray(self.data.oMf[j6].translation, dtype=float)
        r = np.asarray(T_tcp.rotation, dtype=float).T @ (
            w - np.asarray(T_tcp.translation, dtype=float))
        return (float(r[0]), float(r[1]), float(r[2]))

    # ---------------- 正解 ----------------
    def fk_base(self, q: Sequence[float]) -> pin.SE3:
        """关节角 -> 末端位姿（**基座系**）。"""
        qq = np.asarray(q, dtype=float)
        if qq.shape != (NDOF,):
            raise ValueError(f"{self.arm}: q 必须是 {NDOF} 维（收到 {qq.shape}）")
        pin.forwardKinematics(self.model, self.data, qq)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.frame_id]
        # 去掉 URDF world -> base 的固定变换，只留 base -> tcp
        return self.T_urdf_world_base.inverse() * pin.SE3(
            np.array(M.rotation, dtype=float),
            np.array(M.translation, dtype=float))

    def fk_world(self, q: Sequence[float]) -> pin.SE3:
        """关节角 -> 末端位姿（**控制器世界系**）。"""
        return self.T_world_base * self.fk_base(q)

    def fk_world_pose(self, q: Sequence[float]):
        """便利封装：返回 ``(pos, quat_xyzw)``。"""
        T = self.fk_world(q)
        return (tuple(float(v) for v in T.translation),
                matrix_to_quat_xyzw(T.rotation))

    # ---------------- 逆解 ----------------
    def ik_base(self, T_target: pin.SE3, q_seed: Sequence[float], *,
                max_iters: int = 60, tol: float = 1e-6,
                tol_pos: float | None = None, tol_rot: float | None = None,
                damping: float = 1e-6, step: float = 0.5,
                posture_gain: float = 0.0,
                joint_weights: Sequence[float] | None = None) -> IkResult:
        """阻尼最小二乘（DLS）逆解（**基座系**目标）。

        ``joint_weights`` —— 逐关节"移动代价"权重（>0，越大越不愿动）。
        解的是加权最小范数：``dq = W⁻¹Jᵀ(JW⁻¹Jᵀ+λI)⁻¹ e``。大关节给大权重、
        腕部给 1，冗余下位姿误差优先由腕部关节吸收，只转手腕时大关节基本不动。
        不改变可达性，只改变冗余自由度的分配；``None`` = 全 1（原行为）。

        对应 tele_ik 的求解结构：

        * ``q_seed`` —— warm start（tele_ik 的 ``self.init_data``）。
        * ``posture_gain`` —— 零空间姿态回拉，把解往 ``q_seed`` 拉，
          对应 tele_ik 的 ``0.1 * smooth_cost``（拉向上一拍解）。
          用零空间投影而不是全局正则 —— 主任务（位姿误差）不受影响，
          只在冗余自由度上把解钉在同一支，避免七轴跳解支导致关节突跳。
        * ``tol_pos`` / ``tol_rot`` —— 位置与姿态**分开**判阈值，
          对应 tele_ik 的 ``translational_cost`` / ``rotational_cost``。
        * ``step`` —— 每步关节增量的折扣系数（0.5 稳，1.0 快但易振荡）。
        * 关节限位**每步都裁剪** —— 对应 tele_ik 的
          ``opti.bounded(lower, var_q, upper)``。顶到限位时残差降不下去
          会体现为 ``ok=False``，而不是悄悄越界。

        返回 ``ok=False`` 时调用方**必须**丢包并回滚影子目标 ——
        详见 :mod:`teleop_loop`。
        """
        q = self.clamp(q_seed)
        q_seed_arr = np.asarray(q_seed, dtype=float)
        T_target_urdf = self.T_urdf_world_base * T_target
        eye7 = np.eye(NDOF)
        if joint_weights is None:
            w_inv = eye7
        else:
            w = np.asarray(joint_weights, dtype=float)
            if w.shape != (NDOF,) or np.any(w <= 0.0):
                raise ValueError(f"joint_weights 须是 {NDOF} 个正数（收到 {w}）")
            w_inv = np.diag(1.0 / w)
        iters = 0
        ok = False

        for iters in range(1, max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            M = self.data.oMf[self.frame_id]
            err = np.asarray(pin.log6(M.actInv(T_target_urdf)).vector, dtype=float)
            e_pos, e_rot = pose_error(M, T_target_urdf)
            if tol_pos is not None or tol_rot is not None:
                if e_pos <= (tol_pos if tol_pos is not None else 1e9) and \
                        e_rot <= (tol_rot if tol_rot is not None else 1e9):
                    ok = True
                    break
            elif float(np.linalg.norm(err)) <= tol:
                ok = True
                break

            J = pin.computeFrameJacobian(self.model, self.data, q,
                                         self.frame_id, pin.LOCAL)
            J = np.asarray(J, dtype=float)[:, :NDOF]
            JJt = J @ w_inv @ J.T + damping * np.eye(6)
            Jpinv = w_inv @ J.T @ np.linalg.inv(JJt)
            dq = Jpinv @ err
            if posture_gain > 0.0:
                # 零空间姿态回拉（tele_ik smooth_cost 的 pinocchio 化）
                dq = dq + (eye7 - Jpinv @ J) @ (posture_gain * (q_seed_arr - q))
            q = self.clamp(pin.integrate(self.model, q, step * dq))

        # ---- FK 对账（tele_ik solve_fk 出 L_ee/R_ee 的等价物）----
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.frame_id]
        e_pos, e_rot = pose_error(M, T_target_urdf)
        at_limit = bool(np.any(np.isclose(q, self.lower, atol=1e-9))
                        or np.any(np.isclose(q, self.upper, atol=1e-9)))
        return IkResult(q=q, err_pos=e_pos, err_rot=e_rot, ok=ok,
                        iters=iters, at_limit=at_limit)

    def ik_world(self, pos: Sequence[float], quat_xyzw: Sequence[float],
                 q_seed: Sequence[float], **kwargs) -> IkResult:
        """逆解（**控制器世界系**目标位姿 -> 关节角）。"""
        T_world = pin.SE3(quat_xyzw_to_matrix(quat_xyzw),
                         np.asarray(pos, dtype=float))
        return self.ik_base(self.T_world_base.inverse() * T_world, q_seed, **kwargs)

    def ik_multistart(self, T_target: pin.SE3, q_seed: Sequence[float], *,
                      n_starts: int = 24, seed: int = 0,
                      **kwargs) -> IkResult:
        """多起点逆解 —— 从 ``q_seed`` 出发解不出来时换种子再解。

        用途：**首次**把末端送到某个绝对位姿（比如归位准备点）时，
        当前关节角可能离目标太远、或落在不同的解支上，单起点容易失败。
        每帧跟随（每帧只差一点点）用 :meth:`ik_base` 就好，这里太慢。
        """
        best = self.ik_base(T_target, q_seed, **kwargs)
        if best.ok:
            return best
        rng = np.random.default_rng(seed)
        for _ in range(n_starts):
            q0 = rng.uniform(self.lower, self.upper)
            r = self.ik_base(T_target, q0, **kwargs)
            if r.ok and (not best.ok or r.err_pos < best.err_pos):
                best = r
            if best.ok:
                break
        return best


# ===========================================================================
# 双臂（对外便利入口）
# ===========================================================================
class DualArmKinematics:
    """``{臂名: ArmKinematics}`` 的薄封装，给遥操主循环直接用。"""

    def __init__(self, urdf_paths: Optional[dict[str, Path | str]] = None,
                 arms: Sequence[str] = ("LeftArm", "RightArm"),
                 tcp_frames: Optional[dict[str, str]] = None,
                 base_frames: Optional[dict[str, dict]] = None):
        """``urdf_paths``：``{臂名: URDF 路径}``，缺省各用 ``DEFAULT_URDFS``。"""
        self.paths: dict[str, Path] = {
            arm: Path((urdf_paths or {}).get(arm) or DEFAULT_URDFS[arm])
            for arm in arms
        }
        self.arms: dict[str, ArmKinematics] = {}
        for arm in arms:
            k = ArmKinematics(arm, self.paths[arm],
                              tcp_frame=(tcp_frames or {}).get(arm))
            bf = (base_frames or {}).get(arm)
            if bf is not None:
                k.set_base_from_arm_frame_dict(bf)
            self.arms[arm] = k

    def __getitem__(self, arm: str) -> ArmKinematics:
        return self.arms[arm]

    def __contains__(self, arm: str) -> bool:
        return arm in self.arms

    def fk_base(self, arm: str, q: Sequence[float]) -> pin.SE3:
        return self.arms[arm].fk_base(q)

    def fk_world(self, arm: str, q: Sequence[float]) -> pin.SE3:
        return self.arms[arm].fk_world(q)

    def ik_world(self, arm: str, pos: Sequence[float],
                 quat_xyzw: Sequence[float], q_seed: Sequence[float],
                 **kwargs) -> IkResult:
        return self.arms[arm].ik_world(pos, quat_xyzw, q_seed, **kwargs)

    def sync_base_from_client(self, rc, arms: Optional[Sequence[str]] = None) -> dict:
        """从控制器读 ``baseFrame`` 覆盖每条臂的基座位姿。

        返回 ``{arm: (pos, rpy)}``，读不到的臂保持 URDF 缺省并给出 ``None``。

        ⚠️ **上机前必须跑这一步** —— URDF 的 world 与控制器世界系不是同一个系，
        不同步的话 IK 目标整体偏，表现为"手柄往右走、末端往别的方向走"。
        """
        out: dict[str, Optional[tuple]] = {}
        for arm in (arms or self.arms.keys()):
            if arm not in self.arms:
                continue
            try:
                bf = rc.base_frame(arm) or {}
            except Exception:
                bf = {}
            ok = self.arms[arm].set_base_from_arm_frame_dict(bf)
            out[arm] = (self.arms[arm].base_pos, self.arms[arm].base_rpy) if ok else None
        return out


def build_dual_arm_kinematics(
        urdf_paths: Optional[dict[str, Path | str]] = None,
        arms: Sequence[str] = ("LeftArm", "RightArm"),
        base_frames: Optional[dict[str, dict]] = None) -> DualArmKinematics:
    """一行构造双臂运动学（``base_frames`` 传控制器 ``baseFrame`` 字典）。"""
    return DualArmKinematics(urdf_paths, arms=arms, base_frames=base_frames)


# ===========================================================================
# 关节通道：目标位姿 -> 目标关节（含限速与种子维护）
# ===========================================================================
class JointTracker:
    """把「目标末端位姿」逐帧解成「目标关节角」，并按关节速度限幅。

    逐帧 IK 的两个坑，这里都处理掉：

    1. **种子**：每帧拿**上一帧的目标关节**当种子（不是实测 —— 实测有
       伺服滞后，跟随目标时会持续落在后面，种子越用越偏）。首次调用前
       必须用 :meth:`seed_from` 灌入当前实测关节角。
    2. **跳解支**：七轴冗余有无数解。种子变了就容易跳到另一支，表现为
       关节突跳、甚至整臂甩出去。用「以上一帧目标为种子 + 打开
       ``posture_gain`` 零空间回拉 + 每帧关节增量限幅」三重保险把它钉住。

    限速单位是 **rad/帧**：``max_step_deg`` 是单帧单关节允许的最大增量。
    """

    def __init__(self, kin: ArmKinematics, *,
                 max_step_deg: float = 3.0,
                 posture_gain: float = 0.02,
                 tol_pos_m: float = 1e-4,
                 tol_rot_rad: float = 1e-3,
                 max_iters: int = 60,
                 damping: float = 1e-6,
                 step: float = 0.5,
                 joint_weights: Optional[Sequence[float]] = None):
        self.kin = kin
        self.joint_weights = (None if joint_weights is None
                              else tuple(float(v) for v in joint_weights))
        self.max_step = math.radians(max_step_deg)
        self.posture_gain = posture_gain
        self.tol_pos = tol_pos_m
        self.tol_rot = tol_rot_rad
        self.max_iters = max_iters
        self.damping = damping
        self.step = step
        self.q: Optional[np.ndarray] = None
        self.last_ok = True
        #: 最近一次 IK 的详细结果（诊断用）
        self.last_result: Optional[IkResult] = None

    # ---------------- 种子 ----------------
    def seed_from(self, q_measured: Optional[Sequence[float]]) -> None:
        """用当前实测关节角重置种子（接管 / 重锚 / 归位开始时调用）。"""
        if q_measured is None:
            return
        q = np.asarray(q_measured, dtype=float)
        if q.shape != (NDOF,):
            return
        self.q = self.kin.clamp(q)

    def reset(self) -> None:
        self.q = None
        self.last_ok = True
        self.last_result = None

    @staticmethod
    def _limit_step(dq: np.ndarray, step: float) -> np.ndarray:
        """单帧关节增量限幅：**等比缩放**，保持各关节比例。

        不用逐关节 ``np.clip``：各关节独立截断会打乱它们之间的比例，
        关节空间的方向变了，末端就从直线变成弧线。取最大分量缩到
        ``step``，其余关节同比例缩小，方向不变。
        """
        m = float(np.max(np.abs(dq)))
        if m > step:
            dq = dq * (step / m)
        return dq

    # ---------------- 求解 ----------------
    def solve(self, pos: Sequence[float], quat_xyzw: Sequence[float], *,
              max_step_deg: Optional[float] = None) -> Optional[np.ndarray]:
        """目标世界系位姿 -> 限速后的目标关节角。解不出来返回 ``None``。

        返回 ``None`` 时调用方**不要下发**，并应把影子目标回滚 ——
        否则影子会漂到控制器跟不到的地方。
        """
        if self.q is None:
            return None
        r = self.kin.ik_world(pos, quat_xyzw, self.q,
                              max_iters=self.max_iters,
                              tol_pos=self.tol_pos, tol_rot=self.tol_rot,
                              posture_gain=self.posture_gain,
                              damping=self.damping, step=self.step,
                              joint_weights=self.joint_weights)
        self.last_result = r
        if not r.ok:
            self.last_ok = False
            return None
        self.last_ok = True
        # 单帧关节增量限幅（含限位裁剪）—— 防跳解支、防冲出去
        step = (math.radians(max_step_deg) if max_step_deg is not None
                else self.max_step)
        dq = self._limit_step(r.q - self.q, step)
        q_new = self.kin.clamp(self.q + dq)
        self.q = q_new
        return q_new

    def solve_multistart(self, pos: Sequence[float],
                         quat_xyzw: Sequence[float],
                         **kwargs) -> Optional[np.ndarray]:
        """绝对目标（归位准备点）用：单起点失败就多起点再解。"""
        if self.q is None:
            return None
        T_world = pin.SE3(quat_xyzw_to_matrix(quat_xyzw),
                          np.asarray(pos, dtype=float))
        r = self.kin.ik_multistart(self.kin.T_world_base.inverse() * T_world,
                                   self.q, max_iters=self.max_iters,
                                   tol_pos=self.tol_pos, tol_rot=self.tol_rot,
                                   posture_gain=self.posture_gain,
                                   damping=self.damping, step=self.step,
                                   **kwargs)
        self.last_result = r
        if not r.ok:
            self.last_ok = False
            return None
        self.last_ok = True
        dq = self._limit_step(r.q - self.q, self.max_step)
        self.q = self.kin.clamp(self.q + dq)
        return self.q

    # ---------------- 观测 ----------------
    def residual(self, pos: Sequence[float],
                 quat_xyzw: Sequence[float]) -> tuple[float, float]:
        """当前关节目标对应的末端位姿与给定位姿的 ``(位置误差, 姿态误差)``。"""
        if self.q is None:
            return (float("inf"), float("inf"))
        T = self.kin.fk_world(self.q)
        T_ref = pin.SE3(quat_xyzw_to_matrix(quat_xyzw),
                        np.asarray(pos, dtype=float))
        return pose_error(T, T_ref)


# ===========================================================================
# 关节约定探测（同序 / 反序 / 取反）
# ===========================================================================
#: 候选口径名 -> 把控制器实测关节角变成 URDF 关节角的变换
JOINT_CONVENTIONS = {
    "同序": lambda q: q,
    "反序": lambda q: q[::-1].copy(),
    "全取反": lambda q: -q,
    "反序+取反": lambda q: -q[::-1],
}


def detect_joint_convention(kin: "ArmKinematics",
                            q_measured: Sequence[float],
                            tcp_pose: dict,
                            *,
                            pos_tol_m: float = 0.02,
                            rot_tol_rad: float = math.radians(2.0),
                            ) -> list[tuple[str, float, float]]:
    """探测 URDF 关节角与控制器实测关节角之间的口径。

    背景：URDF 里的 ``j1..j7`` 与控制器 ``jointActualPosition[0..6]``
    **是否同序同向没有文档保证**。猜错的后果是机械臂朝完全相反的方向动，
    而且**不报任何错**。

    做法：把实测关节角按每个候选口径变换后做 FK，与控制器上报的
    ``tcpPose``（世界系位置 + RPY）对比，取误差最小的口径。

    参数
    ----
    ``kin``
        已按控制器 ``baseFrame`` 标定好的运动学对象。
    ``q_measured``
        控制器实测的 7 个关节角。
    ``tcp_pose``
        控制器上报的 ``tcpPose`` 字典（需含 ``x,y,z,a,b,c``）。

    返回
    ----
    ``[(口径名, 位置误差 m, 姿态误差 rad), ...]``，按误差从小到大排序。
    调用方自行判断哪个对得上。
    """
    q = np.asarray(q_measured, dtype=float)
    if q.shape != (NDOF,):
        raise ValueError(f"实测关节角必须是 {NDOF} 维（收到 {q.shape}）")
    if not all(k in tcp_pose for k in ("x", "y", "z", "a", "b", "c")):
        raise ValueError("tcp_pose 需含 x,y,z,a,b,c")
    pos = (tcp_pose["x"], tcp_pose["y"], tcp_pose["z"])
    rpy = (tcp_pose["a"], tcp_pose["b"], tcp_pose["c"])

    out: list[tuple[str, float, float]] = []
    for name, fn in JOINT_CONVENTIONS.items():
        qq = np.asarray(fn(q), dtype=float)
        T = kin.fk_world(qq)
        e_pos, e_rot = pose_error_world(T, pos, rpy)
        out.append((name, e_pos, e_rot))
    out.sort(key=lambda t: t[1] + 0.01 * t[2])
    return out


def convention_matches(e_pos: float, e_rot: float, *,
                       pos_tol_m: float = 0.02,
                       rot_tol_rad: float = math.radians(2.0)) -> bool:
    """某个口径的误差是否算「对得上」。"""
    return e_pos < pos_tol_m and e_rot < rot_tol_rad

