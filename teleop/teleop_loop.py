"""
teleop/teleop_loop.py
=====================

PICO **双臂增量式**遥操主循环 —— 把 VR 手柄的相对运动变成机械臂的
**绝对关节目标**。

整体链路
--------
.. code-block:: text

   VR 手柄位姿 --teleop_map--> 世界系增量 --累加--> 影子目标位姿
                                                        │
                                          arm_kinematics (pinocchio IK)
                                                        │
                                                  绝对关节角 (7维)
                                                        │
                                     robot_client.send_joint_target()
                                     controlSpace=0 / commandMode=0 / si
                                                        │
                                                     控制器

**增量式（相对模式）口径**：手柄位姿增量 -> 世界系增量 -> 累加到
**客户端影子目标位姿** -> IK -> **绝对**关节角下发。控制器侧不做累加
（``commandMode: 0``），累加器全在客户端，因此影子与实发目标严格同源，
不会发散。

按键语义（已确认）
------------------
============  ==================  ==============================================
物理键        逻辑键                动作
============  ==================  ==============================================
X（左手柄）    ``home.left``       **按住** -> 左臂缓慢回到准备点；松手即停
A（右手柄）    ``home.right``      **按住** -> 右臂缓慢回到准备点；松手即停
Y（左手柄）    ``teleop.enable``   **按下**（边沿）-> 双臂上电使能 + 建实时会话
B（右手柄）    ``teleop.disable``  **按下**（边沿）-> 双臂停控 + 下电失能
左握持         ``left_arm.active`` 拉满 -> 左臂**开始**遥操；松开 -> 停止下发左臂
右握持         ``right_arm.active``拉满 -> 右臂**开始**遥操；松开 -> 停止下发右臂
左菜单键       ``agv.toggle``      AGV 开环控制 连接 <-> 断开（边沿；不自动连接）
左摇杆拨动     （无绑定）          AGV 连接后直接拨动即开环运动，回中立即停
右摇杆按下     ``lift.toggle``     升降柱 使能 <-> 下电（边沿；不自动使能）
右摇杆拨动     （无绑定）          升降柱使能后直接上下拨动即点动，回中立即停
============  ==================  ==============================================

AGV 与升降柱同双臂的 Y 上电 / B 下电完全独立，各有独立连接和线程。

两类事件
--------
* **边沿锁存**（``consume_latched``）：Y / B / 重锚。这类"不容漏"的一次性
  动作必须走锁存通道 —— 控制循环 90 Hz、采集 200 Hz 不同步，一次快速轻点
  （按下到松开可能只有 30~80 ms）完全可能整个按压过程落在两次轮询之间，
  电平查询会彻底漏掉。
* **电平查询**（``query``）：X / A 按住归位、握持选臂。这类语义本来就是
  "按住多久就做多久"，用瞬时电平才对；用锁存会在松手后继续动作。

归位与遥操互斥
--------------
按住 X / A 归位时，**该臂**的握持遥操通道被冻结。两者都是往同一个影子
目标上叠加增量，混用会让影子发散、永远归不到位。

归位三个必须做对的点
--------------------
1. 归位基准用**控制器上一目标**而非实测位姿。实测滞后会让每包补滞后量
   => 冲过准备点 => 在附近振荡不收敛。
2. 到位判据用**实测**位姿，且**位置与姿态都要到位**。只判位置会
   "位置对了但朝向没转过去"。
3. 归位期间该臂遥操**冻结**，与遥操互斥。

关节模式下发包规则
------------------
* ``controlSpace: 0``、``commandMode: 0``（绝对关节角）、``unitType: si``（rad）。
* **全程只发关节包**，不混发末端包（§21.9：空间切换会让「上一目标」基准失效）。
* 首包必须是当前实测关节角，否则控制器判定 ``firstFrameJumpRejected`` 自动停控。
* IK 失败 => 丢包 + 影子**不推进**（否则影子漂到跟不到的地方，之后每帧都在
  追一个从未下发的目标）。
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from agv_client import AgvParams, AgvWorker                      # noqa: E402
from hand_client import HandParams, HandWorker, Revo2Hand       # noqa: E402
from lift_client import LiftParams, LiftWorker                   # noqa: E402
from robot_client import RobotClient                             # noqa: E402
from robot_config import load_config                             # noqa: E402
from teleop_map import (                                         # noqa: E402
    ROT_MODES, Alignment, AnchorDeadzone, Deadband, quat_angle, quat_conj,
    mat_vec, quat_from_rpy, quat_identity, quat_mul, quat_norm, quat_scale,
    quat_to_mat,
)
from xrt_reader import XrtReader                                 # noqa: E402

try:                                                            # noqa: E402
    from arm_kinematics import (                                 # noqa: E402
        DEFAULT_URDFS, JointTracker, build_dual_arm_kinematics,
    )
    _HAS_KIN = True
except Exception as _exc:  # pinocchio 缺失 —— 本方案必需，故后续会报错
    DEFAULT_URDFS = {}                # type: ignore[assignment]
    JointTracker = None               # type: ignore[assignment]
    build_dual_arm_kinematics = None  # type: ignore[assignment]
    _HAS_KIN = False
    _KIN_IMPORT_ERR = _exc
else:
    _KIN_IMPORT_ERR = None

log = logging.getLogger("teleop")

# ---------------------------------------------------------------------------
# 逻辑按键名（见 button_map.json）
# ---------------------------------------------------------------------------
KEY_LEFT_ACTIVE = "left_arm.active"
KEY_RIGHT_ACTIVE = "right_arm.active"
KEY_AGV_TOGGLE = "agv.toggle"         # 左菜单键：AGV 连接 <-> 断开
KEY_LIFT_TOGGLE = "lift.toggle"       # 右摇杆按下：升降柱 使能 <-> 下电
KEY_TELEOP_ON = "teleop.enable"       # Y 键（左 secondary）
KEY_TELEOP_OFF = "teleop.disable"     # B 键（右 secondary）
KEY_HOME = {"LeftArm": "home.left",   # X 键（左 primary）—— **按住**
            "RightArm": "home.right"}  # A 键（右 primary）—— **按住**

HAND_OF_ARM = {"LeftArm": "left", "RightArm": "right"}
KEY_HAND_TRIGGER = {"left": "hand.left", "right": "hand.right"}   # 扳机 -> 灵巧手
ACTIVE_KEY_OF_HAND = {"left": KEY_LEFT_ACTIVE, "right": KEY_RIGHT_ACTIVE}

HERE = Path(__file__).resolve().parent


# ===========================================================================
# 单臂遥操状态
# ===========================================================================
class ArmTeleop:
    """一条臂的遥操状态机。

    影子目标（``shadow_pos`` / ``shadow_quat``）是**客户端的绝对目标位姿**，
    每次遥操增量都往它上面累加。IK 把它解成关节角下发 —— 影子与实发目标
    严格同源（不像 ``commandMode: 1`` 那样由控制器维护基准）。
    """

    __slots__ = ("arm", "hand", "active", "align", "prev_hand",
                 "n_sent", "n_clamped", "moved_m", "turned_rad",
                 "shadow_pos", "shadow_quat", "n_cmd",
                 "warmup_left", "warmup_total", "singular_hits",
                 "deadzone", "joint_tracker", "n_ik_fail", "pivot")

    def __init__(self, arm: str, hand: str, warmup_frames: int = 0):
        self.arm = arm
        self.hand = hand
        self.active = False
        self.align: Alignment | None = None
        self.prev_hand = None
        self.n_sent = 0
        self.n_clamped = 0
        self.moved_m = 0.0        # 本次接管累计位移（m）
        self.turned_rad = 0.0     # 本次接管累计转角（rad）
        # 起跑缓冲剩余帧数：接管后这几帧只更新基准、不下发。
        # 摇杆中心与 VR 追踪原点不重合，接管瞬间的位姿噪声（实测 2~4mm）
        # 会直接变成一整个增量包，表现为"启动瞬间抽一下"。
        self.warmup_total = warmup_frames
        self.warmup_left = warmup_frames
        # 连续检测到接近奇异的帧数（用于旋转降级判定）
        self.singular_hits = 0
        # 影子目标位姿（**世界系**，绝对）。遥操增量往它上面累加，
        # IK 解出的关节角就是下发的绝对目标。
        self.shadow_pos: tuple = (0.0, 0.0, 0.0)
        self.shadow_quat: tuple = quat_identity()
        self.n_cmd = 0
        # 锚点滞回式死区：锚在**累积位姿**上，EMA 平滑 + 速度闸门。
        # 逐帧增量阈值会把慢速运动整个吃掉
        # （90Hz + 3.5mm 死区 ⇒ 阈值速度 315mm/s），必须用锚点式。
        self.deadzone = AnchorDeadzone()
        # 影子位姿 -> 目标关节的追踪器（pinocchio IK）
        self.joint_tracker = None
        self.n_ik_fail = 0
        # 旋转支点：腕心在 TCP 体坐标系下的位置。(0,0,0) = 绕 TCP 转（旧行为）。
        # 只有遥操增量用它；归位（home）不用，仍按 TCP 走。
        self.pivot: tuple = (0.0, 0.0, 0.0)

    # ---------------- 接管 / 释放 ----------------
    def engage(self, hand_frame, robot_quat, base_rpy=None,
               rot_mode: str = "world") -> None:
        """建立对齐并进入接管态。

        ``base_rpy`` 给定时走**每臂基座标对齐**（手柄的前/上/右 对齐到该臂
        基座标的前/上/右）；给 None 则退回世界系对齐（恒等平移）。

        ``rot_mode`` 决定**旋转口径**（见 :data:`teleop_map.ROT_MODES`）：

        * ``"world"``（默认）—— 世界系刚体传递，与参考实现
          `XRoboToolkit-Teleop-Sample-Python` 逐位一致。
        * ``"tcp_body"`` —— 末端体坐标系传递（"把手柄当工具握"手感）。

        ``warmup_left`` 在此重置为满值 —— 每次接管（含松手后再握上）
        都要重新走一遍起跑缓冲，否则"松手期间手在动、再握上"会立刻
        把那一大段位移当成增量发出去（这是增量式最典型的跳变）。
        """
        if base_rpy is not None:
            self.align = Alignment.build_base_frame(
                hand_frame.quat, robot_quat, base_rpy, rot_mode=rot_mode)
        else:
            self.align = Alignment.build(hand_frame.quat, robot_quat,
                                         rot_mode=rot_mode)
        self.prev_hand = hand_frame
        self.active = True
        self.moved_m = 0.0
        self.turned_rad = 0.0
        self.warmup_left = self.warmup_total
        self.singular_hits = 0
        # 接管瞬间重置死区：锚点/EMA 都要落在"接管时的手柄位姿"上，
        # 否则上一轮接管的残留会让新的一开始就误判。
        self.deadzone.reset(hand_frame.pos, hand_frame.quat)

    def release(self) -> None:
        """停止下发该臂的控制（**会话保留**，机器人停在最后目标）。

        影子目标**不清零** —— 松手再握上时若清零，影子与真实位置会脱节，
        重新接管会把这段偏差一次性补上（猛冲）。重新接管时由
        :meth:`set_shadow` 重新对齐到实测位姿。
        """
        self.active = False
        self.align = None
        self.prev_hand = None
        self.deadzone.reset()

    def set_shadow(self, pos, quat) -> None:
        """把影子目标对齐到当前实测位姿（接管/重锚时调用）。"""
        self.shadow_pos = tuple(pos)
        self.shadow_quat = quat_norm(quat)
        self.n_cmd = 0

    # ---------------- 影子累加 ----------------
    def compose(self, d_pos, d_quat, use_pivot: bool = False):
        """**不推进**影子，只算出「加了这个增量后的目标」—— 供逆解预演。

        每包先算目标位姿再逆解；逆解失败时这一包不能落地，
        影子也不能推进（否则影子漂到控制器跟不到的地方）。所以先
        :meth:`compose` 试算，成功后才 :meth:`apply_delta` 正式推进。

        ``use_pivot=True`` 时旋转绕**腕心**转（``self.pivot``）而不是绕 TCP：
        腕心 ``c = p + R·pivot`` 只随平移增量走，旋转不改变它，所以
        ``p_new = p + d_pos + (I - ΔR)·(R·pivot)``。
        """
        pos = [self.shadow_pos[0] + d_pos[0],
               self.shadow_pos[1] + d_pos[1],
               self.shadow_pos[2] + d_pos[2]]
        if use_pivot and any(self.pivot):
            r_w = mat_vec(quat_to_mat(self.shadow_quat), self.pivot)  # 世界系腕心偏移
            r_rot = mat_vec(quat_to_mat(d_quat), r_w)                 # ΔR·r_w
            for i in range(3):
                pos[i] += r_w[i] - r_rot[i]
        quat = quat_norm(quat_mul(d_quat, self.shadow_quat))
        return tuple(pos), quat

    def apply_delta(self, d_pos, d_quat, use_pivot: bool = False):
        """把一包增量累加到影子上，返回新的影子目标 ``(pos, quat)``。

        语义：**平移分量相加、旋转左乘**（``R_new = Δq ⊗ R_old``）——
        与 ``tele_ik`` 的姿态累加口径一致。

        ⚠️ 影子是**客户端的绝对目标**，与下发包严格同源（``commandMode: 0``）。
        控制器不会再叠一层基准，所以影子不会"漂"。
        """
        self.n_cmd += 1
        self.shadow_pos, self.shadow_quat = self.compose(d_pos, d_quat,
                                                         use_pivot)
        return self.shadow_pos, self.shadow_quat

    # ---------------- 每帧 ----------------
    def step(self, hand_frame, dt: float = 1.0 / 90.0):
        """算出一帧的 (平移增量, 旋转增量)，均为世界系。None = 无内容。

        **起跑缓冲**：接管后的前 ``warmup_total`` 帧只把 ``prev_hand``
        推到当前帧、不返回增量。这几帧的位移全部是「握持瞬间的姿态噪声」
        （Pico 静止抖动实测 2~4mm），缓冲把这些噪声在
        「只更新基准」的过程中自然吸收掉。

        **死区**：由 :class:`teleop_map.AnchorDeadzone` 统一处理
        （EMA 平滑 + 锚点滞回 + 速度闸门）。它返回的是「该不该下发」，
        而本方法返回的是「这一帧该下发的累积增量」——
        两者不是一回事：死区内的帧返回 ``(0,0,0)`` / 单位四元数，
        调用方据此跳过下发。
        """
        if not self.active or self.align is None or self.prev_hand is None:
            return None
        if not hand_frame.pose_valid:
            return None

        self.prev_hand = hand_frame

        if self.warmup_left > 0:
            self.warmup_left -= 1
            return None

        # 死区 + 平滑一体处理：返回该下发的累积增量
        d_pos, d_quat = self.deadzone.update(
            hand_frame.pos, hand_frame.quat, self.align, dt)
        self.moved_m += math.dist(d_pos, (0.0, 0.0, 0.0))
        self.turned_rad += quat_angle(d_quat)
        return d_pos, d_quat


# ===========================================================================
# 准备点（归位）
# ===========================================================================
@dataclass(frozen=True)
class HomePose:
    """一条臂的准备点配置 + 归位限速参数。

    内部一律用 ``(qx,qy,qz,qw)``；JSON 里存的是 **wxyz**（见
    :func:`load_home_poses`），因为那与下发 UDP 报文的旋转分量同序。
    """

    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]   # qx,qy,qz,qw
    max_speed_mps: float = 0.05
    arrive_tol_m: float = 0.005
    arrive_tol_rad: float = 0.035
    min_step_m: float = 0.0002
    rotate: bool = True
    max_rot_per_frame: float = math.radians(2.0)

    def is_placeholder(self) -> bool:
        """是否还是占位零值（没填真实准备点）。"""
        return math.dist(self.pos, (0.0, 0.0, 0.0)) < 1e-9


def load_home_poses(path: Path) -> tuple[dict[str, HomePose], bool]:
    """读 ``ready_pose.json``。返回 ``(每臂配置, only_when_idle)``。

    ``quat`` 用 **wxyz** 顺序（``qw, qx, qy, qz``）—— 与下发 UDP 报文的
    前三个旋转分量同序，避免手抄时把 w 和 x 搞反（w 排错位置算出来的
    姿态完全错乱，但仍是合法四元数，不会报错）。

    允许缺字段（给缺省值），但 ``pos`` / ``quat`` 必须存在且长度为 3 / 4。
    """
    raw = json.loads(path.read_text(encoding="utf-8"))
    out: dict[str, HomePose] = {}
    for arm, cfg in (raw.get("arms") or {}).items():
        pos = tuple(float(v) for v in (cfg.get("pos") or ()))
        quat = tuple(float(v) for v in (cfg.get("quat") or ()))
        if len(pos) != 3:
            raise ValueError(
                f"ready_pose.json: {arm} 的 pos 必须是 3 个数（pos={pos}）")
        if len(quat) != 4:
            raise ValueError(
                f"ready_pose.json: {arm} 的 quat 必须是 4 个数"
                f"（w,x,y,z 顺序，当前读到 {len(quat)} 个：{quat}）")
        norm = math.sqrt(sum(v * v for v in quat))
        if norm < 1e-9:
            raise ValueError(
                f"ready_pose.json: {arm} 的 quat 全零，不是合法单位四元数")
        if abs(norm - 1.0) > 1e-3:
            print(f"  ⚠ {arm} quat 模长 {norm:.6f} ≠ 1，已自动归一化"
                  f"（差 {(norm - 1) * 100:.2f}%，疑似手抄误差）")
        # wxyz -> 内部用的 xyzw
        out[arm] = HomePose(
            pos=pos,                                   # type: ignore[arg-type]
            quat=quat_norm((quat[1], quat[2], quat[3], quat[0])),
            max_speed_mps=float(cfg.get("max_speed_mps", 0.05)),
            arrive_tol_m=float(cfg.get("arrive_tol_mm", 5.0)) / 1000.0,
            arrive_tol_rad=float(cfg.get("arrive_tol_rad", 0.035)),
            min_step_m=float(cfg.get("min_step_mm", 0.2)) / 1000.0,
            rotate=bool(cfg.get("rotate", True)),
            max_rot_per_frame=math.radians(
                float(cfg.get("max_rot_per_frame_deg", 2.0))),
        )
    return out, bool(raw.get("only_when_idle", False))


# ===========================================================================
# 参数加载（teleop_params.json）
# ===========================================================================
@dataclass
class TeleopParams:
    """``teleop_params.json`` 的结构化视图。"""

    raw: dict

    # ---- stream ----
    @property
    def rate_hz(self) -> float:
        return float(self.raw.get("stream", {}).get("rate_hz", 90.0))

    @property
    def command_mode(self) -> int:
        return int(self.raw.get("stream", {}).get("command_mode", 0))

    @property
    def control_space(self) -> int:
        return int(self.raw.get("stream", {}).get("control_space", 0))

    @property
    def unit_type(self) -> str:
        return str(self.raw.get("stream", {}).get("unit_type", "si"))

    @property
    def hold_on_idle(self) -> bool:
        return bool(self.raw.get("stream", {}).get("hold_on_idle", True))

    @property
    def cartesian_ik_path(self) -> str:
        return str(self.raw.get("stream", {}).get("cartesian_ik_path", "none"))

    @property
    def joint_limit(self) -> dict:
        return dict(self.raw.get("stream", {}).get("joint_limit", {}))

    # ---- ik ----
    @property
    def ik(self) -> dict:
        return dict(self.raw.get("ik", {}))

    # ---- deadzone ----
    @property
    def deadzone(self) -> dict:
        return dict(self.raw.get("deadzone", {}))

    # ---- limits ----
    @property
    def limits(self) -> dict:
        return dict(self.raw.get("limits", {}))

    # ---- align ----
    @property
    def align(self) -> dict:
        return dict(self.raw.get("align", {}))

    # ---- home ----
    @property
    def home(self) -> dict:
        return dict(self.raw.get("home", {}))

    # ---- pd ----
    @property
    def pd(self) -> dict:
        return dict(self.raw.get("pd", {}))

    # ---- hand ----
    @property
    def hand(self) -> HandParams:
        return HandParams.from_dict(self.raw.get("hand"))

    # ---- agv ----
    @property
    def agv(self) -> AgvParams:
        return AgvParams.from_dict(self.raw.get("agv"))

    # ---- lift ----
    @property
    def lift(self) -> LiftParams:
        return LiftParams.from_dict(self.raw.get("lift"))


def load_params(path: Path) -> TeleopParams:
    """读 ``teleop_params.json``。文件缺失时返回全缺省值。"""
    p = Path(path)
    if not p.exists():
        print(f"  ⚠ 找不到 {p}，用内置缺省参数")
        return TeleopParams(raw={})
    try:
        raw = json.loads(p.read_text(encoding="utf-8")) or {}
    except (OSError, json.JSONDecodeError) as exc:
        print(f"  ⚠ 读 {p} 失败：{exc}，用内置缺省参数")
        return TeleopParams(raw={})
    return TeleopParams(raw=raw)


# ===========================================================================
# 主循环
# ===========================================================================
class TeleopRunner:
    # 类级缺省：测试里用 ``TeleopRunner.__new__`` 绕过 ``__init__`` 造实例，
    # 属性必须在这里有缺省，否则新加的字段会 AttributeError。
    params: TeleopParams = TeleopParams(raw={})
    kin = None
    _kin_ready: bool = False
    teleop_on: bool = False
    session_active: bool = False
    home_state: dict = {}
    home_cfg: dict = {}

    def __init__(self, args):
        self.args = args
        self.rc: RobotClient | None = None
        self.hand_workers: dict[str, HandWorker] = {}
        self._hand_links: list = []
        self.agv_worker: AgvWorker | None = None
        self.lift_worker: LiftWorker | None = None
        self._lift_link = None
        self.reader: XrtReader | None = None
        self.stop_flag = threading.Event()

        # ---- 参数 ----
        self.params = load_params(Path(getattr(args, "params", None)
                                      or HERE / "teleop_params.json"))
        p_ik = self.params.ik
        p_dz = self.params.deadzone
        p_lim = self.params.limits
        p_align = self.params.align

        self.arms: dict[str, ArmTeleop] = {
            arm: ArmTeleop(arm, hand,
                           warmup_frames=int(round(
                               getattr(args, "warmup_ms", 150.0)
                               * getattr(args, "freq", self.params.rate_hz) / 1000.0)))
            for arm, hand in HAND_OF_ARM.items()
        }
        # 把死区参数装进每臂（锚点滞回式：EMA + 锚点 + 速度闸门）。
        # 逐帧增量死区在 90 Hz 下阈值速度高达 315 mm/s，会把慢速遥操
        # 整个吃掉 —— 必须用锚点式，见 :class:`teleop_map.AnchorDeadzone`。
        for st in self.arms.values():
            st.deadzone = AnchorDeadzone(
                pos_deadzone_m=float(p_dz.get("pos_deadzone_m", 0.005)),
                pos_hysteresis_m=float(p_dz.get("pos_hysteresis_m", 0.002)),
                rot_deadzone_rad=float(p_dz.get("rot_deadzone_rad", 0.0175)),
                rot_hysteresis_rad=float(p_dz.get("rot_hysteresis_rad", 0.0087)),
                vel_deadzone_mps=float(p_dz.get("vel_deadzone_mps", 0.020)),
                vel_hysteresis_frac=float(p_dz.get("vel_hysteresis_frac", 0.5)),
                smooth_tau_s=float(p_dz.get("smooth_tau_s", 0.050)),
                leak=float(p_dz.get("leak", 0.05)),
            )

        self.speed = float(p_lim.get("scale", 1.0))
        # ---- 关节模式（本方案默认开启：客户端自己逆解）----
        self.joint_mode = bool(getattr(args, "joint_mode", True))
        self.kin = None
        self._kin_ready = False
        # ---- 遥操总开关（Y 开 / B 关）----
        self.teleop_on = False
        # ---- 实时会话是否已建立（Y 之后为 True，B 之后为 False）----
        self.session_active = False
        # ---- 归位状态：arm -> (travel_m, last_t, base_pos, base_quat)
        # 归位进行中时该臂的遥操被禁止（互斥），松手即停。
        # base_pos/base_quat 是**控制器基准**（本轮归位自己下发的目标），
        # 不是实测位姿 —— 见 _home_start 的说明。
        self.home_state: dict[str, tuple[float, float, tuple, tuple]] = {}
        self.home_cfg: dict[str, HomePose] = {}
        self.home_only_when_idle = False
        # 每臂 baseFrame 的 rpy 缓存（安装位姿是静态的，缓存一次即可）
        self._base_rpy_cache: dict[str, tuple | None] = {}
        # 对齐模式：base_frame = 每臂基座标对齐；world = 世界系对齐
        self.align_mode = str(p_align.get("mode", "base_frame"))
        # 旋转口径：world = 世界系刚体传递；tcp_body = 末端体坐标系传递
        self.rot_mode = str(p_align.get("rot_mode", "world"))

        # 单包限幅（死区由 AnchorDeadzone 负责，这里只做限幅）
        self.deadband = Deadband(
            pos_deadzone_m=0.0,
            rot_deadzone_rad=0.0,
            max_step_m=float(p_lim.get("max_step_m", 0.020)),
            max_step_rad=float(p_lim.get("max_step_rad", 0.0873)),
        )

    # ---------------- 机器人侧 ----------------
    def _robot_pose_cached(self, arm: str):
        """末端位姿 ``(pos, quat)``（世界系），**只读推送面缓存**。

        绝不做 TCP 回退 —— 本方法在控制热路径上（每包调用），
        一次 TCP 往返就会把 90 Hz 的控制环拖垮。拿不到就返回 ``None``。
        """
        st = self.rc.get_state(arm)
        if st is None or not st.tcp_pose:
            return None
        tp = st.tcp_pose
        if not all(k in tp for k in ("a", "b", "c")):
            return None
        return ((tp["x"], tp["y"], tp["z"]),
                quat_from_rpy(tp["a"], tp["b"], tp["c"]))

    def _robot_pose(self, arm: str):
        """末端当前位姿 ``(pos, quat)``（世界系）。

        优先读推送面缓存的 ``tcpPose``（其 ``a,b,c`` 是世界系 RPY，与
        ``robot/status.cartesian`` 完全重合，实测误差 0.00 mm）——
        **不在控制热路径上做 TCP 请求**，否则每帧一次往返会把 90 Hz
        的控制环拖垮。推送面还没到货时退回一次 ``robot/status``。
        """
        cached = self._robot_pose_cached(arm)
        if cached is not None:
            return cached
        cart = (self.rc.status(arm).get("cartesian") or {})
        if all(k in cart for k in ("a", "b", "c")):
            return ((cart["x"], cart["y"], cart["z"]),
                    quat_from_rpy(cart["a"], cart["b"], cart["c"]))
        return None

    # ---------------- 关节模式（pinocchio 逆解）----------------
    def _setup_kinematics(self, rc: RobotClient) -> bool:
        """建运动学模型并把基座位姿同步到控制器实测 ``baseFrame``。

        失败（没装 pinocchio / URDF 路径错）返回 False，调用方应直接退出 ——
        半个模型跑起来比不跑更危险。
        """
        if not self.joint_mode:
            return True
        if not _HAS_KIN:
            print(f"  ✗ 需要 pinocchio：{_KIN_IMPORT_ERR}")
            print("    安装：conda install -c conda-forge pinocchio")
            return False
        arms = list(self.args.arm)
        # 左臂读左臂 URDF、右臂读右臂 URDF，互不依赖
        cli = {"LeftArm": getattr(self.args, "urdf_left", None),
               "RightArm": getattr(self.args, "urdf_right", None)}
        urdfs = {arm: Path(cli.get(arm) or DEFAULT_URDFS[arm]) for arm in arms}
        for arm, p in urdfs.items():
            if not p.exists():
                print(f"  ✗ {arm} URDF 不存在：{p}")
                return False
            print(f"  · {arm} URDF：{p}")
        try:
            self.kin = build_dual_arm_kinematics(urdfs, arms=arms)
        except Exception as exc:
            print(f"  ✗ 建运动学模型失败：{exc}")
            return False
        if not getattr(self.args, "no_sync_base", False):
            synced = self.kin.sync_base_from_client(rc, arms)
            for arm, v in synced.items():
                if v is None:
                    print(f"  ⚠ {arm} 读不到 baseFrame，沿用 URDF 装法"
                          f"（可能与真机不符）")
                else:
                    pos, rpy = v
                    print(f"  · {arm} baseFrame 已同步：pos=({pos[0]:+.3f},"
                          f"{pos[1]:+.3f},{pos[2]:+.3f}) rpy=({rpy[0]:+.3f},"
                          f"{rpy[1]:+.3f},{rpy[2]:+.3f})")
        # 每臂挂一个关节追踪器
        ikp = self.params.ik
        max_step_deg = float(ikp.get("max_step_deg", 3.0))
        # 客户端单帧步长必须不超过控制器的关节限速，否则超出的部分由控制器
        # 逐关节独立截断，各关节比例被打乱，末端就从直线变成弧线。
        # 这里让客户端先按「等比缩放」限到控制器限速以内（留 10% 余量），
        # 控制器就不会再截断。
        jl = self.params.joint_limit
        if jl.get("enable", True):
            v_max = float(getattr(self.args, "max_vel", None)
                          or jl.get("max_velocity", 0.5))
            ctrl_step_deg = math.degrees(
                0.9 * v_max / float(getattr(self.args, "freq", 0) or
                                    self.params.rate_hz))
            if ctrl_step_deg < max_step_deg:
                print(f"  · 单帧关节步长 {max_step_deg:.2f}° → "
                      f"{ctrl_step_deg:.2f}°（对齐控制器限速 {v_max} rad/s）")
                max_step_deg = ctrl_step_deg
        # 逐关节移动代价权重：[大关节 J1-J4] × big，[腕 J5-J7] × 1。
        # 位姿误差优先由腕部吸收，只转手腕时大关节基本不动。
        big = float(ikp.get("big_joint_weight", 1.0))
        joint_weights = None if big == 1.0 else [big] * 4 + [1.0] * 3
        if joint_weights is not None:
            print(f"  · IK 关节权重：J1-J4 ×{big:g}，J5-J7 ×1")
        # 旋转支点：腕心（从各臂 URDF 算出，左右各用各的）
        pivot_on = bool(self.params.align.get("rotate_about_wrist", True))
        for arm in arms:
            st = self.arms[arm]
            if pivot_on:
                st.pivot = self.kin[arm].wrist_offset_in_tcp()
                print(f"  · {arm} 旋转支点=腕心，TCP 系偏移 "
                      f"({st.pivot[0]*1000:+.1f},{st.pivot[1]*1000:+.1f},"
                      f"{st.pivot[2]*1000:+.1f}) mm")
            st.joint_tracker = JointTracker(
                self.kin[arm],
                max_step_deg=max_step_deg,
                posture_gain=float(ikp.get("posture_gain", 0.02)),
                tol_pos_m=float(ikp.get("tol_pos_m", 1e-4)),
                tol_rot_rad=float(ikp.get("tol_rot_rad", 1e-3)),
                max_iters=int(ikp.get("max_iters", 60)),
                damping=float(ikp.get("damping", 1e-6)),
                step=float(ikp.get("step", 0.5)),
                joint_weights=joint_weights,
            )
        print(f"  · 关节模式：客户端逆解（pinocchio）→ 关节流下发")
        self._kin_ready = True
        return True

    def _seed_tracker(self, arm: str) -> bool:
        """用实测关节角给该臂的逆解种子打底，返回是否可用。

        关节模式必须先有种子才能逆解；拿不到实测关节角就**不接管**，
        否则只能从零位硬解，第一包会离当前位形很远（控制器会判
        ``firstFrameJumpRejected``）。
        """
        st = self.arms.get(arm)
        if st is None or st.joint_tracker is None:
            return True
        q = self.rc.joints(arm) if self.rc is not None else None
        if q is None or len(q) != 7:
            return False
        st.joint_tracker.seed_from(q)
        return True

    def _base_rpy(self, arm: str):
        """该臂 ``baseFrame`` 的 rpy（世界系），带缓存。

        用于**每臂基座标对齐**：手柄的前/上/右 对齐到该臂自己的基座标。
        ``baseFrame`` 是**静态**的（安装位姿），所以缓存一次即可，
        不必在控制热路径上反复请求。
        """
        if arm in self._base_rpy_cache:
            return self._base_rpy_cache[arm]
        try:
            bf = self.rc.base_frame(arm) or {}
        except Exception:
            bf = {}
        rpy = None
        if all(k in bf for k in ("a", "b", "c")):
            rpy = (bf["a"], bf["b"], bf["c"])
        self._base_rpy_cache[arm] = rpy
        return rpy

    def _align_base(self, arm: str):
        """本次对齐该用的基座 rpy（``None`` = 退回世界系对齐）。

        ``align_mode == "base_frame"`` 时用该臂自己的 ``baseFrame``；
        读不到（返回 ``None``）就退回世界系对齐，保证不会因为一次读取
        失败就把整条遥操链搞坏。
        """
        if self.align_mode != "base_frame":
            return None
        return self._base_rpy(arm)

    # ---------------- 奇异降级 ----------------
    def _near_singular(self, arm: str) -> bool:
        """本臂是否处于/接近运动学奇异，姿态增量应当降级。

        判据取两个互补信号（都来自推送面，不占控制热路的 TCP 往返）：

        1. **解析档位** —— ``analyticIkStage`` 不是 ``ok``（``poseClamped``
           / ``segmentBisected``）说明控制器已经在裁剪目标了。
        2. **应用位姿偏差** —— ``analyticIkPositionDeviation`` 非零意味着
           「指令的位姿」与「实际能到达的位姿」已经对不上。

        为什么姿态要先降级（而不是连平移一起停）：

        笛卡尔空间经过腕部奇异时，**位置增量始终连续**（TCP 目标本身没
        问题），但逆解为了满足姿态必须剧烈重分配关节角 —— 表现就是
        「手柄只转了一点点，末端却大幅扭转」，这就是你说的「乱走」。
        降级成纯 3-DoF 平移后，奇异点附近至少不会失控；等出了奇异区
        连续若干帧恢复 ``ok``，姿态跟随再自动回来。
        """
        st = self.rc.get_state(arm)
        if st is None:
            return False
        if st.analytic_stage and st.analytic_stage != "ok":
            return True
        return st.analytic_deviation > 1e-3

    # ---------------- 归位（home）----------------
    def _home_start(self, arm: str) -> None:
        """X/A 键**按住**：启动该臂归位。

        归位走的是**独立增量通道**（与遥操同一条影子目标），启动时会把
        该臂的遥操接管释放掉 —— 两者不能同时往同一个影子基准上加增量，
        否则影子与控制器真实基准会发散（归位只朝一个点走，遥操增量会
        把它带偏，永远到不了）。
        """
        hp = self.home_cfg.get(arm)
        if hp is None:
            print(f"  ✗ {arm} 没有准备点配置（ready_pose.json 缺 arms.{arm}）")
            return
        if hp.is_placeholder():
            print(f"  ✗ {arm} 准备点还是占位零值，先跑 "
                  f"python3 teleop/teach_ready_pose.py 示教")
            return
        if arm in self.home_state:
            return                       # 已在归位中，不重复启动
        rp = self._robot_pose(arm)
        if rp is None:
            print(f"  ✗ {arm} 拿不到当前位姿，无法启动归位")
            return
        # ⚠️ "已在准备点"的判据必须**位置与姿态都满足**。
        # 只判位置会「位置对了但朝向没转过去」—— 机械臂看起来到位了，
        # 实际上还歪着，操作者以为归位完成就松手，结果姿态永远不正。
        if math.dist(rp[0], hp.pos) < hp.arrive_tol_m:
            rot_now = 0.0
            if hp.rotate:
                rot_now = quat_angle(quat_norm(
                    quat_mul(hp.quat, quat_conj(rp[1]))))
            if rot_now < hp.arrive_tol_rad:
                print(f"  ✓ {arm} 已在准备点（位置差 "
                      f"{math.dist(rp[0], hp.pos) * 1000:.2f}mm，姿态差 "
                      f"{math.degrees(rot_now):.2f}°），无需归位")
                return
            print(f"  ⇤ {arm} 位置已到位但姿态差 "
                  f"{math.degrees(rot_now):.2f}°，继续归位")
        # 释放遥操接管，避免两条增量通道打架
        st = self.arms.get(arm)
        if st is not None and st.active:
            st.release()
        # 影子基准对齐到当前实测位姿
        if st is not None:
            st.set_shadow(rp[0], rp[1])
        # 关节模式下归位也走逆解 —— 种子用当前实测关节角
        if self.joint_mode:
            if not self._seed_tracker(arm):
                print(f"  ✗ {arm} 拿不到实测关节角，无法在关节模式下归位")
                return
        d0 = math.dist(rp[0], hp.pos)
        # ⚠️ 归位的**增量基准**是「控制器上一个被接受的目标」，不是当前实测。
        #
        # 真机有伺服滞后：实测位姿落后于目标，若拿**实测**当基准算增量，
        # 每包都会补上这段滞后量 -> 实际目标跑在准备点前面 -> 到点后继续补
        # -> 在准备点附近来回振荡，表现为「归位循环不收尾」。
        #
        # 所以这里显式记下「本轮归位自己下发的目标」，后续每包都基于它
        # 算增量，与控制器的基准严格同源。首包的目标就是当前实测
        # （刚入控时两者一致）。
        self.home_state[arm] = (d0, time.monotonic(), rp[0], rp[1])
        print(f"  ⇤ {arm} 开始归位：距准备点 {d0 * 1000:.1f}mm"
              f"（归位期间该臂遥操暂停）")

    def _home_step(self, arm: str) -> None:
        """推进一帧归位：算一包限速增量下发，抵达则收尾。

        增量语义与遥操完全一致（**平移相加、旋转左乘**），区别只在**限速**：
        按 ``max_speed_mps`` 把剩余距离摊到本帧，并保证单包不小于
        ``min_step_m``（否则最后 1~2mm 会因为整除取整成 0 而永远走不完）。

        ⚠️ 基准是**控制器上一目标**（记在 ``home_state`` 里），不是实测。
        见 :meth:`_home_start` 的说明。
        """
        hp = self.home_cfg.get(arm)
        entry = self.home_state.get(arm)
        if hp is None or entry is None:
            return
        travel, last_t, base_pos, base_quat = entry
        now = time.monotonic()
        dt = max(now - last_t, 1e-3)
        self.home_state[arm] = (travel, now, base_pos, base_quat)

        rp = self._robot_pose_cached(arm)
        if rp is None:
            return                       # 推送面没数据，本帧不动
        actual_pos, actual_quat = rp

        # 剩余距离：**基于目标基准**算（控制器就是这么累加的）。
        rem = math.dist(base_pos, hp.pos)
        # 姿态剩余角（rotate=false 时恒为 0）
        rot_rem = 0.0
        if hp.rotate:
            rot_rem = quat_angle(quat_norm(quat_mul(hp.quat, quat_conj(base_quat))))
        # ⚠️ 收尾条件必须**位置和姿态都到位**。只判位置的话，位置先到
        #   而姿态还在慢慢转（旋转限速 2°/帧，比位置慢得多）就会提前收尾，
        #   姿态永远停在半路 —— 真机上表现为"位置对了但朝向没转过去"。
        #
        #   判据用**实测**位姿（不是目标）：目标到位 ≠ 机械臂到位。
        #   真机实测滞后一帧左右，用实测判可以确保真的停稳了。
        rem_actual = math.dist(actual_pos, hp.pos)
        rot_actual = 0.0
        if hp.rotate:
            rot_actual = quat_angle(quat_norm(
                quat_mul(hp.quat, quat_conj(actual_quat))))
        if (rem_actual < hp.arrive_tol_m
                and rot_actual < hp.arrive_tol_rad
                and rem < 1e-9):
            self._home_finish(arm, "已到达准备点")
            return

        # 限速步长
        step = hp.max_speed_mps * dt
        if step < hp.min_step_m:
            step = hp.min_step_m
        if step > rem:
            step = rem                  # 最后一包直接到位，避免过冲

        k = step / rem if rem > 1e-12 else 0.0
        d_pos = tuple((hp.pos[i] - base_pos[i]) * k for i in range(3))

        # 位置已到位（基准已到准备点）但姿态未到位时，只发旋转增量
        # （位置增量必须为 0，否则会往回走一点点再纠正，抖）。
        if rem <= 1e-9:
            d_pos = (0.0, 0.0, 0.0)

        if hp.rotate:
            d_quat = quat_norm(quat_mul(hp.quat, quat_conj(base_quat)))
            ang = quat_angle(d_quat)
            if ang > hp.max_rot_per_frame:
                d_quat = quat_scale(d_quat, hp.max_rot_per_frame / ang)
        else:
            d_quat = quat_identity()

        # 下发前推进基准：下一包的基准就是本包的目标（与控制器同源）。
        new_base_pos = tuple(base_pos[i] + d_pos[i] for i in range(3))
        new_base_quat = quat_norm(quat_mul(d_quat, base_quat))

        st = self.arms.get(arm)
        if self.joint_mode:
            # 关节模式下**先逆解再推进基准**：解不出来就不动，保持基准
            # 停在原地，下一帧再试（否则基准会跑到跟不到的地方）。
            if st is None or st.joint_tracker is None:
                self.home_state[arm] = (travel, now, base_pos, base_quat)
                return
            q = st.joint_tracker.solve(new_base_pos, new_base_quat)
            if q is None:
                st.n_ik_fail += 1
                if st.n_ik_fail % 30 == 1:
                    print(f"  ! {arm} 归位逆解失败（本包丢弃，"
                          f"累计 {st.n_ik_fail} 次）")
                self.home_state[arm] = (travel, now, base_pos, base_quat)
                return
            self.rc.send_joint_target(arm, q,
                                      command_mode=self.params.command_mode)
            self.home_state[arm] = (travel, now, new_base_pos, new_base_quat)
        else:
            self.home_state[arm] = (travel, now, new_base_pos, new_base_quat)
            from robot_client import Pose
            self.rc.send_target(arm, Pose(d_pos, d_quat),
                                command_mode=1, pose_type=1)
        if st is not None:
            st.apply_delta(d_pos, d_quat)

    def _home_finish(self, arm: str, why: str) -> None:
        self.home_state.pop(arm, None)
        print(f"  ✓ {arm} {why}")

    def _home_stop(self, arm: str) -> None:
        """松开 X/A：**立即停止**归位（停在当前位置）。

        基准保持不动 —— 下次再按住时重新对齐到实测位姿。
        """
        if arm in self.home_state:
            self.home_state.pop(arm, None)
            print(f"  ⇤ {arm} 归位已停止（停在当前位置）")

    def _dist_to_home(self, arm: str):
        """当前末端到准备点的距离（m）。拿不到位姿时返回 None。"""
        hp = self.home_cfg.get(arm)
        if hp is None:
            return None
        rp = self._robot_pose_cached(arm)
        if rp is None:
            return None
        return math.dist(rp[0], hp.pos)

    # ---------------- 逐包 trace ----------------
    def _fmt3(self, v, nd: int = 4) -> str:
        """三元组格式化，带符号。``nd`` 小数位。"""
        return "(" + ", ".join(f"{v[i]:+.{nd}f}" for i in range(3)) + ")"

    def _trace_joint(self, arm: str, st: ArmTeleop, d_pos, d_quat, q) -> None:
        """打印一包下发的关节目标，以及控制器侧的对照读数。

        打三组数，缺一不可：

        1. **下发关节角**（deg）—— 这一包实际发出去的 7 个关节。
        2. **影子目标** —— 客户端累加出来的绝对目标位姿。
        3. **实测位姿** —— 推送面上报的当前 TCP 位姿。影子与实测的差
           就是"指令还没落地"的部分（伺服滞后 / 丢包）。

        第 3 组是滞后的（推送面有延迟），所以差值只作趋势参考。
        """
        rot_deg = math.degrees(quat_angle(d_quat))
        trans_mm = math.dist(d_pos, (0.0, 0.0, 0.0)) * 1000.0
        actual = self._robot_pose_cached(arm)
        if actual is None:
            gap_s = ""
        else:
            a_pos, _ = actual
            gap = math.dist(st.shadow_pos, a_pos) * 1000.0
            gap_s = f" 影子-实测={gap:+.1f}mm"
        qd = " ".join(f"{math.degrees(v):+7.2f}" for v in q)
        print(f"[{arm}#{st.n_cmd:04d}] "
              f"Δpos={trans_mm:6.2f}mm Δrot={rot_deg:6.2f}° "
              f"q(deg)=[{qd}]{gap_s}")

    # ---------------- Y / B 会话管理 ----------------
    def _enable_session(self, rc: RobotClient, arms: list[str]) -> bool:
        """Y 键：双臂上电使能 + 建立实时会话（关节流）。

        顺序（API 文档 §21.4）：
        1. ``device/setOpsMode(auto)``
        2. 若 ``faulted`` -> ``robot/clearServoError``
        3. ``robot/enable`` 双臂
        4. ``realtime/startControl`` 双臂（``cartesianIkPath: none`` = 关节流）
        5. **首包发当前实测关节角**占住首包等待窗
        """
        try:
            if rc.get_ops_mode() != "auto":
                print(f"  操作模式 {rc.get_ops_mode()} → auto")
                rc.set_ops_mode("auto")

            for arm in arms:
                st = rc.status(arm)
                if st.get("faulted"):
                    print(f"  {arm} 安全锁定中，先清错")
                    rc.clear_servo_error(arm)
                if not st.get("enabled"):
                    rc.enable(arm)
                    print(f"  {arm} 上使能")

            rc.start_state_push(arms)
            time.sleep(0.5)

            jl = self.params.joint_limit
            req_v = float(getattr(self.args, "max_vel", None)
                          or jl.get("max_velocity", 0.5))
            req_a = float(getattr(self.args, "max_acc", None)
                          or jl.get("max_acceleration", 1.0))
            for arm in arms:
                info = rc.start_control(
                    arm,
                    frequency=float(self.args.freq),
                    cartesian_ik_path=self.params.cartesian_ik_path,
                    hold_on_idle=self.params.hold_on_idle,
                    max_joint_velocity=req_v,
                    max_joint_acceleration=req_a,
                )
                print(f"  {arm} 入控成功  path={info.get('cartesianIkPath')}")
                # 控制器把请求值与安全上限取严后回显 effective*。以它为准：
                # 请求值被压低就是安全上限在起作用，必须让客户端单帧步长跟着收紧，
                # 否则超出的部分被控制器逐关节截断，末端又会走成弧线。
                eff_v = info.get("effectiveMaxVelocity")
                eff_a = info.get("effectiveMaxAcceleration")
                if eff_v:
                    v_lo = min(float(v) for v in eff_v)
                    a_lo = min(float(a) for a in eff_a) if eff_a else float("nan")
                    print(f"    生效限速 {v_lo:.2f} rad/s（请求 {req_v:.2f}），"
                          f"生效限加速度 {a_lo:.2f} rad/s²（请求 {req_a:.2f}）")
                    trk = self.arms[arm].joint_tracker if arm in self.arms else None
                    if trk is not None:
                        cap = 0.9 * v_lo / float(self.args.freq)
                        if cap < trk.max_step:
                            print(f"    单帧关节步长收紧到 {math.degrees(cap):.2f}°")
                            trk.max_step = cap
        except Exception as exc:
            print(f"  ✗ 入控失败：{exc}")
            return False

        # ---- 首包：连发实测关节角占住等待窗 ----
        for arm in arms:
            q = rc.joints(arm)
            if q is None or len(q) != 7:
                print(f"  ✗ {arm} 拿不到实测关节角，无法起步")
                return False
            st = self.arms.get(arm)
            if st is not None and st.joint_tracker is not None:
                st.joint_tracker.seed_from(q)
            for _ in range(10):
                rc.send_joint_target(arm, q, command_mode=self.params.command_mode)
                time.sleep(0.02)
        print("  首包已发（实测关节角占窗）")
        return True

    def _disable_session(self, rc: RobotClient, arms: list[str]) -> None:
        """B 键：双臂停控 + 下电失能。**程序不退出**，Y 可再次上电。"""
        for arm in arms:
            try:
                rc.stop_control(arm)
            except Exception as exc:
                log.warning("停控 %s 失败: %s", arm, exc)
            try:
                rc.disable(arm)
            except Exception as exc:
                log.warning("断使能 %s 失败: %s", arm, exc)
        # 复位本地状态
        self.session_active = False
        self.teleop_on = False
        self.home_state.clear()
        for st in self.arms.values():
            st.release()
            if st.joint_tracker is not None:
                st.joint_tracker.reset()
        try:
            st = rc.status(arms[0])
        except Exception as exc:
            log.warning("读状态失败: %s", exc)
            return
        print(f"  已停控并断使能：channel={st.get('channel')} "
              f"enabled={st.get('enabled')} faulted={st.get('faulted')}")
        print("  按 Y 可再次上电使能。")

    # ---------------- 主循环 ----------------
    def run(self) -> int:
        args = self.args
        arms = list(args.arm)

        with RobotClient(args.host,
                         port=getattr(args, "control_port", None),
                         command_port=getattr(args, "command_port", None),
                         config=getattr(args, "robot_cfg", None)) as rc:
            self.rc = rc

            # ⚠️ 预检：清掉上一轮遗留的 realtime 会话。
            #    控制器上残留的会话（尤其 state="stopped" 的）会占住该臂，
            #    导致 startControl 失败或包被旧会话吞掉 —— 表现为"发了指令
            #    但机械臂不动"，且不报任何错。
            try:
                stale = [s.get("robot") for s in rc.sessions() if s.get("robot")]
            except Exception:
                stale = []
            for robot in stale:
                if robot in arms:
                    try:
                        rc.stop_control(robot)
                        print(f"  清残留会话: {robot}")
                    except Exception as exc:
                        log.warning("清残留会话 %s 失败: %s", robot, exc)

            # 关节模式：建 pinocchio 模型 + 同步 baseFrame。必须**在入控前**
            # 完成 —— 入控返回的起点位姿、以及首包是否合法，都依赖它。
            if not self._setup_kinematics(rc):
                print("  ✗ 运动学初始化失败，退出")
                return 2

            # ---- 加载准备点配置（归位功能）----
            hp_path = Path(args.ready_pose)
            try:
                self.home_cfg, self.home_only_when_idle = load_home_poses(hp_path)
                for arm in arms:
                    hp = self.home_cfg.get(arm)
                    if hp is None:
                        print(f"  · {arm} 无准备点配置（X/A 键对此臂无效）")
                    elif hp.is_placeholder():
                        print(f"  · {arm} 准备点为占位零值"
                              f"（需先跑 teach_ready_pose.py 示教）")
                    else:
                        print(f"  · {arm} 准备点 pos={self._fmt3(hp.pos)}")
            except Exception as exc:
                print(f"  ⚠ 读 {hp_path} 失败：{exc}（归位功能不可用）")

            # ---- VR 采集 ----
            self.reader = XrtReader(
                args.config, args.handedness, rate_hz=args.vr_rate)
            self.reader.start()
            print(f"  VR 采集启动（{args.vr_rate:.0f} Hz）")

            # ---- 灵巧手：扳机 -> 开合（独立连接 + 独立线程，与臂控互不阻塞）----
            self._start_hands(rc, arms)

            # ---- AGV / 升降柱：只起线程，不连接、不使能（按键触发）----
            self._start_peripherals(rc)

            print("\n" + "=" * 78)
            print(" 遥操待命（尚未上电）")
            print("=" * 78)
            print(" 【上电】左手柄【Y 键按下】= 双臂上电使能 + 建立实时会话")
            print(" 【下电】右手柄【B 键按下】= 双臂停控 + 下电失能")
            print(" 【遥操】左手柄【左握持拉满】= 控左臂；右手柄【右握持拉满】= 控右臂")
            print("        （松开握持 = 该臂停止下发，机器人停在最后目标）")
            print(" 【归位】按住左手柄【X 键】= 左臂缓慢回准备点（松手即停）")
            print("        按住右手柄【A 键】= 右臂缓慢回准备点（松手即停）")
            print(" 【灵巧手】左手柄【扳机】= 左手开合；右手柄【扳机】= 右手开合")
            print("          （与臂的使能 / 握持无关，随时可用）")
            print(" 【AGV】左手柄【菜单键】= 连接 / 断开（启动时不连）；")
            print("       连上后直接拨动左摇杆 = 前后 / 转向，回中即停（连上时摇杆须在中位）")
            print(" 【升降柱】右手柄【摇杆按下】= 使能 / 下电（启动时不使能）；")
            print("       使能后直接上下拨动右摇杆 = 点动，回中即停（使能时摇杆须在中位）")
            print("       （AGV / 升降柱与双臂的 Y 上电、B 下电互相独立）")
            print(" 退出：Ctrl-C")
            print("=" * 78)
            print(" 绑定：" + "   ".join(
                f"{HAND_OF_ARM[arm]}手柄→{arm}" for arm in arms))

            t0 = time.monotonic()
            period = 1.0 / args.freq
            next_tick = t0
            n_sent = 0
            # 归位按住状态：arm -> bool（用于检测"松手"）
            home_holding: dict[str, bool] = {arm: False for arm in arms}

            try:
                while not self.stop_flag.is_set():
                    now = time.monotonic()
                    if now < next_tick:
                        time.sleep(min(next_tick - now, 0.002))
                        continue
                    next_tick += period
                    if next_tick < time.monotonic() - period:
                        next_tick = time.monotonic() + period

                    snap, _events = self.reader.get_latest()

                    # ---- B 键：停控 + 断使能（边沿锁存）----
                    # ⚠️ 用**锁存**而不是瞬时电平：本循环 90 Hz、采集 200 Hz，
                    # 一次快速轻点（B 是急停，绝不能漏）可能整个按压过程都落在
                    # 两次轮询之间，电平查询会彻底漏掉。
                    if self.reader.consume_latched(KEY_TELEOP_OFF):
                        print("\n■ 停控 + 下电失能")
                        self._disable_session(rc, arms)
                        print("  按 Y 可再次上电。")
                        # 不退出 —— Y 键可以重新上电

                    # ---- Y 键：上电使能 + 建立实时会话（边沿锁存）----
                    if self.reader.consume_latched(KEY_TELEOP_ON):
                        if self.session_active:
                            print("  · 已在控中，忽略")
                        else:
                            print("\n▶ 上电使能 + 建立实时会话")
                            if self._enable_session(rc, arms):
                                self.session_active = True
                                self.teleop_on = True
                                print("  ✓ 实时会话已建立，握持即可遥操")

                    # ---- 归位：X / A 键**按住**（电平）----
                    # ⚠️ 这里用**电平查询**而不是锁存：语义是"按住多久就
                    # 归位多久"，用锁存会在松手后继续动作。
                    for arm in arms:
                        key = KEY_HOME.get(arm)
                        if key is None:
                            continue
                        holding = self.reader.query(key)
                        if holding and not home_holding[arm]:
                            # 刚按下 -> 启动归位
                            if self.session_active:
                                self._home_start(arm)
                            else:
                                print(f"  ✗ {arm} 未上电，无法归位")
                                home_holding[arm] = True
                                continue
                        elif not holding and home_holding[arm]:
                            # 刚松开 -> 立即停止归位
                            self._home_stop(arm)
                        home_holding[arm] = holding

                    # ---- 归位推进（互斥：归位中的臂跳过遥操）----
                    for arm in arms:
                        if arm in self.home_state:
                            self._home_step(arm)

                    # ---- AGV / 升降柱：与臂的上电、握持完全无关 ----
                    # 左菜单键 / 右摇杆按下走边沿锁存（不容漏）；两根摇杆的拨动值直接喂给各自 worker。
                    # 这里只做廉价的非阻塞喂值，真正的连接和发包在各自线程里。
                    self._poll_peripherals(snap)

                    # ---- 数据新鲜度 ----
                    if not self.reader.is_data_fresh():
                        continue

                    # ---- 灵巧手：只喂扳机值，真正的下发在各自线程里 ----
                    # 与臂的使能 / 握持无关：手走独立的 RS485 通道，
                    # 扳机随时可以开合。
                    for hand, worker in self.hand_workers.items():
                        worker.feed(self.reader.query_value(KEY_HAND_TRIGGER[hand]))

                    # ---- 每臂：接管判定 + 增量下发 ----
                    for arm in arms:
                        hand = HAND_OF_ARM[arm]
                        st = self.arms[arm]
                        # 未上电 / 未入控 -> 全部不接管
                        if not self.session_active or not self.teleop_on:
                            if st.active:
                                st.release()
                                print(f"  ■ 释放 {arm}")
                            st.prev_hand = None
                            continue
                        # 归位中 -> 该臂冻结（互斥）
                        if arm in self.home_state:
                            if st.active:
                                st.release()
                            st.prev_hand = None
                            continue
                        want = self.reader.query(ACTIVE_KEY_OF_HAND[hand])
                        fr = getattr(snap, hand)

                        if want and not st.active:
                            rp = self._robot_pose(arm)
                            if rp is None:
                                log.warning("%s 拿不到末端朝向，暂不接管", arm)
                                continue
                            if self.joint_mode and not self._seed_tracker(arm):
                                log.warning("%s 拿不到实测关节角，暂不接管", arm)
                                continue
                            st.engage(fr, rp[1], self._align_base(arm),
                                      self.rot_mode)
                            st.set_shadow(rp[0], rp[1])
                            print(f"  ▶ 接管 {arm}（对齐已建立）")

                        elif not want and st.active:
                            # 松开握持 -> 停止下发（会话保留，机器人停在最后目标）
                            st.release()
                            print(f"  ■ 释放 {arm}（停止下发，停在最后目标）")

                        elif not want:
                            st.prev_hand = None
                            continue

                        if not st.active:
                            continue

                        res = st.step(fr, dt=period)
                        if res is None:
                            continue
                        d_pos, d_quat = res

                        # 限幅（倍率单独作用于位移）。
                        # ⚠️ 位置死区已在 :class:`teleop_map.AnchorDeadzone`
                        #    里做完（EMA + 锚点滞回 + 速度闸门）——
                        #    这里**不能再套**逐帧增量死区，否则慢速运动
                        #    会被二次吃掉（90Hz 下阈值速度高达 315mm/s）。
                        if d_pos != (0.0, 0.0, 0.0):
                            d_pos = self.deadband.clamp_pos(
                                tuple(v * self.speed for v in d_pos))

                        # ---- 奇异降级：姿态优先让路 ----
                        if self._near_singular(arm):
                            st.singular_hits += 1
                            if st.singular_hits == 5:
                                print(f"  ⚠ {arm} 接近奇异，姿态跟随自动降级为 3-DoF 平移")
                        elif st.singular_hits > 0:
                            st.singular_hits -= 1
                            if st.singular_hits == 0:
                                print(f"  ✓ {arm} 已离开奇异区，姿态跟随恢复")

                        if st.singular_hits >= 5:
                            d_quat = quat_identity()
                        else:
                            # ⚠️ 这里**只做限幅**，不再套逐帧姿态死区。
                            d_quat = self.deadband.clamp_rot(d_quat)

                        if d_pos == (0.0, 0.0, 0.0) and d_quat == quat_identity():
                            continue

                        # 不设累计行程上限：目标是否可达由 IK 的关节限位判定
                        # （解不出来就丢包，见下），不需要再叠一层人为护栏。

                        # ---- 关节模式：影子位姿 -> 逆解 -> 关节流 ----
                        # 先「试算」目标位姿（不推进影子），逆解成功才
                        # 真正下发并推进影子。逆解失败就丢弃这一包 ——
                        # 影子推进了但包没发出去，影子就与控制器基准
                        # 发散，之后每帧都在追一个从未下发的目标。
                        tgt_pos, tgt_quat = st.compose(d_pos, d_quat,
                                                       use_pivot=True)
                        q = st.joint_tracker.solve(tgt_pos, tgt_quat)
                        if q is None:
                            st.n_ik_fail += 1
                            if st.n_ik_fail % 30 == 1:
                                print(f"  ! {arm} 逆解失败（本包丢弃，"
                                      f"累计 {st.n_ik_fail} 次）")
                            continue
                        rc.send_joint_target(arm, q,
                                             command_mode=self.params.command_mode)
                        n_sent += 1
                        st.n_sent += 1
                        st.apply_delta(d_pos, d_quat, use_pivot=True)
                        if args.trace:
                            self._trace_joint(arm, st, d_pos, d_quat, q)

            except KeyboardInterrupt:
                print("\n收到 Ctrl-C，退出")
            finally:
                self._shutdown(rc, arms)

            print(f"\n共下发 {n_sent} 个关节包")
            for arm in arms:
                st = self.arms[arm]
                print(f"  {arm}: 下发 {st.n_sent} 包，逆解失败 {st.n_ik_fail} 次")
        return 0

    # ---------------- 灵巧手 ----------------
    def _start_hands(self, rc: RobotClient, arms: list[str]) -> None:
        """为每条被遥操臂对应的那只手起一个后台线程。

        初始化失败只停用那只手，绝不影响臂控。
        """
        if getattr(self.args, "no_hand", False):
            print("  · 灵巧手已关闭（--no-hand）")
            return
        try:
            hp = self.params.hand
        except ValueError as exc:
            print(f"  ✗ teleop_params.json 的 hand 段不合法，灵巧手停用：{exc}")
            return
        if not hp.enabled:
            print("  · 灵巧手已关闭（hand.enabled=false）")
            return
        for arm in arms:
            side = HAND_OF_ARM[arm]
            link = rc.new_control_link()
            self._hand_links.append(link)
            hand = Revo2Hand(link, side, hp.slaves[side], hp.modbus_addr[side],
                             baud_rate=hp.baud_rate)
            worker = HandWorker(hand, hp)
            self.hand_workers[side] = worker
            worker.start()
        print(f"  灵巧手线程已启动：{'、'.join(self.hand_workers)}"
              f"（扳机={hp.mode}，最快约 {1000.0 / max(hp.min_interval_ms, 1):.0f} 次/秒）")

    def _stop_hands(self) -> None:
        for worker in self.hand_workers.values():
            worker.stop()
        for link in self._hand_links:
            link.close()
        self.hand_workers.clear()
        self._hand_links.clear()

    # ---------------- AGV / 升降柱 ----------------
    def _start_peripherals(self, rc: RobotClient) -> None:
        """起 AGV / 升降柱的后台线程。**只起线程，不连接、不使能**：
        连接和使能都要操作者按键触发。
        """
        try:
            ap = self.params.agv
        except ValueError as exc:
            print(f"  ✗ teleop_params.json 的 agv 段不合法，AGV 停用：{exc}")
            ap = None
        if ap is not None and ap.enabled and not getattr(self.args, "no_agv", False):
            self.agv_worker = AgvWorker(ap)
            self.agv_worker.start()
            print(f"  AGV 开环控制待命（{ap.ip}:{ap.port}，"
                  f"线速度≤{ap.max_linear_mps:g} m/s，角速度≤{ap.max_angular_radps:g} rad/s）")
        else:
            print("  · AGV 开环控制已关闭")

        try:
            lp = self.params.lift
        except ValueError as exc:
            print(f"  ✗ teleop_params.json 的 lift 段不合法，升降柱停用：{exc}")
            lp = None
        if lp is not None and lp.enabled and not getattr(self.args, "no_lift", False):
            self._lift_link = rc.new_control_link()        # 独立连接：enable 最长 5 s
            self.lift_worker = LiftWorker(self._lift_link, lp)
            self.lift_worker.start()
            print("  升降柱点动待命（右摇杆按下使能）")
        else:
            print("  · 升降柱点动已关闭")

    def _poll_peripherals(self, snap) -> None:
        """主循环每拍调用：消费按键边沿、喂摇杆值。廉价、不阻塞。"""
        if self.reader is None or not self.reader.is_data_fresh():
            return              # 数据过期时不喂：让两个 worker 的 stale 保护生效 => 视为松手
        if self.agv_worker is not None:
            if self.reader.consume_latched(KEY_AGV_TOGGLE):
                self.agv_worker.toggle()
            x, y = snap.left.axis
            self.agv_worker.feed(x, y)
        else:
            self.reader.consume_latched(KEY_AGV_TOGGLE)    # 关闭时也取走，别积压
        if self.lift_worker is not None:
            if self.reader.consume_latched(KEY_LIFT_TOGGLE):
                self.lift_worker.toggle()
            _x, y = snap.right.axis
            self.lift_worker.feed(y)
        else:
            self.reader.consume_latched(KEY_LIFT_TOGGLE)

    def _stop_peripherals(self) -> None:
        """退出收尾：AGV 发停止并断开；升降柱 jogStop（保持使能，靠抱闸）。"""
        if self.agv_worker is not None:
            self.agv_worker.stop()
            self.agv_worker = None
        if self.lift_worker is not None:
            self.lift_worker.stop()
            self.lift_worker = None
        if self._lift_link is not None:
            self._lift_link.close()
            self._lift_link = None

    # ---------------- 收尾 ----------------
    def _shutdown(self, rc: RobotClient, arms: list[str]) -> None:
        self._stop_peripherals()
        self._stop_hands()
        print("\n  收尾：停控 …")
        for arm in arms:
            try:
                rc.stop_control(arm)
                st = rc.status(arm)
                print(f"  {arm} 停控 → channel={st.get('channel')} "
                      f"faulted={st.get('faulted')}")
            except Exception as exc:
                log.warning("停控 %s 失败: %s", arm, exc)
        if self.reader is not None:
            self.reader.stop()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="PICO 双臂增量式遥操主循环（客户端逆解 + 关节流下发）")
    ap.add_argument("--robot-config",
                    default=str(HERE / "robot_config.json"),
                    help="控制器连接配置 JSON（IP / 端口）")
    ap.add_argument("--host", default=None,
                    help="控制器 IP（缺省取 robot_config.json 里的值）")
    ap.add_argument("--control-port", type=int, default=None,
                    help="控制面 TCP 端口（缺省取 robot_config.json）")
    ap.add_argument("--command-port", type=int, default=None,
                    help="指令面 UDP 端口（缺省取 robot_config.json）")
    ap.add_argument("--params",
                    default=str(HERE / "teleop_params.json"),
                    help="遥操参数 JSON（发包频率 / IK 容差 / 死区 / 限幅 …）")
    ap.add_argument("--arm", nargs="+", default=None,
                    help="要遥操的臂（LeftArm / RightArm），可多选")
    ap.add_argument("--config",
                    default=str(HERE / "button_map.json"),
                    help="按键映射 JSON")
    ap.add_argument("--ready-pose",
                    default=str(HERE / "ready_pose.json"),
                    help="准备点配置 JSON（X/A 键归位用）。用 "
                         "teach_ready_pose.py 示教生成")
    ap.add_argument("--freq", type=float, default=0.0,
                    help="指令下发频率 Hz，范围 10~1000（缺省取 "
                         "teleop_params.json 的 stream.rate_hz）。"
                         "同时作为会话 expectedFrequency")
    ap.add_argument("--max-vel", type=float, default=None,
                    help="关节限速 rad/s（缺省取 stream.joint_limit.max_velocity）。"
                         "控制器会与安全上限取严")
    ap.add_argument("--max-acc", type=float, default=None,
                    help="关节限加速度 rad/s²（缺省取 "
                         "stream.joint_limit.max_acceleration）")
    ap.add_argument("--vr-rate", type=float, default=200.0,
                    help="VR 采集频率 Hz")
    ap.add_argument("--handedness", choices=["RH_Z_back", "LH_Z_forward"],
                    default="RH_Z_back")
    ap.add_argument("--trace", action="store_true",
                    help="逐包打印下发的关节目标")
    ap.add_argument("--warmup-ms", type=float, default=150.0,
                    help="起跑缓冲时长（ms）：接管后这段时间内只更新"
                         "增量基准、不下发，把握持瞬间的姿态噪声吸收掉")
    ap.add_argument("--no-hand", action="store_true",
                    help="不启用灵巧手（扳机不控制手）")
    ap.add_argument("--no-agv", action="store_true",
                    help="不启用 AGV 开环控制（菜单键 / 左摇杆无效）")
    ap.add_argument("--no-lift", action="store_true",
                    help="不启用升降柱点动（右摇杆无效）")
    ap.add_argument("--urdf-left", default=None,
                    help="左臂 URDF 路径。缺省取项目内的 urdf_dualarm/left_arm.urdf")
    ap.add_argument("--urdf-right", default=None,
                    help="右臂 URDF 路径。缺省取项目内的 urdf_dualarm/right_arm.urdf")
    ap.add_argument("--no-sync-base", action="store_true",
                    help="不读控制器 baseFrame，用 URDF 自带的装法。"
                         "⚠️ 真机几乎总是要读 —— URDF 的 world 与控制器"
                         "的世界系不是同一个系")
    args = ap.parse_args()

    # 控制器连接配置：--robot-config 指定的文件 → --host 覆盖 → 环境变量
    cfg = load_config(args.robot_config)
    args.robot_cfg = cfg
    if args.host is None:
        args.host = cfg.host
    if args.arm is None:
        args.arm = list(cfg.arms)

    # 频率 / 限速 / 限加速度：命令行 > teleop_params.json > 内置缺省。
    # 这里一次解析完，后面全程只读 args，避免 JSON 与命令行各读各的。
    params = load_params(Path(args.params))
    jl = params.joint_limit
    if args.freq <= 0:
        args.freq = params.rate_hz
    if not 10.0 <= args.freq <= 1000.0:
        ap.error(f"下发频率须在 10~1000 Hz 内（收到 {args.freq:g}），"
                 f"这是控制器 expectedFrequency 的取值域")
    # 关节限速/限加速度：命令行 > teleop_params.json
    jl_cfg = params.joint_limit
    args.max_vel = (args.max_vel if args.max_vel is not None
                    else float(jl_cfg.get("max_velocity", 0.5)))
    args.max_acc = (args.max_acc if args.max_acc is not None
                    else float(jl_cfg.get("max_acceleration", 1.0)))
    if args.max_vel <= 0 or args.max_acc <= 0:
        ap.error("--max-vel / --max-acc 必须 > 0")
    if args.max_vel <= 0:
        args.max_vel = float(jl.get("max_velocity", 1.5))
    if args.max_acc <= 0:
        args.max_acc = float(jl.get("max_acceleration", 5.0))
    # 控制器 expectedFrequency 取值域 [10, 1000] Hz（API 24119）。
    # 它必须等于实际发包频率：控制器按它折算插值周期，不一致就会欠/过插值。
    if not 10.0 <= args.freq <= 1000.0:
        ap.error(f"下发频率 {args.freq:g} Hz 超出控制器允许范围 [10, 1000]")
    args.expected_freq = args.freq
    if args.vr_rate < args.freq:
        print(f"  ⚠ VR 采集 {args.vr_rate:g} Hz 低于下发 {args.freq:g} Hz："
              f"多出的帧是重复位姿，不会更跟手。可加 --vr-rate 提高采集频率")

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(message)s",
                        datefmt="%H:%M:%S")
    print("=" * 78)
    print(f" PICO 双臂增量遥操   {args.host}   臂={args.arm}   "
          f"指令={args.freq:.0f}Hz")
    print("=" * 78)
    print(" ⚠️  会产生真实机械臂运动。请确认周围无人、无障碍物。")
    print("     Ctrl-C 退出。")
    print("=" * 78)

    runner = TeleopRunner(args)
    signal.signal(signal.SIGINT, lambda *_: runner.stop_flag.set())
    return runner.run()


if __name__ == "__main__":
    raise SystemExit(main())
