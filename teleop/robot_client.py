"""
teleop/robot_client.py
======================

机器人控制器通信层（工程方案 §八 的 Thread 3）。

实测环境（2026-10-22）
--------------------
* 控制器 IP / 端口见 ``robot_config.json``（实测环境是 ``192.168.100.16``，
  控制面 TCP 9001，指令面 UDP 9030）—— **不再硬编码在代码里**，
  换控制器只改那个 JSON。
* 双臂 ``LeftArm`` / ``RightArm``，``type=HmRobot7DoF``，``subType=NDA-06-L/R``
* 能力位：``analyticIk=true`` / ``numericalIk=false``
  → 末端遥操必须走 ``cartesianIkPath="analytic"``；``numerical`` 入控会被直接拒绝
* 夹爪 ``type=Generic485``、``maxPosition=0`` → **不可控**（文档：Generic485 型号直接 err）
* 工具目录只有 ``flange``（零偏置）→ TCP 即法兰

三条数据通路（互相独立，别混）
-----------------------------
============  ======  ==============================  ==========================
面            协议    方向                            用途
============  ======  ==============================  ==========================
控制面        TCP     双向 请求—响应 ``{id,ty,db}``   入控 / 停控 / 使能 / 查询
指令面        UDP     客户端 → 控制器 9030            发送末端目标（**不回包**）
推送面        UDP     控制器 → 客户端 注册的 ip:port   观测实测位姿与解析档位
============  ======  ==============================  ==========================

坐标系警告（读之前务必看完）
--------------------------
⚠️ 2026-10-23：本段旧警告**已与实测不符**，保留原文仅作对照。

原文：``analytic`` 通路的末端位姿在**该臂基座系**（与 ``robot/status.cartesian``
同系同点）。**不要**把它当成世界系，也不要和 ``tcpPose`` 直接互相代入。
双臂的 ``baseFrame`` 不同（左臂 ``(0, 0.2, 0)``、右臂 ``(0, -0.2, 0)``，
旋转也不同），所以每臂要独立标定 —— 见 ``teleop_map.py``。

实测结论（FK + 位移实验，本机 ``HmRobot7DoF`` 自建模板）::

    startControl.pose == status.cartesian == push.tcpPose == FK_world
    三者互相重合，误差 0.00 mm

    下发「+20mm 沿 X」-> status 位移 (+20.11, +0.05, +0.10) mm
                        -> push   位移 (+20.00, -0.00, -0.00) mm
    即**下发方向与世界系 1:1 同向**，可当作世界系用。

世界系轴向（2026-10-27 更正，**未在真机复测**）：Z 竖直向上、X 指向机器人
前方、Y 指向机器人左侧，右手系；依据与疑点见 ``teleop_map.WORLD_UP`` 处的
说明。``baseFrame`` 只用于**读参考**，不参与换算。

绝对位姿累加器在客户端
---------------------
``commandMode=0``（绝对）的累加器是**客户端自己的**。控制器裁剪过位姿后，
下一包仍按你发来的绝对值解释。于是"自行积分绝对位姿的客户端须在
``analyticIkPositionDeviation`` 非 0 或档位不是 ``ok`` 时重锚"，否则顶到工作
空间边界后反向会先走一段空行程。

本模块据此实现了 :meth:`RobotClient.auto_reanchor`：监测解析观测，一旦发生
裁剪/保持就把累加器重置到当前实测位姿。
"""

from __future__ import annotations

import json
import logging
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_config import RobotConfig, load_config  # noqa: E402

log = logging.getLogger(__name__)


# ===========================================================================
# 数据结构
# ===========================================================================
@dataclass(slots=True)
class Pose:
    """位姿。位置米，姿态四元数 (qx,qy,qz,qw)。

    注意 UDP 报文里四元数是 **qw 在前**，见 :meth:`to_udp_value`。
    """

    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # qx,qy,qz,qw

    def to_udp_value(self) -> list[float]:
        """转成末端空间四元数报文：``[x,y,z, qw,qx,qy,qz]``。"""
        qx, qy, qz, qw = self.quat
        return [self.pos[0], self.pos[1], self.pos[2], qw, qx, qy, qz]

    @staticmethod
    def from_udp_value(v: list[float]) -> "Pose":
        """从 ``[x,y,z, qw,qx,qy,qz]`` 还原。"""
        return Pose((v[0], v[1], v[2]), (v[4], v[5], v[6], v[3]))


@dataclass(slots=True)
class RobotState:
    """从推送面收到的单臂状态。"""

    robot: str = ""
    timestamp_ms: float = 0.0
    sequence: int = 0
    joints: list[float] = field(default_factory=list)
    tcp_pose: Optional[dict] = None          # 基座系 + 工具（世界系，见文档）
    motion_mode: str = ""
    command_channel: str = ""
    enabled: bool = False
    faulted: bool = False
    safety_response: int = 0
    # realtimeStatus
    state: str = ""
    error: str = ""
    received_count: int = 0
    published_count: int = 0
    dropped_count: int = 0
    queue_depth: int = 0
    # analytic 解析观测
    analytic_stage: str = ""                # ok | poseClamped | segmentBisected | held
    analytic_deviation: float = 0.0         # 应用位姿 vs 指令位姿的位置偏差 (m)
    analytic_arm_angle: Optional[float] = None

    @property
    def is_healthy(self) -> bool:
        return (self.state == "active" and self.error == "none"
                and not self.faulted and self.enabled)


class RobotError(RuntimeError):
    """控制器返回了 ``err``。"""

    def __init__(self, ty: str, err: dict):
        self.ty = ty
        self.err = err
        super().__init__(f"{ty} 失败: [{err.get('code')}] {err.get('message')}")


# ===========================================================================
# 控制面（TCP 9001，请求—响应）
# ===========================================================================
class _ControlLink:
    """控制面 TCP 连接。持久连接 + 失败重连。

    控制器按 JSON 边界回包，但**不保证一次 recv 就是一个完整帧**，
    所以这里累积缓冲后反复尝试解析。
    """

    def __init__(self, host: str, port: int = 9001, timeout: float = 8.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._buf = b""
        self._req_id = 0
        self._lock = threading.Lock()
        # 是否曾经成功连上过。close() 用它避免对"从未连接"的控制器
        # 发起 stopStatePush —— 否则未连接就 close 会卡住两个超时周期。
        self._ever_connected = False

    def _connect(self) -> socket.socket:
        s = socket.create_connection((self.host, self.port), timeout=self.timeout)
        s.settimeout(self.timeout)
        self._ever_connected = True
        return s

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None
            self._buf = b""

    @property
    def ever_connected(self) -> bool:
        """是否曾经成功建立过 TCP 连接。"""
        return self._ever_connected

    def call(self, ty: str, db: dict | None = None) -> Any:
        """发一个请求并等响应。返回 ``db``；有 ``err`` 则抛 :class:`RobotError`。"""
        with self._lock:
            self._req_id += 1
            req_id = self._req_id
            payload = json.dumps({"id": req_id, "ty": ty, "db": db or {}}).encode()

            for attempt in (0, 1):
                try:
                    if self._sock is None:
                        self._sock = self._connect()
                    self._sock.sendall(payload)
                    resp = self._recv_frame()
                    break
                except (OSError, socket.timeout, ValueError) as exc:
                    # 连接可能已被控制器单方面关掉，重连一次再试
                    log.warning("控制面 %s 第%d次失败(%s)，重连重试", ty, attempt, exc)
                    self.close()
                    if attempt == 1:
                        raise
            else:  # pragma: no cover - for 循环必break或raise
                raise RobotError(ty, {"code": -1, "message": "控制面无响应"})

        err = resp.get("err")
        if err:
            raise RobotError(ty, err)
        return resp.get("db")

    def _recv_frame(self) -> dict:
        """读到能解析成一个 JSON 对象为止。"""
        deadline = time.monotonic() + self.timeout
        while True:
            # 先尝试从缓冲里解出完整帧
            obj = self._try_parse()
            if obj is not None:
                return obj
            if time.monotonic() > deadline:
                raise socket.timeout("控制面响应超时")
            chunk = self._sock.recv(65536)  # type: ignore[union-attr]
            if not chunk:
                raise OSError("控制器关闭了连接")
            self._buf += chunk

    def _try_parse(self) -> Optional[dict]:
        """尝试从缓冲解出一个 JSON 对象；解不出返回 None（保留缓冲）。"""
        if not self._buf:
            return None
        text = self._buf.decode("utf-8", errors="ignore").lstrip()
        depth = 0
        in_str = False
        esc = False
        end = -1
        for i, ch in enumerate(text):
            if in_str:
                if esc:
                    esc = False
                elif ch == "\\":
                    esc = True
                elif ch == '"':
                    in_str = False
                continue
            if ch == '"':
                in_str = True
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        if end < 0:
            return None
        try:
            obj = json.loads(text[:end])
        except json.JSONDecodeError:
            return None
        # 丢掉已消费的部分
        consumed = len(self._buf) - len(text) + end
        self._buf = self._buf[consumed:]
        return obj


# ===========================================================================
# 推送面（UDP 接收）
# ===========================================================================
class _StateReceiver:
    """在后台线程里收推送面 UDP 报文，维护每臂最新状态。"""

    def __init__(self, port: int = 0):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.5)
        self.port = self._sock.getsockname()[1]
        self._states: dict[str, RobotState] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._on_update: Optional[Callable[[RobotState], None]] = None

    def start(self, on_update: Optional[Callable[[RobotState], None]] = None) -> None:
        self._on_update = on_update
        self._thread = threading.Thread(target=self._loop, name="robot-push", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self._sock.close()
        except OSError:
            pass

    def get(self, robot: str) -> Optional[RobotState]:
        with self._lock:
            return self._states.get(robot)

    def all(self) -> dict[str, RobotState]:
        with self._lock:
            return dict(self._states)

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = self._sock.recvfrom(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8", errors="ignore"))
            except json.JSONDecodeError:
                continue
            st = self._parse(msg)
            if st is None:
                continue
            with self._lock:
                self._states[st.robot] = st
            if self._on_update is not None:
                try:
                    self._on_update(st)
                except Exception:
                    log.exception("推送回调异常")

    @staticmethod
    def _parse(msg: dict) -> Optional[RobotState]:
        robot = msg.get("robot")
        if not robot:
            return None
        st = RobotState(
            robot=robot,
            timestamp_ms=float(msg.get("timestamp", 0.0)),
            sequence=int(msg.get("sequence", 0)),
        )
        if "jointActualPosition" in msg:
            st.joints = list(msg["jointActualPosition"])
        if "tcpPose" in msg:
            st.tcp_pose = msg["tcpPose"]
        rs = msg.get("robotState") or {}
        st.motion_mode = rs.get("motionMode", "")
        st.command_channel = rs.get("commandChannel", "")
        st.enabled = bool(rs.get("enabled", False))
        st.faulted = bool(rs.get("faulted", False))
        st.safety_response = rs.get("safetyResponse", 0)
        rt = msg.get("realtimeStatus") or {}
        st.state = rt.get("state", "")
        st.error = rt.get("error", "")
        st.received_count = int(rt.get("receivedCount", 0))
        st.published_count = int(rt.get("publishedCount", 0))
        st.dropped_count = int(rt.get("droppedCount", 0))
        st.queue_depth = int(rt.get("queueDepth", 0))
        st.analytic_stage = rt.get("analyticIkStage", "")
        st.analytic_deviation = float(rt.get("analyticIkPositionDeviation", 0.0))
        if "analyticIkArmAngle" in rt:
            st.analytic_arm_angle = rt["analyticIkArmAngle"]
        return st


# ===========================================================================
# 主客户端
# ===========================================================================
class RobotClient:
    """机器人控制器客户端：入控 / 发末端目标 / 收状态。

    连接参数默认从 ``robot_config.json`` 读（见 :mod:`robot_config`），
    也可以显式传参覆盖。

    典型用法::

        with RobotClient() as rc:                 # 用配置文件里的 IP
            rc.prepare(arms=["LeftArm", "RightArm"])
            for arm, pose in targets.items():
                rc.send_target(arm, pose)

        with RobotClient("10.0.0.5") as rc:        # 临时换 IP
            ...
    """

    def __init__(self, host: str | None = None, port: int | None = None,
                 command_port: int | None = None,
                 push_period_ms: int | None = None,
                 push_ip: str | None = None,
                 config: RobotConfig | None = None):
        cfg = config or load_config()
        self.config = cfg
        self.host = host or cfg.host
        self.control_port = int(port if port is not None else cfg.control_port)
        self.command_port = int(command_port if command_port is not None
                                else cfg.command_port)
        self.push_period_ms = int(push_period_ms if push_period_ms is not None
                                  else cfg.push_period_ms)
        # 推送面目标 IP。默认按"到控制器的源地址"自动推断 ——
        # 控制器在另一台机器上，写 127.0.0.1 会让它把状态推给它自己，
        # 本机永远收不到，表现为"发了包但没有任何实测反馈"。
        self._push_ip = push_ip if push_ip is not None else cfg.push_ip
        self._link = _ControlLink(self.host, self.control_port)
        self._recv = _StateReceiver()
        self._cmd_sock: Optional[socket.socket] = None
        self._active_arms: list[str] = []
        self._reanchor_cb: Optional[Callable[[str], Optional[Pose]]] = None

    @classmethod
    def from_config(cls, **kwargs) -> "RobotClient":
        """从配置文件建客户端（等价于不传参数）。"""
        return cls(config=load_config(), **kwargs)

    def push_ip(self) -> str:
        """注册推送面时用的本机地址。"""
        if self._push_ip:
            return self._push_ip
        # 用一条临时 UDP 连接让内核选出到控制器的源地址（不真正发数据）
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((self.host, self.command_port))
            self._push_ip = probe.getsockname()[0]
        except OSError:
            self._push_ip = "127.0.0.1"
        finally:
            probe.close()
        return self._push_ip

    # ---------------- 生命周期 ----------------
    def __enter__(self) -> "RobotClient":
        self._recv.start(self._on_push)
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def close(self) -> None:
        for arm in list(self._active_arms):
            try:
                self.stop_control(arm)
            except Exception:
                log.warning("停控 %s 失败", arm, exc_info=True)
        self._active_arms.clear()
        # 只在**曾经连上过**时才对控制器发 stopStatePush。
        # 否则"没连上就 close"会白等两个超时周期（默认 16s）。
        if self._link.ever_connected:
            try:
                self._link.call("realtime/stopStatePush", {"ip": "", "port": 0})
            except Exception:
                pass
        self._recv.stop()
        self._link.close()
        if self._cmd_sock is not None:
            try:
                self._cmd_sock.close()
            except OSError:
                pass
            self._cmd_sock = None

    # ---------------- 控制面 ----------------
    def call(self, ty: str, db: dict | None = None) -> Any:
        """直接调一个控制面动作（调试用）。"""
        return self._link.call(ty, db)

    def new_control_link(self) -> "_ControlLink":
        """另开一条控制面 TCP 连接（调用方负责 ``close()``）。

        给慢速、阻塞型的外设用（如灵巧手的 ``rs485`` 透传，一次写读两次往返）。
        ``_ControlLink.call`` 持锁，若与臂控共用一条连接，外设一次调用就会把
        状态查询、归位等臂控请求全部堵住。
        """
        return _ControlLink(self.host, self.control_port)

    def list_robots(self) -> list[dict]:
        return self._link.call("robot/list", {}) or []

    def get_state(self, arm: str) -> Optional[RobotState]:
        return self._recv.get(arm)

    def get_ops_mode(self) -> str:
        db = self._link.call("device/getOpsMode", {}) or {}
        return db.get("mode", "")

    def set_ops_mode(self, mode: str) -> None:
        self._link.call("device/setOpsMode", {"mode": mode})

    def enable(self, arm: str) -> bool:
        return bool(self._link.call("robot/enable", {"name": arm}))

    def disable(self, arm: str) -> bool:
        return bool(self._link.call("robot/disable", {"name": arm}))

    def clear_servo_error(self, arm: str) -> bool:
        return bool(self._link.call("robot/clearServoError", {"name": arm}))

    def status(self, arm: str) -> dict:
        return self._link.call("robot/status", {"name": arm}) or {}

    def base_frame(self, arm: str) -> dict:
        """该臂安装基坐标系在世界系下的位姿（``{x,y,z,a,b,c}``）。"""
        db = self._link.call("robot/getKinematicGeometry", {"name": arm}) or {}
        return db.get("baseFrame") or {}

    # ---------------- 入控 / 停控 ----------------
    def start_control(self, arm: str, *, frequency: float = 90.0,
                      analytic: bool = True,
                      cartesian_ik_path: str | None = None,
                      hold_on_idle: bool = True,
                      max_joint_velocity: float = 0.5,
                      max_joint_acceleration: float = 1.0) -> dict:
        """入控。返回会话描述（含 ``pose``：入控瞬间该臂末端位姿，**同系**）。

        ``cartesian_ik_path`` —— **末端逆解通路**，本项目的主用入口：

        * ``"none"``（**本方案默认**）—— 只接受**关节包**
          （``controlSpace: 0``）。客户端用 :mod:`arm_kinematics` 自己逆解，
          控制器只吃关节角，不做任何末端逆解。
        * ``"analytic"`` —— 控制器自己走解析臂型角解，本会话额外接受
          末端包（``controlSpace: 1``）。真机自建模板要用这个。
        * ``"numerical"`` —— 机内数值解，仅部分机型模板 + x86 产物可用。

        ``analytic=True`` 是向后兼容的快捷方式（等价于
        ``cartesian_ik_path="analytic"``）；两者同时给时以
        ``cartesian_ik_path`` 为准。

        ⚠️ **不要混发关节包与末端包**（§21.9）：空间切换会让「上一目标」
        基准失效，增量语义全乱。本方案全程只发关节包。

        ``jointLimit`` 显式打开 —— 缺省是**关**的（``enable`` 缺省 false）。

        ``hold_on_idle=True``（本模块缺省，与控制器缺省一致）：停发多久都不掉线，
        机器人停在最后一个目标上。**分阶段测试必须用它** —— 置 ``False`` 时
        停发超过保持窗即自动停控（``stoppedByStreamLoss``），阶段之间的打印与
        等待就会把会话掐掉，后续包全被丢弃，表现为"发了包但机械臂没动"。
        真正跑实时遥操、要求"停发即回收"时才置 ``False``，并自行看门狗。
        """
        cfg: dict[str, Any] = {
            "robot": arm,
            "expectedFrequency": float(frequency),
            "unitType": "si",
            "timeMode": "none",
            "interpolator": "linear",
            # 控制器缺省即 true。这里显式写出，因为分阶段测试一旦置 false，
            # 阶段间的空等就会触发 stoppedByStreamLoss。
            "holdOnIdle": bool(hold_on_idle),
            "jointLimit": {
                "enable": True,
                "maxVelocity": float(max_joint_velocity),
                "maxAcceleration": float(max_joint_acceleration),
            },
        }
        path = cartesian_ik_path if cartesian_ik_path is not None else (
            "analytic" if analytic else "none")
        if path:
            cfg["cartesianIkPath"] = path
        db = self._link.call("realtime/startControl", {"config": cfg}) or {}
        if arm not in self._active_arms:
            self._active_arms.append(arm)
        return db

    def stop_control(self, arm: str) -> bool:
        ok = bool(self._link.call("realtime/stopControl", {"robot": arm}))
        if arm in self._active_arms:
            self._active_arms.remove(arm)
        return ok

    def sessions(self) -> list[dict]:
        db = self._link.call("realtime/getStatus", {"robot": None}) or {}
        return db.get("sessions") or []

    # ---------------- 推送面 ----------------
    def start_state_push(self, arms: list[str], *, period_ms: int | None = None,
                         fields: list[str] | None = None) -> dict:
        """注册推送目标，收本机 UDP 报文。"""
        ip = self.push_ip()
        target = {
            "robot": arms if len(arms) > 1 else arms[0],
            "ip": ip,
            "port": self._recv.port,
            "period": int(period_ms or self.push_period_ms),
            "unitType": "si",
            "fields": fields or ["jointActualPosition", "tcpPose",
                                 "robotState", "realtimeStatus"],
        }
        log.info("注册推送面: %s:%d <- %s", ip, self._recv.port, arms)
        return self._link.call("realtime/startStatePush", {"target": target}) or {}

    # ---------------- 指令面（UDP） ----------------
    def _ensure_cmd_sock(self) -> socket.socket:
        if self._cmd_sock is None:
            self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return self._cmd_sock

    def send_target(self, arm: str, pose: Pose, *,
                    command_mode: int = 0, pose_type: int = 1) -> None:
        """发一包末端目标到指令面。

        ``command_mode``：0=绝对（客户端自己累加）、1=相对上一目标、2=相对当前实测。
        ``pose_type``：1=四元数、0=欧拉 ZYX。
        """
        msg: dict[str, Any] = {
            "robot": arm,
            "controlSpace": 1,
            "commandMode": int(command_mode),
            "poseType": int(pose_type),
            "value": pose.to_udp_value(),
        }
        data = json.dumps(msg).encode()
        self._ensure_cmd_sock().sendto(data, (self.host, self.command_port))

    def send_joint_target(self, arm: str, joints,
                          *, command_mode: int = 0) -> None:
        """发一包**关节目标**到指令面（``controlSpace: 0``）。

        这条通路不经过控制器逆解 —— 客户端用 :mod:`arm_kinematics`
        （pinocchio）自己把末端位姿解成关节角再按关节下发。

        ``command_mode``：``0``=绝对关节角（缺省，推荐）、
        ``1``=相对上一目标、``2``=相对当前实测。遥操按关节下发时一律用
        **绝对**：目标关节由客户端积分出来的影子位姿逆解而来，本身就是
        绝对值；用相对模式会让控制器再叠一层自己的基准，反而对不上。

        ⚠️ **第一包**会过控制器接管核验：关节包若远离入控时的起始位形
        会被拒绝并自动停控（``firstFrameJumpRejected``）。所以第一包务必
        贴近当前实测关节角（本方法调用方负责从实测关节起步）。

        ``joints`` 长度必须等于该臂关节数（7），单位由会话 ``unitType``
        决定（本项目一律 ``si`` ⇒ **rad**）。

        文档依据：API 说明 §21.9 指令面 UDP 报文 —— ``controlSpace: 0``
        时 ``value`` 就是该臂各关节角。
        """
        vals = [float(v) for v in joints]
        msg: dict[str, Any] = {
            "robot": arm,
            "controlSpace": 0,
            "commandMode": int(command_mode),
            "value": vals,
        }
        data = json.dumps(msg).encode()
        self._ensure_cmd_sock().sendto(data, (self.host, self.command_port))

    def joints(self, arm: str):
        """推送面里的最新实测关节角（rad）。拿不到返回 ``None``。"""
        st = self.get_state(arm)
        if st is None or not st.joints:
            return None
        return list(st.joints)

    # ---------------- 解析观测 -> 重锚 ----------------
    def set_reanchor_callback(self, cb: Callable[[str], Optional[Pose]]) -> None:
        """注册"给我这一臂当前实测位姿"的回调。

        控制器裁剪过位姿后，客户端的绝对位姿累加器必须清掉并重锚到实测，
        否则反向时先走完顶边界期间积下的空行程才开始动。
        """
        self._reanchor_cb = cb

    def _on_push(self, st: RobotState) -> None:
        if self._reanchor_cb is None:
            return
        # 档位不是 ok（poseClamped/segmentBisected/held）或偏差非零 → 已被裁剪
        if st.analytic_stage and st.analytic_stage != "ok":
            log.warning("%s 解析档位 %s（偏差 %.4f m）——需重锚累加器",
                        st.robot, st.analytic_stage, st.analytic_deviation)
        elif st.analytic_deviation > 1e-4:
            log.warning("%s 应用位姿偏差 %.4f m ——需重锚累加器",
                        st.robot, st.analytic_deviation)

    # ---------------- 一站式准备 ----------------
    def prepare(self, arms: list[str], *, frequency: float = 90.0) -> dict[str, dict]:
        """切自动模式 → 上使能 → 注册推送 → 入控。返回每臂的入控响应。"""
        if self.get_ops_mode() != "auto":
            log.info("操作模式 %s → auto", self.get_ops_mode())
            self.set_ops_mode("auto")

        self.start_state_push(arms)

        out: dict[str, dict] = {}
        for arm in arms:
            st = self.status(arm)
            if st.get("faulted"):
                log.warning("%s 处于安全锁定，先清错", arm)
                self.clear_servo_error(arm)
            if not st.get("enabled"):
                self.enable(arm)
            out[arm] = self.start_control(arm, frequency=frequency)
            pose = out[arm].get("pose") or {}
            log.info("%s 入控成功 起点位姿=%s", arm, json.dumps(pose, ensure_ascii=False))
        return out

    # ---------------- 关节流会话（客户端自己逆解）----------------
    def prepare_joint_stream(self, arms: list[str], *,
                             frequency: float = 90.0,
                             cartesian_ik_path: str = "none",
                             hold_on_idle: bool = True,
                             max_joint_velocity: float = 0.5,
                             max_joint_acceleration: float = 1.0,
                             seed_packets: int = 10,
                             seed_interval: float = 0.02) -> dict[str, dict]:
        """入控 + **占住首包等待窗**（关节流专用）。

        流程（严格按 API 文档 §21.4 / §21.9）：

        1. 切自动模式；
        2. 注册状态推送（否则 ``jointActualPosition`` 拿不到）；
        3. 逐臂：清安全锁定 -> 上使能 -> ``realtime/startControl``；
        4. **立刻连发当前实测关节角**（绝对，``commandMode: 0``）。

        第 4 步是关键：入控后控制器有一个**首包等待窗**（缺省 2000 ms），
        窗满仍没发过包就按断流停控（``stoppedByStreamLoss``），之后所有包
        都被丢弃。而关节包的第一包还要过一次**接管核验** —— 若远离入控
        时的起始位形会被拒绝并自动停控（``firstFrameJumpRejected``）。

        所以第一包必须是**当前实测关节角**。这里连发 ``seed_packets`` 包
        （间隔 ``seed_interval``）把窗口稳稳占住。

        返回 ``{arm: 入控响应}``。任何一臂拿不到实测关节角都会抛
        ``RuntimeError`` —— 宁可起不来，也不要发一个会被控制器拒掉的首包。
        """
        if self.get_ops_mode() != "auto":
            log.info("操作模式 %s → auto", self.get_ops_mode())
            self.set_ops_mode("auto")

        self.start_state_push(arms)
        # 推送面需要一点时间把第一帧推过来
        time.sleep(0.5)

        out: dict[str, dict] = {}
        for arm in arms:
            st = self.status(arm)
            if st.get("faulted"):
                log.warning("%s 处于安全锁定，先清错", arm)
                self.clear_servo_error(arm)
            if not st.get("enabled"):
                self.enable(arm)
                log.info("%s 上使能", arm)
            out[arm] = self.start_control(
                arm, frequency=frequency,
                cartesian_ik_path=cartesian_ik_path,
                hold_on_idle=hold_on_idle,
                max_joint_velocity=max_joint_velocity,
                max_joint_acceleration=max_joint_acceleration,
            )

        # ---- 占住首包等待窗：连发实测关节角 ----
        for arm in arms:
            q = self.joints(arm)
            if q is None or len(q) != 7:
                raise RuntimeError(
                    f"{arm} 拿不到实测关节角（推送面未就绪？）—— "
                    f"关节流无法起步，放弃入控")
            for _ in range(max(1, seed_packets)):
                self.send_joint_target(arm, q, command_mode=0)
                time.sleep(seed_interval)
            log.info("%s 首包已发（实测关节角占窗）", arm)
        return out
