"""Revo2 灵巧手：扳机 -> 开合。

Modbus 帧与手的协议移植自 ``RobotControls/communication/revo2_hand.py``，
本模块自包含，不依赖那个工程。控制器侧走 API 文档 §33 的 ``rs485/*``
（2026-10-28 起取代 ``ec2rs485/*``，按 ``channel`` 操作，``gripper/setType`` 已删除）：

    rs485/list          按从站名（EtherCAT485L / EtherCAT485R）找到通道标识 channel
    rs485/init          115200 8N1（通道被厂商夹爪占用时返回 23252）
    rs485/write         发 Modbus RTU 帧（0x10 写 6 个自由度）
    rs485/read          读 8 字节应答并校验

6 个自由度（线上归一化 0~1000）::

    0 拇指弯曲   1 拇指侧摆   2 食指   3 中指   4 无名指   5 小指

参考实现里的两个预设就是本模块的两端：``一把抓`` = 扳机拉满，``释放`` = 扳机松开。

⚠️ 速率：一次完整的"写+读"是两次控制面往返，还要等通信板发送、等手应答，
远慢于 90 Hz 的臂控循环。旧版文档曾给出"每帧不少于 110 ms"的下限，新版文档
不再写这个数，实际能多快要上机实测（``min_interval_ms`` 可调）。所以灵巧手
必须放在独立线程里（:class:`HandWorker`），**不能**在臂控循环里同步调用。

⚠️ 独立连接：``_ControlLink`` 内部有锁，一次调用会占着连接。灵巧手用
``RobotClient.new_control_link()`` 另开一条 TCP 连接，不与臂控的控制面共用。
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Optional

from robot_client import RobotError

log = logging.getLogger("teleop.hand")

DOF_COUNT = 6
WRITE_TIME_MS_MIN = 1
WRITE_TIME_MS_MAX = 2000

REG_WRITE_ALL = 0x03F2           # 1010：6 × (position, time_ms)

# 参考实现的预设（一把抓 / 释放）
POSE_GRASP = [1000, 0, 1000, 1000, 1000, 1000]
POSE_RELEASE = [0, 0, 0, 0, 0, 0]

ERR_BUSY = 20003                 # 目标资源正忙（可重试）
ERR_OCCUPIED = 23252             # 通道正被厂商夹爪协议占用
ERR_NOT_CONFIGURED = 23254       # 串口未配置：需要重新 rs485/init
ERR_IO = 23258                   # 串口 I/O 失败：文档要求错误关闭后重新 init
ERR_NEED_REINIT = (ERR_NOT_CONFIGURED, ERR_IO)


class HandError(RuntimeError):
    """灵巧手通信 / 配置失败。"""


# ===========================================================================
# Modbus RTU（与参考实现一致）
# ===========================================================================
def crc16(data: bytes) -> int:
    """Modbus RTU CRC16（多项式 0xA001）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def build_frame(slave: int, fc: int, payload: list[int]) -> list[int]:
    body = [slave & 0xFF, fc & 0xFF] + [b & 0xFF for b in payload]
    c = crc16(bytes(body))
    return body + [c & 0xFF, (c >> 8) & 0xFF]


def verify_crc(frame: list[int]) -> bool:
    if len(frame) < 4:
        return False
    return crc16(bytes(frame[:-2])) == ((frame[-1] << 8) | frame[-2])


def write_all_dofs_frame(slave: int, positions: list[int], time_ms: int) -> list[int]:
    """0x10 写 6 个自由度：起始 0x03F2，12 个寄存器 = 6 × (位置, 时间)。"""
    if len(positions) != DOF_COUNT:
        raise ValueError(f"positions 长度必须为 {DOF_COUNT}")
    payload = [(REG_WRITE_ALL >> 8) & 0xFF, REG_WRITE_ALL & 0xFF,
               0x00, 0x0C,      # 寄存器数 = 12
               0x18]            # 字节数 = 24
    for p in positions:
        payload += [(p >> 8) & 0xFF, p & 0xFF,
                    (time_ms >> 8) & 0xFF, time_ms & 0xFF]
    return build_frame(slave, 0x10, payload)


def check_write_ack(rx: list[int]) -> None:
    """校验 0x10 的应答，异常则抛 :class:`HandError`。"""
    if not rx:
        raise HandError("无应答")
    if not verify_crc(rx):
        raise HandError("应答 CRC 错误")
    if len(rx) >= 3 and (rx[1] & 0xFF) == 0x90:
        raise HandError(f"从站异常码 0x{rx[2] & 0xFF:02X}")
    if (rx[1] & 0xFF) != 0x10:
        raise HandError(f"非预期功能码 0x{rx[1] & 0xFF:02X}")


# ===========================================================================
# 参数
# ===========================================================================
@dataclass
class HandParams:
    """``teleop_params.json`` 的 ``hand`` 段。"""
    enabled: bool = True
    mode: str = "proportional"            # proportional | binary
    open_pose: list[int] = field(default_factory=lambda: list(POSE_RELEASE))
    close_pose: list[int] = field(default_factory=lambda: list(POSE_GRASP))
    trigger_low: float = 0.05             # 低于它 = 完全张开（死区）
    trigger_high: float = 0.95            # 高于它 = 完全合拢
    binary_on: float = 0.6                # binary 模式：超过它合拢
    binary_off: float = 0.45              # binary 模式：跌破它张开（迟滞）
    write_time_ms: int = 120              # 每帧下发给手的"到位时间"
    min_interval_ms: float = 120.0        # 两次下发的最小间隔
    min_change: int = 20                  # 变化小于它（0~1000）不下发
    baud_rate: int = 115200
    slaves: dict[str, str] = field(default_factory=lambda: {
        "left": "EtherCAT485L", "right": "EtherCAT485R"})
    modbus_addr: dict[str, int] = field(default_factory=lambda: {
        "left": 126, "right": 127})

    @classmethod
    def from_dict(cls, raw: dict | None) -> "HandParams":
        p = cls()
        raw = raw or {}
        for key in ("enabled", "mode", "trigger_low", "trigger_high",
                    "binary_on", "binary_off", "write_time_ms",
                    "min_interval_ms", "min_change", "baud_rate"):
            if key in raw:
                setattr(p, key, raw[key])
        for key in ("open_pose", "close_pose"):
            if key in raw:
                setattr(p, key, [int(v) for v in raw[key]])
        if "slaves" in raw:
            p.slaves = {**p.slaves, **raw["slaves"]}
        if "modbus_addr" in raw:
            p.modbus_addr = {**p.modbus_addr,
                             **{k: int(v) for k, v in raw["modbus_addr"].items()}}
        p.validate()
        return p

    def validate(self) -> None:
        if self.mode not in ("proportional", "binary"):
            raise ValueError(f"hand.mode 只能是 proportional / binary，收到 {self.mode!r}")
        for name in ("open_pose", "close_pose"):
            pose = getattr(self, name)
            if len(pose) != DOF_COUNT or any(not 0 <= v <= 1000 for v in pose):
                raise ValueError(f"hand.{name} 须为 {DOF_COUNT} 个 0~1000 的整数")
        if not 0.0 <= self.trigger_low < self.trigger_high <= 1.0:
            raise ValueError("hand: 须满足 0 <= trigger_low < trigger_high <= 1")
        if not 0.0 <= self.binary_off < self.binary_on <= 1.0:
            raise ValueError("hand: 须满足 0 <= binary_off < binary_on <= 1")
        if not WRITE_TIME_MS_MIN <= int(self.write_time_ms) <= WRITE_TIME_MS_MAX:
            raise ValueError(f"hand.write_time_ms 须在 "
                             f"{WRITE_TIME_MS_MIN}~{WRITE_TIME_MS_MAX} ms 内")
        if self.min_interval_ms < 0 or self.min_change < 0:
            raise ValueError("hand.min_interval_ms / min_change 不能为负")


# ===========================================================================
# 单只手
# ===========================================================================
class Revo2Hand:
    """一只 Revo2 灵巧手（经控制面 ``rs485`` 透传）。线程不安全：由一个线程独占。"""

    def __init__(self, link: Any, side: str, slave: str, addr: int,
                 baud_rate: int = 115200):
        self._link = link
        self.side = side
        self.slave = slave            # 从站名，如 EtherCAT485L（用来在 rs485/list 里找通道）
        self.channel = ""             # rs485 通道标识，init() 时从 rs485/list 取
        self.addr = addr
        self.baud_rate = baud_rate

    def _find_channel(self) -> str:
        """在 ``rs485/list`` 里按从站名找双臂 EC2485 通道。

        文档要求通道标识原样取自 list、不自行拼接（里面含主站名）。
        """
        chans = self._link.call("rs485/list", {}) or []
        for c in chans:
            if not isinstance(c, dict) or c.get("backend") != "dualEc2485":
                continue
            if (c.get("backendStatus") or {}).get("slave") != self.slave:
                continue
            if not c.get("supported", True):
                raise HandError(f"通道 {c.get('channel')} 当前平台不支持")
            if c.get("occupiedBy"):
                raise HandError(self._occupied_msg(c.get("channel", "")))
            return str(c["channel"])
        names = [(c.get("channel"), (c.get("backendStatus") or {}).get("slave"))
                 for c in chans if isinstance(c, dict)]
        raise HandError(f"rs485/list 里没有从站 {self.slave} 对应的 dualEc2485 通道，"
                        f"现有通道：{names}。请核对 teleop_params.json 的 hand.slaves")

    def _occupied_msg(self, channel: str) -> str:
        return (f"通道 {channel} 正被厂商夹爪协议占用（23252），无法透传。"
                f"该手的 gripper 设备型号需为 Generic485，或先用 "
                f"peripheral/setDeviceEnabled 停用该通道的夹爪协议"
                f"（停用需要现场确认夹爪静止，本程序不会替你确认）")

    # 左右手的 rs485/init 必须串行。文档 §33 总则写"不同通道可独立进行"，但上机实测
    # 两只手同时 init，必有一只返回 20003（文档 §33.6 / 升降柱章也把 rs485/init 归为
    # "从站参数读写请求"，同类请求互斥）。以实测为准：进程内加锁，再对 20003 兜底重试。
    _init_lock = threading.Lock()
    INIT_BUSY_RETRIES = 10
    INIT_BUSY_WAIT_S = 0.15

    def init(self) -> None:
        """找通道 -> 初始化串口。失败抛 :class:`HandError`。"""
        try:
            self.channel = self._find_channel()
            self._init_channel()
        except RobotError as exc:
            if exc.err.get("code") == ERR_OCCUPIED:
                raise HandError(self._occupied_msg(self.channel)) from exc
            raise HandError(str(exc)) from exc

    def _init_channel(self) -> None:
        """串行 + 20003 重试。其它错误（占用、未就绪等）重试没有意义，原样抛出。"""
        params = {"channel": self.channel, "baudRate": int(self.baud_rate),
                  "stopBits": 1, "parity": 0}
        with self._init_lock:
            for attempt in range(self.INIT_BUSY_RETRIES + 1):
                try:
                    self._link.call("rs485/init", params)
                    return
                except RobotError as exc:
                    if exc.err.get("code") != ERR_BUSY or attempt == self.INIT_BUSY_RETRIES:
                        raise
                    time.sleep(self.INIT_BUSY_WAIT_S)

    def reinit_channel(self) -> None:
        try:
            self._init_channel()
        except RobotError as exc:
            raise HandError(str(exc)) from exc

    def write_all(self, positions: list[int], time_ms: int) -> None:
        """写 6 个自由度并校验应答。失败抛 :class:`HandError` / :class:`RobotError`。"""
        pos = [max(0, min(1000, int(p))) for p in positions]
        frame = write_all_dofs_frame(self.addr, pos, int(time_ms))
        self._link.call("rs485/write", {"channel": self.channel, "data": frame})
        rx = self._link.call("rs485/read", {
            "channel": self.channel, "length": 8, "timeout": 400})
        check_write_ack(list(rx) if isinstance(rx, list) else [])


# ===========================================================================
# 后台线程：扳机 -> 手
# ===========================================================================
class HandWorker:
    """一只手一个线程。主循环 :meth:`feed` 扳机值（廉价、非阻塞），
    线程按最小间隔把最新目标下发给手。

    只保留**最新**目标：下发慢于扳机变化时，中间值直接丢弃，不排队，
    所以手永远追的是扳机当前的位置而不是一串过期指令。
    """

    def __init__(self, hand: Revo2Hand, params: HandParams):
        self.hand = hand
        self.p = params
        self._trigger: Optional[float] = None
        self._binary_closed = False
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_sent: Optional[list[int]] = None
        self._last_send_t = 0.0
        self._last_err = ""
        self.ready = threading.Event()
        self.n_sent = 0
        self.n_err = 0

    # ---------- 主循环侧 ----------
    def feed(self, trigger: float) -> None:
        self._trigger = trigger

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name=f"hand-{self.hand.side}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # ---------- 映射 ----------
    def ratio(self, trigger: float) -> float:
        """扳机 0~1 -> 合拢比例 0~1。"""
        t = max(0.0, min(1.0, trigger))
        if self.p.mode == "binary":
            if self._binary_closed:
                self._binary_closed = t >= self.p.binary_off
            else:
                self._binary_closed = t >= self.p.binary_on
            return 1.0 if self._binary_closed else 0.0
        lo, hi = self.p.trigger_low, self.p.trigger_high
        return max(0.0, min(1.0, (t - lo) / (hi - lo)))

    def pose(self, ratio: float) -> list[int]:
        o, c = self.p.open_pose, self.p.close_pose
        return [int(round(a + ratio * (b - a))) for a, b in zip(o, c)]

    # ---------- 线程 ----------
    def _run(self) -> None:
        side = self.hand.side
        try:
            self.hand.init()
        except Exception as exc:                       # 初始化失败：本只手停用，不影响臂控
            print(f"  ✗ 灵巧手 {side} 初始化失败，本手不可用：{exc}")
            return
        print(f"  ✓ 灵巧手 {side} 就绪（{self.hand.slave}，地址 {self.hand.addr}）")
        self.ready.set()

        min_gap = self.p.min_interval_ms / 1000.0
        while not self._stop.wait(0.02):
            trig = self._trigger
            if trig is None:
                continue
            ratio = self.ratio(trig)
            target = self.pose(ratio)
            if not self._should_send(target, ratio):
                continue
            if time.monotonic() - self._last_send_t < min_gap:
                continue
            self._send(target)

    def _should_send(self, target: list[int], ratio: float) -> bool:
        if self._last_sent is None:
            return True
        diff = max(abs(a - b) for a, b in zip(target, self._last_sent))
        if diff >= self.p.min_change:
            return True
        # 到达端点（全开 / 全合）时，哪怕差值不足 min_change 也要补发，
        # 否则手会停在离端点差一点的位置。
        return diff > 0 and ratio in (0.0, 1.0)

    def _send(self, target: list[int]) -> None:
        self._last_send_t = time.monotonic()
        try:
            self.hand.write_all(target, int(self.p.write_time_ms))
        except Exception as exc:
            self.n_err += 1
            msg = str(exc)
            if msg != self._last_err:                  # 同一错误只提示一次，避免刷屏
                print(f"  ! 灵巧手 {self.hand.side} 下发失败：{msg}")
                self._last_err = msg
            code = exc.err.get("code") if isinstance(exc, RobotError) else None
            if code in ERR_NEED_REINIT:
                try:
                    self.hand.reinit_channel()
                except Exception as exc2:
                    log.warning("灵巧手 %s 重新 init 失败: %s", self.hand.side, exc2)
            return
        self._last_err = ""
        self._last_sent = list(target)
        self.n_sent += 1
