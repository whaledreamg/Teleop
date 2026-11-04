"""
teleop/agv_client.py
====================

AGV 开环运动客户端（Seer 协议，控制口 19205），供 PICO 左手柄使用。

* 左手柄【菜单键】按一下 = 连接 AGV，再按一下 = 断开（不自动连接）。
* 连接后直接拨动左手柄摇杆 = 开环运动（无需按压）；摇杆回中立即停车。

移植自 RobotControl/communication/agv_tcp_client.py 的 ``AGVControlClient``，
只保留开环运动需要的部分，并改掉三处不适合模拟摇杆的地方：

1. 参考实现的 ``start_continuous_move`` 每次调用都清零速度并另起线程，
   模拟量每拍都在变，会叠线程。这里只有**一个**发送线程，按固定节拍发最新速度。
2. 参考实现的 ``send_no_wait`` 在 ``_pending_requests`` 里留条目且永不清理，
   长时间运行会无限增长；并且不看应答，AGV 拒绝指令时毫无提示。
   这里不登记待应答表，发完顺手读应答，``ret_code != 0`` 就提示。
3. 指令 ``duration`` 只有 200 ms：程序崩溃 / 断线 / 主循环卡死时 AGV 自己停下，
   不依赖我们还能发出"停止"。

安全门：连接成功后必须先看到一次"摇杆回中（在死区内）"才允许运动，
避免按菜单键连接的瞬间摇杆还拨着，AGV 一连上就窜出去。
"""

from __future__ import annotations

import json
import logging
import select
import socket
import struct
import threading
import time
from dataclasses import dataclass, fields
from typing import Optional

log = logging.getLogger("teleop.agv")

SYNC_HEADER = 0x5A
PROTOCOL_VERSION = 0x01
HEADER_FMT = "!BBHIH6s"                  # sync ver seq dlen cmd reserved
HEADER_SIZE = 16
MAX_PAYLOAD = 1024 * 1024

CMD_TEACH_STOP = 2000                    # 停止开环运动
CMD_TEACH_MOVE = 2010                    # 开环运动 {vx, vy, w, duration}


def build_frame(cmd: int, body: Optional[dict], seq: int) -> bytes:
    raw = json.dumps(body).encode("utf-8") if body is not None else b""
    head = struct.pack(HEADER_FMT, SYNC_HEADER, PROTOCOL_VERSION,
                       seq & 0xFFFF, len(raw), cmd & 0xFFFF, b"\x00" * 6)
    return head + raw


@dataclass
class AgvParams:
    """``teleop_params.json`` 的 ``agv`` 段。"""
    enabled: bool = True
    ip: str = "192.168.100.101"
    port: int = 19205
    max_linear_mps: float = 0.2          # 拨杆推满的前进 / 后退速度
    max_angular_radps: float = 0.3       # 拨杆推满的转向角速度
    stick_deadzone: float = 0.15         # 拨杆死区（0~1），之外重新归一
    forward_sign: int = 1                # 拨杆向上 => 前进；方向反了改 -1
    turn_sign: int = 1                   # 拨杆向右 => 顺时针(w<0)；方向反了改 -1
    send_hz: float = 10.0                # 开环指令重发频率
    duration_ms: int = 200               # 指令有效期；断线 / 卡死时 AGV 自行停下
    connect_timeout_s: float = 3.0
    stale_s: float = 0.3                 # 超过它没收到摇杆数据 => 视为松手

    @classmethod
    def from_dict(cls, raw: dict | None) -> "AgvParams":
        p = cls()
        names = {f.name for f in fields(cls)}
        for key, val in (raw or {}).items():
            if key in names:
                setattr(p, key, val)
        p.validate()
        return p

    def validate(self) -> None:
        if not 0.0 < self.max_linear_mps <= 1.0:
            raise ValueError("agv.max_linear_mps 须在 (0, 1] m/s 内")
        if not 0.0 < self.max_angular_radps <= 1.0:
            raise ValueError("agv.max_angular_radps 须在 (0, 1] rad/s 内")
        if not 0.0 <= self.stick_deadzone < 1.0:
            raise ValueError("agv.stick_deadzone 须在 [0, 1) 内")
        if self.forward_sign not in (1, -1) or self.turn_sign not in (1, -1):
            raise ValueError("agv.forward_sign / turn_sign 只能是 1 或 -1")
        if not 1.0 <= self.send_hz <= 50.0:
            raise ValueError("agv.send_hz 须在 1~50 内")
        if self.duration_ms <= 1000.0 / self.send_hz:
            raise ValueError("agv.duration_ms 必须大于一个发送周期"
                             f"（{1000.0 / self.send_hz:.0f} ms），"
                             "否则指令会在两次发送之间过期，AGV 一抖一抖")


def shape_axis(v: float, dz: float) -> float:
    """死区 + 重新归一：|v|<=dz => 0，之外线性映射回 0~1，保留符号。"""
    a = abs(v)
    if a <= dz:
        return 0.0
    return (1.0 if v > 0 else -1.0) * min(1.0, (a - dz) / (1.0 - dz))


class AgvLink:
    """AGV 控制口的一条 TCP 连接。发送 + 非阻塞读应答。"""

    def __init__(self, ip: str, port: int, timeout: float):
        self.ip, self.port, self.timeout = ip, port, timeout
        self._sock: Optional[socket.socket] = None
        self._seq = 0
        self._buf = b""

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        s = socket.create_connection((self.ip, self.port), timeout=self.timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(self.timeout)
        self._sock = s
        self._buf = b""

    def close(self) -> None:
        s, self._sock = self._sock, None
        if s is not None:
            try:
                s.close()
            except OSError:
                pass

    def send(self, cmd: int, body: Optional[dict]) -> None:
        if self._sock is None:
            raise OSError("AGV 未连接")
        self._seq = (self._seq + 1) & 0xFFFF
        self._sock.sendall(build_frame(cmd, body, self._seq))

    def drain_replies(self) -> list[dict]:
        """读走已到达的应答（不阻塞），返回解析出的 JSON 列表。"""
        out: list[dict] = []
        s = self._sock
        if s is None:
            return out
        while True:
            ready, _, _ = select.select([s], [], [], 0)
            if not ready:
                break
            chunk = s.recv(4096)
            if not chunk:
                raise OSError("AGV 关闭了连接")
            self._buf += chunk
        while len(self._buf) >= HEADER_SIZE:
            if self._buf[0] != SYNC_HEADER:
                nxt = self._buf.find(bytes([SYNC_HEADER]), 1)
                self._buf = b"" if nxt < 0 else self._buf[nxt:]
                continue
            _, _, _, dlen, _, _ = struct.unpack(HEADER_FMT, self._buf[:HEADER_SIZE])
            if dlen > MAX_PAYLOAD:
                self._buf = self._buf[1:]
                continue
            if len(self._buf) < HEADER_SIZE + dlen:
                break
            body = self._buf[HEADER_SIZE:HEADER_SIZE + dlen]
            self._buf = self._buf[HEADER_SIZE + dlen:]
            try:
                obj = json.loads(body.decode("utf-8", "replace")) if body else {}
            except ValueError:
                continue
            if isinstance(obj, dict):
                out.append(obj)
        return out


class AgvWorker:
    """AGV 开环控制：连接 / 断开由按键触发，运动由摇杆驱动。

    主循环（90 Hz）只调 :meth:`toggle`、:meth:`feed`，都不阻塞；
    连接、发包、读应答全在后台线程。
    """

    def __init__(self, params: AgvParams):
        self.p = params
        self._lock = threading.Lock()
        self._want_connected = False       # 操作者想要的状态
        self._busy = False                 # 连接 / 断开进行中
        self._stick: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._armed = False                # 连接后见过"摇杆回中"才允许动
        self._warned_unarmed = False
        self._link: Optional[AgvLink] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_err = ""
        self._was_moving = False
        self.n_sent = 0

    # ---------- 主循环侧 ----------
    @property
    def connected(self) -> bool:
        return self._link is not None and self._link.connected

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="agv", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def toggle(self) -> None:
        """菜单键：连接 <-> 断开。连接 / 断开进行中忽略重复按键。"""
        with self._lock:
            if self._busy:
                print("  · AGV 正在连接 / 断开，忽略这次按键")
                return
            self._want_connected = not self._want_connected
            want = self._want_connected
        print(f"\n{'▶ AGV 开环控制：连接中 …' if want else '■ AGV 开环控制：断开'}")

    def feed(self, x: float, y: float) -> None:
        """摇杆 (x右正, y上正)。廉价、非阻塞。连接后拨动即运动，无需按压。"""
        self._stick = (x, y, time.monotonic())

    # ---------- 速度映射 ----------
    def velocity(self) -> tuple[float, float]:
        """当前应发的 (vx, w)。数据过期（视为松手）返回 (0, 0)。"""
        x, y, t = self._stick
        if time.monotonic() - t > self.p.stale_s:
            return 0.0, 0.0
        dz = self.p.stick_deadzone
        vx = self.p.forward_sign * shape_axis(y, dz) * self.p.max_linear_mps
        # 拨杆向右 => 顺时针 => w 为负（右手系，z 向上）
        w = -self.p.turn_sign * shape_axis(x, dz) * self.p.max_angular_radps
        return vx, w

    # ---------- 线程 ----------
    def _run(self) -> None:
        period = 1.0 / self.p.send_hz
        try:
            while not self._stop.wait(period):
                with self._lock:
                    want = self._want_connected
                if want and not self.connected:
                    self._do_connect()
                elif not want and self._link is not None:
                    self._do_disconnect()
                if self.connected:
                    self._tick()
        finally:
            if self._link is not None:
                self._do_disconnect()

    def _do_connect(self) -> None:
        with self._lock:
            self._busy = True
        link = AgvLink(self.p.ip, self.p.port, self.p.connect_timeout_s)
        try:
            link.connect()
        except OSError as exc:
            print(f"  ✗ AGV 连接失败（{self.p.ip}:{self.p.port}）：{exc}")
            with self._lock:
                self._want_connected = False
                self._busy = False
            return
        self._link = link
        self._armed = False
        self._warned_unarmed = False
        self._last_err = ""
        with self._lock:
            self._busy = False
        print(f"  ✓ AGV 已连接（{self.p.ip}:{self.p.port}）。"
              f"直接拨动左摇杆 = 开环运动（连上时摇杆须在中位）")

    def _do_disconnect(self) -> None:
        with self._lock:
            self._busy = True
        link, self._link = self._link, None
        if link is not None:
            try:
                link.send(CMD_TEACH_STOP, None)       # 先停车再断
            except OSError:
                pass
            link.close()
        with self._lock:
            self._busy = False
        print("  ✓ AGV 已断开（已发停止）")

    def _tick(self) -> None:
        link = self._link
        assert link is not None
        vx, w = self.velocity()
        moving = vx != 0.0 or w != 0.0
        if not moving:
            self._armed = True                        # 见过松手 => 解除安全门
        if moving and not self._armed and not self._warned_unarmed:
            self._warned_unarmed = True
            print("  · AGV 安全门未解除：请先让左摇杆回中，再拨动")
        try:
            if moving and self._armed:
                if not self._was_moving:
                    print(f"  → AGV 运动 vx={vx:+.2f} m/s  w={w:+.2f} rad/s")
                link.send(CMD_TEACH_MOVE, {"vx": vx, "vy": 0.0, "w": w,
                                           "duration": int(self.p.duration_ms)})
                self.n_sent += 1
            elif self._was_moving:
                link.send(CMD_TEACH_STOP, None)
            self._was_moving = moving and self._armed
            for rep in link.drain_replies():
                code = rep.get("ret_code", 0)
                if code not in (0, None):
                    msg = f"[{code}] {rep.get('err_msg', '')}"
                    if msg != self._last_err:         # 同一错误只提示一次
                        print(f"  ! AGV 拒绝指令：{msg}")
                        self._last_err = msg
        except OSError as exc:
            print(f"  ✗ AGV 连接中断：{exc}（按菜单键重新连接）")
            link.close()
            self._link = None
            self._was_moving = False
            with self._lock:
                self._want_connected = False
