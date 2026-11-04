"""
teleop/xrt_reader.py
====================

PICO + XRoboToolkit 采集层（工程方案 §八 的 Thread 1）。

职责
----
1. 通过 xrobotoolkit_sdk 读取头显 / 左右手柄的 6-DoF 位姿与按键
2. 配置驱动的按键映射（analog / button / hold / combo / axis）
3. 线程安全的"最新值覆盖"接口，供 Thread 2 控制循环直接取用
4. 可切换的坐标系手性适配

不在本模块做的事
----------------
* 滤波 / 姿态映射 -> teleop_map.py
* 逆解           -> arm_kinematics.py
* 控制器通信     -> robot_client.py
* 按键语义决策   -> teleop_loop.py

运行
----
本模块是**库**，由 ``teleop_loop.py`` 驱动：

    python teleop_loop.py

⚠️ ``xrobotoolkit_sdk`` 不可用时**直接报错退出**，不降级造假数据 ——
造出来的手柄位姿会让主循环误以为"人在操作"，是危险的。

设计要点
--------
* **最新值覆盖，不是 FIFO 队列**。VR 90 Hz、控制 200 Hz，输入输出频率不同，
  FIFO 必然积压并引入无界延迟。控制循环只读最新一帧，旧帧直接丢弃。
* 按键映射与物理按键解耦：本模块只产出"逻辑事件"，
  接管 / 急停 / 模式切换由下游订阅（见 button_map.json）。
* 时间戳双轨：SDK 的 Unix 纳秒用于跨设备对时，本地 monotonic 用于判新鲜度，
  两者不可混用。
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Optional

log = logging.getLogger(__name__)

Handedness = Literal["RH_Z_back", "LH_Z_forward"]
Which = Literal["left", "right", "head"]

try:
    import xrobotoolkit_sdk as xrt  # type: ignore
    _SDK_AVAILABLE = True
except Exception as exc:  # 未安装，或二进制与环境不匹配
    xrt = None
    _SDK_AVAILABLE = False
    log.warning("xrobotoolkit_sdk 不可用（%s）", exc)


# ===========================================================================
# 数据结构
# ===========================================================================
@dataclass(slots=True)
class HandFrame:
    """单帧单手柄（含头显）的全部观测。"""
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)             # 米
    quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # qx,qy,qz,qw
    pose_valid: bool = False

    grip: float = 0.0        # 握持键 0~1
    trigger: float = 0.0     # 扳机   0~1
    axis: tuple[float, float] = (0.0, 0.0)
    axis_click: bool = False

    primary: bool = False    # 左手柄=X，右手柄=A
    secondary: bool = False  # 左手柄=Y，右手柄=B
    menu: bool = False       # 左手柄=Menu，右手柄=截图/录制


@dataclass(slots=True)
class XrtSnapshot:
    """同一时刻的完整观测 + 两条时间轨。"""
    left: HandFrame = field(default_factory=HandFrame)
    right: HandFrame = field(default_factory=HandFrame)
    head: HandFrame = field(default_factory=HandFrame)
    device_ts_ns: int = 0      # SDK Unix 纳秒（跨设备对时用）
    local_ts_ns: int = 0       # 本地 monotonic 纳秒（判新鲜度用）


@dataclass(slots=True)
class ButtonEvent:
    """一帧内被触发的逻辑事件。"""
    name: str
    active: bool
    value: float
    edge: Optional[str]   # press | release | hold_fire | None


# ===========================================================================
# 原始读数 + 手性适配
# ===========================================================================
class _MirrorZ:
    """Z 轴镜像：左手系(X右/Y上/Z前) <-> 右手系(X右/Y上/Z后)。

    反射矩阵 S = diag(1,1,-1)，位姿变换为 p' = S p，R' = S R S。
    在四元数上等价于只对向量部分做同样的轴反射：
        (qx, qy, qz, qw)  ->  (-qx, -qy, qz, qw)

    ⚠️ 仅翻转位置 Z 而不动四元数是错的，会得到一个与镜像不自洽的旋转。
    """

    @staticmethod
    def pos(p: tuple[float, float, float]) -> tuple[float, float, float]:
        return (p[0], p[1], -p[2])

    @staticmethod
    def quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        return (-q[0], -q[1], q[2], q[3])


class _RawXrt:
    """封装 xrobotoolkit_sdk，屏蔽命名差异与手性差异。"""

    def __init__(self, handedness: Handedness = "RH_Z_back"):
        self.handedness = handedness
        if not _SDK_AVAILABLE:
            raise RuntimeError(
                "xrobotoolkit_sdk 不可用：无法采集 PICO 手柄。"
                "请在 teleop conda 环境里安装 SDK 并确认 PC-Service 在运行。")
        xrt.init()
        self._t0 = time.monotonic()

    def close(self) -> None:
        try:
            xrt.close()
        except Exception:
            pass

    # ---------- 位姿 ----------
    _POSE_FN = {"left": "get_left_controller_pose",
                "right": "get_right_controller_pose",
                "head": "get_headset_pose"}

    def get_pose(self, which: Which) -> tuple[tuple[float, float, float],
                                              tuple[float, float, float, float]] | None:
        v = getattr(xrt, self._POSE_FN[which])()
        if v is None or len(v) < 7:
            return None
        pos = (float(v[0]), float(v[1]), float(v[2]))
        quat = self._norm_quat(v[3], v[4], v[5], v[6])

        if self.handedness == "LH_Z_forward":
            pos, quat = _MirrorZ.pos(pos), _MirrorZ.quat(quat)
        return pos, quat

    # ---------- 模拟量 ----------
    def get_analog(self, which: Which, kind: str) -> float:
        pre = "left_" if which == "left" else "right_"
        try:
            if kind == "grip":
                return float(getattr(xrt, f"get_{pre}grip")())
            if kind == "trigger":
                return float(getattr(xrt, f"get_{pre}trigger")())
        except Exception as exc:
            log.warning("get_analog(%s,%s) failed: %s", which, kind, exc)
        return 0.0

    def get_axis(self, which: Which) -> tuple[float, float]:
        pre = "left_" if which == "left" else "right_"
        try:
            a = getattr(xrt, f"get_{pre}axis")()
            return float(a[0]), float(a[1])
        except Exception as exc:
            log.warning("get_axis(%s) failed: %s", which, exc)
            return 0.0, 0.0

    # ---------- 离散键 ----------
    def get_button(self, which: Which, kind: str) -> bool:
        """kind ∈ {primary, secondary, menu, axis_click}

        注意 A/B/X/Y 四个键在 pybind 里**没有 left/right 前缀**，
        只能靠 (which, kind) 显式分派 —— 不能靠字符串拼函数名。
        """
        if kind == "axis_click":
            pre = "left_" if which == "left" else "right_"
            try:
                return bool(getattr(xrt, f"get_{pre}axis_click")())
            except Exception:
                return False

        if kind == "primary":
            fn = xrt.get_X_button if which == "left" else xrt.get_A_button
        elif kind == "secondary":
            fn = xrt.get_Y_button if which == "left" else xrt.get_B_button
        elif kind == "menu":
            # PC-Service 协议层有 menuButton，但 pybind v1.1.0 未见导出。
            # 这里做属性探测，拿不到就返回 False，不抛错。
            name = f"get_{'left' if which == 'left' else 'right'}_menu_button"
            fn = getattr(xrt, name, None)
            if fn is None:
                return False
        else:
            raise ValueError(f"unknown button kind: {kind}")

        try:
            return bool(fn())
        except Exception as exc:
            log.warning("get_button(%s,%s) failed: %s", which, kind, exc)
            return False

    def get_timestamp_ns(self) -> int:
        try:
            return int(xrt.get_time_stamp_ns())
        except Exception:
            return time.time_ns()

    @staticmethod
    def _norm_quat(a, b, c, d) -> tuple[float, float, float, float]:
        n = math.sqrt(a * a + b * b + c * c + d * d)
        if n < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (a / n, b / n, c / n, d / n)


# ===========================================================================
# 按键映射
# ===========================================================================
@dataclass
class ButtonBinding:
    name: str
    kind: str                                   # analog|button|hold|combo|axis
    source: dict
    threshold: float = 0.5
    duration: float = 1.0
    # 迟滞带宽：已激活时用 threshold - hysteresis 作为释放阈值。
    # 没有迟滞的话，握持值在阈值附近微抖就会让 dead-man 反复 press/release，
    # 机械臂跟着反复冻结/解冻 —— 实机上是危险行为。
    hysteresis: float = 0.15
    # 是否产生 **press 边沿锁存**（consume_latched 通道）。
    #
    # 默认 True —— 一次性动作（Y 使能 / B 失能 / 重锚）必须走锁存：
    # 控制循环 90 Hz、采集 200 Hz 不同步，一次快速轻点（按下到松开可能
    # 只有 30~80 ms）完全可能整个按压过程落在两次轮询之间，电平查询
    # 会彻底漏掉。
    #
    # ⚠️ 但**电平语义**的绑定必须显式置 False —— 例如"按住 X 键归位，
    # 松手立即停止"。若这类绑定也锁存，消费方一旦读了锁存位，就会在
    # 松手之后仍然继续归位，直接违背语义。
    latch: bool = True
    # 运行态
    _active: bool = field(default=False, init=False, repr=False)
    _value: float = field(default=0.0, init=False, repr=False)
    _press_t: float = field(default=0.0, init=False, repr=False)
    _fired: bool = field(default=False, init=False, repr=False)
    # press 边沿锁存位：置位后一直保持，直到消费者 consume_latched()。
    # 用于"不容漏"的一次性动作（结束遥操 / 急停）。
    _latched: bool = field(default=False, init=False, repr=False)


class ButtonDispatcher:
    """按键状态机：每帧 update()，返回本帧的逻辑事件。"""

    _KINDS = {"analog", "button", "hold", "combo", "axis"}

    def __init__(self, bindings: list[dict]):
        self._b: list[ButtonBinding] = []
        for cfg in bindings:
            kind = cfg["kind"]
            if kind not in self._KINDS:
                raise ValueError(f"{cfg.get('name')}: unknown kind {kind!r}")
            self._b.append(ButtonBinding(
                name=cfg["name"], kind=kind, source=cfg["source"],
                threshold=float(cfg.get("threshold", 0.5)),
                duration=float(cfg.get("duration", 1.0)),
                hysteresis=float(cfg.get("hysteresis", 0.15)),
                latch=bool(cfg.get("latch", True)),
            ))
        self._idx = {b.name: b for b in self._b}
        if len(self._idx) != len(self._b):
            raise ValueError("button_map.json 存在重复的 name")

    def active(self, name: str) -> bool:
        b = self._idx.get(name)
        return b._active if b else False

    def latched(self, name: str) -> bool:
        """该绑定**曾经被按下过**且尚未被消费（``consume_latched``）。

        ⚠️ 这是给"结束遥操/急停"这类**不容漏**的一次性动作准备的。

        为什么不能只用 :meth:`active`：``active`` 是瞬时电平，控制循环
        90 Hz 轮询、采集 200 Hz，两者不同步 —— 一次快速轻点（按下到松开
        可能只有 30~80 ms）完全可能整个按压过程落在两次轮询之间，
        于是被**彻底漏掉**。对急停来说漏一次就是致命的。

        :meth:`fired` 解决的是另一个问题（``hold`` 长按），它依赖
        "按住足够久"这一前提；而快速轻点连 ``hold`` 的边沿都来不及
        产生，所以仍然需要独立的**锁存**通道。

        锁存语义：press 边沿产生时置位，直到消费者显式调用
        :meth:`consume_latched` 清除为止 —— 因此"漏读"不存在，
        只会"多读一次"（而多读一次对急停是无害的）。
        """
        b = self._idx.get(name)
        return bool(b._latched) if b else False

    def consume_latched(self, name: str) -> bool:
        """取走并清除锁存位。返回清除前是否处于锁存状态。"""
        b = self._idx.get(name)
        if b is None or not b._latched:
            return False
        b._latched = False
        return True

    def fired(self, name: str) -> bool:
        """``hold`` 类绑定是否**已经触发过**。

        与 :meth:`active` 的区别：``active`` 是"按钮当前按着"，
        ``fired`` 是"长按已达标、事件已发出"。``hold`` 触发后 ``_fired``
        保持为 True 直到按钮松开。

        ⚠️ 为什么需要它：控制循环跑 90 Hz、采集跑 200 Hz，
        ``get_latest()`` 只带回**最后一帧**的事件 —— 中间帧的事件会被
        漏掉。急停这种一次性事件漏一次就致命，所以改成**电平查询**：
        每 tick 问"急停已经触发了吗"，事件不会丢。
        """
        b = self._idx.get(name)
        return bool(b._fired) if b else False

    def value(self, name: str) -> float:
        b = self._idx.get(name)
        return b._value if b else 0.0

    def update(self, snap: XrtSnapshot, t: float) -> list[ButtonEvent]:
        events: list[ButtonEvent] = []
        for b in self._b:
            new_active, new_value = self._eval(b, snap)
            edge = self._edge(b, new_active, t)
            b._active, b._value = new_active, new_value
            if edge:
                events.append(ButtonEvent(b.name, new_active, new_value, edge))
        return events

    # ---------- 求值 ----------
    def _eval(self, b: ButtonBinding, snap: XrtSnapshot) -> tuple[bool, float]:
        k = b.kind
        if k == "analog":
            v = self._read_analog(snap, b.source)
            return self._hyst(b, v, b.threshold), v
        if k == "axis":
            idx = int(b.source.get("axis_index", 0))
            fr = snap.left if b.source["which"] == "left" else snap.right
            v = fr.axis[idx] if idx < len(fr.axis) else 0.0
            # 轴向有正负，用幅值比较
            return self._hyst(b, abs(v), b.threshold), v
        if k == "button":
            return self._read_button(snap, b.source), 0.0
        if k == "hold":
            return self._read_button(snap, b.source), 0.0
        if k == "combo":
            return all(self._read_button(snap, s) for s in b.source["all"]), 0.0
        return False, 0.0

    @staticmethod
    def _hyst(b: ButtonBinding, value: float, threshold: float) -> bool:
        """带迟滞的阈值比较。

        已激活 -> 需跌破 (threshold - hysteresis) 才释放；
        未激活 -> 需超过 threshold 才激活。

        ⚠️ 释放判据用 >= 而非 >：threshold - hysteresis 在浮点下可能是
        0.44999999999999996 这种值，用 > 会让恰好等于边界的值无法释放，
        死区看起来"卡住"。
        """
        if b._active:
            release = max(0.0, threshold - b.hysteresis)
            return value >= release
        return value >= threshold

    @staticmethod
    def _read_analog(snap: XrtSnapshot, src: dict) -> float:
        fr = snap.left if src["which"] == "left" else snap.right
        return fr.grip if src["source"] == "grip" else fr.trigger

    @staticmethod
    def _read_button(snap: XrtSnapshot, src: dict) -> bool:
        fr = snap.left if src["which"] == "left" else snap.right
        return {
            "primary": fr.primary, "secondary": fr.secondary,
            "menu": fr.menu, "axis_click": fr.axis_click,
        }.get(src["source"], False)

    # ---------- 边沿 ----------
    @staticmethod
    def _edge(b: ButtonBinding, now_active: bool, t: float) -> Optional[str]:
        prev = b._active
        if b.kind == "hold":
            if now_active and not b._fired:
                if b._press_t == 0.0:
                    b._press_t = t
                elif t - b._press_t >= b.duration:
                    b._fired = True
                    return "hold_fire"
            elif not now_active:
                b._press_t, b._fired = 0.0, False
            return None
        if now_active and not prev:
            # 锁存 press 边沿：消抖（快速轻点也不丢），且必须等消费者
            # 显式 consume_latched() 才清除。
            # ⚠️ 电平语义的绑定（``latch: false``）**不**锁存 ——
            #    「按住 X 归位、松手即停」这类语义靠锁存会变成
            #    「松手后仍在归位」。
            if b.latch:
                b._latched = True
            return "press"
        if not now_active and prev:
            return "release"
        return None


# ===========================================================================
# 采集器
# ===========================================================================
class XrtReader:
    """采集线程 + 最新值覆盖。"""

    def __init__(self,
                 button_config: dict | str | Path,
                 handedness: Handedness = "RH_Z_back",
                 rate_hz: float = 200.0,
                 stale_ms: float = 500.0):
        if isinstance(button_config, (str, Path)):
            with open(button_config, "r", encoding="utf-8") as f:
                cfg = json.load(f)
        else:
            cfg = button_config
        self._raw = _RawXrt(handedness)
        self._disp = ButtonDispatcher(cfg.get("bindings", []))
        self._period = 1.0 / max(50.0, float(rate_hz))
        self._rate_hz = 1.0 / self._period
        self._stale_ns = int(stale_ms * 1e6)

        self._lock = threading.Lock()
        self._latest = XrtSnapshot()
        self._events: list[ButtonEvent] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sample_count = 0
        self._error_count = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="xrt-reader", daemon=True)
        self._thread.start()
        log.info("XrtReader 启动: %.0f Hz, hand=%s",
                 self._rate_hz, self._raw.handedness)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._raw.close()
        log.info("XrtReader 停止（共采样 %d 帧，错误 %d 帧）",
                 self._sample_count, self._error_count)

    # ---------- 消费者接口 ----------
    def get_latest(self) -> tuple[XrtSnapshot, list[ButtonEvent]]:
        with self._lock:
            return self._latest, list(self._events)

    def is_data_fresh(self) -> bool:
        """基于本地 monotonic 时钟判断数据是否新鲜。

        ⚠️ 不能用 SDK 的 Unix 时间戳做这个判断 —— 那是墙钟，
        与 monotonic 不同源，混用会导致永远判定为过期。
        """
        with self._lock:
            if self._latest.local_ts_ns == 0:
                return False
            return (time.monotonic_ns() - self._latest.local_ts_ns) < self._stale_ns

    def query(self, name: str) -> bool:
        """直接查询逻辑按键当前激活态（跨线程安全）。"""
        return self._disp.active(name)

    def query_fired(self, name: str) -> bool:
        """查询 ``hold`` 类绑定是否已触发过（电平，见 :meth:`ButtonDispatcher.fired`）。"""
        return self._disp.fired(name)

    def query_latched(self, name: str) -> bool:
        """查询该键是否被按下过且尚未消费（见 :meth:`ButtonDispatcher.latched`）。"""
        return self._disp.latched(name)

    def consume_latched(self, name: str) -> bool:
        """取走该键的锁存 press（见 :meth:`ButtonDispatcher.consume_latched`）。"""
        return self._disp.consume_latched(name)

    def query_value(self, name: str) -> float:
        return self._disp.value(name)

    def stats(self) -> dict:
        with self._lock:
            return {
                "samples": self._sample_count,
                "errors": self._error_count,
                "rate_hz": self._rate_hz,
                "pose_valid": {
                    "head": self._latest.head.pose_valid,
                    "left": self._latest.left.pose_valid,
                    "right": self._latest.right.pose_valid,
                },
            }

    # ---------- 采集循环 ----------
    def _loop(self) -> None:
        deadline = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            if now < deadline:
                time.sleep(min(deadline - now, 0.002))
                continue
            try:
                snap = self._sample()
                events = self._disp.update(snap, now)
                with self._lock:
                    self._latest = snap
                    self._events = events
                    self._sample_count += 1
            except Exception:
                self._error_count += 1
                log.exception("采样异常，该帧丢弃")
            deadline += self._period
            # 落后太多则重置，避免忙等
            if deadline < time.monotonic() - self._period:
                deadline = time.monotonic()

    def _sample(self) -> XrtSnapshot:
        snap = XrtSnapshot(device_ts_ns=self._raw.get_timestamp_ns(),
                           local_ts_ns=time.monotonic_ns())
        for which in ("left", "right", "head"):
            fr: HandFrame = getattr(snap, which)
            pose = self._raw.get_pose(which)          # type: ignore[arg-type]
            if pose is not None:
                fr.pos, fr.quat = pose
                fr.pose_valid = True
            fr.grip = self._raw.get_analog(which, "grip")          # type: ignore[arg-type]
            fr.trigger = self._raw.get_analog(which, "trigger")    # type: ignore[arg-type]
            fr.axis = self._raw.get_axis(which)                    # type: ignore[arg-type]
            fr.primary = self._raw.get_button(which, "primary")
            fr.secondary = self._raw.get_button(which, "secondary")
            fr.menu = self._raw.get_button(which, "menu")
            fr.axis_click = self._raw.get_button(which, "axis_click")
        return snap

