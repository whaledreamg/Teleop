"""
teleop/lift_client.py
=====================

升降柱点动，供 PICO 右手柄使用（走控制器 9001 的 ``liftingColumn/*``，手册第 34 章）。

* 右手柄【摇杆按下】按一下 = 使能，再按一下 = 下电（不自动使能）。
* 使能后直接拨动右摇杆上 / 下 = 上升 / 下降点动（无需按压）；回中立即停。

要点（都来自手册 34.1 / 34.11 / 34.12）：

* 点动靠心跳维持：``jogStart`` 之后须在 ``timeoutMs`` 内持续 ``jogHeartbeat``，
  停发即减速停下。所以程序崩溃 / 断线时升降柱自己会停，不依赖"停止"指令发得出去。
* ``moving`` 为真时（含 ``jogStop`` 后的减速阶段）新的 ``jogStart`` 返回 23268，
  所以换方向 / 重新开始前要等 ``moving`` 变假。
* "是否已使能"以控制器的 ``getState.enabled`` 为准，不自己记标志——急停会让
  升降柱自动下电，自己记的标志会和真实状态对不上。
* 下电前先停：运动中 ``disable`` 是直接断电不减速。
* 用独立的控制面连接 + 独立线程：``enable`` 的请求期限最长 5 s，不能堵住臂控。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, fields
from typing import Any, Optional

from robot_client import RobotError

log = logging.getLogger("teleop.lift")

ERR_BUSY = 20003
ERR_OFFLINE = 23261
ERR_FAULT = 23262
ERR_NOT_ENABLED = 23263
ERR_ESTOP = 23266
ERR_MOVING = 23268
ERR_AT_LIMIT = 23273


@dataclass
class LiftParams:
    """``teleop_params.json`` 的 ``lift`` 段。"""
    enabled: bool = True
    device: Optional[str] = None         # null => 取 liftingColumn/list 的第一台
    speed_mps: Optional[float] = None    # null => 用控制器的 jogVelocity
    up_sign: int = 1                     # 拨杆向上 => 上升；方向反了改 -1
    stick_on: float = 0.5                # 拨杆超过它开始点动
    stick_off: float = 0.3               # 回落到它以下才停（迟滞，防边界抖动）
    timeout_ms: int = 500                # 心跳超时（100~5000）
    heartbeat_ms: int = 150              # 心跳间隔，须 < timeout_ms / 2
    stale_s: float = 0.3                 # 超过它没收到摇杆数据 => 视为松手
    stop_wait_s: float = 3.0             # jogStop 后等 moving 变假的上限

    @classmethod
    def from_dict(cls, raw: dict | None) -> "LiftParams":
        p = cls()
        names = {f.name for f in fields(cls)}
        for key, val in (raw or {}).items():
            if key in names:
                setattr(p, key, val)
        p.validate()
        return p

    def validate(self) -> None:
        if self.up_sign not in (1, -1):
            raise ValueError("lift.up_sign 只能是 1 或 -1")
        if not 0.0 < self.stick_off < self.stick_on <= 1.0:
            raise ValueError("lift 须满足 0 < stick_off < stick_on <= 1")
        if not 100 <= int(self.timeout_ms) <= 5000:
            raise ValueError("lift.timeout_ms 须在 100~5000 内")
        if not 20 <= int(self.heartbeat_ms) <= self.timeout_ms / 2:
            raise ValueError("lift.heartbeat_ms 须在 20 ~ timeout_ms/2 内")
        if self.speed_mps is not None and self.speed_mps <= 0:
            raise ValueError("lift.speed_mps 须 > 0（或 null 用控制器缺省）")


class LiftWorker:
    """升降柱：使能 / 下电由按键触发，点动由摇杆驱动。

    主循环只调 :meth:`toggle`、:meth:`feed`，都不阻塞；所有控制面调用
    （最长 5 s）都在后台线程里。
    """

    TICK_S = 0.02

    def __init__(self, link: Any, params: LiftParams):
        self.link = link
        self.p = params
        self.device: Optional[str] = params.device
        self._toggle_req = threading.Event()
        self._stick: tuple[float, float] = (0.0, 0.0)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        # 以下只在线程内读写
        self._enabled = False
        self._jog_dir = 0
        self._last_hb = 0.0
        self._armed = False              # 使能后见过"松手"才允许动
        self._latched_dir = 0            # 撞限位后，同方向要先松手再试
        self._last_err = ""

    # ---------- 主循环侧 ----------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="lift", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=8.0)
            self._thread = None

    def toggle(self) -> None:
        self._toggle_req.set()

    def feed(self, y: float) -> None:
        """摇杆 y（上正）。廉价、非阻塞。使能后拨动即点动，无需按压。"""
        self._stick = (y, time.monotonic())

    # ---------- 摇杆 -> 方向 ----------
    def _want_dir(self) -> int:
        y, t = self._stick
        if time.monotonic() - t > self.p.stale_s:
            return 0
        a = abs(y)
        threshold = self.p.stick_off if self._jog_dir != 0 else self.p.stick_on
        if a < threshold:
            return 0
        return self.p.up_sign if y > 0 else -self.p.up_sign

    # ---------- 线程 ----------
    def _run(self) -> None:
        try:
            while not self._stop.wait(self.TICK_S):
                try:
                    if self._toggle_req.is_set():
                        self._toggle_req.clear()
                        self._do_toggle()
                    if self._enabled:
                        self._tick()
                except RobotError as exc:
                    self._report(exc)
                except OSError as exc:
                    self._report_text(f"连接异常：{exc}")
                    self._jog_dir = 0
        finally:
            self._safe_stop_jog()

    def _resolve_device(self) -> bool:
        if self.device:
            return True
        devs = self.link.call("liftingColumn/list", {}) or []
        if not devs:
            print("  ✗ 控制器上没有升降柱")
            return False
        self.device = devs[0]["name"]
        return True

    def _state(self) -> dict:
        return self.link.call("liftingColumn/getState", {"device": self.device}) or {}

    def _do_toggle(self) -> None:
        if not self._resolve_device():
            return
        try:
            st = self._state()
        except (RobotError, OSError) as exc:
            print(f"  ✗ 升降柱读状态失败：{exc}")
            return
        if st.get("enabled"):
            self._disable()
        else:
            self._enable(st)

    def _enable(self, st: dict) -> None:
        print(f"\n▶ 升降柱 {self.device}：使能中 …")
        try:
            if not st.get("online", True):
                print("  ✗ 升降柱不在线：请检查网线与驱动器电源（启动时未连上的须重启控制器）")
                return
            if st.get("emergencyStopActive"):
                print("  ✗ 机器人急停生效中，升降柱拒绝使能")
                return
            if st.get("fault"):
                print(f"  · 驱动器故障（错误码 {st.get('errorCode')}），先清除")
                self.link.call("liftingColumn/clearError", {"device": self.device})
            self.link.call("liftingColumn/enable", {"device": self.device})
        except RobotError as exc:
            self._report(exc)
            return
        self._enabled = True
        self._armed = False
        self._jog_dir = 0
        self._latched_dir = 0
        print(f"  ✓ 升降柱已使能（高度 {st.get('height', float('nan')):+.3f} m）。"
              f"直接上下拨动右摇杆 = 点动（使能时摇杆须在中位）")

    def _disable(self) -> None:
        print(f"\n■ 升降柱 {self.device}：下电中 …")
        self._safe_stop_jog()                          # 先停，运动中 disable 是直接断电
        try:
            self.link.call("liftingColumn/disable", {"device": self.device})
        except (RobotError, OSError) as exc:
            print(f"  ✗ 下电失败：{exc}")
            return
        self._enabled = False
        print("  ✓ 升降柱已下电（靠驱动器抱闸保持高度）")

    def _tick(self) -> None:
        want = self._want_dir()
        if want == 0:
            self._armed = True
            self._latched_dir = 0
        if not self._armed:
            return
        if want != 0 and want == self._latched_dir:
            return                                      # 撞过限位，先松手

        if want != self._jog_dir:
            if self._jog_dir != 0:
                self._stop_jog_and_wait()
            if want != 0:
                self._start_jog(want)
            return

        now = time.monotonic()
        if self._jog_dir != 0 and now - self._last_hb >= self.p.heartbeat_ms / 1000.0:
            self.link.call("liftingColumn/jogHeartbeat", {"device": self.device})
            self._last_hb = now

    def _start_jog(self, direction: int) -> None:
        db: dict = {"device": self.device, "direction": direction,
                    "timeoutMs": int(self.p.timeout_ms)}
        if self.p.speed_mps is not None:
            db["speed"] = float(self.p.speed_mps)
        try:
            self.link.call("liftingColumn/jogStart", db)
        except RobotError as exc:
            code = exc.err.get("code")
            if code == ERR_MOVING:
                return                                  # 还在减速，下一拍再试
            if code == ERR_AT_LIMIT:
                self._latched_dir = direction
                print(f"  · 升降柱已到{'上' if direction == self.p.up_sign else '下'}限位")
                return
            raise
        self._jog_dir = direction
        self._last_hb = time.monotonic()
        self._last_err = ""

    def _stop_jog_and_wait(self) -> None:
        self._jog_dir = 0
        try:
            self.link.call("liftingColumn/jogStop", {"device": self.device})
        except RobotError as exc:
            self._report(exc)
        deadline = time.monotonic() + self.p.stop_wait_s
        while time.monotonic() < deadline:
            try:
                if not self._state().get("moving"):
                    return
            except (RobotError, OSError):
                return
            time.sleep(0.05)

    def _safe_stop_jog(self) -> None:
        """退出 / 下电前的收尾：确保没有点动在进行。"""
        if self.device is None:
            return
        try:
            self.link.call("liftingColumn/jogStop", {"device": self.device})
        except (RobotError, OSError):
            pass
        self._jog_dir = 0

    # ---------- 提示 ----------
    def _report(self, exc: RobotError) -> None:
        code = exc.err.get("code")
        self._jog_dir = 0
        if code in (ERR_NOT_ENABLED, ERR_ESTOP, ERR_OFFLINE):
            self._enabled = False                       # 控制器那边已经下电了
            self._report_text(f"{exc}。升降柱已不在使能态，按右摇杆重新使能")
        elif code == ERR_FAULT:
            self._enabled = False
            self._report_text(f"{exc}。按右摇杆会先清故障再使能")
        else:
            self._report_text(str(exc))

    def _report_text(self, msg: str) -> None:
        if msg != self._last_err:                       # 同一错误只提示一次，避免刷屏
            print(f"  ! 升降柱：{msg}")
            self._last_err = msg
