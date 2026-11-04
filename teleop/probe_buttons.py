"""
teleop/probe_buttons.py
=======================

只读探针：打印 PICO 手柄所有按键 / 摇杆 / 扳机 / 握持的**变化**，用来确认
SDK 到底能读到哪些键（尤其是右手柄的"截屏键"能不能读到）。

不连机器人、不发任何指令。

用法（teleop conda 环境，PICO 头显连着 PC-Service）：
    python teleop/probe_buttons.py

逐个按你关心的键，看哪一行变化：
    right.menu       => 截屏键可用，button_map.json 里 lift.toggle 就是它
    都没有变化        => 截屏键是系统保留键，应用读不到，需要换一个键给升降柱使能

Ctrl-C 退出。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import xrobotoolkit_sdk as xrt  # noqa: E402

# (名字, 取值函数)
PROBES = [
    ("left.menu", lambda: xrt.get_left_menu_button()),
    ("right.menu", lambda: xrt.get_right_menu_button()),
    ("left.axis_click", lambda: xrt.get_left_axis_click()),
    ("right.axis_click", lambda: xrt.get_right_axis_click()),
    ("X", lambda: xrt.get_X_button()),
    ("Y", lambda: xrt.get_Y_button()),
    ("A", lambda: xrt.get_A_button()),
    ("B", lambda: xrt.get_B_button()),
    ("left.trigger", lambda: round(float(xrt.get_left_trigger()), 2)),
    ("right.trigger", lambda: round(float(xrt.get_right_trigger()), 2)),
    ("left.grip", lambda: round(float(xrt.get_left_grip()), 2)),
    ("right.grip", lambda: round(float(xrt.get_right_grip()), 2)),
    ("left.axis", lambda: tuple(round(float(v), 2) for v in xrt.get_left_axis())),
    ("right.axis", lambda: tuple(round(float(v), 2) for v in xrt.get_right_axis())),
]


def main() -> int:
    xrt.init()
    print("按键探针已启动。逐个按键，观察哪一行变化（Ctrl-C 退出）。\n")
    last: dict[str, object] = {}
    try:
        while True:
            for name, fn in PROBES:
                try:
                    v = fn()
                except Exception as exc:                 # 某个函数不存在 / 读失败
                    v = f"读取失败: {exc}"
                if last.get(name) != v:
                    if name in last:                     # 第一轮只记基线，不打印
                        print(f"{time.strftime('%H:%M:%S')}  {name:<18} {last[name]!s:>14} -> {v}")
                    last[name] = v
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        xrt.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
