"""
teleop/robot_config.py
=======================

机械臂控制器**连接配置的读取层** —— IP / 端口不再硬编码在各脚本里。

配置文件
--------
``teleop/robot_config.json``（与本文件同目录）。换控制器只改那个 JSON，
所有脚本（遥操、示教）自动跟着变。

取值优先级
----------
1. **环境变量** ``TELOP_ROBOT_HOST``（临时改 IP 调试用，最优先）
2. ``robot_config.json`` 里的 ``controller.host``
3. 内置缺省 ``192.168.100.16``（配置文件丢失时不至于直接崩）

用法
----
::

    from robot_config import load_config

    cfg = load_config()
    cfg.host            # '192.168.100.16'
    cfg.control_port    # 9001
    cfg.command_port    # 9030

    # 或者直接用模块级便捷函数（每次调用都重新读文件，改完 JSON 立刻生效）
    from robot_config import host, arms
    host()
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "RobotConfig",
    "load_config",
    "config_path",
    "host",
    "control_port",
    "command_port",
    "push_period_ms",
    "push_ip",
    "arms",
    "DEFAULT_HOST",
]

# 配置文件缺失时的兜底（实测环境：控制器 192.168.100.16）
DEFAULT_HOST = "192.168.100.16"
DEFAULT_CONTROL_PORT = 9001
DEFAULT_COMMAND_PORT = 9030
DEFAULT_PUSH_PERIOD_MS = 20
DEFAULT_ARMS = ("LeftArm", "RightArm")

# 环境变量名
ENV_HOST = "TELOP_ROBOT_HOST"

HERE = Path(__file__).resolve().parent
CONFIG_FILE = HERE / "robot_config.json"


@dataclass(slots=True)
class RobotConfig:
    """控制器连接参数。"""

    host: str = DEFAULT_HOST
    control_port: int = DEFAULT_CONTROL_PORT
    command_port: int = DEFAULT_COMMAND_PORT
    push_period_ms: int = DEFAULT_PUSH_PERIOD_MS
    # 本机接收推送的 IP。None = 自动按「到控制器的源地址」推断。
    push_ip: str | None = None
    arms: tuple[str, ...] = field(default_factory=lambda: DEFAULT_ARMS)
    # 配置文件路径（None 表示用了缺省值）
    source: Path | None = None

    @property
    def host_label(self) -> str:
        """人类可读的连接标识（带端口）。"""
        return f"{self.host}:{self.control_port}"


def config_path() -> Path:
    """默认配置文件路径。"""
    return CONFIG_FILE


def load_config(path: str | os.PathLike[str] | None = None) -> RobotConfig:
    """读配置文件并返回 :class:`RobotConfig`。

    文件不存在或字段缺失时用内置缺省值补齐，不抛异常 —— 让"忘带配置文件"
    退化成"用默认 IP"而不是"脚本起不来"。

    环境变量 ``TELOP_ROBOT_HOST`` 优先于文件里的 host。
    """
    p = Path(path) if path is not None else CONFIG_FILE
    raw: dict = {}
    if p.exists():
        try:
            raw = json.loads(p.read_text(encoding="utf-8")) or {}
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"读机器人配置 {p} 失败: {exc}") from exc

    ctl = raw.get("controller") or {}
    if not isinstance(ctl, dict):
        raise ValueError(f"{p}: controller 必须是对象")

    arms_raw = raw.get("arms") or list(DEFAULT_ARMS)
    if isinstance(arms_raw, dict):          # 容错：写成 {"LeftArm": {...}}
        arms_raw = list(arms_raw.keys())
    arms_tuple = tuple(str(a) for a in arms_raw) or DEFAULT_ARMS

    push_ip = ctl.get("push_ip")
    if push_ip is not None:
        push_ip = str(push_ip).strip() or None

    cfg = RobotConfig(
        host=str(ctl.get("host") or DEFAULT_HOST).strip(),
        control_port=int(ctl.get("control_port") or DEFAULT_CONTROL_PORT),
        command_port=int(ctl.get("command_port") or DEFAULT_COMMAND_PORT),
        push_period_ms=int(ctl.get("push_period_ms") or DEFAULT_PUSH_PERIOD_MS),
        push_ip=push_ip,
        arms=arms_tuple,
        source=p if p.exists() else None,
    )

    # 环境变量覆盖（调试时临时改 IP）
    env_host = os.environ.get(ENV_HOST, "").strip()
    if env_host:
        cfg.host = env_host
    return cfg


# --------------------------------------------------------------------------
# 便捷函数：每次调用都重新读文件（改完 JSON 立刻生效，不需重启进程）
# --------------------------------------------------------------------------
def host() -> str:
    return load_config().host


def control_port() -> int:
    return load_config().control_port


def command_port() -> int:
    return load_config().command_port


def push_period_ms() -> int:
    return load_config().push_period_ms


def push_ip() -> str | None:
    return load_config().push_ip


def arms() -> tuple[str, ...]:
    return load_config().arms


if __name__ == "__main__":
    # 查看当前生效的连接配置：python3 teleop/robot_config.py
    c = load_config()
    print(f"配置文件   {c.source or '(缺失，用内置缺省)'}")
    print(f"控制器     {c.host}")
    print(f"控制面     TCP {c.control_port}")
    print(f"指令面     UDP {c.command_port}")
    print(f"推送周期   {c.push_period_ms} ms")
    print(f"推送源 IP  {c.push_ip or '(自动推断)'}")
    print(f"机械臂     {', '.join(c.arms)}")
