"""
teleop/teach_ready_pose.py
==========================

**准备点示教工具** —— 把机械臂当前位姿登记进 ``ready_pose.json``。

用途
----
遥操前需要先把每只臂摆到一个"准备点"，之后**按住** X / A 键就能让它自己
慢慢回到这个位姿。本脚本读当前实测位姿并写进配置文件。

⚠️ 本脚本**只读机械臂**（``robot/status``），不会让它产生任何运动。
   但它会切自动模式、上使能 —— 运行前请确认机械臂周围无人。

怎么用
------
1. 手动把机械臂摆到准备点（拖动示教杆或用控制器自带的示教功能）。
2. 跑::

       python3 teleop/teach_ready_pose.py

3. 确认打印出来的 pos/quat 与你摆的位姿一致。
4. 之后遥操时**按住** X（左臂）/ A（右臂）即可归位。

写入的格式
----------
``quat`` 用 **wxyz** 顺序（``qw, qx, qy, qz``）—— 与下发 UDP 报文的
旋转分量同序，避免手抄时把 w 和 x 搞反（w 排错位置算出来的姿态完全
错乱，但仍是合法四元数，**不会报错**，所以格外容易漏掉）。
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_client import RobotClient                           # noqa: E402
from robot_config import load_config                          # noqa: E402
from teleop_loop import load_home_poses                       # noqa: E402
from teleop_map import quat_from_rpy                           # noqa: E402

HERE = Path(__file__).resolve().parent

# 控制器连接参数来自 robot_config.json（换控制器只改那个文件）
_CFG = load_config()
HOST = _CFG.host
ARMS = _CFG.arms


def main() -> int:
    path = HERE / "ready_pose.json"
    if not path.exists():
        print(f"✗ 找不到 {path}")
        return 1

    print("=" * 74)
    print(f" 准备点示教   {HOST}")
    print("=" * 74)
    print(" 1) 手动把机械臂摆到准备点")
    print(" 2) 保持不动，再回车")
    print(" ⚠ 本脚本只读取位姿，不会让机械臂运动。")
    print("=" * 74)

    try:
        input(" 摆好了？按回车继续（Ctrl-C 取消）: ")
    except (EOFError, KeyboardInterrupt):
        print("\n取消")
        return 1

    # 读现有配置（保留速度等参数）
    try:
        cfg = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"✗ 读 {path} 失败: {exc}")
        return 1
    cfg.setdefault("arms", {})

    captured = {}
    with RobotClient(HOST, config=_CFG) as rc:
        for arm in ARMS:
            st = rc.status(arm)
            cart = st.get("cartesian") or {}
            if not all(k in cart for k in ("x", "y", "z", "a", "b", "c")):
                print(f"✗ {arm} 读不到 cartesian 位姿")
                continue
            tp = (float(cart["x"]), float(cart["y"]), float(cart["z"]))
            rpy = (float(cart["a"]), float(cart["b"]), float(cart["c"]))
            captured[arm] = (tp, rpy)
            qx, qy, qz, qw = quat_from_rpy(*rpy)
            print(f"\n {arm}:")
            print(f"   位置 pos = [{tp[0]:+.6f}, {tp[1]:+.6f}, {tp[2]:+.6f}]")
            print(f"   四元数 quat (w,x,y,z) = "
                  f"[{qw:+.6f}, {qx:+.6f}, {qy:+.6f}, {qz:+.6f}]")
            print(f"   （等效欧拉角 {rpy[0]:+.6f}, {rpy[1]:+.6f}, {rpy[2]:+.6f} rad）")

    if not captured:
        print("\n✗ 没读到任何臂的位姿，未写入")
        return 1

    print("\n" + "=" * 74)
    ans = input(f" 确认写入这 {len(captured)} 条准备点？(y/N): ").strip().lower()
    if ans not in ("y", "yes"):
        print("取消，未写入")
        return 1

    for arm, (tp, rpy) in captured.items():
        entry = cfg["arms"].setdefault(arm, {})
        entry["pos"] = [round(tp[0], 6), round(tp[1], 6), round(tp[2], 6)]
        # quat_from_rpy 返回 (qx,qy,qz,qw)，这里按 wxyz 存
        qx, qy, qz, qw = quat_from_rpy(*rpy)
        entry["quat"] = [round(qw, 6), round(qx, 6), round(qy, 6), round(qz, 6)]
        entry.pop("rpy", None)
        # 归位限速参数（缺省值与 teleop_params.json 的 home 段一致）
        entry.setdefault("max_speed_mps", 0.05)
        entry.setdefault("arrive_tol_mm", 5.0)
        entry.setdefault("arrive_tol_rad", 0.035)
        entry.setdefault("min_step_mm", 0.2)
        entry.setdefault("rotate", True)
        entry.setdefault("max_rot_per_frame_deg", 2.0)
        entry["comment"] = (
            f"{'左' if arm.startswith('Left') else '右'}臂准备点 "
            f"—— 由 teach_ready_pose.py 于 "
            f"{time.strftime('%Y-%m-%d %H:%M:%S')} 示教写入")

    path.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")
    print(f"\n✓ 已写入 {path}")

    # 回读校验：解析回来必须与写入值等价
    try:
        loaded, _ = load_home_poses(path)
    except Exception as exc:
        print(f"⚠ 回读校验失败: {exc}")
        return 1
    from teleop_map import quat_angle, quat_conj, quat_mul
    for arm, (tp, rpy) in captured.items():
        hp = loaded.get(arm)
        if hp is None:
            print(f"⚠ {arm} 回读不到")
            continue
        d_pos = math.dist(hp.pos, tp) * 1000.0
        d_rot = math.degrees(quat_angle(quat_mul(hp.quat, quat_conj(
            quat_from_rpy(*rpy)))))
        flag = "✓" if (d_pos < 1e-3 and d_rot < 1e-3) else "✗"
        print(f"  {flag} {arm} 回读位置差 {d_pos:.6f}mm 姿态差 {d_rot:.6f}°")
    print("\n✓ 完成。遥操时**按住** X（左臂）/ A（右臂）归位。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
