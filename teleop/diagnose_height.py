"""
teleop/diagnose_height.py
=========================

只读诊断：不上使能、不发任何运动包。复用 ``TeleopRunner`` 的真实链路
（基座对齐 -> 死区 -> 限幅 -> 影子累加 -> pinocchio IK），逐层记录 Z 向累计量，
定位"水平来回推，高度却单向增长"出在哪一层。

四层数据（都是接管后的累计量，单位 mm）：

    手柄 VR Y   手柄在房间系里实际抬高了多少（Y 是 VR 的竖直轴）
    映射 Σd_z   经对齐/死区/限幅之后，累加进影子的 Z 增量之和
    影子 Z      影子目标位姿相对接管时的 Z 变化（应等于上一列）
    FK(IK) Z    IK 解出的关节角做 FK 得到的 Z（与影子 Z 之差 = IK 残差）

读法：
    手柄 VR Y 在涨   -> 手本身在抬高（动作习惯 / 追踪漂移）
    映射 Σd_z 涨而手柄不涨 -> 对齐矩阵或死区引入了竖直分量
    影子 Z 与 Σd_z 不一致 -> 影子累加有额外项
    FK(IK) Z 偏离影子 Z    -> IK 跟踪残差

用法：握住握持键（接管），水平来回推几次，松开或等时间到。
    python3 teleop/diagnose_height.py --arm LeftArm --duration 20
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
from argparse import Namespace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robot_client import RobotClient                       # noqa: E402
from robot_config import load_config                       # noqa: E402
from teleop_loop import (                                  # noqa: E402
    ACTIVE_KEY_OF_HAND, HAND_OF_ARM, HERE, TeleopRunner,
)
from xrt_reader import XrtReader                           # noqa: E402

ZERO3 = (0.0, 0.0, 0.0)


def main() -> int:
    ap = argparse.ArgumentParser(description="高度漂移只读诊断")
    ap.add_argument("--robot-config", default=str(HERE / "robot_config.json"))
    ap.add_argument("--params", default=str(HERE / "teleop_params.json"))
    ap.add_argument("--config", default=str(HERE / "button_map.json"))
    ap.add_argument("--arm", default="LeftArm", choices=list(HAND_OF_ARM))
    ap.add_argument("--handedness", default="RH_Z_back",
                    choices=["RH_Z_back", "LH_Z_forward"])
    ap.add_argument("--duration", type=float, default=20.0, help="总时长 s")
    ap.add_argument("--csv", default=None, help="逐帧数据存 CSV")
    a = ap.parse_args()
    logging.basicConfig(level=logging.WARNING)

    arm, hand = a.arm, HAND_OF_ARM[a.arm]
    cfg = load_config(a.robot_config)
    args = Namespace(params=a.params, warmup_ms=150.0, freq=90.0, arm=[arm],
                     urdf_left=None, urdf_right=None, no_sync_base=False,
                     joint_mode=True)
    runner = TeleopRunner(args)
    period = 1.0 / args.freq
    st = runner.arms[arm]

    rows: list[list[float]] = []
    with RobotClient(cfg.host, port=cfg.control_port,
                     command_port=cfg.command_port, config=cfg) as rc:
        runner.rc = rc
        rc.start_state_push([arm])
        time.sleep(0.5)
        if not runner._setup_kinematics(rc):
            return 2
        reader = XrtReader(a.config, a.handedness, rate_hz=200.0)
        reader.start()
        print(f"\n只读诊断 {arm}（{hand} 手柄）。握住握持键接管，水平来回推。"
              f" {a.duration:.0f}s 后结束。\n")
        print(f"{'t(s)':>6} {'手柄VR_Y':>9} {'映射Σd_z':>9} {'影子Z':>8} "
              f"{'FK(IK)Z':>8}   (mm，相对接管时)")

        hand0 = fk0 = shadow0 = None
        sum_dz = 0.0
        n_fail = 0
        t0 = next_tick = last_print = time.monotonic()
        try:
            while time.monotonic() - t0 < a.duration:
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(min(next_tick - now, 0.002))
                    continue
                next_tick += period
                snap, _ = reader.get_latest()
                fr = getattr(snap, hand)
                want = reader.query(ACTIVE_KEY_OF_HAND[hand])

                if want and not st.active:
                    rp = runner._robot_pose(arm)
                    if rp is None or not runner._seed_tracker(arm):
                        continue
                    st.engage(fr, rp[1], runner._align_base(arm), runner.rot_mode)
                    st.set_shadow(rp[0], rp[1])
                    hand0, shadow0, sum_dz = fr.pos, rp[0], 0.0
                    fk0 = runner.kin[arm].fk_world_pose(st.joint_tracker.q)[0]
                    print("  ▶ 接管（基准已记录）")
                elif not want and st.active:
                    st.release()
                    print("  ■ 释放")
                if not st.active:
                    continue

                res = st.step(fr, dt=period)
                if res is None:
                    continue
                d_pos, d_quat = res
                if d_pos != ZERO3:
                    d_pos = runner.deadband.clamp_pos(
                        tuple(v * runner.speed for v in d_pos))
                d_quat = runner.deadband.clamp_rot(d_quat)
                tgt = st.compose(d_pos, d_quat)
                q = st.joint_tracker.solve(*tgt)
                if q is None:
                    n_fail += 1
                    continue
                st.apply_delta(d_pos, d_quat)
                sum_dz += d_pos[2]
                fk = runner.kin[arm].fk_world_pose(q)[0]
                row = [now - t0, (fr.pos[1] - hand0[1]) * 1e3, sum_dz * 1e3,
                       (st.shadow_pos[2] - shadow0[2]) * 1e3,
                       (fk[2] - fk0[2]) * 1e3]
                rows.append(row)
                if now - last_print >= 0.5:
                    last_print = now
                    print(f"{row[0]:6.1f} {row[1]:+9.1f} {row[2]:+9.1f} "
                          f"{row[3]:+8.1f} {row[4]:+8.1f}")
        except KeyboardInterrupt:
            pass
        finally:
            reader.stop()

    print(f"\n采样 {len(rows)} 帧，逆解失败 {n_fail} 次。末帧各层 Z（mm）：")
    if rows:
        r = rows[-1]
        print(f"  手柄VR_Y={r[1]:+.1f}  映射Σd_z={r[2]:+.1f}  "
              f"影子Z={r[3]:+.1f}  FK(IK)Z={r[4]:+.1f}")
    if a.csv and rows:
        with open(a.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "hand_vr_y_mm", "sum_dz_mm", "shadow_z_mm", "fk_z_mm"])
            w.writerows(rows)
        print(f"  已存 {a.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
