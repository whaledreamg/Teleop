# PICO 双臂增量式遥操

PICO VR 手柄 → 双 7-DoF 机械臂的**增量式遥操**系统。
客户端自己做逆解（pinocchio DLS），按**绝对关节角**通过实时会话下发。

---

## 1. 整体链路

```
PICO 手柄 (xrobotoolkit_sdk)
   │  6-DoF 位姿 + 按键 + 握持量          [采集 200 Hz]
   ▼
xrt_reader.XrtReader  ── 最新值覆盖 + 按键状态机（边沿锁存 / 电平查询）
   │
   ▼
teleop_map.Alignment  ── 接管瞬间建对齐基准（每臂基座标）
teleop_map.AnchorDeadzone ── EMA + 锚点滞回 + 速度闸门
   │  房间系增量：Δpos(m) / Δquat
   ▼
teleop_loop.ArmTeleop  ── 累加到**客户端影子目标位姿**（绝对）
   │
   ▼
arm_kinematics.JointTracker ── pinocchio DLS IK（warm-start + 零空间回拉）
   │  绝对关节角 7 维 (rad)
   ▼
robot_client.send_joint_target  ── UDP 9030, controlSpace=0 / commandMode=0 / si
   │
   ▼
控制器 realtime 会话 → 机械臂
```

**增量式（相对模式）口径**：手柄位姿增量 → 世界系增量 → 累加到客户端影子目标
位姿 → IK → 绝对关节角下发。控制器侧**不做累加**（`commandMode: 0`），
累加器全在客户端，因此影子与实发目标严格同源，不会发散。

---

## 2. 按键语义

| 物理键 | 逻辑键 | 动作 |
|---|---|---|
| **X**（左手柄 primary） | `home.left` | **按住** → 左臂缓慢回到准备点；**松手立即停止** |
| **A**（右手柄 primary） | `home.right` | **按住** → 右臂缓慢回到准备点；**松手立即停止** |
| **Y**（左手柄 secondary） | `teleop.enable` | **按下**（边沿）→ 双臂上电使能 + 建立实时会话 |
| **B**（右手柄 secondary） | `teleop.disable` | **按下**（边沿）→ 双臂停控 + 下电失能 |
| **左握持 ≥ 0.6** | `left_arm.active` | 左臂**开始**遥操（建立对齐）；松开 → 停止下发左臂 |
| **右握持 ≥ 0.6** | `right_arm.active` | 右臂**开始**遥操；松开 → 停止下发右臂 |
| **左菜单键** | `agv.toggle` | **按下**（边沿）→ AGV 开环控制 连接 ⇄ 断开；启动时**不**自动连接 |
| **左摇杆拨动** | （无绑定） | AGV 连接后直接拨动即动：上下 = 前进 / 后退，左右 = 转向；**回中立即停** |
| **右摇杆按下** | `lift.toggle` | **按下**（边沿）→ 升降柱 使能 ⇄ 下电；启动时**不**自动使能 |
| **右摇杆拨动** | （无绑定） | 升降柱使能后直接拨动即点动：拨上 = 上升，拨下 = 下降；**回中立即停** |

> AGV 与升降柱和双臂的 Y 上电 / B 下电**互相独立**。原来的"重锚"键（左摇杆按下）已移除；
> 想重新对齐，松开握持再握上即可（握持边沿本来就会重建对齐）。
>
> 右手柄"截屏键"被 PICO 系统占用，升降柱使能改用**右摇杆按下**。
> 注意：右摇杆同时用来拨动升降柱，推满摇杆时可能顺带按下，误触发使能 / 下电；点动中误下电前会先停。

### 两类事件：边沿锁存 vs 电平查询

控制循环 90 Hz、采集 200 Hz，两者不同步 —— 一次快速轻点（按下到松开可能只有
30~80 ms）完全可能整个按压过程落在两次轮询之间。所以：

- **边沿锁存**（`consume_latched`）：Y / B / AGV 菜单键 / 升降柱使能（右摇杆按下） —— "不容漏"的一次性动作。
  press 边沿产生时置位，直到消费者显式取走才清除。漏读不可能，只可能多读一次。
- **电平查询**（`query`）：X / A 按住归位、握持选臂 —— 语义本来就是"按住多久
  就做多久"，用锁存会在松手后继续动作。

对应 `button_map.json` 里的 `latch` 字段（`true` = 锁存，`false` = 电平）。

### 三条互斥 / 安全规则

1. **归位与遥操互斥**：按住 X/A 归位时，该臂的握持遥操通道被冻结。
   两者都是往同一个影子目标叠加增量，混用会让影子发散、永远归不到位。
2. **松手只停"下发"**，会话保留、机器人停在最后目标（入控用 `holdOnIdle: true`）。
3. **归位到位判据用实测位姿，且位置与姿态都要到位** —— 只判位置会"位置对了但
   朝向没转过去"。

---

## 3. 文件清单

```
teleop/
├── teleop_loop.py        主循环：按键 -> 接管/归位 -> IK -> 关节流下发
├── agv_client.py         AGV 开环控制（Seer 19205，指令 2010；菜单键连接 + 左摇杆，独立线程）
├── lift_client.py        升降柱点动（liftingColumn/*；右摇杆按下使能 + 右摇杆拨动，独立连接 + 线程）
├── probe_buttons.py      只读探针：打印所有按键 / 摇杆变化，确认 SDK 能读到哪些键
├── arm_kinematics.py     pinocchio FK/IK（7-DoF）、JointTracker、baseFrame 同步
├── teleop_map.py         四元数数学、Alignment（对齐）、AnchorDeadzone（死区）
├── robot_client.py       控制面 TCP 9001 + 指令面 UDP 9030 + 状态推送接收
├── hand_client.py        Revo2 灵巧手：扳机 -> 开合（rs485 透传 + Modbus RTU，独立线程）
├── robot_config.py       连接配置读取层（robot_config.json）
├── xrt_reader.py         PICO 采集 + 按键状态机（边沿锁存 / 电平查询）
├── teach_ready_pose.py   准备点示教工具（写 ready_pose.json）
│
├── robot_config.json     控制器 IP / 端口 / 臂名
├── button_map.json       按键映射（含 latch 语义与阈值）
├── ready_pose.json       准备点位姿（示教生成）
├── teleop_params.json    发包节拍 / IK / 死区 / 限幅 / 对齐 / 归位参数
└── requirements.txt
```

URDF 在项目根的 `urdf_dualarm/`：左臂读 `left_arm.urdf`，右臂读 `right_arm.urdf`，
两条臂各自独立建模，互不依赖（`dualarm.urdf` 不再被遥操使用）。

---

## 4. 配置

### `robot_config.json` —— 连接

```json
{
  "controller": {
    "host": "192.168.100.16",
    "control_port": 9001,
    "command_port": 9030,
    "push_period_ms": 20,
    "push_ip": null
  },
  "arms": ["LeftArm", "RightArm"]
}
```

- `control_port` / `command_port`：控制面（请求-响应：入控/停控/使能/查询）与
  指令面（发关节目标，不回包）。
- `push_ip`：**不要**写 `127.0.0.1`（那会让控制器把状态推给它自己）；`null`
  表示按"到控制器的源地址"自动推断。
- 临时改 IP 也可用环境变量 `TELOP_ROBOT_HOST`（优先级高于文件）。

### `teleop_params.json` —— 算法参数

| 段 | 关键项 | 说明 |
|---|---|---|
| `stream` | `rate_hz=90`、`control_space=0`、`command_mode=0`、`cartesian_ik_path="none"` | 关节空间 / 绝对关节角 / 只收关节包 |
| `ik` | `posture_gain=0.02`、`max_step_deg=3`、`tol_pos_m` / `tol_rot_rad` | 零空间回拉防跳解支；位置与姿态**分开**判阈值 |
| `deadzone` | `pos_deadzone_m=0.005`、`vel_deadzone_mps=0.020` | 锚点滞回式（**不是**逐帧增量阈值） |
| `limits` | `max_step_m=0.020`、`max_step_rad` | 单包位移 / 旋转上限 |
| `align` | `mode="base_frame"`、`rot_mode="world"` | 每臂基座标对齐；旋转世界系刚体传递 |
| `home` | `max_speed_mps=0.05`、`arrive_tol_mm=5` | 归位限速与到位判据 |
| `pd` | `enabled=false` | **预留段**，PD 暂不做 |

⚠️ `deadzone` 必须是**锚点滞回式**。逐帧增量阈值在 90 Hz 下等于 315 mm/s 的
速度闸门，会把正常遥操手速整个吃掉。

### `button_map.json` —— 按键

改阈值 / 换键位都在这里。`latch: false` 表示电平语义（按住多久做多久），
`latch: true`（缺省）表示边沿锁存（一次性动作不容漏）。

---

## 5. 上机流程

### 5.1 环境

```bash
conda activate teleop
pip install -r teleop/requirements.txt
# pinocchio 建议：conda install -c conda-forge pinocchio=4.1.0
```

### 5.2 准备点示教（首次或换准备点时）

1. 手动把机械臂摆到准备点。
2. 跑：

```bash
python teleop/teach_ready_pose.py
```

3. 确认打印的 pos/quat 与你摆的一致，确认后写入 `ready_pose.json`。

脚本**只读机械臂**（`robot/status`），不会让它运动。

### 5.3 遥操

```bash
python teleop/teleop_loop.py
```

启动后按提示操作：

```
【上电】左手柄【Y 键按下】= 双臂上电使能 + 建立实时会话
【下电】右手柄【B 键按下】= 双臂停控 + 下电失能
【遥操】左手柄【左握持拉满】= 控左臂；右手柄【右握持拉满】= 控右臂
        （松开握持 = 该臂停止下发，机器人停在最后目标）
【归位】按住左手柄【X 键】= 左臂缓慢回准备点（松手即停）
        按住右手柄【A 键】= 右臂缓慢回准备点（松手即停）
【AGV】左手柄【菜单键】= 连接 / 断开（启动时不连）；
       连上后直接拨动左摇杆 = 前后 / 转向，回中即停
【升降柱】右手柄【摇杆按下】= 使能 / 下电（启动时不使能）；
       使能后直接上下拨动右摇杆 = 点动，回中即停
```

Ctrl-C 退出。

### 5.4 常用命令行参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--host` | 取 `robot_config.json` | 控制器 IP |
| `--control-port` / `--command-port` | 9001 / 9030 | 控制面 / 指令面端口 |
| `--arm LeftArm RightArm` | 配置里的臂名 | 只遥操某条臂 |
| `--freq` | 90 Hz（取 `teleop_params.json`） | 关节包下发频率 |
| `--vr-rate` | 200 Hz | VR 采集频率 |
| `--handedness` | `RH_Z_back` | 坐标系手性 |
| `--no-hand` | 关 | 不启用灵巧手（扳机不控制手） |
| `--warmup-ms` | 150 | 起跑缓冲（吸收接管瞬间的姿态噪声） |
| `--urdf-left` | `urdf_dualarm/left_arm.urdf` | 左臂 URDF 路径 |
| `--urdf-right` | `urdf_dualarm/right_arm.urdf` | 右臂 URDF 路径 |
| `--no-sync-base` | 关 | 不读控制器 `baseFrame`（**真机几乎总要读**） |
| `--trace` | 关 | 逐包打印下发关节目标 + 影子/实测偏差 |

### 5.5 灵巧手（扳机）

左手柄扳机 → 左手，右手柄扳机 → 右手。**与臂是否使能、是否握持无关**，启动后随时可用。
参数在 `teleop_params.json` 的 `hand` 段：

| 键 | 默认 | 说明 |
|---|---|---|
| `mode` | `proportional` | `proportional` 扳机线性映射，半按半合；`binary` 超过 `binary_on` 一把抓，跌破 `binary_off` 才释放 |
| `open_pose` / `close_pose` | 释放 / 一把抓 | 6 自由度 0~1000：拇指弯曲、拇指侧摆、食指、中指、无名指、小指 |
| `trigger_low` / `trigger_high` | 0.05 / 0.95 | 扳机死区 |
| `write_time_ms` | 120 | 每帧给手的到位时间 |
| `min_interval_ms` / `min_change` | 120 / 20 | 下发节流 |

实现要点：

* 协议、寄存器、从站地址（左 126 / 右 127，`EtherCAT485L/R`）与 `RobotControls/communication/revo2_hand.py` 一致，Modbus 帧已与它逐字节比对。
* 控制器侧用 API §33 的 `rs485/*`（2026-10-28 起取代 `ec2rs485/*`，改为按 `channel` 操作）。程序启动时按 `hand.slaves` 里的从站名，在 `rs485/list` 里找 `dualEc2485` 通道，**通道标识不要手填**。
* **手比臂控慢得多**：一次下发是 `rs485/write` + `rs485/read` 两次往返，还要等通信板发送和手应答。所以手在独立线程里下发，只保留最新目标，不在 90 Hz 的臂控循环里同步调用。新版文档没再给具体下限，实际能多快上机实测，再调 `min_interval_ms`。
* 灵巧手另开一条控制面 TCP 连接，避免一次写读堵住臂控的状态查询和归位。
* 左右手的 `rs485/init` **必须串行**。文档 §33 总则写"不同通道可独立进行"，但实测两只手同时 `init` 必有一只返回 `20003`（文档把 `rs485/init` 归为"从站参数读写请求"，同类请求互斥）。代码里加了进程内锁，并对 `20003` 兜底重试；`write` / `read` 不受此限制。
* 某只手初始化失败只停用那只手，臂控不受影响。
* **`gripper/setType` 已删除，程序不再自动改型号。** 该手的 gripper 设备型号必须是 `Generic485`，否则通道被厂商夹爪协议占用，`rs485/init` 返回 `23252`，程序会给出提示并停用这只手。型号现在只能经 `peripheral/saveConfig` + `applyConfig` 修改；停用夹爪协议的 `peripheral/setDeviceEnabled` 需要现场确认夹爪静止，这一步我不代办。

---

## 6. 关键实现细节

### 6.1 首帧保护

会话建立后，**第一包必须是当前实测关节角**。否则控制器判定
`firstFrameJumpRejected` 自动停控。代码在 `_enable_session` 里先发一包占住
首包等待窗。

### 6.2 baseFrame 同步

URDF 里的 `world` 与控制器上报的**世界系不是同一个系**（实测两条臂的基座
`baseFrame` 分别为 `pos=(0, ±0.2, 0)`，与 URDF 自带装法差 1.2~1.3 m）。
启动时读 `robot/getKinematicGeometry -> baseFrame` 覆盖 URDF 装法，
否则 IK 目标整体偏，表现为"手柄往右走、末端往别的方向走"。

### 6.3 IK 失败处理

IK 失败 ⇒ **丢包 + 影子不推进**。影子推进了但包没发出去，影子就会漂到
控制器跟不到的地方，之后每帧都在追一个从未下发的目标。

### 6.4 奇异降级

接近运动学奇异时（`analyticIkStage != "ok"` 或 `analyticIkPositionDeviation`
非零），姿态跟随降级为纯 3-DoF 平移；连续 5 帧命中才降级，离开后自动恢复。

### 6.5 关节约定

URDF 的 `j1..j7` 与控制器 `jointActualPosition[0..6]` 是否同序同向**没有文档
保证**。`arm_kinematics.detect_joint_convention(kin, q_measured, tcp_pose)` 把
实测关节角代入 FK，与控制器上报的 `tcpPose` 对比，返回按误差排序的候选口径
（同序 / 反序 / 全取反 / 反序+取反）。对不上就先别下发关节包。

### 6.6 增量式的跳变防护

| 风险 | 防护 |
|---|---|
| 接管瞬间手柄噪声变成整包增量 | 起跑缓冲（`--warmup-ms`，默认 150 ms 只更新基准不下发） |
| 松手期间手在动，再握上时一次补上 | 重新接管时 `set_shadow` 把影子对齐到实测位姿 |
| 摇杆中心与追踪原点不重合 | 同上（warmup + 重新对齐） |
| 七轴冗余跳解支 | warm-start 种子 + 零空间姿态回拉 + 单帧关节增量限幅 |
| 单包猛冲 | `max_step_m` / `max_step_rad` 限幅 |
| 持续往一个方向推到工作空间边界 | IK 关节限位：解不出来就丢包，机械臂停在边界 |
| 追踪丢帧导致手柄瞬移 | 锚点滞回死区 + 速度闸门 |

---

## 7. 排障

| 现象 | 排查方向 |
|---|---|
| 发了指令但机械臂不动 | 控制器上残留 realtime 会话吞包 —— 程序启动时会清残留；再看 `robot/<name>/status` 的 `channel` |
| 首包被拒 / 自动停控 | 首包不是实测关节角。看 `_enable_session` 是否跑通、`joints(arm)` 是否取到 7 个值 |
| 手柄往右走、末端往别的方向走 | `baseFrame` 没同步（`--no-sync-base` 被打开）或对齐模式不对 |
| 末端朝向乱扭 | 接近奇异 —— 看 `analyticIkStage`；或 `rot_mode` 选错（`world` vs `tcp_body`） |
| 慢速移动没反应 | 死区过大，或误用了逐帧增量阈值（应为锚点滞回式） |
| 启动瞬间抽一下 | 起跑缓冲太小（调 `--warmup-ms`）或死区锚点没随接管重置 |
| 归位冲过准备点来回振荡 | 归位基准必须用**控制器上一目标**而非实测位姿 |
| 归位看着到了但姿态歪着 | 到位判据必须**位置 + 姿态**都满足 |
| VR 采集没数据 | PC-Service 是否启动、头显是否连上、`xrobotoolkit_sdk` 是否装在 `teleop` 环境里 |

---

## 8. 参考

- 控制协议：`API说明文档.md` 第 21 章（Realtime 接口）
- 逆解算法结构：`tele_ik/`（本项目按同一结构重写为 pinocchio 4.x 实现）
- 设备 SDK：`xrobotoolkit-sdk`（XRoboToolkit 官方）
