"""
teleop/teleop_map.py
====================

VR（PICO）增量 -> 机器人末端增量的映射层。纯计算，不碰硬件。

坐标系事实（已用 FK 实测定死，见本文件末尾"实测依据"）
----------------------------------------------------
* 机器人侧一切末端位姿读数 —— ``startControl.pose`` / ``robot/status.cartesian``
  / 推送面 ``tcpPose`` —— **全部在世界系**，且三者互相重合（误差 0.00 mm）。
  文档中"基座系 + 工具"的措辞对本机（``HmRobot7DoF`` 自建模板）不成立。
* ``baseFrame`` 给出该臂基座系在世界系下的位姿；反推可知
  **世界系 Z 轴竖直向上**、X 轴指向机器人前方、Y 轴指向机器人左侧
  （右手系；未在真机上复测，见 ``WORLD_UP`` 处的说明）。VR 房间系是
  X 右 / Y 上 / Z 后，两系轴向不同，靠 ``Alignment.trans_rot`` 换算。
* VR 侧（OpenXR 约定）：X 右 / Y 上 / Z 后，右手系；
  四元数排布 ``(qx, qy, qz, qw)``（SDK 的 ``body_joints_pose`` docstring 佐证）。

增程式遥操的映射原理
--------------------
**不要**试图把 VR 手柄的绝对位置对齐到机器人末端的绝对位置 —— 那需要
标定 VR 原点与机器人世界原点的关系，现场根本做不到。

增程式（relative / incremental）遥操只关心**相对运动**，于是：

1. **接管瞬间记下基准**。按下 grip 接管该臂时记下手柄朝向 :math:`R_{h0}`
   与机器人末端当前朝向 :math:`R_{r0}`。

2. **每帧只发增量**，且**平移与旋转各走各的框架**：

   .. math::
      \\Delta p = R_{trans} \\cdot \\Delta p_v
      \\qquad
      \\Delta q = f(\\Delta q_v)

   其中 :math:`\\Delta p_v = p_{cur} - p_{prev}`、:math:`\\Delta q_v = q_{cur} q_{prev}^{-1}`。
   两者都**已经在世界系里**：:math:`\\Delta p_v` 是房间系里的位移向量，
   :math:`\\Delta q_v = R_{cur} R_{prev}^{-1}` 作用在世界上。

两种旋转口径
------------
旋转增量怎么搬到机器人侧，有**两种都说得通**的语义，本实现两种都提供：

===========  ==========================  ==========================  ==============
口径         含义                        公式                        适用
===========  ==========================  ==========================  ==============
``world``    **世界系刚体传递**          :math:`\\Delta q` 直接用     参考实现口径
``tcp_body`` **末端体坐标系传递**        :math:`R_{align}\\Delta q R_{align}^{-1}`  "当工具握"手感
===========  ==========================  ==========================  ==============

* ``world``（**默认**）—— 与参考实现 `XRoboToolkit-Teleop-Sample-Python`
  **逐位一致**。手柄在世界系里怎么转，末端就在世界系里跟着怎么转：

  .. math:: R_{r,new} = \\Delta q \\cdot R_{r,old}

  :math:`\\Delta q` 直接就是世界系增量，控制器 ``commandMode: 1`` 的
  "旋转左乘"正好消费它。**平移与旋转同在世界系里传递** —— 手柄在房间里的
  刚体运动被整体搬到末端上，口径统一、不会打架。

* ``tcp_body`` —— 传统的"把手柄当工具握"手感：手柄**相对自己初始朝向**的
  转动量，映射到末端**自身体系**的同名轴上：

  .. math:: R_{align} = R_{r0} \\cdot R_{h0}^{-1},
            \\qquad R_{r,new} = R_{align} \\Delta q R_{align}^{-1} \\cdot R_{r,old}

  推导：目标是 :math:`R_{r}(t) = R_{align} R_h(t)`，而
  :math:`R_h(t) = \\Delta q R_h(t-1)`，代入即得上式。

  ⚠️ 两种口径**结果不同**（手柄绕房间竖直轴转 30°、末端初始朝下时：
  ``world`` 给 yaw +30°，``tcp_body`` 给 roll -30°）。不是 bug，是两种语义。

3. **平移的映射** ``R_trans`` 有两个变体：

   * ``world``：常量旋转，VR 系 -> 机器人世界系。两系轴向不同
     （VR：X 右 / Y 上 / Z 后；机器人：X 前 / Y 左 / Z 上），不是单位阵。
   * ``base_frame``：**每臂基座标**对齐。手柄的"前/上/右"分别对齐到
     **该臂基座标**的"前/上/右"。之所以要乘 :math:`R_{hand}^{-1}`：
     位移增量是**在 VR 系里表达的**（手柄朝哪，"右"就是哪），要把它换到
     世界系就得用"当前手柄朝向"消去这层依赖。于是：

     * 手柄指向该臂的"前" -> 末端沿该臂的前方走
     * 手柄指向该臂的"右" -> 末端沿该臂的右方走
     * 手柄上抬 -> 末端往上走

     且**与手柄绕自身指向轴的滚转无关**（滚转不改变前/上的世界方向）。

4. **下发用 ``commandMode: 1``**（相对上一目标）。控制器自己累加，
   客户端不维护累加器 —— 这正好绕开文档警告的"绝对目标被裁剪后累加器
   漂移、反向先走一段空行程"的问题。

    对齐只依赖**朝向**，不依赖位置，所以 VR 原点与机器人原点差多少都无所谓。

死区：为什么不能用逐帧增量阈值
------------------------------
死区本意是滤掉 Pico 静止抖动（实测 2~4 mm）。但**最直觉的写法是错的**：

    .. code:: text

      if len(delta) < deadzone: return      # ❌ 错

这一帧的位移增量 = 速度 / 帧率，所以阈值实际上是一个**速度阈值**：

    ==============  ====================  ==========================
    死区             指令率                 等效阈值速度
    ==============  ====================  ==========================
    1.5 mm          90 Hz                 135 mm/s
    3.5 mm          90 Hz                 315 mm/s
    5.0 mm          90 Hz                 450 mm/s
    ==============  ====================  ==========================

正常遥操的手速是 50~300 mm/s —— **几乎全程被吃掉**，表现为
"按了没反应、机械臂纹丝不动"。姿态同理：0.8° 死区 @90 Hz ⇒
阈值角速度 72°/s，慢速转动也发不出去。

正确做法是 :class:`AnchorDeadzone`：**EMA 平滑 + 锚点滞回 + 速度闸门**。
关键在于阈值作用在「累积位移」而非「单帧增量」上 —— 慢速移动时累积量
持续增长、突破阈值后一次性发出，**位移不丢**（手柄走 100 mm，
末端就走 100 mm）。详见该类 docstring。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

# 四元数一律用 (qx, qy, qz, qw)，与 xrt_reader / robot_client 一致。
Quat = tuple[float, float, float, float]
Vec3 = tuple[float, float, float]


# ===========================================================================
# 四元数工具
# ===========================================================================
def quat_identity() -> Quat:
    return (0.0, 0.0, 0.0, 1.0)


def quat_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton 四元数乘 :math:`a \\otimes b`。

    对应旋转 :math:`R(a\\otimes b) = R(a) R(b)`，即**先作用 b 再作用 a**。
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conj(q: Quat) -> Quat:
    """共轭。单位四元数的逆等于共轭。"""
    return (-q[0], -q[1], -q[2], q[3])


def quat_norm(q: Quat) -> Quat:
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return quat_identity()
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_scale(q: Quat, s: float) -> Quat:
    """四元数按标量缩放（**不**重新归一化）。

    只用于"把一个增量旋转的角度缩到上限"这种场合：四元数的
    向量部分长度与 sin(θ/2) 成正比，所以要按 **sin** 而不是 θ
    线性缩放，否则缩放后的角度不是期望值。

    正确做法是构造同轴的新旋转：取 q 的向量部分方向，模长换成
    sin(θ' / 2)，其中 θ' = θ * s。这里用 slerp 到单位四元数再乘 s
    来实现 —— 对 s ∈ [0,1] 精确。
    """
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12 or s >= 1.0:
        return q
    # 归一化后，向量部分长度 = sin(θ/2)
    x, y, z, w = q[0] / n, q[1] / n, q[2] / n, q[3] / n
    sn = math.sqrt(x * x + y * y + z * z)
    if sn < 1e-12:
        return q                       # 零旋转，缩放无意义
    theta = 2.0 * math.asin(min(1.0, sn))
    sign = 1.0 if w >= 0 else -1.0     # 保持半球一致
    w2 = sign * math.cos(theta * 0.5 * s)
    s2 = math.sin(theta * 0.5 * s)
    return (x / sn * s2, y / sn * s2, z / sn * s2, w2)


def quat_from_rpy(a: float, b: float, c: float) -> Quat:
    """R = Rz(c)·Ry(b)·Rx(a) 对应的四元数 (qx,qy,qz,qw)。

    ⚠️ 这个展开式极易写错符号 —— 手搓的乘积展开会得到**模长不为 1** 的
    "四元数"，而四元数乘法假定单位化，后续旋转全错且**不报错**（只是结果
    慢慢歪掉）。本公式与 ``robot_smoke.tcp_to_pose`` 一致，并已用控制器
    ``startControl`` 返回的 ``euler`` / ``quaternion`` 两字段交叉验证：
    2026-10-22 实测 LeftArm 端点误差 < 1e-16（矩阵逐元素比较）。

    约定：``a,b,c`` 即 ``R = Rz(c)·Ry(b)·Rx(a)`` 的三个角，ZYX 顺序。
    """
    ca, sa = math.cos(a * 0.5), math.sin(a * 0.5)
    cb, sb = math.cos(b * 0.5), math.sin(b * 0.5)
    cc, sc = math.cos(c * 0.5), math.sin(c * 0.5)
    return quat_norm((
        cc * cb * sa - sc * sb * ca,
        cc * sb * ca + sc * cb * sa,
        sc * cb * ca - cc * sb * sa,
        cc * cb * ca + sc * sb * sa,
    ))


def quat_to_mat(q: Quat) -> list[list[float]]:
    """四元数 -> 3×3 旋转矩阵（行主序）。"""
    x, y, z, w = quat_norm(q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def mat_to_quat(m: list[list[float]]) -> Quat:
    """3×3 旋转矩阵 -> 四元数（Shepperd 分支法，取数值最稳的那个分支）。"""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return quat_norm((x, y, z, w))


def quat_angle(q: Quat) -> float:
    """四元数对应的旋转角（rad），取 [0, π]。"""
    w = abs(quat_norm(q)[3])
    return 2.0 * math.acos(max(-1.0, min(1.0, w)))


def quat_slerp_slow(a: Quat, b: Quat, t: float) -> Quat:
    """归一化线性插值（够快够用，不追求球面匀速）。

    只在两个四元数夹角不大时用；夹角接近 π 时 nlerp 会走"短路"。
    """
    qa, qb = quat_norm(a), quat_norm(b)
    if qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3] < 0:
        qb = (-qb[0], -qb[1], -qb[2], -qb[3])
    return quat_norm(tuple((1 - t) * qa[i] + t * qb[i] for i in range(4)))


# ===========================================================================
# 向量工具
# ===========================================================================
def mat_vec(m: list[list[float]], v: Vec3) -> Vec3:
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(v: Vec3, s: float) -> Vec3:
    return (v[0] * s, v[1] * s, v[2] * s)


def vec_cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


# ===========================================================================
# 对齐
# ===========================================================================
# 旋转口径（Alignment.rot_mode）：
#   "world"    世界系刚体传递 —— Δq 直接用。与参考实现
#              XRoboToolkit-Teleop-Sample-Python 逐位一致。**默认**。
#   "tcp_body" 末端体坐标系传递 —— R_align·Δq·R_align^{-1}。
#              "把手柄当工具握"手感。
# 两种口径结果不同，不是 bug 是两种语义（详见模块 docstring）。
ROT_MODES = ("world", "tcp_body")

# 机器人世界系轴向（2026-10-27 用 FK 重新核定，推翻此前"Y 向上"的假设）：
#
#   证据（互相一致）：
#     * q=0 的 FK：左臂末端在世界 (0, +1.08, 0)，右臂在 (0, -1.08, 0)
#       —— 两臂沿 ±Y 向两侧水平伸直（T 字位），所以 **Y 是左右轴，左 = +Y**；
#     * baseFrame 平移 (0, ±0.2, 0)：两肩沿 Y 相隔 0.4 m；
#     * 示教的准备点 x≈+0.42、z≈-0.26：在肩下方、身体前方；
#     * 用户对机械臂坐标系图的描述：Z_W 垂直向上（我没能读出图，未独立核对）。
#   ⇒ 采用 **X 向前 / Y 向左 / Z 向上**（右手系：X×Y=Z）。
#
#   ⚠️ 未核实的两点：
#     1) "X 向前"是我按两臂准备点都在 x≈+0.42 推的（两手在肩前方，
#        很自然）；用户对图的描述却写"X 水平向左"，与此不符，未解决。
#        左右轴为 Y（左 = +Y）由 baseFrame 平移 + 准备点 y=±0.43 支持，较可靠。
#     2) 此前"Y 向上"只是由"基座 Z 竖直"这个假设推出的，并非实测；
#        本次改成 Z 向上，依据是准备点（y 一正一负、z 同为 -0.26）与 q=0 FK，
#        没在真机上复测。
#   上机后请先小幅推手柄（前/右/上各一次）核对末端实际走向。
#
#   ⚠️ 旧版把 Y 当竖直、基座 +X 当"前方"。实际基座 +X 指世界 -Z（向下），
#      所以"手往前推"会变成"往下走"、"手往上抬"会变成"往左走"。
WORLD_FORWARD: Vec3 = (1.0, 0.0, 0.0)
WORLD_LEFT: Vec3 = (0.0, 1.0, 0.0)
WORLD_UP: Vec3 = (0.0, 0.0, 1.0)


def base_frame_axes(rpy: Vec3) -> list[Vec3]:
    """baseFrame 的 rpy -> 该臂基座三轴在世界系下的方向（仅供诊断）。"""
    R = quat_to_mat(quat_from_rpy(*rpy))
    return [(R[0][j], R[1][j], R[2][j]) for j in range(3)]


def base_align_rot(base_rpy: Vec3 | None = None,
                   up_world: Vec3 = WORLD_UP) -> list[list[float]]:
    """"手柄该长什么样"的世界系基：列 = (右, 上, **后**)。

    手柄/VR 房间系是 ``+X 右 / +Y 上 / -Z 前``，机器人世界系是
    ``X 前 / Y 左 / Z 上``，所以：

    * VR 右(+X) -> 世界 -Y
    * VR 上(+Y) -> 世界 +Z
    * VR 后(+Z) -> 世界 -X

    与 ``baseFrame`` 无关：两臂共用同一个世界系，baseFrame 只决定基座
    装在哪（供 IK 换算），不决定"前/上/右"。``base_rpy`` 参数仅为兼容旧调用。
    """
    right_w = vec_scale(WORLD_LEFT, -1.0)
    up_w = up_world
    back_w = vec_scale(WORLD_FORWARD, -1.0)
    return [[right_w[0], up_w[0], back_w[0]],
            [right_w[1], up_w[1], back_w[1]],
            [right_w[2], up_w[2], back_w[2]]]


@dataclass(slots=True)
class Alignment:
    """VR -> 机器人的对齐（接管瞬间建立）。

    这里有**两个语义完全不同的旋转**，绝不能混用（2026-10-23 实测定位）：

    ==========  ==========================  =============  ================
    矩阵        含义                        用途           是否依赖手柄姿态
    ==========  ==========================  =============  ================
    ``rot``     **VR 系 -> TCP 体坐标系**    ``map_quat``   是
    ``trans_rot``  **VR 系 -> 世界系**        ``map_vec``    **否**
    ==========  ==========================  =============  ================

    旋转：``R_align = R_robot0 · R_hand0^{-1}``。``R_robot0`` 是 TCP 体系 ->
    世界系，``R_hand0`` 是 VR 系 -> 世界系，相乘得到 **VR -> TCP 体系**。
    旋转增量本来就在"物体自身系"里表达，所以用它是对的。

    平移：``d_pos = cur_pos - prev_pos``。PICO 报的 ``pos`` 本来就表达在
    **VR 世界系（房间系）** 里 —— X 右 / Y 上 / Z 后，右手系，1:1 就是米
    （实测：``y≈1.30`` 是手的高度、``z≈-0.40`` 是身前距离，
    正是房间系而非手柄体坐标系）。所以 ``d_pos`` **已经是世界系向量**，
    只需一次**常量**的坐标系换算，不该再乘任何由手柄/末端姿态推出的旋转。

    ⚠️ 曾经把 ``R_align``（VR->TCP 体）拿去过平移，于是末端朝向被算进了
    平移方向里。实测：手柄与世界完全对齐时，VR +X 被映射到世界
    ``(-0.256, -0.536, +0.804)``，竖直分量 -0.536，方向歪 100° 以上 ——
    这正是「水平移动方向与机械臂方向不对齐」。

    平移需要的常量旋转 ``R_trans`` 只由**世界系与 VR 系的轴向差异**决定
    （默认两者同为 X 右 / Y 上 / Z 后，即恒等），与手柄怎么握、末端什么
    姿态**完全无关**。这样"向右移动"在任何握持姿态下都是同一世界方向。

    旋转口径（``rot_mode``）
    ----------------------
    ``map_quat`` 怎么把 VR 侧的旋转增量搬到机器人侧，有两种语义：

    * ``"world"``（**默认**）—— **世界系刚体传递**，与参考实现
      `XRoboToolkit-Teleop-Sample-Python` **逐位一致**：

      .. math:: R_{r,new} = \\Delta q \\cdot R_{r,old}

      :math:`\\Delta q` 就是世界系增量，直接被控制器 ``commandMode: 1``
      的"旋转左乘"消费。平移与旋转**同在世界系**里传递。

    * ``"tcp_body"`` —— **末端体坐标系传递**（"把手柄当工具握"手感）：

      .. math:: R_{r,new} = R_{align} \\Delta q R_{align}^{-1} R_{r,old}

      ⚠️ 两种口径**结果不同**（手柄绕房间竖直轴转 30°、末端初始朝下时：
      ``world`` 给 yaw +30°，``tcp_body`` 给 roll -30°）。不是 bug 是两种语义。
    """

    rot: list[list[float]]        # R_align = R_robot0 · R_hand0^{-1}（VR->TCP体）
    inv_rot: list[list[float]]    # R_align^{-1}
    trans_rot: list[list[float]]  # R_trans：VR 系 -> 机器人世界系（常量）
    trans_inv: list[list[float]]  # R_trans^{-1}
    hand_quat: Quat               # 接管时的手柄朝向（仅记录，供诊断/重锚）
    robot_quat: Quat              # 接管时的机器人末端朝向
    # 旋转口径："world" = 世界系刚体传递（参考实现口径）；
    #             "tcp_body" = 末端体坐标系传递
    rot_mode: str = "world"

    @classmethod
    def build(cls, hand_quat: Quat, robot_quat: Quat,
              trans_rot: list[list[float]] | None = None,
              rot_mode: str = "world") -> "Alignment":
        rh = quat_to_mat(hand_quat)
        rr = quat_to_mat(robot_quat)
        # R_align = R_robot0 · R_hand0^{-1}  ->  VR 系 -> TCP 体坐标系
        ri = [[rh[j][i] for j in range(3)] for i in range(3)]   # R_hand0^T
        rot = [[sum(rr[i][k] * ri[k][j] for k in range(3)) for j in range(3)]
               for i in range(3)]
        inv = [[rot[j][i] for j in range(3)] for i in range(3)]
        # 平移专用：VR 系 -> 世界系。默认恒等 —— 两系轴向本就一致
        # （X 右 / Y 上 / Z 后，右手系，1:1 米制）。
        tr = trans_rot if trans_rot is not None else [
            [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        tri = [[tr[j][i] for j in range(3)] for i in range(3)]
        if rot_mode not in ROT_MODES:
            raise ValueError(f"未知 rot_mode: {rot_mode!r}（可选：{ROT_MODES}）")
        return cls(rot=rot, inv_rot=inv,
                   trans_rot=tr, trans_inv=tri,
                   hand_quat=quat_norm(hand_quat),
                   robot_quat=quat_norm(robot_quat),
                   rot_mode=rot_mode)

    @classmethod
    def build_base_frame(cls, hand_quat: Quat, robot_quat: Quat,
                         base_rpy: Vec3,
                         rot_mode: str = "world") -> "Alignment":
        """**每臂基座标对齐**：手柄与该臂自己的基座标对齐。

        语义：手柄的"前 / 上 / 右"分别对齐到**该臂基座标**的
        "前 / 上 / 右"，其中

        * **前** = 世界 ``+X``，**上** = 世界 ``+Z``，**右** = 世界 ``-Y``
          （两臂共用同一个世界系，与 baseFrame 无关）

        对齐后的手柄朝向记为 :math:`R_{want}`，再令

        .. math:: R_{trans} = R_{want} \\cdot R_{hand}^{-1}

        之所以要乘 :math:`R_{hand}^{-1}`：位移增量是**在 VR 系里表达的**
        （手柄朝哪，"右"就是哪），要把它换到世界系就得用"当前手柄朝向"
        消去这层依赖。这样：

        * 手柄指向该臂的"前" -> 末端沿该臂的前方走
        * 手柄指向该臂的"右" -> 末端沿该臂的右方走
        * 手柄上抬 -> 末端往上走

        且**与手柄绕自身指向轴的滚转无关**（滚转不改变前/上的世界方向）。
        """
        want = base_align_rot(base_rpy)          # 列 = (右, 上, 后) 世界系
        rh = quat_to_mat(hand_quat)
        # ⚠️ 只取手柄的**偏航**（朝向在水平面上的投影），不用完整姿态。
        # 位移增量是 VR 房间系向量，竖直轴（Y）本来就是真竖直；若连俯仰/
        # 滚转一起消掉（R_want·R_hand^{-1}），抓取瞬间手柄前端朝下 45°，
        # 之后"手往前推"会被换算成"斜向上走"，滚转同理 —— 即实测的
        # "方向没对齐"。偏航保留是为了让操作者可以朝任意方向站着遥操。
        fx = -rh[0][2]                           # 手柄前向(-Z 局部轴)的房间系分量
        fz = -rh[2][2]
        n = math.hypot(fx, fz)
        if n < 0.35:
            # 手柄几乎竖直指天/指地，前向的水平投影不可靠：不修偏航
            yaw_cos, yaw_sin = 1.0, 0.0
        else:
            # R_yaw 绕 Y 旋转，使 R_yaw·(0,0,-1) = (fx,0,fz)/n
            yaw_cos, yaw_sin = -fz / n, -fx / n
        # R_yaw^{-1}（绕 Y 的转置）
        ryi = [[yaw_cos, 0.0, -yaw_sin],
               [0.0, 1.0, 0.0],
               [yaw_sin, 0.0, yaw_cos]]
        # R_trans = R_want · R_yaw^{-1}
        tr = [[sum(want[i][k] * ryi[k][j] for k in range(3)) for j in range(3)]
              for i in range(3)]
        return cls.build(hand_quat, robot_quat, trans_rot=tr, rot_mode=rot_mode)

    def map_vec(self, v: Vec3) -> Vec3:
        """把 VR 系的位移增量搬到**世界系**（用于平移）。

        常量旋转，与手柄姿态、末端姿态均无关。
        """
        return mat_vec(self.trans_rot, v)

    def map_quat(self, q: Quat) -> Quat:
        """把 VR 系表达的旋转增量搬到机器人侧（用于旋转）。

        按 ``rot_mode`` 分两种：

        * ``"world"``（默认）—— **原样返回**。:math:`\\Delta q` 本身
          就是世界系增量（:math:`R_{cur} R_{prev}^{-1}` 作用在世界上），
          控制器 ``commandMode: 1`` 的"旋转左乘"直接消费它。
          与参考实现 `XRoboToolkit-Teleop-Sample-Python` 逐位一致。
        * ``"tcp_body"`` —— :math:`R_{align}\\cdot q\\cdot R_{align}^{-1}`，
          把手柄的自身系增量搬到末端自身体系的同名轴上。
        """
        if self.rot_mode == "world":
            # Δq 是 VR 房间系里的增量，必须换到机器人世界系：
            #   q_w = R_trans · Δq · R_trans^{-1}
            # 两系轴向不同时直接原样用会把"绕竖直轴转"变成"绕水平轴转"。
            m = quat_to_mat(q)
            t = self.trans_rot
            r = [[sum(t[i][k] * m[k][j] for k in range(3)) for j in range(3)]
                 for i in range(3)]
            out = [[sum(r[i][k] * self.trans_inv[k][j] for k in range(3))
                    for j in range(3)] for i in range(3)]
            return mat_to_quat(out)
        m = quat_to_mat(q)
        r = [[sum(self.rot[i][k] * m[k][j] for k in range(3)) for j in range(3)]
             for i in range(3)]
        out = [[sum(r[i][k] * self.inv_rot[k][j] for k in range(3))
                for j in range(3)] for i in range(3)]
        return mat_to_quat(out)


# ===========================================================================
# 增量提取
# ===========================================================================
@dataclass(slots=True)
class HandDelta:
    """手柄两帧之间的变化。

    ``d_pos`` 是**世界系**位移向量（PICO 报的 pos 本就在房间系里）。
    ``d_quat`` 是**世界系**旋转增量（``q_cur · q_prev⁻¹``，作用在世界上）。
    详见 :func:`hand_delta` 的说明。
    """

    d_pos: Vec3 = (0.0, 0.0, 0.0)      # 位移增量（世界系）
    d_quat: Quat = quat_identity()     # 旋转增量（世界系）
    valid: bool = False


def hand_delta(prev_pos: Vec3, prev_quat: Quat,
               cur_pos: Vec3, cur_quat: Quat) -> HandDelta:
    """两帧手柄位姿的差分。

    旋转增量取 :math:`\\Delta q = q_{cur} q_{prev}^{-1}`。

    ⚠️ **它是世界系增量，不是体坐标系增量**。已数值验证：
    :math:`R_{cur} = \\Delta q \\cdot R_{prev}`（误差 2e-16）成立，而
    :math:`R_{cur} = R_{prev} \\cdot \\Delta q` 不成立（误差 3e-1）。

    也就是说 :math:`\\Delta q` **作用在世界上**（pre-multiplication），
    与参考实现 `XRoboToolkit-Teleop-Sample-Python` 的
    ``quat_diff_as_angle_axis(q_cur, q_prev)`` 口径**完全一致**。

    这正是 :data:`ROT_MODES` 里 ``"world"`` 口径能直接消费它的原因；
    ``"tcp_body"`` 口径则要额外做 ``R_align·Δq·R_align⁻¹`` 的换算。
    """
    dq = quat_mul(quat_norm(cur_quat), quat_conj(quat_norm(prev_quat)))
    return HandDelta(d_pos=vec_sub(cur_pos, prev_pos),
                     d_quat=quat_norm(dq),
                     valid=True)


# ===========================================================================
# 死区 / 限幅
# ===========================================================================
@dataclass(slots=True)
class AnchorDeadzone:
    """**锚点滞回式**死区（位置 + 姿态）。

    为什么不能用逐帧增量阈值
    ------------------------
    最直觉的做法是"这一帧的位移增量小于阈值就丢掉"。但那是**错的**：

    * 死区作用在**相邻两帧增量**上，而增量 = 速度 / 帧率。
      90 Hz 指令率 + 3.5 mm 死区 ⇒ 阈值速度 = 3.5 mm × 90 = **315 mm/s**。
      正常遥操的手速是 50~300 mm/s —— **几乎全程被吃掉**，
      表现为"按了没反应、机械臂纹丝不动"。
    * 姿态同理：0.8° 死区 @90 Hz ⇒ 阈值角速度 72°/s，慢速转动也发不出去。
    * 死区本意是滤掉 Pico 静止抖动（实测 2~4 mm）。但抖动是
      **围绕锚点来回摆**，不是单向累积；逐帧阈值把它当成了"真实运动"。

    本类怎么解决
    -------------
    三件事叠加：

    1. **EMA 平滑**手柄位姿（时间常数 ``smooth_tau_s``）——
       抖动是高频零均值噪声，EMA 把它的幅度压掉数倍；
       慢速运动是低频信号，几乎不受影响。
    2. **锚点滞回**判定：把平滑后的增量累加成 ``acc``，与「上次触发时的
       位姿」比较。慢速移动时 ``acc`` **持续累积**，突破 ``pos_deadzone_m``
       就下发 —— 一次性把累积量发出去，**位移不丢**。
       空闲时锚点按 ``leak`` 朝 ``acc`` 缓慢漂移：抖动/漂移被锚点吸收，
       持续单向运动仍会累积突破阈值。
    3. **速度闸门**：平滑后的速度低于 ``vel_deadzone_mps`` 视为"停了"，
       释放并把锚点拉回当前位姿。低于该速度的运动本来就感觉不到。

    迟滞
    ----
    位置触发用 ``pos_deadzone_m``、释放用
    ``pos_deadzone_m - pos_hysteresis_m``；姿态同理。
    速度闸门触发用 ``vel_deadzone_mps``、恢复用 ``×(1+vel_hysteresis_frac)``，
    避免在阈值附近反复触发/释放造成末端抖动。

    ⚠️ **位移不丢**：触发时下发的是"自上次触发以来的累积位移"，
    不是当帧增量。所以"手柄走 100 mm → 末端走 100 mm"始终成立。
    """

    pos_deadzone_m: float = 0.005      # 位置触发阈值（m）
    pos_hysteresis_m: float = 0.002    # 位置释放阈值回退量（m）
    rot_deadzone_rad: float = 0.0175   # 姿态触发阈值（rad, ≈1°）
    rot_hysteresis_rad: float = 0.0087 # 姿态释放阈值回退量（rad, ≈0.5°）
    vel_deadzone_mps: float = 0.020    # 速度闸门（m/s），低于此视为停止
    vel_hysteresis_frac: float = 0.5   # 速度闸门恢复系数（×(1+frac)）
    smooth_tau_s: float = 0.050        # EMA 时间常数（s）
    leak: float = 0.05                 # 空闲时锚点朝累积位姿漂移的比例

    # ---- 运行时状态（不参与构造）----
    _smooth_pos: Optional[Vec3] = field(default=None, init=False, repr=False)
    _smooth_quat: Optional[Quat] = field(default=None, init=False, repr=False)
    _acc_pos: Vec3 = field(default=(0.0, 0.0, 0.0), init=False, repr=False)
    _acc_quat: Quat = field(default=quat_identity(), init=False, repr=False)
    _anchor_pos: Vec3 = field(default=(0.0, 0.0, 0.0), init=False, repr=False)
    _anchor_quat: Quat = field(default=quat_identity(), init=False, repr=False)
    _latch_pos: bool = field(default=False, init=False, repr=False)
    _latch_rot: bool = field(default=False, init=False, repr=False)

    # ---------------- 生命周期 ----------------
    def reset(self, hand_pos: Optional[Vec3] = None,
              hand_quat: Optional[Quat] = None) -> None:
        """重置全部状态（接管 / 释放 / 重锚时调用）。

        给了手柄位姿就顺手把它设成锚点 —— 锚点必须落在"接管那一刻"，
        否则上一轮接管的残留会让新一轮一开始就误判。
        """
        self._smooth_pos = None
        self._smooth_quat = None
        self._acc_pos = (0.0, 0.0, 0.0)
        self._acc_quat = quat_identity()
        self._anchor_pos = (0.0, 0.0, 0.0)
        self._anchor_quat = quat_identity()
        self._latch_pos = False
        self._latch_rot = False
        if hand_pos is not None:
            self._smooth_pos = tuple(hand_pos)   # type: ignore[assignment]
        if hand_quat is not None:
            self._smooth_quat = quat_norm(hand_quat)

    # ---------------- 内部工具 ----------------
    @staticmethod
    def _alpha(dt: float, tau: float) -> float:
        """EMA 系数：``1 - exp(-dt/tau)``（与采样率无关）。"""
        if tau <= 0.0:
            return 1.0
        return 1.0 - math.exp(-max(0.0, dt) / tau)

    # ---------------- 主流程 ----------------
    def update(self, hand_pos: Vec3, hand_quat: Quat,
               align: "Alignment", dt: float) -> tuple[Vec3, Quat]:
        """喂入一帧手柄位姿，返回「这一帧该下发的增量」``(d_pos, d_quat)``。

        ``d_pos`` —— 世界系位移增量（米），与控制器 ``commandMode=1``
        的"平移分量相加"直接兼容。
        ``d_quat`` —— 世界系旋转增量，与控制器"旋转左乘"直接兼容。

        两个返回值都可能是零 —— 那就是"这一帧不下发"。
        """
        # ---- 1. EMA 平滑 ----
        if self._smooth_pos is None or self._smooth_quat is None:
            self._smooth_pos = tuple(hand_pos)      # type: ignore[assignment]
            self._smooth_quat = quat_norm(hand_quat)
            self._anchor_pos = (0.0, 0.0, 0.0)
            self._anchor_quat = quat_identity()
            return (0.0, 0.0, 0.0), quat_identity()

        a = self._alpha(dt, self.smooth_tau_s)
        prev_pos, prev_quat = self._smooth_pos, self._smooth_quat
        self._smooth_pos = tuple(prev_pos[i] + a * (hand_pos[i] - prev_pos[i])
                                 for i in range(3))
        self._smooth_quat = quat_slerp_slow(prev_quat, quat_norm(hand_quat), a)

        # 平滑后的一帧增量（VR 系 -> 世界系）
        d_pos_v = vec_sub(self._smooth_pos, prev_pos)
        d_quat_v = quat_norm(quat_mul(self._smooth_quat, quat_conj(prev_quat)))
        d_pos = align.map_vec(d_pos_v)
        d_quat = align.map_quat(d_quat_v)

        # ---- 2. 累积（与控制器同语义：平移相加、旋转左乘）----
        self._acc_pos = (self._acc_pos[0] + d_pos[0],
                         self._acc_pos[1] + d_pos[1],
                         self._acc_pos[2] + d_pos[2])
        self._acc_quat = quat_norm(quat_mul(d_quat, self._acc_quat))

        dt_eff = max(dt, 1e-6)
        speed = vec_norm(d_pos) / dt_eff
        rot_speed = quat_angle(d_quat) / dt_eff
        vel_gate = self.vel_deadzone_mps
        vel_resume = vel_gate * (1.0 + self.vel_hysteresis_frac)

        # ---- 3. 位置判定 ----
        out_pos: Vec3 = (0.0, 0.0, 0.0)
        err_pos = vec_sub(self._acc_pos, self._anchor_pos)
        if self._latch_pos:
            if speed < vel_gate:
                # 停了：释放，锚点拉回当前累积位姿
                self._latch_pos = False
                self._anchor_pos = self._acc_pos
            else:
                out_pos = err_pos
                self._anchor_pos = self._acc_pos
        else:
            if vec_norm(err_pos) > self.pos_deadzone_m:
                # 突破死区：把**累积位移**一次发出去（位移不丢）
                self._latch_pos = True
                out_pos = err_pos
                self._anchor_pos = self._acc_pos
            else:
                # 空闲：锚点缓慢漂移，吸收抖动与漂移
                self._anchor_pos = (self._anchor_pos[0] + self.leak * err_pos[0],
                                    self._anchor_pos[1] + self.leak * err_pos[1],
                                    self._anchor_pos[2] + self.leak * err_pos[2])

        # ---- 4. 姿态判定 ----
        out_quat: Quat = quat_identity()
        err_quat = quat_norm(quat_mul(self._acc_quat, quat_conj(self._anchor_quat)))
        ang = quat_angle(err_quat)
        if self._latch_rot:
            if rot_speed < vel_gate:
                self._latch_rot = False
                self._anchor_quat = self._acc_quat
            else:
                out_quat = err_quat
                self._anchor_quat = self._acc_quat
        else:
            if ang > self.rot_deadzone_rad:
                self._latch_rot = True
                out_quat = err_quat
                self._anchor_quat = self._acc_quat
            else:
                self._anchor_quat = quat_slerp_slow(
                    self._anchor_quat, self._acc_quat, self.leak)

        return out_pos, out_quat


@dataclass(slots=True)
class Deadband:
    """死区 + 单包限幅。

    为什么要死区：VR 追踪有毫米级抖动，不滤的话末端会一直微微蠕动。
    为什么要限幅：追踪丢帧 / 手柄瞬移会让单帧增量很大，一包发出去
    就是一次猛冲。

    ⚠️ 位置死区请用 :class:`AnchorDeadzone`（锚点滞回式）。
    本类里的 ``deadzone_pos`` 是**逐帧增量阈值**，只适合单元测试与
    "单帧跳变"场景；拿它做实时死区会把慢速运动整个吃掉。
    """

    pos_deadzone_m: float = 0.0015     # 1.5 mm
    rot_deadzone_rad: float = 0.0087   # 0.5°
    max_step_m: float = 0.020          # 单包位移上限 20 mm
    max_step_rad: float = 0.0873       # 单包旋转上限 5°
    scale: float = 1.0                 # 位置缩放（1.0 = 1:1）

    def clamp_pos(self, d: Vec3) -> Vec3:
        """单包位移限幅。"""
        n = vec_norm(d)
        if n > self.max_step_m:
            return vec_scale(d, self.max_step_m / n)
        return d

    def deadzone_pos(self, d: Vec3) -> Vec3:
        """位移死区。"""
        if vec_norm(d) < self.pos_deadzone_m:
            return (0.0, 0.0, 0.0)
        return d

    def apply_pos(self, d: Vec3) -> Vec3:
        """死区 -> 缩放 -> 限幅（一步到位）。"""
        d = vec_scale(self.deadzone_pos(d), self.scale)
        return self.clamp_pos(d)

    def apply_rot(self, q: Quat) -> Quat:
        """姿态死区 + 限幅（一步到位）。

        ⚠️ 实时路径请用 :meth:`clamp_rot`（**只限幅**）——
        姿态死区由 :class:`AnchorDeadzone` 统一处理。这里再套一次
        等于把死区按帧作用于增量角，90 Hz 下阈值角速度高达 90°/s，
        慢速转动会完全发不出去。
        """
        ang = quat_angle(q)
        if ang < self.rot_deadzone_rad:
            return quat_identity()
        return self.clamp_rot(q)

    def clamp_rot(self, q: Quat) -> Quat:
        """单包旋转限幅（**不含死区**）。"""
        ang = quat_angle(q)
        if ang > self.max_step_rad:
            # 绕同一轴按比例缩角
            axis = _quat_axis(q)
            half = ang * 0.5 * (self.max_step_rad / ang)
            s = math.sin(half)
            return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half))
        return q


def _quat_axis(q: Quat) -> Vec3:
    """单位四元数的旋转轴（q 的向量部分；w≈0 时退化为任意垂直轴）。"""
    v = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2])
    if v < 1e-9:
        return (1.0, 0.0, 0.0)
    return (q[0] / v, q[1] / v, q[2] / v)


# 实测依据
# ===========================================================================
# 2026-10-22 用 FK（device/getDhParams 的 MDH + baseFrame）实测：
#
#   LeftArm   startControl.pose == status.cartesian == push.tcpPose == FK_world
#             误差 0.00 mm；与 FK_base 差 1297.21 mm
#   RightArm  同上，与 FK_base 差 1165.67 mm
#
# 由 baseFrame(rpy) 反推世界系轴向：
#   LeftArm   基座+X -> 世界 (0,0,-1)   基座+Y -> 世界 (-1,0,0)   基座+Z -> 世界 (0,1,0)
#   RightArm  基座+X -> 世界 (0,0,-1)   基座+Y -> 世界 ( 1,0,0)   基座+Z -> 世界 (0,-1,0)
#
# （2026-10-27 更正）此前由"基座 Z 竖直"推出"世界 Y 向上"，该假设已废弃；
# 现采用 X 前 / Y 左 / Z 上，依据见 WORLD_UP 处的说明。
