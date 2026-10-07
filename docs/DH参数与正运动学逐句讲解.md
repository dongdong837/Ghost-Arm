# Ghost Arm：DH 参数与正运动学逐句讲解

## 1. 这一步完成什么

输入四个关节角 `q1,q2,q3,q4`，计算两指中间的 `grasp_center` 相对于球体 `base_link` 的位置和朝向。正运动学是“角度 → 位姿”，不会移动机器人，也不是逆运动学。

只修改 `/home/ubuntu/ghost_arm`；原 `/home/ubuntu/ghost_sim` 保留不变。

## 2. 按现有模型确定尺寸

读取 `simulation/tail_structure.py`，得到机械链：

```text
base_link 球心
  → 后支架 (-0.27, 0, -0.04)
  → 尾根关节 (-0.06, 0, -0.04)，绕 Z 转 q1
  → 肩关节 (0, 0, -0.045)，绕 Y 转 q2
  → 上臂 0.22 m
  → 肘关节，绕 Y 转 q3
  → 前臂 0.20 m
  → 腕关节，绕 Y 转 q4
  → 抓取中心在腕下方 0.115 m
```

尾根关节相对球心的固定平移为 `(-0.33,0,-0.08)`。肩关节比尾根低 0.045 m，因此肩部高度为 `-0.125 m`。后三个俯仰轴在零位时平行，均沿 +Y。

夹指的两个滑动关节不加入四轴串联链，因为 `grasp_center` 固定在掌部。两指不对称开合时，它也不会自动变成实际接触中心；此时需要另外计算接触位置。

## 3. 采用标准 DH，明确坐标轴

使用列向量与标准 DH 顺序：

$$ A_i = R_z(\theta_i)T_z(d_i)T_x(a_i)R_x(\alpha_i). $$

这里的关节 i 围绕前一坐标系的 `z_(i-1)` 轴旋转。不是改进 DH，不能把两种表直接混用。

- DH 坐标系 0：放在尾根转轴处，初始方向与 base_link 一致，Z 朝上。
- DH 坐标系 1：放在肩关节处，`alpha1=-π/2` 将下一根 Z 轴转到物理 +Y 方向。
- DH 坐标系 2：放在肘关节处。`theta2=q2+π/2` 使零位时正 X 连杆方向对应实际向下的上臂。
- DH 坐标系 3：放在腕关节处，沿前臂继续延伸。
- DH 坐标系 4：放在抓取中心。最后一根连杆长度 0.115 m 包含腕到抓取中心的工具偏移。

### 标准 DH 参数表

| i | 物理关节 | a_i（m） | alpha_i（rad） | d_i（m） | theta_i |
| --- | --- | --- | --- | --- | --- |
| 1 | tail_yaw | 0 | -π/2 | -0.045 | q1 |
| 2 | tail_shoulder | 0.22 | 0 | 0 | q2+π/2 |
| 3 | tail_elbow | 0.20 | 0 | 0 | q3 |
| 4 | tail_wrist | 0.115 | 0 | 0 | q4 |

`d1` 为负，因为肩位于尾根下方；`theta2` 的 π/2 是坐标系零位换算，不表示 Gazebo 肩关节要额外转 90°。程序接收的仍是物理关节反馈角度。

### 两个固定变换不能漏

$$T_{base,grasp}=T_{base,0} A_1 A_2 A_3 A_4 C.$$

`T_base,0` 只有平移 `(-0.33,0,-0.08)`。DH 最后一个坐标系与 URDF 的抓取坐标系方向不同，所以还要乘：

$$C=\begin{bmatrix}0&0&-1&0\-1&0&0&0\0&1&0&0\0&0&0&1\end{bmatrix}.$$

其旋转是 `Rz(-π/2) Rx(π/2)`，只换方向，不再平移；0.115 m 已经写入第 4 行，不能重复添加。

## 4. 可以手算检查的表达式

设 `phi=q2+q3+q4`，则

```text
rho = -0.22 sin(q2) - 0.20 sin(q2+q3) - 0.115 sin(phi)
x = -0.33 + cos(q1) rho
y =          sin(q1) rho
z = -0.125 - 0.22 cos(q2) - 0.20 cos(q2+q3) - 0.115 cos(phi)
R = Rz(q1) Ry(phi)
```

这与 DH 连乘结果一致，便于检查方向和长度。当前代码使用矩阵连乘，方便之后扩展雅可比与坐标转换。

- 零位 `[0,0,0,0]`：抓取中心为 `(-0.33,0,-0.66)` m，旋转矩阵为单位阵。
- 前伸 `[0,-0.8,-0.6,1.4]`：约为 `(0.0249083,0,-0.4272689)` m；三个俯仰角相加为零，所以抓取坐标轴方向与 base_link 相同。夹指几何仍沿其局部 -Z 向下。

这些是相对球心的位置。若球心在世界 `(0,0,1.2)` 且姿态为单位旋转，前伸抓取中心世界高度约为 `0.7727311 m`。球体转动后必须完整使用 `T_world,base × T_base,grasp`，不能只给 Z 加高度。

## 5. 参数文件 arm_dh.json

文件：`config/arm_dh.json`。

```json
{
  "convention": "standard_DH_Rz_Tz_Tx_Rx",
  "base_frame": "base_link",
  "tip_frame": "grasp_center",
  "base_translation_m": [
    -0.33,
    0.0,
    -0.08
  ],
  "rows": [
    {
      "joint": "tail_yaw",
      "a_m": 0.0,
      "alpha_rad": -1.5707963267948966,
      "d_m": -0.045,
      "theta_offset_rad": 0.0
    },
    {
      "joint": "tail_shoulder",
      "a_m": 0.22,
      "alpha_rad": 0.0,
      "d_m": 0.0,
      "theta_offset_rad": 1.5707963267948966
    },
    {
      "joint": "tail_elbow",
      "a_m": 0.2,
      "alpha_rad": 0.0,
      "d_m": 0.0,
      "theta_offset_rad": 0.0
    },
    {
      "joint": "tail_wrist",
      "a_m": 0.115,
      "alpha_rad": 0.0,
      "d_m": 0.0,
      "theta_offset_rad": 0.0
    }
  ],
  "tip_rotation": [
    [
      0,
      0,
      -1
    ],
    [
      -1,
      0,
      0
    ],
    [
      0,
      1,
      0
    ]
  ]
}
```

- `convention`：记录矩阵乘法约定，程序拒绝其他 DH 约定。
- `base_frame/tip_frame`：说明位姿两端坐标系。
- `base_translation_m`：球心到尾根 DH 原点的固定平移。
- `rows`：四行 DH 参数，必须与物理关节顺序一致。
- 每行 `joint`：实时读取 JointState 时按这个名称取角度；不依赖消息数组排列。
- `a_m`、`alpha_rad`、`d_m`：对应表格的连杆长度、扭转角、轴向偏移。
- `theta_offset_rad`：与实际关节角相加的坐标零位偏置。
- `tip_rotation`：上述 C 的 3×3 旋转部分。

改变模型长度或零位后，需要同步更新这里的参数，再运行 URDF 对照测试；参数表不会自动随模型修改。

## 6. 正运动学核心代码逐句讲解

文件：`src/ghost_sim/kinematics/forward.py`。

```python
"""Standard DH forward kinematics; distances in metres, angles in radians."""
import json
from pathlib import Path
import numpy as np
from ghost_sim.paths import CONFIG


def load_parameters(path=CONFIG / 'arm_dh.json'):
    """Read the explicit DH convention and fixed base/tool transforms."""
    data = json.loads(Path(path).read_text())
    if data['convention'] != 'standard_DH_Rz_Tz_Tx_Rx':
        raise ValueError('Only standard DH is supported.')
    return data


def dh_matrix(a, alpha, d, theta):
    """A = Rz(theta) Tz(d) Tx(a) Rx(alpha)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(positions, parameters=None):
    """Return base_link -> grasp_center, not a world-frame pose."""
    cfg = load_parameters() if parameters is None else parameters
    q = np.asarray(positions, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('Expected four finite joint angles in radians.')
    transform = np.eye(4)
    transform[:3, 3] = cfg['base_translation_m']
    for angle, row in zip(q, cfg['rows']):
        transform = transform @ dh_matrix(
            row['a_m'], row['alpha_rad'], row['d_m'],
            angle + row['theta_offset_rad'])
    tool = np.eye(4)
    tool[:3, :3] = cfg['tip_rotation']
    return transform @ tool
```

| 行号 | 说明 |
| --- | --- |
| 1 | 说明本模块使用标准 DH，单位是米和弧度。 |
| 2 | 导入 JSON 解析库。 |
| 3 | Path 用于读取参数文件。 |
| 4 | NumPy 用于三角函数、数组和矩阵乘法。 |
| 5 | 导入项目实际 config 目录，不写死项目绝对路径。 |
| 8 | 定义参数读取函数，默认读取 arm_dh.json，也可以传入另一个文件。 |
| 9 | 函数文档：参数中包含 DH 约定和固定变换。 |
| 10 | 先把 path 转成 Path，再读文本，最后解析成 Python 字典。 |
| 11 | 检查约定是否是标准 Rz→Tz→Tx→Rx。 |
| 12 | 不匹配就报错，避免误把改进 DH 表代入标准矩阵。 |
| 13 | 返回参数字典。当前这是受控项目配置，尚未做完整 JSON schema 校验。 |
| 16 | 定义单节 DH 矩阵函数，参数顺序为 a、alpha、d、theta。 |
| 17 | 注明四次旋转/平移的矩阵乘法顺序。 |
| 18 | 计算 theta 的余弦和正弦，保存到 ct、st。 |
| 19 | 计算 alpha 的余弦和正弦，保存到 ca、sa。 |
| 20 | 开始构造 4×4 NumPy 数组。 |
| 21 | 矩阵第一行；前三项属于旋转，最后一项 a*cos(theta) 是 X 平移。 |
| 22 | 矩阵第二行；a*sin(theta) 是 Y 平移。 |
| 23 | 矩阵第三行；Z 平移是 d。 |
| 24 | 齐次矩阵最后一行为 [0,0,0,1]，使旋转和平移能统一连乘。 |
| 25 | 结束数组并返回矩阵。 |
| 28 | 定义四轴正运动学函数；可传入已加载参数以避免重复读文件。 |
| 29 | 明确返回 base_link 到 grasp_center 的变换，不是世界坐标。 |
| 30 | 未传参数时加载默认文件，否则复用传入字典。 |
| 31 | 把关节角转换成浮点数组，兼容列表等输入。 |
| 32 | 要求形状严格为一维四元素，并且全部为有限数；拒绝 NaN、Inf 和错误数量。 |
| 33 | 输入不合法时抛出 ValueError。这里不检查物理限位，正运动学可以数学上计算限位外的角度，但不代表机械臂能执行。 |
| 34 | 先建立 4×4 单位矩阵。 |
| 35 | 给最后一列的前三项填入固定基座平移，得到 T_base,0。 |
| 36 | 将每个实际角度与对应 DH 参数行配对。 |
| 37 | 用 @ 做矩阵乘法，并把这一节结果累乘到已有变换右侧。不能用 *，后者是逐元素乘法。 |
| 38 | 从配置取该节的 a、alpha、d。 |
| 39 | theta=物理关节角+零位偏置，再完成单节矩阵计算。 |
| 40 | 建立工具固定变换的单位矩阵。 |
| 41 | 将左上角设为 C 的旋转部分；平移保持零，因为工具长度已计入 DH 第 4 行。 |
| 42 | 乘上固定工具换轴，得到最终矩阵。[:3,3] 是位置，[:3,:3] 是方向。 |

## 7. 命令行程序 fk_cli.py 如何验证

文件：`src/ghost_sim/kinematics/fk_cli.py`。它提供两条只读路径：

```bash
./ghost.sh fk --joints 0 -0.8 -0.6 1.4
./ghost.sh fk --live
```

第一条只算输入角度；第二条读取正在运行的 Gazebo 数据，不发运动命令，因此不会覆盖你在滑块中设置的姿态。

### pose_matrix(transform)

1. `t,q=...`：取 ROS Transform 中的位置与四元数。
2. 将 `[x,y,z,w]` 四元数归一化。
3. `matrix=np.eye(4)`：准备齐次矩阵。
4. 按四元数旋转公式填写左上 3×3；不把欧拉角当四元数使用。
5. 将 t.x/t.y/t.z 填进平移列并返回。

### live_sample()

1. 函数内部导入 rclpy 和消息类型，离线计算不需要建立 ROS 节点。
2. 创建只读检查节点与 states、poses 两个缓存字典。
3. 将仿真时间戳换成整数纳秒，避免浮点时间比较误差。
4. joints 回调把消息中的关节名称与位置组成字典，以时间戳为键缓存；超过 2000 项移除最旧插入项。
5. physical 回调只保存 `ghost/base_link` 和 `ghost/grasp_center` 的实际位姿；超过 200 个时间戳则清理旧项。
6. 订阅 `/joint_states` 与 `/simulation/ground_truth/poses`，不把 `/tf` 中 URDF 推算结果当 Gazebo 真值。
7. 从 DH 表提取四个关节名字，设置 15 秒等待截止时间。
8. `spin_once` 处理 ROS 回调。
9. `states.keys() & poses.keys()` 找出时间戳完全相同的数据；优先检查最新一组，避免机械臂运动时比较不同时间的状态。
10. 检查两端位姿与四个关节都存在，且两端位姿具有相同父坐标系。
11. `inv(T_model,base) @ T_model,tip` 把两个物理位姿转换成 base_link 下的末端实际位姿。这样球体是否移动或转动不影响局部比较。
12. 返回四角度、实际矩阵和时间戳；超时则报告缺少同步数据，最后清理节点。

### main()

1. 使用互斥参数组，要求 `--joints` 与 `--live` 二选一。
2. `--joints` 接收四个浮点数；`--live` 调用实时取样。
3. `forward_kinematics(q)` 计算 DH 结果。
4. 输出父子坐标系、四个角度、位置和完整矩阵。
5. 实时模式计算位置欧氏距离 `norm(p_DH-p_Gazebo)`。
6. 姿态误差使用 `acos((trace(R_DH.T @ R_Gazebo)-1)/2)`；clip 到 [-1,1] 防止舍入误差使 acos 无定义。
7. 位置误差小于 1e-4 m 且姿态误差小于 1e-3 rad 才标记通过。
8. 不通过时退出码为 1，便于脚本验收。

`--live` 检查的是读取到的同一仿真时刻，不是等待机械臂运动到某个目标。

## 8. 怎样保证不是“自己验证自己”

新增 `tests/unit/test_forward_kinematics.py` 使用独立路径：

- 从生成的 URDF 解析 parent/child/origin/axis，沿实际连杆树计算末端变换。
- 旋转用轴角 Rodrigues 公式，固定姿态用 Rz×Ry×Rx，不使用 DH 表。
- 检查零位的手算位置和单位旋转矩阵。
- 检查 ready 姿态的末端朝向。
- 用固定随机种子在四轴限位内产生 200 组角度，逐一比较完整 4×4 矩阵。
- 对少角度、多角度、NaN、Inf 检查拒绝行为。

原有 17 项测试加本次 4 项，共 21 项通过。矩阵比较容差为 1e-12，属于软件几何一致性检查，不代表真实机械臂能达到这个精度。

Gazebo 当前姿态另做了实际反馈验证，结果保存在 `runtime/reports/fk_gazebo_check.json`。本次样本位置误差约 6.3e-8 m、姿态误差约 5.9e-7 rad；这是当前理想仿真的一次样本，不是实机定位精度或全部姿态的物理误差保证。

## 9. 操作示例

在新项目中：

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh fk --joints 0 0 0 0
./ghost.sh fk --joints 0 -0.8 -0.6 1.4
./ghost.sh fk --live
./ghost.sh test
```

仿真未启动时，离线角度计算仍可运行；实时验证需要 `./ghost.sh start gui:=true` 已运行并正常发布状态。`fk` 通过统一入口加载新项目的 src 路径，不会调用旧项目同名包。

下一阶段才是逆运动学：给定期望抓取中心位置和俯仰角，求 q1～q4，并处理多解、限位、不可达和碰撞。

## 解析逆运动学（新增）

已实现位置＋俯仰角解析逆解，包括双肘分支、限位、奇异提示与参考姿态选解。运行 `./ghost.sh ik 0.024908286 0 -0.427268905 --pitch 0 --live` 读取当前角度选解；省略 `--live` 可离线计算。命令只计算，不运动，尚未做尾臂碰撞检测。详见 [逆运动学逐句讲解](逆运动学逐句讲解.md)。
