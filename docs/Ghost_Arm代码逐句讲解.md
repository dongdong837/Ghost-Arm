# Ghost Arm 代码逐句讲解（当前版）

更新日期：2026-10-07。项目：`/home/ubuntu/ghost_arm`。本篇与当前源文件逐句对照，默认抓取末端角度为 **−60°**，支持 **−60°～0°**。

## 先知道现在能做什么

已实现球形幽灵与四轴尾臂、两指夹爪、DH 正运动学、解析逆运动学、球体间隙检查、地面木块摩擦抓取、底部 RGB-D 测量、50 Hz 实测关节反馈，以及原有飞行/三维导航与大模型入口。抓取目前使用仿真物块位姿进行对准，**不是视觉识别后的自动抓取闭环**；球体路径检查也不是完整的机械臂环境避障规划。

`/home/ubuntu/ghost_sim` 是保留的原项目，本次讲解只对应 `ghost_arm`。内部 Python 包名仍为 `ghost_sim`，只是模块名。两个项目共用 ROS 域、Gazebo 分区及启动锁，不要同时运行。

## 最少运行指令

终端一保持运行：

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh start gui:=false
```

终端二按需打开 RViz（或在终端一用 `gui:=true` 打开 Gazebo GUI）：

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh view
```

终端三执行抓取，等完成后再运行松开：

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh grasp pick --pitch-deg -60
./ghost.sh grasp release
```

`gui:=false` 仍运行物理后台和传感器，只是不打开 Gazebo 窗口。需要点击测量再用 `./ghost.sh camera`。不要同时运行多个运动命令或一边自动抓取一边拖动关节滑块。

## 先分清四个概念

| 名称 | 在项目中的作用 |
| --- | --- |
| Gazebo 后台 | 计算关节、碰撞、摩擦、木块落地并生成模拟图像 |
| ROS 节点与桥 | 求解运动学、执行流程、把控制/反馈消息在两套通信中转换 |
| RViz | 显示 ROS 模型、TF、图像、点云和路径，不负责物理抓取 |
| OpenCV 相机窗口 | 将同时间戳 RGB-D 拼在一起，点击测量表面点 |

`base_link` 是球体坐标；世界坐标记录小球和木块在房间中的位置；光学坐标用来从像素反投影。`grasp_center` 是夹爪中的数学参考点，没有可见几何实体。

六关节顺序为：尾根偏航、肩、肘、腕、左指、右指。四轴单位是弧度，两指单位是米。**`grasp --pitch-deg -60` 使用度；`ik --pitch` 和 `fk --joints` 使用弧度。** 末端俯仰角是肩、肘、腕之和，并不是只把腕转到 −60°。

## 消息如何形成闭环

```text
命令 → grasp_demo / ik_execute → 关节目标与飞行速度
                                     ↓
                       ROS/Gazebo 桥 → 物理控制器
                                     ↓
                         Gazebo 碰撞、摩擦、运动
                                     ↓
             50 Hz JointFeedback + 模型/木块 PosePublisher
                                     ↓
                  ROS 测量反馈 → 下一步控制 / 抓取验证

底部 RGB + Depth + CameraInfo → 同步缓存 → 图像与表面点测量
```

手指位置到达预期范围只是初步接触判断，最终还需看木块是否实际抬升。持久向下力让木块有重量；没有把木块固定到夹爪，也没有用修改物块坐标制造抓取动画。

## 本文读法

每章包含**当前完整源码**和**原文件行号对应的中文说明**。表格一行通常解释一条语句；跨行的函数调用、列表或字典用行号范围合并解释。一行里有多个分号连接的操作会在说明中依次展开。空行仅分隔代码，不执行。源码块可与真实文件比较；不要把表格行号复制进程序。

从零写可按“结构表 → 木块 → 运动学 → 碰撞 → 抓取 → 相机 → 发布插件 → 启动脚本”的顺序阅读。已有运行问题可直接从抓取主程序看起。

## 文件导航

1. [抓取程序：从命令到实际抬升](#module-1)：`src/ghost_sim/control/grasp_demo.py`
2. [球体避碰：为什么末端正确仍可能穿模](#module-2)：`src/ghost_sim/kinematics/body_collision.py`
3. [RGB-D 同步缓存：解决重启后画面冻结](#module-3)：`src/ghost_sim/perception/rgbd_sync.py`
4. [深度相机窗口：显示、点击测量与过期保护](#module-4)：`src/ghost_sim/perception/depth_camera.py`
5. [50 Hz 关节反馈插件：物理不降频，只减少消息](#module-5)：`src/gazebo_plugins/joint_feedback.cc`
6. [机械结构、指间摩擦和相机安装](#module-6)：`src/ghost_sim/simulation/tail_structure.py`
7. [地面木块：质量、重力与真实碰撞](#module-7)：`src/ghost_sim/simulation/wood_block.py`
8. [单独执行坐标 IK：与抓取流程共用的控制基础](#module-8)：`src/ghost_sim/control/ik_execute.py`
9. [手动预设姿态：与自动抓取有什么不同](#module-9)：`src/ghost_sim/control/arm_pose.py`
10. [启动文件：后台、界面与桥接](#module-10)：`launch/sim.launch.py`
11. [统一命令入口](#module-11)：`ghost.sh`
12. [运行环境](#module-12)：`scripts/env.sh`
13. [启动与防止重复仿真](#module-13)：`scripts/start_sim.sh`
14. [自动编译插件](#module-14)：`scripts/build_plugins.sh`
15. [CMake 如何生成共享库](#module-15)：`src/gazebo_plugins/CMakeLists.txt`
16. [只打开 RViz](#module-16)：`scripts/view_ghost.sh`

<a id="module-1"></a>

## 1. 抓取程序：从命令到实际抬升

源文件：[src/ghost_sim/control/grasp_demo.py](../src/ghost_sim/control/grasp_demo.py)（246 行）。

这是动作流程的主文件。先读当前姿态，再求逆解、检查路径，最后逐步发送速度和关节目标。它使用 Gazebo 物块位姿对准，不使用相机识别结果。代码中的 `q[:4]` 是四个旋转关节，`q[4:]` 是两根滑动手指；前者用弧度，后者用米。

<!-- source: src/ghost_sim/control/grasp_demo.py sha256: 5f4b396eb0792198044f499a9db880e0499448039cfec33263690956a7411955 -->

```python
"""Fixed-scene, contact-friction pick / release demonstration (not visual grasp planning)."""
import argparse
import math
import fcntl
import json
import time
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from ghost_sim.control.ik_execute import interpolation
from ghost_sim.kinematics.fk_cli import pose_matrix
from ghost_sim.kinematics.forward import forward_kinematics
from ghost_sim.kinematics.inverse import inverse_kinematics
from ghost_sim.kinematics.body_collision import select_body_safe_solution, check_body_path, arm_body_clearance
from ghost_sim.paths import REPORTS
from ghost_sim.simulation.tail_structure import JOINTS
from ghost_sim.simulation.wood_block import SIZE


class Demo:
    def __init__(self, pitch_degrees=-60.):
        self.pitch = math.radians(pitch_degrees)
        self.node = rclpy.create_node('wood_grasp_demo')
        self.q = self.base = self.block = None
        self.stamp = None
        self.fresh = {}
        self.min_body_clearance = float("inf")
        self.pubs = [self.node.create_publisher(Float64, '/ghost/arm/'+name+'/cmd_pos', 10) for name in JOINTS]
        self.flight = self.node.create_publisher(Twist, '/ghost/cmd_vel', 1)
        self.subs = [
            self.node.create_subscription(JointState, '/joint_states', self.joints, 10),
            self.node.create_subscription(TFMessage, '/simulation/ground_truth/poses', self.robot, 10),
            self.node.create_subscription(TFMessage, '/model/wood_block/pose', self.wood, 10)]

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        stamp = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
        if all(name in values for name in JOINTS) and stamp != self.stamp:
            self.q = np.array([values[name] for name in JOINTS])
            self.stamp = stamp
            self.fresh['joints'] = time.monotonic()

    def robot(self, message):
        for pose in message.transforms:
            if pose.child_frame_id == 'ghost':
                self.base = pose_matrix(pose.transform)
                self.fresh['base'] = time.monotonic()

    def wood(self, message):
        for pose in message.transforms:
            if pose.child_frame_id == 'wood_block':
                self.block = pose_matrix(pose.transform)
                self.fresh['block'] = time.monotonic()

    def tick(self, check=True):
        rclpy.spin_once(self.node, timeout_sec=.02)
        if check:
            stale=[f'{name}: {time.monotonic()-self.fresh[name]:.1f}s' if name in self.fresh else f'{name}: missing'
                   for name in ('joints','base','block')
                   if name not in self.fresh or time.monotonic()-self.fresh[name]>2]
            if stale:raise RuntimeError('反馈中断，已停止动作（'+', '.join(stale)+'）。检查仿真是否暂停或卡顿。')
        if check and self.q is not None:
            clearance, link = arm_body_clearance(self.q)
            self.min_body_clearance = min(self.min_body_clearance, clearance)
            if clearance < .002:
                raise RuntimeError(f'实际机械臂靠近球体：{link}，间隙 {clearance*1000:.1f} mm。')
        time.sleep(.01)

    def ready(self):
        end = time.monotonic()+20
        while len(self.fresh) != 3 or not all(p.get_subscription_count() for p in self.pubs):
            self.tick(False)
            if time.monotonic() > end:
                raise RuntimeError('缺少仿真反馈或控制桥；请用更新后的场景重启仿真。')
        client = self.node.create_client(Trigger, '/ghost/cancel_navigation')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('导航取消服务未就绪。')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3)
        if future.result() is None:
            raise RuntimeError('无法取消原导航任务。')

    def send(self, q):
        for pub, value in zip(self.pubs, q):
            pub.publish(Float64(data=float(value)))

    def joint_move(self, target, seconds=4, contact=False):
        start = self.q.copy()
        check_body_path(start, target)
        seconds = max(seconds, 1.5*float(np.max(np.abs(target[:4]-start[:4])))/.35)
        initial = self.stamp
        deadline = time.monotonic()+seconds*20+20
        while time.monotonic() < deadline:
            self.tick()
            elapsed = self.stamp-initial
            if elapsed < 0:
                raise RuntimeError('仿真时间重置，请重新开始。')
            self.send(interpolation(start, target, elapsed, seconds))
            if elapsed >= seconds:
                if contact:  # Fingers intentionally cannot reach zero through a solid block.
                    return
                if np.max(np.abs(self.q[:4]-target[:4])) < .015 and np.max(np.abs(self.q[4:]-target[4:])) < .002:
                    return
        raise RuntimeError('关节运动超时。')

    def fly(self, target, speed=.06):
        deadline = time.monotonic()+180
        stable = 0
        try:
            while time.monotonic() < deadline:
                self.tick()
                error = np.array(target)-self.base[:3, 3]
                if np.linalg.norm(error) < .0015:
                    stable += 1
                    if stable >= 10:
                        return
                else:
                    stable = 0
                command = np.clip(error*1.5, -speed, speed)
                message = Twist()
                message.linear.x, message.linear.y, message.linear.z = map(float, command)
                self.flight.publish(message)
            raise RuntimeError('飞行超时，可能被障碍物阻挡。')
        finally:
            self.flight.publish(Twist())

    def wait(self, seconds):
        start = self.stamp
        end = time.monotonic()+seconds*20+10
        while self.stamp-start < seconds:
            self.tick()
            if self.stamp < start or time.monotonic() > end:
                raise RuntimeError('等待时仿真时间停止或重置。')

    def pick(self):
        block_start = self.block[:3, 3].copy()
        world_size = np.abs(self.block[:3, :3]) @ np.array(SIZE)
        if np.linalg.norm(block_start[:2]-[.1, 0]) > .30 or abs(block_start[2]-world_size[2]/2) > .008:
            raise RuntimeError('演示需要木块在初始地面位置附近；请先放下或重启场景。')
        if np.linalg.norm(self.base[:2, 3]) > .5 or self.base[2, 3] < .40:
            raise RuntimeError('请先让幽灵回到原点附近、飞行高度至少 0.40 米，再运行演示。')
        aligned = np.all(np.max(np.abs(self.block[:3, :3]),axis=0) > .995)
        if np.linalg.norm(self.base[:3, :3]-np.eye(3)) > .05 or not aligned:
            raise RuntimeError('演示需要小球朝向接近初始姿态，木块以一个平面落地并与世界轴基本对齐。')
        expected_finger = (world_size[1]-.02)/2
        opening = min(.03, expected_finger+.015)
        if not 0 < expected_finger < .025:
            raise RuntimeError('当前木块宽度不适合夹爪开口。')
        # Solve the same ready geometry with the existing analytical IK.
        tip = np.array([0., 0., -.46])
        solution = select_body_safe_solution(inverse_kinematics(tip, self.pitch, self.q[:4]), self.q[:4])
        angles = np.array(solution['selected']['joint_angles_rad'])
        print('1/5 张开夹爪，机械臂进入抓取姿态', flush=True)
        clearance = self.base[:3, 3].copy()
        clearance[2] = max(.8, clearance[2])
        # Release ground-level contact before rising to the arm-reconfiguration height.
        self.joint_move(np.r_[self.q[:4], opening, opening], 3)
        self.fly(clearance)
        self.joint_move(np.r_[angles, opening, opening])
        actual_tip = forward_kinematics(angles)[:3, 3]
        approach = np.r_[block_start[:2]-actual_tip[:2], max(.8, self.base[2, 3])]
        print('2/5 对准地面木块', flush=True)
        self.fly(approach)
        # Centre 10 mm above the block centre keeps fingertips 10 mm off the floor.
        pickup = approach.copy()
        # Rotating the wrist changes the lowest finger corner: preserve 8 mm floor clearance.
        lowest_offset = .03*math.cos(self.pitch)+.0225*abs(math.sin(self.pitch))
        grasp_height = max(block_start[2]+.01, .008+lowest_offset)
        pickup[2] = grasp_height-actual_tip[2]
        print('3/5 缓慢下降，夹爪包住木块', flush=True)
        self.fly(pickup)
        print('4/5 闭合夹爪，通过接触摩擦夹持', flush=True)
        self.joint_move(np.r_[angles, 0., 0.], 3, contact=True)
        self.wait(1)
        if not all(abs(value-expected_finger) < .004 for value in self.q[4:]):
            raise RuntimeError('两侧夹爪未形成预期夹持，停止抬升。')
        print('5/5 上升 25 厘米，并验证木块实际高度', flush=True)
        lifted = pickup.copy(); lifted[2] += .25
        self.fly(lifted, .035)
        self.wait(3)
        height = float(self.block[2, 3]-block_start[2])
        tip_world = self.base @ forward_kinematics(self.q[:4])
        distance = float(np.linalg.norm(tip_world[:3, 3]-self.block[:3, 3]))
        report = {'method': 'physical_contact_friction', 'vision_used': False, 'pitch_degrees': math.degrees(self.pitch),
                  'block_start_m': block_start.tolist(), 'block_end_m': self.block[:3, 3].tolist(),
                  'lift_height_m': height, 'grasp_distance_m': distance,
                  'finger_positions_m': self.q[4:].tolist(),
                  'min_measured_body_clearance_m': self.min_body_clearance,
                  'body_path_check': solution['body_collision_check'],
                  'passed': height > .20 and distance < .04 and self.min_body_clearance >= .002}
        (REPORTS/'wood_grasp_check.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        if not report['passed']:
            raise RuntimeError('木块未稳定抬起，请查看验证报告。')

    def release(self):
        print('张开夹爪，木块将在重力作用下落地。', flush=True)
        self.joint_move(np.r_[self.q[:4], .025, .025], 3)
        self.wait(3)
        height = float(self.block[2, 3])
        print(f'木块中心离地高度：{height:.4f} m', flush=True)
        support_height = float(np.abs(self.block[2, :3]) @ (np.array(SIZE)/2))
        report = {'block_center_height_m': height, 'support_height_m': support_height,
                  'passed': abs(height-support_height) < .008}
        (REPORTS/'wood_release_check.json').write_text(json.dumps(report, indent=2)+'\n')
        if not report['passed']:
            raise RuntimeError('木块未在预期时间内平稳落地。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['pick', 'release'], nargs='?', default='pick')
    parser.add_argument('--pitch-deg',type=float,default=-60.,help='末端俯仰角（度），当前地面演示支持 -60 到 0')
    args = parser.parse_args()
    if not math.isfinite(args.pitch_deg) or not -60 <= args.pitch_deg <= 0:
        parser.error('当前地面抓取演示的俯仰角范围是 -60 到 0 度。')
    with open('/tmp/ghost_arm_ik.lock', 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.exit(1, '已有机械臂动作在执行。\n')
        rclpy.init()
        demo = Demo(args.pitch_deg)
        try:
            demo.ready()
            getattr(demo, args.action)()
        except (RuntimeError, ValueError, KeyboardInterrupt) as error:
            # Hold arm feedback; keep the existing finger force setpoint to avoid dropping a load.
            if demo.q is not None:
                for pub, value in zip(demo.pubs[:4], demo.q[:4]):
                    pub.publish(Float64(data=float(value)))
            parser.exit(1, f'动作停止：{error}\n')
        finally:
            for _ in range(3):
                demo.flight.publish(Twist())
                time.sleep(.02)
            demo.node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 模块说明：固定场景、接触摩擦抓取演示，不是通用视觉抓取规划器。 |
| 2 | argparse 读取命令行的 pick/release 和角度参数。 |
| 3 | math 提供度转弧度、三角函数与有限数值检查。 |
| 4 | fcntl 提供进程文件锁，防止两个 IK/抓取命令同时执行。 |
| 5 | json 将验证结果保存为可读报告。 |
| 6 | time 提供真实时间计时和短暂让出 CPU 的 sleep。 |
| 7 | NumPy 用于关节数组、向量、矩阵和距离计算；别名为 np。 |
| 8 | rclpy 是 ROS 2 的 Python 客户端库。 |
| 9 | Twist 保存小球的线速度、角速度指令。 |
| 10 | JointState 保存各关节名称与测量位置。 |
| 11 | Float64 用于每个关节的标量位置目标。 |
| 12 | Trigger 是不带业务输入、返回成功状态和说明的服务类型。 |
| 13 | TFMessage 是由多个坐标变换组成的 ROS 消息。 |
| 14 | 复用已有三次平滑插值函数 interpolation。 |
| 15 | pose_matrix 把平移和四元数转成 4×4 位姿矩阵。 |
| 16 | forward_kinematics 根据关节角计算抓取中心相对球体的位置。 |
| 17 | inverse_kinematics 根据目标位置与俯仰角求数学逆解。 |
| 18 | 导入逆解路径筛选、路径检查、当前球体间隙计算三个函数。 |
| 19 | REPORTS 是项目 runtime/reports 的绝对路径。 |
| 20 | JOINTS 指定六个关节的统一排列顺序。 |
| 21 | SIZE 是木块自身坐标系中的长宽高。 |
| 24 | Demo 类把节点、反馈缓存和抓取步骤放在同一对象中。 |
| 25 | 构造函数默认末端角度为 -60 度，不是腕关节本身的角度。 |
| 26 | 先把角度转为弧度；后面数学函数和 IK 都使用弧度。 |
| 27 | 创建名为 wood_grasp_demo 的 ROS 节点。 |
| 28 | 尚未收到关节、小球、木块反馈时，三者都设为 None。 |
| 29 | 保存最近一次关节消息的仿真时间，初始没有数据。 |
| 30 | fresh 记录三路反馈最近到达的真实时间。 |
| 31 | 最小间隙从正无穷开始，后续不断取更小的实测值。 |
| 32 | 列表推导式为六个关节各创建一个发布器；队列深度为 10。 |
| 33 | 小球速度发布到 /ghost/cmd_vel，队列只保留一个位置以减少旧命令积压。 |
| 34 | 保存订阅对象，避免订阅关系提前释放。 |
| 35 | 订阅实测关节状态，消息到达后执行 self.joints。 |
| 36 | 订阅 Gazebo 小球位姿，消息交给 self.robot。 |
| 37 | 订阅木块位姿，消息交给 self.wood；结束订阅列表。 |
| 39 | 关节状态回调；message 由 ROS 传入。 |
| 40 | zip 将关节名和位置配成对，再转成按名称索引的字典。 |
| 41 | 把时间戳的秒和纳秒合成浮点秒。 |
| 42 | 只有六个关节都有数据，且仿真时间确实改变，才更新状态，避免重复旧消息假装新反馈。 |
| 43 | 按 JOINTS 顺序构造数组，不假设消息内部顺序固定。 |
| 44 | 记录本次测量的仿真时间。 |
| 45 | 用单调真实时间标记反馈到达时刻，不受系统时钟校正影响。 |
| 47 | 处理一条包含多个变换的小球位姿消息。 |
| 48 | 逐个取出变换。 |
| 49 | 只选整个 ghost 模型的位姿，跳过其各个 link。 |
| 50 | 将模型在世界中的姿态存成 4×4 矩阵。 |
| 51 | 更新小球位姿的新鲜度。 |
| 53 | 处理木块的位姿消息。 |
| 54 | 遍历消息中的所有变换。 |
| 55 | 只选整个 wood_block 模型。 |
| 56 | 保存木块相对世界的位姿矩阵。 |
| 57 | 更新木块反馈到达时间。 |
| 59 | 每次控制循环调用 tick；check=False 用于启动等待阶段。 |
| 60 | 处理一次 ROS 回调，最多等 0.02 秒；这不是把整个程序固定为 50 Hz。 |
| 61 | 只有启用检查时才检查反馈是否过期。 |
| 62 | 构造失效项的说明：已有数据写出多久没更新，否则写 missing。 |
| 63 | 需要检查关节、小球、木块三路反馈。 |
| 64 | 只把缺失或超过 2 秒未到达的数据加入 stale 列表。 |
| 65 | 发现失效项就抛异常，并把具体是哪一路中断写进报错。 |
| 66 | 已获得关节数组时，才能检查实际机械姿态。 |
| 67 | 计算所有运动连杆碰撞盒与球体的最小间隙，以及对应连杆。 |
| 68 | 保存从动作开始到现在的最小间隙，供最终报告使用。 |
| 69 | 实际间隙小于 2 毫米时停止；计划路径要求更大的 5 毫米余量。 |
| 70 | 报错时把米乘 1000 转成毫米显示。 |
| 71 | 休眠 0.01 秒让出 CPU，避免忙循环。 |
| 73 | ready 负责等待通信就绪，并取消先前导航。 |
| 74 | 启动等待最多使用 20 秒真实时间。 |
| 75 | 三路反馈尚未齐全，或任意关节命令还没有订阅者时继续等。 |
| 76 | 等待阶段允许尚未获得全部反馈，因此暂不执行超时/碰撞检查。 |
| 77 | 检查是否超过启动等待期限。 |
| 78 | 超时退出，并提示需要使用包含木块和桥接的当前仿真。 |
| 79 | 创建取消导航的服务客户端。 |
| 80 | 最多等待该服务 3 秒。 |
| 81 | 找不到服务就停止，避免与旧导航同时控制小球。 |
| 82 | 异步发送取消请求，得到 future 对象。 |
| 83 | 继续处理 ROS 回调，最多等待 3 秒让请求完成。 |
| 84 | 检查是否收到了返回对象；此处没有额外检查返回对象的 success 字段。 |
| 85 | 没收到响应则终止动作。 |
| 87 | send 接收按六关节顺序排列的目标数组。 |
| 88 | 逐个把发布器与对应目标配对。 |
| 89 | 将 NumPy 数值转换为 Python float，再封装为 Float64 发布。 |
| 91 | 执行关节插值；contact=True 表示允许手指被物块挡住而达不到零位置。 |
| 92 | 复制当前测量数组，避免回调更新时改变插值起点。 |
| 93 | 在发送运动命令前，检查起点到终点整段关节空间路径与球体的间隙。 |
| 94 | smoothstep 最大斜率是 1.5，因此用最大角度变化计算时长，把四个转动关节的名义目标速度限制到 0.35 rad/s；这不是手指速度上限。 |
| 95 | 用当前仿真时间作为动作起点。 |
| 96 | 真实时间兜底期限较宽，允许虚拟机以低于实时的速度运行。 |
| 97 | 在真实时间期限内持续执行动作。 |
| 98 | 接收最新反馈并执行新鲜度和球体间隙检查。 |
| 99 | 用仿真时间差计算插值进度，不把虚拟机卡顿误算成已经完成运动。 |
| 100 | 负时间差说明仿真被重置。 |
| 101 | 重置后旧轨迹已经无效，停止本次执行。 |
| 102 | 计算平滑过渡的六关节目标并发布，而不是直接跳到终点。 |
| 103 | 只有计划的仿真时长走完才检查是否结束。 |
| 104 | 夹紧时允许位置目标被实体木块阻挡。 |
| 105 | contact 模式在时长结束后返回；真正的夹持判断由 pick 后面的手指检查和抬升验证完成。 |
| 106 | 普通运动要求四轴误差均小于 0.015 rad、两指误差均小于 0.002 m。 |
| 107 | 满足误差要求则结束关节运动。 |
| 108 | 到达真实时间期限仍未完成则报错。 |
| 110 | 飞行到世界坐标 target；默认每个方向的速度限幅为 0.06 m/s。 |
| 111 | 飞行最多等待 180 秒真实时间。 |
| 112 | stable 统计连续接近目标的控制循环次数。 |
| 113 | try/finally 保证正常退出或异常退出都发送停止速度。 |
| 114 | 期限内不断计算位置误差。 |
| 115 | 更新反馈并检查数据是否有效。 |
| 116 | 当前世界位置是 4×4 矩阵最后一列的前三项；目标减当前位置得到误差。 |
| 117 | 位置误差的欧氏长度小于 1.5 毫米时认为这一轮接近目标。 |
| 118 | 连续接近目标计数加一。 |
| 119 | 要求连续 10 个循环满足条件；这是循环次数，不是严格 10 个独立测量帧。 |
| 120 | 满足稳定条件后返回。 |
| 121 | 否则进入未到达分支。 |
| 122 | 把连续计数清零。 |
| 123 | 比例控制：速度=1.5×位置误差，然后分别限幅 XYZ；不保证合速度恰好等于 speed。 |
| 124 | 创建全零的 Twist 消息。 |
| 125 | 只填入三个线速度分量，角速度保持零。 |
| 126 | 发布给现有飞行控制节点，再由它转换本体速度并做基本遇障停止。 |
| 127 | 一直到不了目标时报告可能受到障碍阻挡。 |
| 128 | 不论成功或异常都执行 finally。 |
| 129 | 发送全零 Twist 请求停止，不会直接修改模型位置。 |
| 131 | 等待指定的仿真秒数，并继续维护反馈检查。 |
| 132 | 保存开始等待时的仿真时间。 |
| 133 | 额外设真实时间上限，避免仿真停住后永久等待。 |
| 134 | 仿真时间未经过 seconds 时继续循环。 |
| 135 | 处理新反馈和检查。 |
| 136 | 检测仿真时间回退或真实等待超时。 |
| 137 | 发现异常就结束等待并报错。 |
| 139 | pick 是完整抓取流程的入口。 |
| 140 | 保存木块开始位置，用于最后计算实际抬升量。 |
| 141 | abs(R)×SIZE 得到木块在世界轴上的包围盒尺寸，可处理侧躺后长宽高交换。 |
| 142 | 要求木块处于初始 XY 周围 0.30 米内，并且其中心高度接近当前竖直尺寸的一半，即一个平面落在地面。 |
| 143 | 木块离开演示区或尚未落地时拒绝开始。 |
| 144 | 小球需在世界原点周围 0.5 米内，高度至少 0.40 米。 |
| 145 | 不满足演示起点要求时给出明确提示。 |
| 146 | 旋转矩阵每一列都应有一个绝对值接近 1 的分量，用来检查木块是否基本与世界轴对齐。 |
| 147 | 同时要求小球朝向接近初始姿态；这里的矩阵范数阈值不是直接的角度值。 |
| 148 | 不支持任意滚动/斜放木块，因此不满足方向检查时终止。 |
| 149 | 两指零位内侧间距为 0.02 m，物块宽度减去该间距再除以二，就是预期单指位移。 |
| 150 | 张开目标比夹持位置多 0.015 m，且不超过单指 0.03 m 行程。 |
| 151 | 要求预期夹持位置在可用范围内，保留张开余量。 |
| 152 | 物块太窄或太宽时拒绝抓取。 |
| 153 | 注释说明下面复用现有解析逆运动学；不是新写另一套 IK。 |
| 154 | 抓取中心相对 base_link 的目标固定为球心正下方 0.46 米。 |
| 155 | 先用当前四轴角作为 seed 求所有逆解，再筛选起点到候选解的球体避碰路径；self.pitch 默认为 -60°对应的弧度。 |
| 156 | 取最终选中的四轴关节目标数组。 |
| 157 | 打印第 1 步并立即刷新终端，方便观察程序走到哪里。 |
| 158 | 复制小球当前位置，准备只改变安全调整高度。 |
| 159 | 调整机械臂前，先把小球升到至少 0.80 米。 |
| 160 | 注释说明先解除地面物块接触，避免夹着木块直接升高。 |
| 161 | 保持当前四轴角，两指先张开到 opening；np.r_ 将四轴和两指拼成六元素数组。 |
| 162 | 升到安全调整高度。 |
| 163 | 两指保持张开，四轴运动到选中的抓取姿态。 |
| 164 | 用 FK 重新计算该组目标角的抓取中心位置；这里是目标角的计算结果，不是实测末端反馈。 |
| 165 | 世界木块 XY 减去末端相对球体 XY，得到小球应飞到的 XY；本流程已限制球体朝向接近初始方向。 |
| 166 | 打印对准阶段。 |
| 167 | 飞到木块上方的接近位置。 |
| 168 | 这是垂直抓取时的直观说明；倾斜抓取的实际高度由后面角点公式决定，不恒等于 1 厘米余量。 |
| 169 | 复制接近位置，准备只改下降目标 Z。 |
| 170 | 注释说明腕部倾斜会改变手指最低角点，需要保留 8 毫米离地距离。 |
| 171 | 0.03 是抓取中心到手指下端的轴向长度，0.0225 是手指 X 方向半宽；两者按俯仰角投影到竖直方向。 |
| 172 | 在“比木块中心高 1 cm”和“最低角点离地 8 mm”之间取较高的抓取中心高度。 |
| 173 | 末端世界 Z=球体 Z+末端相对 Z，因此反推出球体应该下降到多高。 |
| 174 | 打印下降阶段。 |
| 175 | 缓慢下降，不直接瞬移到木块。 |
| 176 | 打印夹紧阶段。 |
| 177 | 保持四轴目标，把两指目标设为零；木块会阻止它们完全闭合，控制器通过位置误差产生夹紧力。 |
| 178 | 等待 1 秒仿真时间，让接触稳定。 |
| 179 | 两指测量位置都要接近期望夹持位置，允许误差 4 mm。 |
| 180 | 不满足预期接触位置则不抬升；这不是力传感器检测。 |
| 181 | 打印抬升验证阶段。 |
| 182 | 复制抓取时球体位置，并把目标高度增加 0.25 米。 |
| 183 | 使用每轴 0.035 m/s 的限幅上升，降低滑落风险。 |
| 184 | 抬升后再等待 3 秒仿真时间检查保持情况。 |
| 185 | 用实际木块末高度减起始高度，不把小球上升量当作木块上升量。 |
| 186 | 世界到球体矩阵乘球体到末端 FK，得到当前末端世界位姿。 |
| 187 | 计算末端与木块中心的实际距离。 |
| 188 | 报告说明使用接触摩擦而非视觉闭环，并记录实际配置的角度值。 |
| 189 | 保存木块起始和结束世界坐标。 |
| 190 | 保存实际抬升量与抓取中心距离。 |
| 191 | 保存两根手指的测量位移。 |
| 192 | 保存执行期间测得的最小球体间隙。 |
| 193 | 保存执行前的路径检查信息。 |
| 194 | 同时要求抬升超过 0.20 m、中心距离小于 0.04 m、实际球体间隙至少 2 mm 才算通过。 |
| 195 | 将本次报告以缩进 JSON 写入 runtime/reports/wood_grasp_check.json。 |
| 196 | 把同一报告打印到终端。 |
| 197 | 检查报告中的总通过标志。 |
| 198 | 未通过时明确报错，不把执行完指令等同于抓取成功。 |
| 200 | release 负责张开夹爪并验证落地。 |
| 201 | 提示当前是松开自由落地；不是下降到地面再轻放。 |
| 202 | 保持四轴当前测量角，两指各张开到 0.025 米，总内侧间距约 0.07 米。 |
| 203 | 等待木块运动 3 秒仿真时间。 |
| 204 | 读取木块中心当前世界高度。 |
| 205 | 打印米为单位、四位小数的中心高度。 |
| 206 | 按当前旋转计算包围盒半高度，用来判断直立或侧躺物块的底部是否接近地面。 |
| 207 | 建立落地报告，保存实测中心高度和几何支撑高度。 |
| 208 | 两者差距小于 8 mm 就通过；这里没有额外要求物块速度等于零。 |
| 209 | 保存落地报告 JSON。 |
| 210 | 检查落地结果。 |
| 211 | 未落到预期高度时报告失败。 |
| 214 | 脚本命令行入口函数。 |
| 215 | 创建解析器，使用模块说明作为帮助文本。 |
| 216 | 动作可选 pick/release；省略动作时默认 pick。 |
| 217 | 角度参数是浮点数，默认 -60 度，允许范围在下一段验证。 |
| 218 | 实际解析用户输入。 |
| 219 | 拒绝 NaN/无穷大及 -60～0 度范围外的输入。 |
| 220 | 通过 argparse 输出用法错误并退出。 |
| 221 | 打开与 IK 执行共用的锁文件；with 保证结束时关闭文件并释放锁。 |
| 222 | 尝试加锁。 |
| 223 | LOCK_EX 表示独占锁，LOCK_NB 表示锁被占用时立即失败而非等待。 |
| 224 | 捕获已有控制进程占用锁的情况。 |
| 225 | 提示有动作正在执行，退出码为 1。 |
| 226 | 初始化 ROS 通信。 |
| 227 | 使用用户角度创建 Demo 对象。 |
| 228 | 启动主动作的异常保护区。 |
| 229 | 等待反馈与桥接，并取消旧导航。 |
| 230 | getattr 根据 action 名称选择 demo.pick 或 demo.release 并调用。 |
| 231 | 捕获运行错误、非法目标和 Ctrl+C。 |
| 232 | 注释说明异常时保持手臂，但不主动松开手指，避免掉落负载。 |
| 233 | 只有拿到测量状态后才能发送保持位置请求。 |
| 234 | 只遍历四个转动关节的发布器和当前角度。 |
| 235 | 把当前测量角发送为新的保持目标；手指原目标保持不变。 |
| 236 | 输出停止原因并以失败码退出。 |
| 237 | 无论成功或异常均执行清理。 |
| 238 | 重复发送三次停止速度，增加接收机会。 |
| 239 | 发布全零飞行速度。 |
| 240 | 两次停止消息之间间隔 0.02 秒。 |
| 241 | 销毁 ROS 节点。 |
| 242 | 关闭 ROS 上下文。 |
| 245 | 只有作为程序入口运行时执行下面调用；被 import 时不自动运行。 |
| 246 | 进入 main。 |

<a id="module-2"></a>

## 2. 球体避碰：为什么末端正确仍可能穿模

源文件：[src/ghost_sim/kinematics/body_collision.py](../src/ghost_sim/kinematics/body_collision.py)（78 行）。

这里只检查运动连杆的有向长方体碰撞盒与球体，不检查房间全部障碍，也不执行通用绕障规划。数学 IK 仍列出运动学解，执行阶段才按整段直接关节路径筛选。

<!-- source: src/ghost_sim/kinematics/body_collision.py sha256: 62ff72b0fdfca34833235fbe2c16d13f1cd130ac3da28a036cae67aac033e74a -->

```python
"""Arm box versus body sphere checks in base_link, including sampled joint paths.

Checks the model's collision geometry, not room obstacles or arm-arm collisions.
The fixed mounting parts intentionally join the body and are excluded.
"""
import math
import numpy as np
from ghost_sim.simulation.tail_structure import PARTS

BODY_RADIUS = .28
MARGIN = .005


def arm_body_clearance(joints):
    q = np.asarray(joints, dtype=float)
    if q.shape not in ((4,), (6,)) or not np.isfinite(q).all():
        raise ValueError('碰撞检查需要 4 或 6 个有限关节值。')
    values = dict(zip(('tail_yaw', 'tail_shoulder', 'tail_elbow', 'tail_wrist',
                       'finger_left_slide', 'finger_right_slide'), q))
    frames = {'base_link': np.eye(4)}
    distances = {}
    for parent, child, name, kind, origin, axis, limits, size, centre, mass in PARTS:
        local = np.eye(4)
        local[:3, 3] = origin
        if kind == 'revolute':
            a = np.asarray(axis, dtype=float)
            cross = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            angle = values[name]
            local[:3, :3] = np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
        elif kind == 'prismatic':
            local[:3, 3] += np.asarray(axis)*values.get(name, 0.)
        frame = frames[parent] @ local
        frames[child] = frame
        if size is None or kind == 'fixed':
            continue
        # Transform the sphere centre into the oriented link box's coordinates.
        sphere_in_box = -frame[:3, :3].T @ frame[:3, 3]-np.asarray(centre)
        closest = np.clip(sphere_in_box, -np.asarray(size)/2, np.asarray(size)/2)
        distances[child] = float(np.linalg.norm(sphere_in_box-closest)-BODY_RADIUS)
    link = min(distances, key=distances.get)
    return distances[link], link


def check_body_path(start, target, step=.01):
    """Sample the straight joint-space path (smoothstep follows this same curve).

    This is a bounded-resolution check, not a continuous collision proof.
    """
    start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
    if start.shape != target.shape or start.shape not in ((4,), (6,)):
        raise ValueError('起点和终点必须有相同的 4 或 6 个关节。')
    if not np.isfinite(start).all() or not np.isfinite(target).all():
        raise ValueError('关节值必须有限。')
    count = max(1, math.ceil(float(np.max(np.abs(target-start)))/step))
    minimum, closest = float('inf'), ''
    for t in np.linspace(0., 1., count+1):
        distance, link = arm_body_clearance(start+t*(target-start))
        if distance < minimum:
            minimum, closest = distance, link
        if distance < MARGIN:
            raise ValueError(f'机械臂路径靠近或穿过球体：{link}，间隙 {distance*1000:.1f} mm，要求至少 {MARGIN*1000:.0f} mm。')
    return {'scope': 'arm_boxes_vs_body_sphere', 'min_clearance_m': minimum,
            'closest_link': closest, 'max_joint_sample_step': step, 'samples': count+1}


def select_body_safe_solution(result, start):
    """Filter analytical IK branches by the entire execution path, retaining seed order."""
    reasons = []
    for candidate in result['solutions']:
        try:
            check = check_body_path(start, candidate['joint_angles_rad'])
        except ValueError as error:
            reasons.append(str(error))
            continue
        result['selected'] = candidate
        result['body_collision_check'] = check
        return result
    raise ValueError('没有避开球体的直接关节路径。'+(' '+reasons[0] if reasons else ''))
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 模块说明：检查连杆盒与本体球；固定连接件有意与球体连接，因此排除。三引号内是说明文字，不执行。 |
| 3–5 | 模块说明：检查连杆盒与本体球；固定连接件有意与球体连接，因此排除。三引号内是说明文字，不执行。 |
| 6 | 导入三角函数和向上取整。 |
| 7 | 导入数组、矩阵与向量运算。 |
| 8 | 读取与 SDF/URDF 模型共用的机械结构表，避免另外抄一套连杆长度。 |
| 10 | 球体物理碰撞半径是 0.28 米，外观半径较小，为 0.25 米。 |
| 11 | 计划路径至少保留 0.005 米间隙。 |
| 14 | 输入四个旋转关节，或四轴加两指的六个关节位置。 |
| 15 | 将输入统一为浮点数组。 |
| 16 | 验证形状及每个数是否有限。 |
| 17 | 错误输入不参与几何计算。 |
| 18–19 | 把数组按约定顺序对应到关节名称；省略手指时，后面按零位计算。 |
| 20 | base_link 相对自身的变换是单位矩阵。 |
| 21 | 保存每个运动连杆到球体的间隙。 |
| 22 | 逐项解包机械结构表；origin 是相对父连杆的关节原点，centre 是盒体相对连杆的中心。 |
| 23 | 先用单位矩阵建立本段局部变换。 |
| 24 | 最后一列填入关节原点的固定平移。 |
| 25 | 转动关节需要根据角度生成旋转矩阵。 |
| 26 | 把关节轴变成向量；结构表中的轴已经是单位轴。 |
| 27 | 构造轴向量的反对称叉乘矩阵。 |
| 28 | 取出该关节当前角度。 |
| 29 | 使用 Rodrigues 公式 I+sin(q)K+(1-cos(q))K²，计算绕任意单位轴的旋转。 |
| 30 | 直线滑动关节使用另一个分支。 |
| 31 | 沿关节轴增加位移；四轴输入没有手指值时默认位移为零。 |
| 32 | 父连杆变换乘当前局部变换，得到子连杆相对球体的变换。 |
| 33 | 缓存子连杆变换，供后续连杆继续递推。 |
| 34 | 无几何形状或固定安装件不做球体碰撞检查。 |
| 35 | 跳过间隙计算，但前面已经保存它的坐标变换，后续仍可使用。 |
| 36 | 说明接下来把球心移到碰撞盒的坐标系中。 |
| 37 | 球心在 base_link 原点；逆变换得到 -Rᵀt，再减盒中心偏移 centre。 |
| 38 | 把球心局部坐标分别夹到盒体各轴的半尺寸范围内，得到盒上最近点。 |
| 39 | 球心到盒体的最近距离减球半径；负值代表与球相交，不是精确的穿透深度解算。 |
| 40 | 在各连杆中选择间隙最小的那个。 |
| 41 | 返回最小间隙和连杆名。 |
| 44 | 检查关节空间直线路径，默认最大单轴采样间隔为 0.01。 |
| 45 | 说明 smoothstep 只改变沿路径的快慢，不改变路径形状；离散采样不等于连续无碰撞证明。 |
| 47–48 | 说明 smoothstep 只改变沿路径的快慢，不改变路径形状；离散采样不等于连续无碰撞证明。 |
| 49 | 起点和终点都转换为浮点数组。 |
| 50 | 要求两端维度一致，并且为四轴或六轴。 |
| 51 | 维度不匹配时停止。 |
| 52 | 拒绝任一端出现 NaN 或无穷大。 |
| 53 | 明确报出有限数值要求。 |
| 54 | 按最大单轴变化量除以步长并向上取整；至少检查一个区间。 |
| 55 | 准备记录整个路径的最小间隙。 |
| 56 | 从 t=0 到 t=1 均匀采样，包含起点和终点。 |
| 57 | 线性插值得到这一点的关节数组，再计算球体间隙。 |
| 58 | 判断当前间隙是否刷新全路径最小值。 |
| 59 | 同时记录数值和最接近的连杆。 |
| 60 | 任一点小于 5 mm 计划余量就拒绝。 |
| 61 | 报错包含具体连杆、实际毫米数和要求的毫米数。 |
| 62–63 | 路径通过后返回检查范围、最小间隙、最近连杆和采样信息；不宣称已经检查环境碰撞。 |
| 66 | 在 IK 返回的全部候选中寻找当前起点可直接执行的解。 |
| 67 | 说明保留原来与 seed 接近的排序，但增加整段路径筛选。 |
| 68 | 保存各个失败候选的原因。 |
| 69 | 依次遍历数学可行解。 |
| 70 | 每个候选独立检查，失败不直接结束全部搜索。 |
| 71 | 检查从当前关节位置到该候选的路径。 |
| 72 | 捕获路径穿球或余量不足。 |
| 73 | 保存错误原因供最终诊断。 |
| 74 | 继续尝试下一组逆解。 |
| 75 | 找到通过的候选后替换 selected。 |
| 76 | 附带球体路径检查报告。 |
| 77 | 立即返回第一组通过的解。 |
| 78 | 没有任何候选路径通过时抛错；程序不会自动绕路，也不会强行使用第一组逆解。 |

<a id="module-3"></a>

## 3. RGB-D 同步缓存：解决重启后画面冻结

源文件：[src/ghost_sim/perception/rgbd_sync.py](../src/ghost_sim/perception/rgbd_sync.py)（29 行）。

每一帧必须有同时间戳的 rgb、depth、info 三份消息。仿真重启使时间归零，不能继续让旧时间戳挤掉新帧。

<!-- source: src/ghost_sim/perception/rgbd_sync.py sha256: 7fd9e257fb4ce06a1156b58a47e1d3157a74c8bf321ab4b77ef55d5ec4b04bf1 -->

```python
"""Bounded exact-stamp RGB/depth/info sync that recovers after simulation resets."""
class RGBDSync:
    def __init__(self, capacity=30):
        self.capacity = capacity
        self.frames = {}
        self.last = {}
        self.resets = 0

    def push(self, kind, stamp, message):
        if kind in self.last and stamp < self.last[kind]:
            self.frames.clear()
            self.last.clear()
            self.resets += 1
        self.last[kind] = stamp
        self.frames.setdefault(stamp, {})[kind] = message
        while len(self.frames) > self.capacity:
            del self.frames[next(iter(self.frames))]

    def latest(self):
        ready = [stamp for stamp, frame in self.frames.items()
                 if all(kind in frame for kind in ('rgb', 'depth', 'info'))]
        if not ready:
            return None
        stamp = max(ready)
        frame = self.frames[stamp]
        for old in list(self.frames):
            if old <= stamp:
                del self.frames[old]
        return frame
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 模块说明：精确时间戳配对、有界缓存、支持仿真时间回退。 |
| 2 | 用一个类封装配对状态，便于不启动 ROS 就进行单元测试。 |
| 3 | 缓存容量默认 30 个不同时间戳。 |
| 4 | 保存容量上限。 |
| 5 | frames 将时间戳映射到这一帧已到达的消息字典。 |
| 6 | last 记录每一路最后到达的时间戳。 |
| 7 | resets 记录检测到多少次时间回退，供界面清空旧图。 |
| 9 | push 接收消息类别、整数纳秒时间戳和原始消息。 |
| 10 | 同一路时间戳变小就视作新时间周期；当前实现也会把同一路乱序旧消息视作回退。 |
| 11 | 删除所有尚未配对的旧帧。 |
| 12 | 清空各路时间记录，等待新周期各消息到齐。 |
| 13 | 增加重置计数。 |
| 14 | 记录这一条消息的时间戳。 |
| 15 | 若该帧不存在则建空字典，然后填入 rgb/depth/info 对应消息。 |
| 16 | 缓存超出容量就淘汰旧条目。 |
| 17 | 按字典插入顺序删除最早到达项，避免新周期的小时间戳被反复淘汰。 |
| 19 | latest 返回最新完整的一组消息，没有完整帧时返回 None。 |
| 20–21 | 筛选同时具备三种消息的时间戳；不同时间的 RGB 和深度不能拼在一起。 |
| 22 | 判断是否没有完整帧。 |
| 23 | 没有则继续等待，不伪造配对。 |
| 24 | 在完整帧中选最大时间戳，尽量查看最近画面。 |
| 25 | 取出对应的三路消息。 |
| 26 | 遍历字典键的副本，以便循环内删除缓存。 |
| 27 | 这帧及更早的帧已不再需要。 |
| 28 | 删除已消费或落后的帧。 |
| 29 | 返回这组原始消息，解码交给相机查看器。 |

<a id="module-4"></a>

## 4. 深度相机窗口：显示、点击测量与过期保护

源文件：[src/ghost_sim/perception/depth_camera.py](../src/ghost_sim/perception/depth_camera.py)（122 行）。

彩色图反映材质，深度图反映距离，颜色不应相同。两张图应对应同一时刻与像素。当前相机固定在 base_link 上，计算点击位置只需静态安装外参，不需要世界位姿。

<!-- source: src/ghost_sim/perception/depth_camera.py sha256: d2936593e5f0b772692ddfee590754b4aa1b6b97ff161b628d7d9833a33d4062 -->

```python
"""View synchronized bottom RGB-D; click to measure a surface point in base_link."""
import argparse
import json
import time
import numpy as np
from ghost_sim.perception.depth_geometry import decode_depth, backproject
from ghost_sim.kinematics.fk_cli import pose_matrix
from ghost_sim.paths import REPORTS
from ghost_sim.perception.rgbd_sync import RGBDSync


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import PointStamped
    from tf2_ros import Buffer, TransformException
    from tf2_msgs.msg import TFMessage
    from cv_bridge import CvBridge
    import cv2
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pixel',nargs=2,type=int,metavar=('U','V'),help='measure once without opening a window')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('ghost_bottom_camera')
    buffer=Buffer()
    # The camera is fixed to base_link: only static extrinsics are needed.
    # Avoid queuing world/body TF and stale-time warnings after a world restart.
    def fixed_transforms(message):
        for transform in message.transforms:buffer.set_transform_static(transform,'robot_static_tf')
    listener=node.create_subscription(TFMessage,'/tf_static',fixed_transforms,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    bridge=CvBridge();sync=RGBDSync();view={'frame':None,'received':0,'clicked':None,'text':'Click RGB to measure a surface point'}
    publisher=node.create_publisher(PointStamped,'/ghost/grasp_camera/selected_point',10)
    def key(header):return header.stamp.sec*1000000000+header.stamp.nanosec
    def receive(message,kind):
        previous=sync.resets
        sync.push(kind,key(message.header),message)
        if sync.resets!=previous:
            view.update(frame=None,received=0,clicked=None,text='Simulation restarted; waiting for RGB-D')
            node.get_logger().info('Simulation time reset: cleared RGB-D cache')
    def calibration(message):receive(message,'info')
    subscriptions=[node.create_subscription(Image,'/grasp_camera/'+topic,lambda m,k=k:receive(m,k),qos_profile_sensor_data)
                   for topic,k in [('image','rgb'),('depth_image','depth')]]
    subscriptions.append(node.create_subscription(CameraInfo,'/grasp_camera/camera_info',calibration,qos_profile_sensor_data))
    def measure(u,v):
        if view['frame'] is None or time.monotonic()-view['received']>2:
            raise ValueError('相机数据未就绪或已过期。')
        pair,cam,depth=view['frame']
        if any(abs(d)>1e-10 for d in cam.d):raise ValueError('当前工具只支持已校正、无畸变图像。')
        if cam.width!=depth.shape[1] or cam.height!=depth.shape[0]:raise ValueError('内参与图像尺寸不一致。')
        if pair['rgb'].header.frame_id!=pair['depth'].header.frame_id or cam.header.frame_id!=pair['depth'].header.frame_id:
            raise ValueError('RGB、深度与内参的坐标系不一致。')
        optical=backproject(depth,u,v,cam.k)
        tf=buffer.lookup_transform('base_link',cam.header.frame_id,Time.from_msg(pair['depth'].header.stamp),timeout=Duration(seconds=0))
        point=(pose_matrix(tf.transform)@np.r_[optical,1])[:3]
        result={'pixel':[u,v],'stamp_ns':key(pair['depth'].header),'depth_m':float(depth[v,u]),
                'optical_frame':cam.header.frame_id,'optical_position_m':optical.tolist(),
                'target_frame':'base_link','position_m':point.tolist(),
                'point_type':'visible_surface','motion_executed':False}
        selected=PointStamped();selected.header.stamp=pair['depth'].header.stamp;selected.header.frame_id='base_link'
        selected.point.x,selected.point.y,selected.point.z=map(float,point);publisher.publish(selected)
        (REPORTS/'bottom_camera_point.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        view['clicked']=(u,v);view['text']='base_link XYZ: '+', '.join(f'{p:.3f}' for p in point)+' m'
        return result
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and view['frame'] is not None:
            h,w=view['frame'][2].shape
            if x>=w:return
            try:measure(x,y)
            except (ValueError,TransformException) as error:
                view['text']='Invalid depth or TF; choose another pixel';print(str(error),flush=True)
    deadline=time.monotonic()+20
    window='Ghost Arm - Bottom RGB | Depth (click RGB, Q to quit)'
    next_draw=0.
    try:
        if args.pixel is None:
            cv2.namedWindow(window,cv2.WINDOW_AUTOSIZE);cv2.setMouseCallback(window,click)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            pair=sync.latest()
            if pair is not None:
                cam=pair['info'];depth=decode_depth(pair['depth'])
                view['frame']=(pair,cam,depth);view['received']=time.monotonic()
            if args.pixel is not None:
                if view['frame'] is not None:
                    try:measure(*args.pixel);return
                    except TransformException:
                        pass
                if time.monotonic()>deadline:raise RuntimeError('等待同步图像或相机 TF 超时。')
                continue
            if time.monotonic()<next_draw:
                continue
            next_draw=time.monotonic()+.1
            if view['frame'] is not None:
                pair,cam,depth=view['frame'];rgb=bridge.imgmsg_to_cv2(pair['rgb'],desired_encoding='bgr8').copy()
                valid=np.isfinite(depth)&(depth>0)
                scaled=np.zeros(depth.shape,dtype=np.uint8)
                near,far=(np.percentile(depth[valid],[2,98]) if valid.any() else (0.,1.))
                far=max(float(far),float(near)+.05)
                scaled[valid]=(np.clip((depth[valid]-near)/(far-near),0,1)*255).astype(np.uint8)
                colored=cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO);colored[~valid]=0
                if view['clicked'] is not None:cv2.drawMarker(rgb,view['clicked'],(0,255,0),cv2.MARKER_CROSS,12,1)
                canvas=np.hstack([rgb,colored]);canvas=cv2.copyMakeBorder(canvas,0,50,0,0,cv2.BORDER_CONSTANT)
                stale=time.monotonic()-view['received']>=2
                label=view['text'] if not stale else 'STALE CAMERA - image hidden; waiting for fresh RGB-D'
                if stale:canvas[:depth.shape[0],:]=0
                cv2.putText(canvas,f'Depth color range: {near:.2f} .. {far:.2f} m', (5,depth.shape[0]+40),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                cv2.putText(canvas,label,(5,depth.shape[0]+20),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                cv2.imshow(window,canvas)
            else:
                waiting=np.zeros((290,640,3),dtype=np.uint8)
                cv2.putText(waiting,'Waiting for synchronized RGB-D...', (15,140),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                cv2.imshow(window,waiting)
            if cv2.waitKey(1)&0xff in (ord('q'),27):break
    finally:
        cv2.destroyAllWindows();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明本程序查看底部同步 RGB-D，点击得到 base_link 下可见表面点。 |
| 2 | 解析 --pixel 命令行参数。 |
| 3 | 保存和打印 JSON 测量结果。 |
| 4 | 使用单调真实时间检测图像是否过期。 |
| 5 | 数组、矩阵、有效像素筛选和图像拼接用 NumPy。 |
| 6 | 复用深度解码和相机反投影算法；两者不负责绘制窗口。 |
| 7 | 把 TF 消息转换为用于乘法的位姿矩阵。 |
| 8 | 读取报告目录。 |
| 9 | 导入新写的精确帧同步器。 |
| 12 | 程序入口函数。 |
| 13 | 在运行时导入 ROS 2 客户端。 |
| 14 | Time 用于把消息时间戳传给 TF 查询。 |
| 15 | Duration 用于指定查询等待时长。 |
| 16 | sensor_data 使用适合传感器的 QoS；另外导入静态 TF 所需的持久性配置。 |
| 17 | Image 是图像消息，CameraInfo 是分辨率、内参和畸变信息。 |
| 18 | PointStamped 是带时间戳与坐标系的三维点。 |
| 19 | Buffer 保存坐标变换，TransformException 捕获缺失变换。 |
| 20 | TFMessage 用于接收静态变换数组。 |
| 21 | CvBridge 将 ROS 图像转换成 OpenCV 数组。 |
| 22 | cv2 负责窗口、颜色映射、文字和鼠标事件。 |
| 23 | 创建参数解析器。 |
| 24 | --pixel 后接两个整数 U、V；指定后只测一次，不打开窗口。 |
| 25 | 读取参数。 |
| 26 | 初始化 ROS，并创建独立相机节点。 |
| 27 | 创建 TF 查询缓存。 |
| 28–29 | 说明相机固定安装，只接收静态外参，避免处理不相关的动态世界 TF 和重启后的旧时间警告。 |
| 30 | 静态 TF 消息回调。 |
| 31 | 遍历每个变换并以静态变换方式写入缓存。 |
| 32–33 | 订阅 /tf_static；Transient Local 允许后来启动的查看器收到此前已发布的固定外参。 |
| 34 | 创建图像转换器、同步器和界面状态字典；初始没有图像、点击点或有效到达时间。 |
| 35 | 创建点击三维点的 ROS 发布器；此话题不直接驱动机械臂。 |
| 36 | 将秒与纳秒合成整数纳秒，避免浮点数配对误差。 |
| 37 | 统一处理 RGB、深度或内参消息。 |
| 38 | 先保存同步器重置计数。 |
| 39 | 把消息按类别与时间戳放进缓存。 |
| 40 | 如果 push 检测到时间回退，重置计数会变化。 |
| 41 | 清空旧画面与点击标记，显示正在等待新周期数据。 |
| 42 | 记录一次清缓存日志，方便排查重启。 |
| 43 | 把 CameraInfo 消息也交给同一个同步器，类别标记为 info。 |
| 44–45 | 分别订阅彩色与深度图；lambda 中 k=k 固定本次循环变量，避免两个回调都变成 depth。 |
| 46 | 订阅相机内参，并保留订阅对象。 |
| 47 | measure 测量一个像素，不做目标检测。 |
| 48 | 没有同步帧，或最近帧超过 2 秒未更新，就拒绝测量。 |
| 49 | 明确提示数据缺失/过期。 |
| 50 | 取出三路配对消息、内参和已解码深度数组。 |
| 51 | 要求图像无畸变；该函数没有对畸变图像自动矫正。 |
| 52 | 检查 CameraInfo 宽高与深度数组一致，防止用错内参。 |
| 53–54 | 要求 RGB、深度和 CameraInfo 的 frame_id 相同，否则不能直接当作同一光学坐标系。 |
| 55 | 根据 K 矩阵和像素深度反投影，得到光学坐标系三维点。 |
| 56 | 查询光学坐标系到 base_link 的变换；相机固定安装，静态变换适用于图像时间；等待时长为零，缺失就立即抛异常。 |
| 57 | 把点补成 [x,y,z,1]，乘 4×4 矩阵，再取前三维。 |
| 58 | 结果记录像素、精确时间戳及该像素的深度米数。 |
| 59 | 记录光学坐标系名字和原始光学三维坐标。 |
| 60 | 记录转换后的 base_link 坐标。 |
| 61 | 注明这是可见表面点，且没有执行机械运动；不能直接把它当作物体中心。 |
| 62 | 构建 PointStamped，使用原图时间戳和 base_link 标签。 |
| 63 | 填入三维坐标并发布。 |
| 64 | 把同一结果写入 JSON 报告，保留中文和缩进。 |
| 65 | 输出到终端并立即刷新。 |
| 66 | 缓存点击像素，更新下方坐标提示文本。 |
| 67 | 返回计算结果。 |
| 68 | OpenCV 鼠标事件回调，x/y 是窗口像素位置。 |
| 69 | 只有左键按下且已经有画面时才处理。 |
| 70 | 读取当前图像的高和宽。 |
| 71 | 右半边是伪彩深度图，点击右半边不执行测量。 |
| 72 | 尝试把左半边点击位置传给 measure。 |
| 73 | 捕获无效深度、超出范围或 TF 未就绪等错误。 |
| 74 | 在窗口提示换一个像素，并把具体原因输出到终端。 |
| 75 | 单次 --pixel 测量最多等待 20 秒真实时间。 |
| 76 | 定义窗口标题，提示点击左图、Q 退出。 |
| 77 | 下次允许绘制的时间初始为零。 |
| 78 | 使用 finally 保证释放窗口和节点。 |
| 79 | 只有未指定 --pixel 才需要 GUI。 |
| 80 | 创建原始像素大小的窗口，并注册鼠标回调。 |
| 81 | 只要 ROS 尚未关闭就继续循环。 |
| 82 | 处理一个 ROS 回调，最多阻塞 0.02 秒。 |
| 83 | 尝试取出最新完整同步帧。 |
| 84 | 有完整帧才更新画面缓存。 |
| 85 | 取出 CameraInfo，并把原始深度消息解码成以米为单位的二维数组。 |
| 86 | 保存最新配对，同时记录真实到达时间。 |
| 87 | 有 --pixel 参数时执行单次测量路径。 |
| 88 | 需要先等到一帧完整图像。 |
| 89 | 用两个参数作为 U、V；成功后立即返回。 |
| 90 | 如果只是 TF 还没到，就暂时等待。 |
| 91 | 忽略这一轮的 TF 异常，下一轮继续尝试；不吞掉其他无效深度错误。 |
| 92 | 超过单次测量期限则报错。 |
| 93 | 单次测量模式不进入窗口绘制逻辑。 |
| 94 | 检查是否尚未到下一次绘图时间。 |
| 95 | 未到时间就继续处理消息，不反复重画同一帧。 |
| 96 | 安排 0.1 秒后才能再次绘制，绘图上限约 10 FPS；传感器帧率仍是 5 Hz 仿真时间。 |
| 97 | 已有图像时进入绘图分支。 |
| 98 | 把 ROS RGB 图转换成 BGR8，并复制后用于画十字标记，避免修改原消息数据。 |
| 99 | 有效深度必须是有限且大于零的数。 |
| 100 | 创建 8 位显示用数组，不修改以米为单位的原始 depth。 |
| 101 | 按有效深度的 2% 和 98% 分位数自动选择显示范围；没有有效点时使用 0～1 的占位范围。 |
| 102 | 范围至少相差 0.05 米，避免近乎平面时除零或噪声被无限放大。 |
| 103 | 把有效深度线性映射到 0～255，范围外截断；这一步仅用于颜色显示。 |
| 104 | 使用 TURBO 伪彩表，把无效深度强制设成黑色。 |
| 105 | 已有点击位置时在彩色图上画绿色十字。 |
| 106 | 左右拼接 RGB 和深度图，并在底部加 50 像素文字区。 |
| 107 | 判断最后一组配对图像是否超过 2 秒未更新。 |
| 108 | 正常时显示测量提示，过期时改成 STALE CAMERA 提示。 |
| 109 | 过期时把两幅图遮黑，不把历史画面冒充实时画面。 |
| 110 | 在第二行文字写出当前深度颜色对应的米数范围。 |
| 111 | 在第一行文字写出坐标提示或过期警告。 |
| 112 | 显示处理后的组合图。 |
| 113 | 没有帧时使用另一个分支。 |
| 114 | 创建黑色等待画面。 |
| 115 | 写入等待同步 RGB-D 的提示。 |
| 116 | 显示等待画面，避免重启后继续残留旧图。 |
| 117 | 处理键盘/窗口事件；Q 或 Esc 结束循环。 |
| 118 | 离开主循环后执行清理。 |
| 119 | 依次关闭 OpenCV 窗口、销毁节点、关闭 ROS。 |
| 122 | 模块直接运行时进入 main；被导入时不自动创建窗口。 |

<a id="module-5"></a>

## 5. 50 Hz 关节反馈插件：物理不降频，只减少消息

源文件：[src/gazebo_plugins/joint_feedback.cc](../src/gazebo_plugins/joint_feedback.cc)（63 行）。

这个 C++ 插件替换高频关节状态发布器。物理引擎仍每 0.002 秒计算一次，即 500 Hz；测量位置每 20 ms 发布一次，即 50 Hz。速度/力字段没有提供测量值，不能把桥接中可能出现的默认零当作真实测量。

<!-- source: src/gazebo_plugins/joint_feedback.cc sha256: ddd19052ed7415b79b7d74dbab2f287d3a097299c5de1aeabd157e4d72abd178 -->

```cpp
// Publish measured single-axis joint positions at 50 Hz, not every physics step.
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/plugin/Register.hh>
#include <chrono>
#include <vector>
#include <string>

namespace ghost_arm {
class JointFeedback : public ignition::gazebo::System,
                      public ignition::gazebo::ISystemConfigure,
                      public ignition::gazebo::ISystemPostUpdate {
  ignition::transport::Node node;
  ignition::transport::Node::Publisher publisher;
  std::vector<std::pair<std::string, ignition::gazebo::Entity>> joints;
  std::chrono::steady_clock::duration last{};
  bool sent = false;
 public:
  void Configure(const ignition::gazebo::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &,
                 ignition::gazebo::EntityComponentManager &ecm,
                 ignition::gazebo::EventManager &) override {
    ignition::gazebo::Model model(entity);
    for (const auto *name : {"tail_yaw", "tail_shoulder", "tail_elbow", "tail_wrist",
                             "finger_left_slide", "finger_right_slide"}) {
      auto id = model.JointByName(ecm, name);
      if (id == ignition::gazebo::kNullEntity) continue;
      if (!ecm.Component<ignition::gazebo::components::JointPosition>(id))
        ecm.CreateComponent(id, ignition::gazebo::components::JointPosition());
      joints.emplace_back(name, id);
    }
    publisher = node.Advertise<ignition::msgs::Model>("/ghost/arm/joint_states");
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo &info,
                  const ignition::gazebo::EntityComponentManager &ecm) override {
    if (info.paused) return;
    // Align to the same 20 ms simulation grid as PosePublisher for exact-stamp FK checks.
    const auto period = std::chrono::milliseconds(20);
    const auto slot = info.simTime / period;
    if (sent && info.simTime >= last && slot == last / period) return;
    sent = true; last = info.simTime;
    ignition::msgs::Model message;
    message.set_name("ghost");
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    message.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    message.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    for (const auto &joint : joints) {
      const auto *position = ecm.Component<ignition::gazebo::components::JointPosition>(joint.second);
      if (!position || position->Data().empty()) continue;
      auto *item = message.add_joint();
      item->set_name(joint.first); item->set_id(joint.second);
      item->mutable_axis1()->set_position(position->Data()[0]);
    }
    publisher.Publish(message);
  }
};
}
IGNITION_ADD_PLUGIN(ghost_arm::JointFeedback, ignition::gazebo::System,
                   ghost_arm::JointFeedback::ISystemConfigure,
                   ghost_arm::JointFeedback::ISystemPostUpdate)
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明目标：按 50 Hz 发布实测单轴关节位置，不在每个物理步都发布。 |
| 2 | 引入 Gazebo 系统插件和生命周期接口。 |
| 3 | Model 包装器用于按关节名称查找模型中的实体。 |
| 4 | JointPosition 组件保存物理引擎算出的实际关节位置。 |
| 5 | Gazebo Transport 节点用于本地仿真消息通信，不是 ROS rclcpp 节点。 |
| 6 | 引入 protobuf 的 Model 消息类型，沿用现有 ROS 桥。 |
| 7 | 提供插件注册宏，让 Gazebo 能从动态库加载类。 |
| 8 | chrono 提供仿真时长和毫秒/纳秒换算。 |
| 9 | vector 保存不定长的关节名与实体编号列表。 |
| 10 | string 保存关节名称。 |
| 12 | 将自定义类放进 ghost_arm 命名空间，避免与系统插件重名。 |
| 13 | 类首先继承 Gazebo System 基类。 |
| 14 | 继承配置阶段接口，声明将在模型加载时完成初始化。 |
| 15 | 继承物理更新后接口，声明在每步计算完成后读取结果。 |
| 16 | 保存 Gazebo Transport 节点。 |
| 17 | 保存反馈发布器对象。 |
| 18 | 每个 pair 的 first 是关节名，second 是 Gazebo 实体编号。 |
| 19 | 保存上次发布的仿真时间，花括号表示初始化为零时长。 |
| 20 | 记录是否至少发布过一次。 |
| 21 | 以下方法对 Gazebo 插件系统公开。 |
| 22 | Configure 的第一个参数是插件挂载模型的实体编号。 |
| 23 | SDF 配置参数未命名，表示当前实现没有读取插件子标签；关节名、话题和频率由本文件定义。 |
| 24 | ecm 是可修改的实体组件管理器，用它查找关节并请求测量组件。 |
| 25 | 事件管理器此处没有用到；override 明确覆盖系统接口。 |
| 26 | 用实体编号建立 Model 包装器。 |
| 27–28 | 遍历当前尾臂的四个转轴和两个滑动手指名称。 |
| 29 | 按名称查询该关节实体。 |
| 30 | 未找到就跳过；后续 Python 等待六关节齐全的检查会发现缺失。 |
| 31 | 如果物理位置测量组件尚未创建，则准备启用它。 |
| 32 | 创建 JointPosition 组件，物理系统后续会填充实际位置。 |
| 33 | 把有效关节的名字和实体编号加入列表。 |
| 34 | 结束关节查找循环。 |
| 35 | 发布 ignition.msgs.Model 到原话题 /ghost/arm/joint_states，已有桥接无需更换话题。 |
| 36 | 结束 Configure。 |
| 37 | PostUpdate 接收这一物理步的信息。 |
| 38 | 实体管理器为 const，说明这个阶段只读取物理状态，不修改它。 |
| 39 | 仿真暂停时不重复发布相同状态。 |
| 40 | 说明频率不仅要是 50 Hz，还要对齐 PosePublisher 使用的 20 ms 时间网格。 |
| 41 | 定义一个 20 毫秒周期。 |
| 42 | 仿真总时间除以周期得到当前时间格编号。 |
| 43 | 同一个格已经发布过就跳过；若时间回退，info.simTime>=last 不成立，会允许立即开始新周期。 |
| 44 | 记录已经发布及当前仿真时间。 |
| 45 | 创建一条 Model 消息。 |
| 46 | 把模型名填为 ghost。 |
| 47 | 把仿真时间转换成整数纳秒，不使用墙上时钟。 |
| 48 | 整数除以十亿得到时间戳秒部分。 |
| 49 | 取余得到不足一秒的纳秒部分。 |
| 50 | 遍历所有找到的关节。 |
| 51 | 从物理引擎的组件表读取该关节测量位置。 |
| 52 | 位置组件缺失或尚无数据时跳过，避免访问空数组。 |
| 53 | 向消息中添加一个 Joint 子消息。 |
| 54 | 分别填入关节名和实体编号。 |
| 55 | 本项目关节均为单轴，因此把第一个测量值写入 axis1.position；不是写命令目标。 |
| 56 | 结束关节消息填充循环。 |
| 57 | 一次发布包含六关节位置的 Model 消息。 |
| 58 | 结束 PostUpdate。 |
| 59 | 结束类定义；C++ 类末尾需要分号。 |
| 60 | 结束 ghost_arm 命名空间。 |
| 61–63 | 注册插件类以及它实现的 Configure/PostUpdate 接口，编译出的共享库才能被 SDF 指定并加载。 |

<a id="module-6"></a>

## 6. 机械结构、指间摩擦和相机安装

源文件：[src/ghost_sim/simulation/tail_structure.py](../src/ghost_sim/simulation/tail_structure.py)（83 行）。

PARTS 同时服务于 Gazebo SDF、ROS URDF 与碰撞检查。每项依次表示：父连杆、子连杆、关节名、类型、关节原点、转轴、限位、盒体尺寸、盒中心偏移、质量。球体坐标 +X 向前、+Y 向左、+Z 向上。

<!-- source: src/ghost_sim/simulation/tail_structure.py sha256: 2e4cdd865bcd4741e98e6227bb0920a103d0f76d79dd7c57f2d6f0b14546d73e -->

```python
"""Shared mechanical geometry for Gazebo and ROS; metres, radians, +X forward."""
import xml.etree.ElementTree as E
import math

JOINTS = ['tail_yaw', 'tail_shoulder', 'tail_elbow', 'tail_wrist', 'finger_left_slide', 'finger_right_slide']
# parent, child, joint, type, joint origin in parent, axis, limits, box size, box centre, mass
PARTS = [
 ('base_link','bottom_mount','bottom_mount_fixed','fixed',(0,0,-.25),(0,0,1),(0,0),(.24,.18,.035),(0,0,0),.08),
 ('bottom_mount','grasp_camera_link','grasp_camera_fixed','fixed',(.065,0,-.04),(0,0,1),(0,0),(.065,.10,.035),(0,0,0),.04),
 ('base_link','tail_bracket','tail_bracket_fixed','fixed',(-.27,0,-.04),(0,0,1),(0,0),(.14,.09,.07),(-.025,0,0),.06),
 ('tail_bracket','tail_yaw_link','tail_yaw','revolute',(-.06,0,-.04),(0,0,1),(-1.3,1.3),(.07,.085,.055),(0,0,-.012),.05),
 ('tail_yaw_link','tail_upper','tail_shoulder','revolute',(0,0,-.045),(0,1,0),(-1.6,.7),(.045,.055,.22),(0,0,-.11),.08),
 ('tail_upper','tail_lower','tail_elbow','revolute',(0,0,-.22),(0,1,0),(-1.8,1.8),(.04,.05,.20),(0,0,-.10),.06),
 ('tail_lower','gripper_palm','tail_wrist','revolute',(0,0,-.20),(0,1,0),(-2.2,2.2),(.065,.115,.055),(0,0,-.035),.06),
 ('gripper_palm','finger_left','finger_left_slide','prismatic',(0,.018,-.06),(0,1,0),(0,.03),(.045,.016,.085),(0,0,-.0425),.02),
 ('gripper_palm','finger_right','finger_right_slide','prismatic',(0,-.018,-.06),(0,-1,0),(0,.03),(.045,.016,.085),(0,0,-.0425),.02),
 ('gripper_palm','grasp_center','grasp_center_fixed','fixed',(0,0,-.115),(0,0,1),(0,0),None,(0,0,0),.001),
]

def add(p,t,v=None,**kw):
 e=E.SubElement(p,t,kw)
 if v is not None:e.text=str(v)
 return e

def vec(v):return ' '.join(map(str,v))

def attach(sdf_model, robot):
 # Fortress scene serialization cannot handle relative_to poses reliably.
 # All zero-pose joint origins have identity rotation, so sum translations.
 zero_positions={'base_link':(0,0,0)}
 for parent,child,name,kind,origin,axis,limits,size,centre,mass in PARTS:
  sl=add(sdf_model,'link',name=child)
  if kind!='fixed':add(sl,'self_collide','true')
  zero_positions[child]=tuple(a+b for a,b in zip(zero_positions[parent],origin))
  add(sl,'pose',vec(zero_positions[child])+' 0 0 0')
  inertia=add(sl,'inertial');add(inertia,'pose',vec(centre)+' 0 0 0');add(inertia,'mass',mass)
  tensor=add(inertia,'inertia');a,b,c=size or (.01,.01,.01)
  for key,value in [('ixx',mass*(b*b+c*c)/12),('iyy',mass*(a*a+c*c)/12),('izz',mass*(a*a+b*b)/12),('ixy',0),('ixz',0),('iyz',0)]:add(tensor,key,value)
  ul=add(robot,'link',name=child)
  if size:
   color='0.10 0.20 0.28 1' if 'finger' in child or 'camera' in child else '0.15 0.65 0.8 1'
   for kind_geom in ('visual','collision'):
    sg=add(sl,kind_geom,name=child+'_'+kind_geom);add(sg,'pose',vec(centre)+' 0 0 0')
    add(add(add(sg,'geometry'),'box'),'size',vec(size))
    ug=add(ul,kind_geom);add(ug,'origin',xyz=vec(centre),rpy='0 0 0');add(add(ug,'geometry'),'box',size=vec(size))
    if kind_geom=='collision' and 'finger' in child:
     friction=add(add(add(sg,'surface'),'friction'),'ode');add(friction,'mu',1.5);add(friction,'mu2',1.5)
    if kind_geom=='visual':
     mat=add(sg,'material');add(mat,'ambient',color);add(mat,'diffuse',color)
     add(add(ug,'material',name=child+'_color'),'color',rgba=color)
   # Visible bearing aligned with the pitch joint axis.
   if kind=='revolute':
    bearing=add(sl,'visual',name='bearing');add(bearing,'pose','0 0 0 1.57079632679 0 0' if axis==(0,1,0) else '0 0 0 0 0 0')
    cyl=add(add(bearing,'geometry'),'cylinder');add(cyl,'radius',.032);add(cyl,'length',.075)
    mat=add(bearing,'material');add(mat,'ambient','0.85 0.55 0.12 1');add(mat,'diffuse','0.85 0.55 0.12 1')
    uv=add(ul,'visual');add(uv,'origin',xyz='0 0 0',rpy='1.57079632679 0 0' if axis==(0,1,0) else '0 0 0');add(add(uv,'geometry'),'cylinder',radius='.032',length='.075');add(add(uv,'material',name='bearing_gold'),'color',rgba='0.85 0.55 0.12 1')
  sj=add(sdf_model,'joint',name=name,type=kind);add(sj,'parent',parent);add(sj,'child',child)
  uj=add(robot,'joint',name=name,type=kind);add(uj,'parent',link=parent);add(uj,'child',link=child);add(uj,'origin',xyz=vec(origin),rpy='0 0 0')
  if kind!='fixed':
   ax=add(sj,'axis');add(ax,'xyz',vec(axis));limit=add(ax,'limit')
   for key,val in [('lower',limits[0]),('upper',limits[1]),('effort',8 if kind=='revolute' else 3),('velocity',1 if kind=='revolute' else .06)]:add(limit,key,val)
   add(add(ax,'dynamics'),'damping',.08)
   add(uj,'axis',xyz=vec(axis));add(uj,'limit',lower=str(limits[0]),upper=str(limits[1]),effort='8',velocity='1' if kind=='revolute' else '.06')
   controller=add(sdf_model,'plugin',filename='ignition-gazebo-joint-position-controller-system',name='ignition::gazebo::systems::JointPositionController')
   if kind=='prismatic':
    # A blocked finger has intentional position error; never integrate it into a persistent closing force.
    for key in ('i_gain','i_max','i_min'):add(controller,key,0)
   for key,val in [('joint_name',name),('topic','/model/ghost/joint/'+name+'/0/cmd_pos'),('p_gain',5 if kind=='revolute' else 30),('d_gain',.15 if kind=='revolute' else .3),('cmd_max',8 if kind=='revolute' else 3),('cmd_min',-8 if kind=='revolute' else -3)]:add(controller,key,val)
 state=add(sdf_model,'plugin',filename='libghost_joint_feedback.so',name='ghost_arm::JointFeedback');add(state,'topic','/ghost/arm/joint_states')
 for name in JOINTS:add(state,'joint_name',name)
 # Gazebo cameras look along local +X; positive pitch points +X downwards.
 camera_link=sdf_model.find("link[@name='grasp_camera_link']")
 sensor=add(camera_link,'sensor',name='grasp_rgbd',type='rgbd_camera')
 add(sensor,'pose','0 0 -0.02 0 1.57079632679 0');add(sensor,'topic','/grasp_camera');add(sensor,'always_on','true');add(sensor,'update_rate',5)
 cam=add(sensor,'camera');add(cam,'horizontal_fov',1.2217304764);add(cam,'optical_frame_id','grasp_camera_optical_frame')
 # Explicit intrinsics avoid Fortress defaults (fx=277) disagreeing with the 70-degree image.
 focal=160/math.tan(1.2217304764/2)
 intrinsics=add(add(cam,'lens'),'intrinsics')
 for key,value in [('fx',focal),('fy',focal),('cx',159.5),('cy',119.5),('s',0)]:add(intrinsics,key,value)
 im=add(cam,'image');add(im,'width',320);add(im,'height',240);add(im,'format','R8G8B8')
 clip=add(cam,'clip');add(clip,'near',.05);add(clip,'far',4)
 optical=add(robot,'link',name='grasp_camera_optical_frame')
 j=add(robot,'joint',name='grasp_optical_fixed',type='fixed');add(j,'parent',link='grasp_camera_link');add(j,'child',link='grasp_camera_optical_frame');add(j,'origin',xyz='0 0 -0.02',rpy='3.14159265359 0 -1.57079632679')
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明这是 Gazebo 与 ROS 共用的机械参数，单位是米、弧度。 |
| 2 | ElementTree 用于构造 SDF 和 URDF 的 XML 标签。 |
| 3 | math 用于计算相机焦距。 |
| 5 | 统一六个关节的名称与顺序，命令、反馈和运动学使用同一约定。 |
| 6 | 注释列出每个 PARTS 元组中十个字段的含义。 |
| 7 | 开始机械结构表。 |
| 8 | 在球底 z=-0.25 m 处固定一块 24×18×3.5 cm 的小平板，质量 80 g。 |
| 9 | 相机壳相对平板向前 6.5 cm、向下 4 cm；此处是相机外壳，不是光学原点。 |
| 10 | 尾部支架固定到球后方；盒中心再向后偏 2.5 cm。 |
| 11 | 尾根偏航轴沿 Z，范围 -1.3～1.3 rad；它连接支架与旋转件。 |
| 12 | 肩关节沿 Y 转动，范围 -1.6～0.7 rad；上臂长 22 cm，盒中心在下方 11 cm。 |
| 13 | 肘关节在上臂末端，沿 Y 转动；前臂长 20 cm。 |
| 14 | 腕关节在前臂末端，沿 Y 转动，允许 -2.2～2.2 rad；掌部盒中心向下偏 3.5 cm。 |
| 15 | 左指位于掌部 +Y 一侧，沿 +Y 滑动，最大行程 3 cm；手指盒高 8.5 cm。 |
| 16 | 右指位于 -Y 一侧，滑动轴是 -Y，因此两指正位移都表示张开。 |
| 17 | 抓取中心固定在掌部下方 11.5 cm；它没有显示/碰撞盒，只作为运动学目标坐标系。 |
| 18 | 结束结构表。 |
| 20 | add 是创建 XML 子标签的辅助函数，kw 接收 name 等属性。 |
| 21 | 在父元素下创建名为 t 的子元素。 |
| 22 | 有文本值时转成字符串写入标签，None 表示不设置文本。 |
| 23 | 返回子元素，方便继续往下添加标签。 |
| 25 | 把向量的数值转为以空格分隔的文本，适用于 SDF 的 pose/size。 |
| 27 | attach 同时接收 Gazebo 模型元素与 ROS robot 元素。 |
| 28–29 | 为绕过本机 Fortress 对 relative_to 的序列化兼容问题，零位原点直接累加；这只适用于这里零位旋转均为单位旋转的结构。 |
| 30 | 球体自身的零位位置为原点。 |
| 31 | 逐项解包参数表，生成每一个零件。 |
| 32 | 在 Gazebo 模型下创建对应 link。 |
| 33 | 运动连杆开启 self_collide，物理层参与自碰撞约束；固定安装件不在这里开启。 |
| 34 | 零位子链接位置等于父链接位置加本关节原点平移。 |
| 35 | 写入模型坐标系下的零位位置，旋转为零。 |
| 36 | 建立 inertial 标签，质心放在盒中心，写入质量。 |
| 37 | 建立转动惯量张量；无盒体的抓取中心用 1 cm 小立方体计算一个正惯量。 |
| 38 | 按均匀长方体公式写三个主惯量，三个惯性积为零。 |
| 39 | 在 URDF 中创建同名链接。 |
| 40 | 只有定义了盒体尺寸的零件才生成可见/碰撞外形。 |
| 41 | 手指和相机壳使用深色，其余部件使用青色；最后一个分量为不透明度。 |
| 42 | 同一尺寸分别创建 visual 与 collision。 |
| 43 | 创建 SDF 几何容器，并把它放在 centre 偏移处。 |
| 44 | 添加 geometry/box/size，写三轴尺寸。 |
| 45 | 在 URDF 同步建立几何、原点和尺寸；URDF 是显示模型，不在这里运行 Gazebo 物理。 |
| 46 | 只有手指的碰撞几何需要额外设置夹持摩擦。 |
| 47 | 创建 surface/friction/ode，并把两个摩擦方向系数设为 1.5。 |
| 48 | 材质只添加到 visual，不添加到 collision。 |
| 49 | 写 SDF 的环境光与漫反射颜色。 |
| 50 | 写 URDF 材质颜色。 |
| 51 | 下面的圆柱轴承仅用于显示转轴位置。 |
| 52 | 只为旋转关节生成金色轴承。 |
| 53 | 在关节原点创建轴承外观；沿 Y 的关节将圆柱转 90° 使轴向对齐。 |
| 54 | 轴承半径 3.2 cm、长度 7.5 cm；没有额外轴承碰撞形状。 |
| 55 | 为轴承添加金色环境光和漫反射材质。 |
| 56 | 在 URDF 同步创建轴承原点、圆柱尺寸和颜色。 |
| 57 | SDF 中创建关节，并声明父、子链接关系。 |
| 58 | URDF 中创建同名关节，同时写清相对父链接的关节原点。 |
| 59 | 固定关节无需转轴限位和位置控制器。 |
| 60 | 创建 SDF 轴向与限位容器。 |
| 61 | 写位置上下限；转轴力矩上限 8、滑动轴力上限 3，速度限值分别 1 rad/s 和 0.06 m/s。 |
| 62 | 添加关节阻尼 0.08，减小自由振动。 |
| 63 | URDF 同步记录轴与限位；当前 URDF effort 统一写 8，实际 Gazebo 手指力限值仍以上面 SDF 的 3 N 为准。 |
| 64 | 每个运动关节加载 Gazebo JointPositionController 插件。 |
| 65 | 手指滑动关节单独设置积分项。 |
| 66 | 木块阻挡手指是预期接触，不能让这个持续位置误差积累成越来越大的夹紧力。 |
| 67 | 将 i_gain、i_max、i_min 都设为 0；避免长时间夹住后张开仍输出夹紧力。 |
| 68 | 指定关节名与原生命令话题；四轴用 P=5、D=0.15，手指用 P=30、D=0.3；输出分别限制为 ±8 和 ±3。 |
| 69 | 加载自定义 50 Hz 反馈插件；此处 topic 子标签仅保留描述，当前 C++ 插件实际使用硬编码的同名话题。 |
| 70 | 写 joint_name 子标签；当前 C++ 也按同一六名称硬编码查找，改关节名时两处必须同步。 |
| 71 | Gazebo 相机默认看本地 +X，绕 Y 转 +90° 后朝向球底。 |
| 72 | 找到前面生成的相机外壳 link。 |
| 73 | 添加一颗同时产生 RGB 和深度的 rgbd_camera 传感器。 |
| 74 | 传感器相对壳体再下移 2 cm，并朝下；话题前缀为 /grasp_camera，保持启用，频率为每仿真秒 5 帧。 |
| 75 | 创建 camera 参数，水平视场约 70°，指定光学坐标系标签。 |
| 76 | 说明不能依赖与实际视场不匹配的默认焦距。 |
| 77 | 用半幅宽 160 像素除以 tan(半视场角)，得到约 228.50 像素的焦距。 |
| 78 | 创建显式镜头内参容器。 |
| 79 | fx、fy 使用同一焦距，主点为 (159.5,119.5)，斜切参数 s 为零。 |
| 80 | 输出分辨率为 320×240，彩色编码为 R8G8B8。 |
| 81 | 深度近裁剪为 5 cm、远裁剪为 4 m。 |
| 82 | 在 URDF 中增加光学坐标系链接。 |
| 83 | 创建相机壳到光学坐标系的固定变换：平移下移 2 cm，旋转将 ROS 光学轴方向与实际朝下相机对应。 |

<a id="module-7"></a>

## 7. 地面木块：质量、重力与真实碰撞

源文件：[src/ghost_sim/simulation/wood_block.py](../src/ghost_sim/simulation/wood_block.py)（50 行）。

世界为了支持理想悬浮仍采用零重力。木块是可运动刚体，并单独持续施加向下的重量；抓取通过物理接触摩擦，没有给木块和夹爪偷偷加固定连接。

<!-- source: src/ghost_sim/simulation/wood_block.py sha256: 12c71edb3eb53daa699d9de75c2f79f8d1fca87015471cd65cad9aefe1b778ac -->

```python
"""A dynamic wooden practice block with local gravity in the floating-ghost world."""
import xml.etree.ElementTree as E

SIZE = (.04, .04, .06)
MASS = .04
POSITION = (.10, 0., .03)


def add(parent, tag, value=None, **attributes):
    element = E.SubElement(parent, tag, attributes)
    if value is not None:
        element.text = str(value)
    return element


def attach(world):
    model = add(world, 'model', name='wood_block')
    add(model, 'static', 'false')
    add(model, 'pose', ' '.join(map(str, POSITION))+' 0 0 0')
    link = add(model, 'link', name='link')
    inertia = add(link, 'inertial')
    add(inertia, 'mass', MASS)
    tensor = add(inertia, 'inertia')
    x, y, z = SIZE
    for key, value in [('ixx', MASS*(y*y+z*z)/12), ('iyy', MASS*(x*x+z*z)/12),
                       ('izz', MASS*(x*x+y*y)/12), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        add(tensor, key, value)
    for kind in ('visual', 'collision'):
        shape = add(link, kind, name='wood_'+kind)
        add(add(add(shape, 'geometry'), 'box'), 'size', ' '.join(map(str, SIZE)))
        if kind == 'visual':
            material = add(shape, 'material')
            add(material, 'ambient', '0.65 0.35 0.12 1')
            add(material, 'diffuse', '0.65 0.35 0.12 1')
        else:
            friction = add(add(add(shape, 'surface'), 'friction'), 'ode')
            add(friction, 'mu', 1.5)
            add(friction, 'mu2', 1.5)
    # Apply only this body's weight, leaving the existing floating robot unchanged.
    force = add(world, 'plugin', filename='ignition-gazebo-apply-link-wrench-system',
                name='ignition::gazebo::systems::ApplyLinkWrench')
    persistent = add(force, 'persistent')
    for key, value in [('entity_name', 'wood_block'), ('entity_type', 'model'),
                       ('force', f'0 0 {-MASS*9.81}')]:
        add(persistent, key, value)
    poses = add(model, 'plugin', filename='ignition-gazebo-pose-publisher-system',
                name='ignition::gazebo::systems::PosePublisher')
    for key, value in [('publish_model_pose', 'true'), ('publish_nested_model_pose', 'true'), ('publish_link_pose', 'true'),
                       ('use_pose_vector_msg', 'true'), ('update_frequency', 50)]:
        add(poses, key, value)
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明木块是在零重力幽灵世界中单独受到重量的动态物体。 |
| 2 | 导入 XML 构造工具。 |
| 4 | 木块自身长宽高是 0.04、0.04、0.06 米。 |
| 5 | 质量 0.04 千克，即 40 克。 |
| 6 | 中心初始位于世界 (0.10,0,0.03)，因高度 6 cm，底面恰好落在 z=0。 |
| 9 | 定义创建 XML 子标签的辅助函数。 |
| 10 | 建立标签和属性。 |
| 11 | 只有存在文本值时才设置内容。 |
| 12 | 转为字符串，适配 XML 文本格式。 |
| 13 | 返回新建元素。 |
| 16 | attach 将木块添加到已有世界元素。 |
| 17 | 创建名为 wood_block 的模型，名称与控制器反馈筛选一致。 |
| 18 | static=false 表示物块可参与动力学运动。 |
| 19 | 写入世界位置，后面三个零是初始 roll、pitch、yaw。 |
| 20 | 创建木块的单个刚性 link。 |
| 21 | 创建惯性属性容器。 |
| 22 | 写入质量。 |
| 23 | 创建转动惯量矩阵元素。 |
| 24 | 把尺寸解包为 x、y、z。 |
| 25–26 | 均匀长方体主惯量分别为 m(y²+z²)/12 等，惯性积为零。 |
| 27 | 逐一写入各惯量值。 |
| 28 | 外观和碰撞使用同一个几何尺寸循环生成。 |
| 29 | 为当前类型创建名为 wood_visual 或 wood_collision 的元素。 |
| 30 | 写 geometry/box/size，保证看到的尺寸与实际碰撞一致。 |
| 31 | 外观分支设置颜色。 |
| 32 | 建立材质容器。 |
| 33 | 设置棕色环境光颜色。 |
| 34 | 设置同色漫反射；当前是木块色外观，不是带真实木纹纹理的网格模型。 |
| 35 | 碰撞分支设置摩擦。 |
| 36 | 创建摩擦参数容器。 |
| 37 | 第一摩擦方向系数设为 1.5。 |
| 38 | 第二摩擦方向系数也设为 1.5。 |
| 39 | 说明重量只施加给木块，不直接改变整个世界的重力。 |
| 40–41 | 世界中加载 ApplyLinkWrench 系统，用于持续施加力。 |
| 42 | persistent 表示每个物理步都施加，而非只给一次冲量。 |
| 43–44 | 指定受力模型为 wood_block，世界 Z 方向力为 -MASS×9.81，即 -0.3924 N。 |
| 45 | 写入这组持续力参数。 |
| 46–47 | 给木块加载 PosePublisher，以便读取它实际是否抬升。 |
| 48–49 | 启用模型及链接位姿、Pose_V 数组消息与 50 Hz 更新；publish_nested_model_pose 是本机获取完整模型位姿所需配置的一部分。 |
| 50 | 逐项写入位姿发布参数。 |

<a id="module-8"></a>

## 8. 单独执行坐标 IK：与抓取流程共用的控制基础

源文件：[src/ghost_sim/control/ik_execute.py](../src/ghost_sim/control/ik_execute.py)（100 行）。

`ik --execute` 只控制四轴，不控制手指、不移动球体。这里的 pitch 是弧度，和 grasp 的 --pitch-deg 度数参数不同。四轴碰撞检查中未提供手指值，按手指零位计算；它不是完整六轴/全场景规划。

<!-- source: src/ghost_sim/control/ik_execute.py sha256: 8bd908b7bf903995bde0ffe02e5a08bf8f95a6e6100ef5be89ba6b3e1db491d7 -->

```python
"""Execute an IK solution with measured joints, bounded interpolation and a hold on failure."""
import fcntl
import math
import time
import numpy as np
from ghost_sim.kinematics.inverse import ARM_NAMES, inverse_kinematics
from ghost_sim.kinematics.forward import forward_kinematics
from ghost_sim.kinematics.body_collision import select_body_safe_solution, arm_body_clearance


class ExecutionError(RuntimeError):
    def __init__(self, message, motion_started=False):
        super().__init__(message)
        self.motion_started = motion_started


def interpolation(start, target, elapsed, duration):
    u = float(np.clip(elapsed / duration, 0, 1))
    return start + (target - start) * (3*u*u - 2*u*u*u)


def execute_target(position, pitch, requested_duration=4.0):
    """Use actual joints as the IK seed. Only command four arm joints, never fingers."""
    if not math.isfinite(requested_duration) or requested_duration <= 0:
        raise ValueError('运动时长必须为正数。')
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64
    lock = open('/tmp/ghost_arm_ik.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise ExecutionError('已有逆运动学动作正在执行。')
    rclpy.init()
    node = rclpy.create_node('ghost_arm_ik_execute')
    state = {'q': None, 'stamp': None, 'received': None}
    started = False
    def receive(message):
        values = dict(zip(message.name, message.position))
        if not all(name in values and math.isfinite(values[name]) for name in ARM_NAMES):
            return
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if state['stamp'] is None or stamp != state['stamp']:
            state.update(q=np.array([values[name] for name in ARM_NAMES]),
                         stamp=stamp, received=time.monotonic())
    sub = node.create_subscription(JointState, '/joint_states', receive, 20)
    pubs = [node.create_publisher(Float64, '/ghost/arm/'+name+'/cmd_pos', 10) for name in ARM_NAMES]
    def send(angles):
        for pub, value in zip(pubs, angles):
            pub.publish(Float64(data=float(value)))
    try:
        deadline = time.monotonic() + 15
        while state['q'] is None or not all(pub.get_subscription_count() for pub in pubs):
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() > deadline:
                raise RuntimeError('仿真关节反馈或命令桥未就绪。')
        if time.monotonic() - state['received'] > 1:
            raise RuntimeError('关节反馈已过期。')
        result = select_body_safe_solution(inverse_kinematics(position, pitch, state['q']), state['q'])
        start = state['q'].copy()
        target = np.array(result['selected']['joint_angles_rad'])
        # smoothstep has peak slope 1.5; cap nominal joint command speed at .35 rad/s.
        duration = max(requested_duration, 1.5*float(np.max(np.abs(target-start)))/.35)
        initial_stamp = state['stamp']
        deadline = time.monotonic() + duration*10 + 20
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if time.monotonic()-state['received'] > 1:
                raise RuntimeError('关节反馈停止更新，已请求保持最近位置。')
            clearance, link = arm_body_clearance(state['q'])
            if clearance < .002:
                raise RuntimeError(f'实际机械臂靠近球体：{link}，已停止。')
            elapsed = state['stamp']-initial_stamp
            if elapsed < 0:
                raise RuntimeError('仿真时间已重置，终止当前动作。')
            send(interpolation(start, target, elapsed, duration))
            started = True
            actual = forward_kinematics(state['q'])
            position_error = float(np.linalg.norm(actual[:3,3]-np.asarray(position)))
            pitch_error = abs(math.atan2(math.sin(sum(state['q'][1:])-pitch), math.cos(sum(state['q'][1:])-pitch)))
            if elapsed >= duration and np.max(np.abs(state['q']-target)) < .01 and position_error < .002 and pitch_error < .01:
                result.update(motion_executed=True, execution_succeeded=True,
                              duration_sim_seconds=duration, actual_joint_angles_rad=state['q'].tolist(),
                              actual_position_m=actual[:3,3].tolist(), tracking_position_error_m=position_error,
                              tracking_pitch_error_rad=pitch_error)
                return result
            time.sleep(.02)
        raise RuntimeError('运动超时，已请求保持最近位置。')
    except (ValueError, RuntimeError, KeyboardInterrupt) as error:
        if started and state['q'] is not None:
            for _ in range(3):
                send(state['q'])
                time.sleep(.03)
        reason = '用户中断，已请求保持最近位置。' if isinstance(error, KeyboardInterrupt) else str(error)
        raise ExecutionError(reason, started) from error
    finally:
        node.destroy_node()
        rclpy.shutdown()
        lock.close()
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 模块说明：根据测量关节执行逆解，并在失败时请求保持。 |
| 2 | 导入文件锁。 |
| 3 | 用于有限数值检查、角度差归一化和三角函数。 |
| 4 | 用于真实时间超时与短暂等待。 |
| 5 | 用于数组、向量与误差运算。 |
| 6 | 四轴名称和解析 IK 函数来自同一运动学模块。 |
| 7 | 导入 FK，用于执行中计算实际末端位置。 |
| 8 | 导入路径筛选和球体间隙检查。 |
| 11 | 扩展 RuntimeError，使错误还携带“是否已经开始运动”的状态。 |
| 12 | 默认认为运动尚未开始。 |
| 13 | 将文字原因交给父类异常。 |
| 14 | 保存实际执行状态，避免开始运动后仍报告成完全未执行。 |
| 17 | 计算任意数组起点到终点的平滑插值。 |
| 18 | 进度=已用仿真时间/总时间，并限制到 0～1。 |
| 19 | 3u²-2u³ 的起终点斜率为零，减小突然启动/停止；函数对数组逐元素计算。 |
| 22 | 执行 base_link 下的目标位置与俯仰角，默认最短请求时长为 4 秒。 |
| 23 | 说明采用实测关节作为 seed，并且不发送手指命令。 |
| 24 | 拒绝无穷、NaN 和非正时长。 |
| 25 | 报参数错误，不启动动作。 |
| 26 | 运行时加载 ROS 客户端。 |
| 27 | 导入关节反馈类型。 |
| 28 | 导入标量目标类型。 |
| 29 | 打开与抓取流程共用的锁文件。 |
| 30 | 开始尝试加锁。 |
| 31 | 申请非阻塞独占锁。 |
| 32 | 捕获被另一个动作占用的情况。 |
| 33 | 未获得锁时关闭文件。 |
| 34 | 返回未开始运动的执行错误。 |
| 35 | 初始化 ROS。 |
| 36 | 创建 IK 执行节点。 |
| 37 | 缓存四轴数组、仿真时间戳和最近真实到达时间。 |
| 38 | 尚未发出运动目标。 |
| 39 | 接收关节状态的回调。 |
| 40 | 构造按名字索引的关节位置字典。 |
| 41 | 要求四轴数据齐全且数值有限。 |
| 42 | 无效消息直接跳过。 |
| 43 | 将时间戳合并为秒。 |
| 44 | 只把仿真时间不同的消息当作新反馈。 |
| 45–46 | 按统一关节顺序缓存四轴数据，同时更新仿真时间和真实到达时间。 |
| 47 | 订阅 /joint_states，队列深度为 20。 |
| 48 | 为四个转轴建立目标发布器，不包含两个手指。 |
| 49 | 定义发送一组角度的内部辅助函数。 |
| 50 | 将四轴发布器与目标配对。 |
| 51 | 转换为 Float64 并发布。 |
| 52 | 以下流程统一由异常处理和 finally 保护。 |
| 53 | 最多等待 15 秒让通信就绪。 |
| 54 | 没收到关节，或控制桥尚未订阅命令时继续等。 |
| 55 | 处理 ROS 回调，最多等待 0.05 秒。 |
| 56 | 检查真实超时。 |
| 57 | 反馈或桥接没就绪就停止。 |
| 58 | 开始求解前再检查测量是否超过 1 秒未更新。 |
| 59 | 拒绝基于过期姿态计算运动起点。 |
| 60 | 先求 IK，再从 seed 排序的候选中挑选球体路径通过的那组。 |
| 61 | 复制测量起点。 |
| 62 | 取筛选后的目标角数组。 |
| 63 | 注释说明 smoothstep 最大斜率和 0.35 rad/s 名义速度限制。 |
| 64 | 按最大角度变化扩大持续时间，不能短于用户要求的时长。 |
| 65 | 记录仿真开始时间。 |
| 66 | 设置允许慢速仿真的真实时间兜底期限。 |
| 67 | 在期限内执行控制循环。 |
| 68 | 接收最新反馈。 |
| 69 | 执行期间检查反馈是否超过 1 秒未更新。 |
| 70 | 反馈中断时抛异常，由后面逻辑请求保持。 |
| 71 | 根据当前测量四轴计算球体最小间隙。 |
| 72 | 间隙不足 2 mm 时停止。 |
| 73 | 错误文字说明是哪根连杆靠近球体。 |
| 74 | 仿真时间差作为插值进度。 |
| 75 | 检测时间回退。 |
| 76 | 仿真重置后旧运动计划不再有效。 |
| 77 | 发送这一时刻的平滑目标。 |
| 78 | 记录已经开始发送运动命令。 |
| 79 | 用当前测量关节计算末端姿态。 |
| 80 | 计算实际位置与目标的欧氏距离。 |
| 81 | 把肩肘腕总角与目标 pitch 的差映射到最短角度差，再取绝对值。 |
| 82 | 需要时间结束、关节误差小于 0.01 rad、末端位置误差小于 2 mm、俯仰误差小于 0.01 rad，才认为到达。 |
| 83–86 | 在结果中添加执行成功、动作时长、实测角度、末端位置及跟踪误差。 |
| 87 | 返回本次执行报告。 |
| 88 | 休眠 0.02 秒让出 CPU。 |
| 89 | 到期限仍不满足要求则报告超时。 |
| 90 | 统一捕获输入、运行和 Ctrl+C 异常。 |
| 91 | 只有动作确实开始且有测量，才发送保持请求。 |
| 92 | 重复发送三次。 |
| 93 | 将当前测量角度作为保持目标。 |
| 94 | 保持消息之间间隔 0.03 秒。 |
| 95 | Ctrl+C 转换成中文原因，其他异常保留原原因。 |
| 96 | 把是否开始运动一并包装成 ExecutionError，并保留原始异常链。 |
| 97 | 任何退出路径均执行清理。 |
| 98 | 销毁 ROS 节点。 |
| 99 | 关闭 ROS。 |
| 100 | 关闭锁文件并释放动作锁。 |

<a id="module-9"></a>

## 9. 手动预设姿态：与自动抓取有什么不同

源文件：[src/ghost_sim/control/arm_pose.py](../src/ghost_sim/control/arm_pose.py)（41 行）。

这是结构演示命令 `arm hang/ready/close`，不等同于 `grasp pick`。它使用真实时间插值，未实现抓取流程的完整反馈新鲜度、负载验证及共享文件锁；不要和 GUI、IK、抓取同时执行。ready 仍是垂直准备姿态，不随 grasp 默认 -60° 改变。

<!-- source: src/ghost_sim/control/arm_pose.py sha256: 2d78aedc92e93143e8b10494265b0789532b47e3a7c48313e56b1ecb078f3da4 -->

```python
"""Slow, joint-space structural demonstrations; not an IK or grasp controller."""
import argparse
import time
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from ghost_sim.simulation.tail_structure import JOINTS
from ghost_sim.kinematics.body_collision import check_body_path

PRESETS = {
 'hang': [0,0,0,0,.025,.025],
 'ready': [0,-.6637090764,-.6713951328,1.3351042091,.025,.025],
 'close': [0,-.6637090764,-.6713951328,1.3351042091,0,0],
}

def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('pose',choices=PRESETS)
 args=parser.parse_args()
 rclpy.init();node=rclpy.create_node('ghost_arm_pose');state={}
 sub=node.create_subscription(JointState,'/joint_states',lambda m:state.update(zip(m.name,m.position)),10)
 pubs=[node.create_publisher(Float64,'/ghost/arm/'+j+'/cmd_pos',10) for j in JOINTS]
 try:
  deadline=time.monotonic()+15
  while not all(j in state for j in JOINTS) or not all(p.get_subscription_count() for p in pubs):
   rclpy.spin_once(node,timeout_sec=.1)
   if time.monotonic()>deadline:raise RuntimeError('关节状态或控制桥未就绪，请先启动仿真。')
  start=[state[j] for j in JOINTS];target=PRESETS[args.pose]
  check_body_path(start,target)
  began=time.monotonic();duration=4
  while time.monotonic()-began<duration+8:
   u=min(1,(time.monotonic()-began)/duration);blend=u*u*(3-2*u)
   for p,a,b in zip(pubs,start,target):p.publish(Float64(data=float(a+(b-a)*blend)))
   rclpy.spin_once(node,timeout_sec=.02);time.sleep(.02)
   errors=[abs(state[j]-t) for j,t in zip(JOINTS,target)]
   if u==1 and all(e<(.002 if i>=4 else .035) for i,e in enumerate(errors)):
    print('姿态完成：',args.pose,{j:round(state[j],4) for j in JOINTS});return
  raise RuntimeError('关节未在时限内到达目标：'+str(state))
 finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明只做慢速关节空间姿态演示，不求 IK、不验证抓取。 |
| 2 | 解析姿态名。 |
| 3 | 真实时间计时与休眠。 |
| 4 | ROS 2 Python 客户端。 |
| 5 | 订阅关节位置。 |
| 6 | 发布标量目标。 |
| 7 | 读取六关节顺序。 |
| 8 | 执行前检查球体路径。 |
| 10 | 开始预设姿态字典。 |
| 11 | hang 四轴均为零、两指各张开 2.5 cm。 |
| 12 | ready 使用避开球体的四轴垂直姿态，两指张开；肩肘腕之和约为零。 |
| 13 | close 的四轴与 ready 相同，两指目标为零；有木块阻挡时可能达不到零位，不能代替 grasp 的接触模式。 |
| 14 | 结束字典。 |
| 16 | 命令入口函数。 |
| 17 | 创建参数解析器。 |
| 18 | 姿态名只能选 PRESETS 中已有的键。 |
| 19 | 读取用户输入。 |
| 20 | 初始化 ROS、建立节点、创建关节字典。 |
| 21 | 每收到一条 JointState 就更新字典中的关节值。 |
| 22 | 为六个关节创建位置发布器。 |
| 23 | 用 finally 保证清理节点。 |
| 24 | 启动等待最多 15 秒真实时间。 |
| 25 | 等六关节反馈齐全且所有目标话题均已有订阅者。 |
| 26 | 等待时仍处理回调。 |
| 27 | 超时则提醒先启动仿真。 |
| 28 | 按名字读取起点，并从预设表选终点。 |
| 29 | 先检查直接关节路径是否靠近球体。 |
| 30 | 记录真实起始时间，名义过渡时长为 4 秒。 |
| 31 | 总共最多用 12 秒真实时间进行过渡和等待。 |
| 32 | 进度限到 1，再计算三次平滑系数。 |
| 33 | 向六个关节发送当前插值目标。 |
| 34 | 处理反馈，再休眠 0.02 秒。 |
| 35 | 逐关节计算实际位置误差。 |
| 36 | 进度结束且四轴误差小于 0.035 rad、手指误差小于 0.002 m 时完成。 |
| 37 | 打印测量姿态并返回。 |
| 38 | 超过期限仍未到位则报错。 |
| 39 | 销毁节点并关闭 ROS。 |
| 41 | 直接运行模块时启动 main。 |

<a id="module-10"></a>

## 10. 启动文件：后台、界面与桥接

源文件：[launch/sim.launch.py](../launch/sim.launch.py)（56 行）。

Gazebo 后台负责物理和传感器，Gazebo GUI 与 RViz 是两个不同的显示窗口。`gui:=false` 只关闭 Gazebo 界面，不会停掉仿真。`[` 表示 Gazebo→ROS，`]` 表示 ROS→Gazebo 的单向桥接。

<!-- source: launch/sim.launch.py sha256: 7aefbb1b5b043ce2c7c41b0df4d2d7e2c83db50bb8d7dae06e5980c50694d9b9 -->

```python
"""Ghost simulation, bridges, model display, camera and flight control."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ghost_sim.paths import WORLD, MODEL, CONFIG
from ghost_sim.simulation.tail_structure import JOINTS


def static_tf(name,parent,child,xyz,rpy=(0,0,0)):
    x,y,z = xyz
    roll,pitch,yaw = rpy
    return Node(package='tf2_ros',executable='static_transform_publisher',name=name,
                arguments=['--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),
                           '--pitch',str(pitch),'--yaw',str(yaw),'--frame-id',parent,'--child-frame-id',child])


def generate_launch_description():
    def gazebo(context):
        command = ['ign','gazebo','-r','-v','3','-s',str(WORLD)]
        processes = [ExecuteProcess(cmd=command,output='screen')]
        if LaunchConfiguration('gui').perform(context) == 'true':
            processes.append(TimerAction(period=3.0,actions=[ExecuteProcess(
                cmd=['ign','gazebo','-g','--gui-config',str(CONFIG/'arm_gazebo.config')],
                additional_env={'LIBGL_ALWAYS_SOFTWARE':'1','QT_OPENGL':'software','QSG_RENDER_LOOP':'basic'},
                output='screen')]))
        return processes

    actions = [DeclareLaunchArgument('gui',default_value='false'),OpaqueFunction(function=gazebo),
               Node(package='robot_state_publisher',executable='robot_state_publisher',
                    parameters=[{'robot_description':(MODEL).read_text(),'use_sim_time':True}])]
    for module in ['visualization.ground_truth_tf','visualization.reference_scene','visualization.cloud_frame','control.flight_control','mapping.voxel_map_node','navigation.navigation3d']:
        actions.append(ExecuteProcess(cmd=['python3','-m','ghost_sim.'+module,'--ros-args','-p','use_sim_time:=true'],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='ghost_bridge',
                        arguments=[
                            '/model/ghost/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                            '/model/ghost/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                            '/model/wood_block/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                            '/ghost/arm/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                            '/grasp_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                        remappings=[('/ghost/arm/joint_states','/joint_states'),('/model/ghost/pose','/simulation/ground_truth/poses'),
                                    ('/camera/points','/camera/points_raw')],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='arm_bridge',
                        parameters=[{'config_file':str(CONFIG/'arm_bridge.yaml')}],output='screen'))
    actions.extend([static_tf('camera_mount','base_link','camera_link',(.29,0,0)),
                    static_tf('camera_optical','camera_link','camera_optical_frame',(0,0,0),
                              (-1.57079632679,0,-1.57079632679))])
    return LaunchDescription(actions)
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 说明此文件组织仿真、模型显示、桥和飞行控制。 |
| 2 | 导入 Path；当前文件中没有直接使用它。 |
| 3 | LaunchDescription 是 ROS 启动动作的集合。 |
| 4 | 分别导入启动参数声明、外部进程、运行时函数和延迟动作。 |
| 5 | LaunchConfiguration 在启动时读取 gui 等参数。 |
| 6 | Node 用于启动 ROS 可执行节点。 |
| 7 | WORLD、MODEL、CONFIG 是项目内已解析的资源路径。 |
| 8 | 导入 JOINTS；当前启动文件没有直接使用它，关节桥由 YAML 定义。 |
| 11 | 定义创建固定 TF 发布器的辅助函数。 |
| 12 | 解包 XYZ 平移。 |
| 13 | 解包 roll、pitch、yaw 欧拉角。 |
| 14 | 返回 tf2_ros 的静态变换节点。 |
| 15–16 | 将平移、旋转和父子 frame 名转换成命令行参数。 |
| 19 | ROS launch 调用的主入口。 |
| 20 | 内部函数在实际启动上下文中决定是否创建 GUI。 |
| 21 | ign gazebo 的 -r 表示运行、-v 3 是日志级别、-s 表示仅后台，并加载 WORLD。 |
| 22 | 创建后台进程动作。 |
| 23 | 读取 gui 参数；只有字符串 true 才增加 GUI。 |
| 24 | 等待 3 秒让后台先初始化。 |
| 25 | 单独启动 -g 图形界面，并加载尾臂面板配置。 |
| 26 | 设置软件 OpenGL 和基础 Qt 渲染循环，以适配本虚拟机。 |
| 27 | 把 GUI 输出交给启动终端。 |
| 28 | 返回这组进程动作。 |
| 30 | 声明 gui 默认 false，并安排运行时创建 Gazebo 进程。 |
| 31 | 启动 robot_state_publisher。 |
| 32 | 读取生成的 URDF 文本作为 robot_description，使用仿真时间。 |
| 33 | 列出世界 TF、参考场景、点云修正、飞行控制、体素地图和导航模块。 |
| 34 | 以 Python 模块方式启动每个模块，并传入 use_sim_time=true。 |
| 35 | 创建主要 ROS/Gazebo 消息桥。 |
| 36 | 开始话题与类型映射列表。 |
| 37 | ROS 小球 Twist 命令单向传到 Gazebo。 |
| 38 | Gazebo 小球位姿数组转换为 ROS TFMessage。 |
| 39 | Gazebo 木块位姿数组转换为 ROS TFMessage。 |
| 40 | Gazebo 仿真时钟转到 ROS /clock。 |
| 41 | 自定义插件的 Model 关节位置转换为 ROS JointState。 |
| 42 | 底部 RGB 图传到 ROS。 |
| 43 | 底部深度图传到 ROS。 |
| 44 | 底部相机内参传到 ROS。 |
| 45 | 保留前向 RGB 相机的桥接。 |
| 46 | 保留前向深度相机的桥接。 |
| 47 | 保留前向相机内参。 |
| 48 | 前向点云转成 PointCloud2，结束映射列表。 |
| 49 | 把关节状态重命名为 /joint_states，把小球真实位姿重命名为 /simulation/ground_truth/poses。 |
| 50 | 前向点云先进入 points_raw，后续由已有修正节点处理坐标系标签。 |
| 51 | 另启动专门的关节目标桥。 |
| 52 | 读取 config/arm_bridge.yaml，避免在 ROS 话题中使用以数字开头的原生路径片段 /0/。 |
| 53 | 建立前向相机相对 base_link 的固定安装变换。 |
| 54–55 | 建立前向相机的 optical 坐标轴旋转；底部相机的固定变换已经写进 URDF，不在此重复广播。 |
| 56 | 把所有动作返回给 ROS launch。 |

<a id="module-11"></a>

## 11. 统一命令入口

源文件：[ghost.sh](../ghost.sh)（31 行）。

所有命令都从脚本所在目录确定项目路径；`"$@"` 原样转发剩余参数。`exec` 用目标程序替换当前 shell，使退出码和 Ctrl+C 信号更直接地传递。

<!-- source: ghost.sh sha256: c8aa15d6eb853700a098050636b1ab6e62f740dc02f4d31949069bab9d57fce9 -->

```bash
#!/usr/bin/env bash
set -e
GHOST_ENTRY_ROOT="$(cd "$(dirname "$0")" && pwd)"
command_name="${1:-help}"
if [ "$#" -gt 0 ]; then shift; fi
case "$command_name" in
  start) exec "$GHOST_ENTRY_ROOT/scripts/start_sim.sh" "$@" ;;
  view) exec "$GHOST_ENTRY_ROOT/scripts/view_ghost.sh" "$@" ;;
  teleop) exec "$GHOST_ENTRY_ROOT/scripts/teleop.sh" "$@" ;;
  goto) exec "$GHOST_ENTRY_ROOT/scripts/navigate.sh" "$@" ;;
  ai) exec "$GHOST_ENTRY_ROOT/scripts/ghost_ai.sh" "$@" ;;
  stop) exec "$GHOST_ENTRY_ROOT/scripts/cancel_navigation.sh" "$@" ;;
  package) exec "$GHOST_ENTRY_ROOT/scripts/package.sh" "$@" ;;
  grasp|camera|ik|fk|arm|test|check-flight|check-navigation|build-model|map)
    source "$GHOST_ENTRY_ROOT/scripts/env.sh"
    case "$command_name" in
      grasp) exec python3 -m ghost_sim.control.grasp_demo "$@" ;;
      camera) exec python3 -m ghost_sim.perception.depth_camera "$@" ;;
      ik) exec python3 -m ghost_sim.kinematics.ik_cli "$@" ;;
      fk) exec python3 -m ghost_sim.kinematics.fk_cli "$@" ;;
      arm) exec python3 -m ghost_sim.control.arm_pose "$@" ;;
      test) exec python3 -m unittest discover -s "$GHOST_SIM_ROOT/tests/unit" -p 'test_*.py' -v ;;
      check-flight) exec python3 "$GHOST_SIM_ROOT/tests/integration/check_flight.py" "$@" ;;
      check-navigation) exec python3 "$GHOST_SIM_ROOT/tests/integration/check_navigation3d.py" "$@" ;;
      build-model) exec python3 -m ghost_sim.simulation.create_scene "$@" ;;
      map) exec python3 -m ghost_sim.mapping.voxel_map "$@" ;;
    esac ;;
  help|-h|--help)
    echo 'Usage: ./ghost.sh {start|view|teleop|grasp [pick|release]|camera [--pixel U V]|ik X Y Z [--pitch P] [--live|--execute]|fk {--joints Q1 Q2 Q3 Q4|--live}|arm {hang|ready|close}|goto X Y Z [--yaw R]|ai "任务"|stop|map|build-model|test|check-flight|check-navigation|package}' ;;
  *) echo "未知命令：$command_name。运行 ./ghost.sh help 查看用法。" >&2; exit 2 ;;
esac
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 由环境查找 bash 来执行脚本。 |
| 2 | 一般命令失败时立即退出，避免在环境不完整时继续启动。 |
| 3 | 进入脚本目录并解析绝对路径，不依赖用户终端当前目录。 |
| 4 | 第一个参数作为子命令；缺省显示 help。 |
| 5 | 有参数时移走子命令，剩下的是需要转发的参数。 |
| 6 | 按子命令选择分支。 |
| 7 | start 启动后台及可选 GUI，gui:=false/true 会继续传给 launch。 |
| 8 | view 打开 RViz。 |
| 9 | teleop 打开人工飞行控制。 |
| 10 | goto 发送三维导航目标。 |
| 11 | ai 调用已有大模型控制入口。 |
| 12 | stop 取消当前导航，不等同于终止 Gazebo 进程。 |
| 13 | package 生成源码压缩包。 |
| 14 | 这些子命令通过 Python 模块或测试脚本执行。 |
| 15 | 先加载项目统一环境，包括 ROS 域、Python 路径和插件路径。 |
| 16 | 在这组 Python 命令内部再次分支。 |
| 17 | grasp 进入本篇逐句解释的抓取控制器，--pitch-deg 等参数原样转发。 |
| 18 | camera 打开或单次查询底部相机。 |
| 19 | ik 计算逆运动学；是否执行由 Python 参数 --execute 决定。 |
| 20 | fk 计算或验证正运动学。 |
| 21 | arm 执行预设姿态。 |
| 22 | test 搜索 tests/unit 下所有 test_*.py，并显示详细测试结果。 |
| 23 | 执行飞行集成检查脚本。 |
| 24 | 执行导航集成检查脚本。 |
| 25 | 重新生成 SDF 场景和 URDF 显示模型；已运行的 Gazebo 不会自动重载。 |
| 26 | 执行体素地图生成模块。 |
| 27 | 结束内部 case，并结束对应外层分支。 |
| 28 | help、-h、--help 共用帮助分支。 |
| 29 | 显示命令概览；完整角度参数帮助可用 ./ghost.sh grasp --help 查看。 |
| 30 | 无法识别的命令写入标准错误，并返回退出码 2。 |
| 31 | 结束主 case。 |

<a id="module-12"></a>

## 12. 运行环境

源文件：[scripts/env.sh](../scripts/env.sh)（11 行）。

这些变量只设置当前脚本及其子进程的环境。旧 ghost_sim 与新 ghost_arm 共用域和锁，不要同时启动。

<!-- source: scripts/env.sh sha256: 1e6a0feffac8b11c9312efd5ccb94ef9da36950e2bc8ae3a64b2504bb2074c9a -->

```bash
#!/usr/bin/env bash
# Shared by all launchers; uses the checkout location, never the caller's cwd.
GHOST_SIM_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=43
export IGN_PARTITION=ghost_sim
export PYTHONPATH="$GHOST_SIM_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$GHOST_SIM_ROOT/runtime/logs" "$GHOST_SIM_ROOT/runtime/reports" "$GHOST_SIM_ROOT/runtime/maps"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$GHOST_SIM_ROOT/runtime/build/plugins${IGN_GAZEBO_SYSTEM_PLUGIN_PATH:+:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"
export LP_NUM_THREADS=2
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 使用 bash。 |
| 2 | 说明路径以项目位置为准。 |
| 3 | BASH_SOURCE[0] 指当前环境脚本，向上一级得到项目根目录。 |
| 4 | 加载 ROS 2 Humble 的工具、库和环境。 |
| 5 | ROS_DOMAIN_ID=43 将本项目 ROS 通信与其他域隔离。 |
| 6 | IGN_PARTITION=ghost_sim 设置 Gazebo Transport 分区，需与后台一致。 |
| 7 | 把项目 src 加到 Python 搜索路径前面，同时保留原有路径。 |
| 8 | 创建日志、报告和地图目录；-p 允许目录已存在。 |
| 10 | 把自动编译的插件目录加入 Gazebo 系统插件搜索路径，并保留用户原路径。 |
| 11 | 限制 Mesa llvmpipe 软件渲染线程数量为 2，减少多窗口争抢 CPU；不是启用硬件加速。 |

<a id="module-13"></a>

## 13. 启动与防止重复仿真

源文件：[scripts/start_sim.sh](../scripts/start_sim.sh)（7 行）。

只需运行 ./ghost.sh start；自定义插件会自动构建，不需要先手动执行 colcon。首次构建需要 CMake、C++ 编译器和 Gazebo 开发依赖。

<!-- source: scripts/start_sim.sh sha256: 88cd0857e4ec4b10ad9374572c0d2774daa97aeb0f5767368f2f3c6bd6f1f555 -->

```bash
#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec 9> /tmp/ghost_sim.lock
flock -n 9 || { echo '幽灵仿真已经运行。'; exit 1; }
bash "$GHOST_SIM_ROOT/scripts/build_plugins.sh" || { cat "$GHOST_SIM_ROOT/runtime/logs/plugin_build.log" >&2; exit 1; }
exec ros2 launch "$GHOST_SIM_ROOT/launch/sim.launch.py" "$@"
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 使用 bash。 |
| 2 | 遇到一般错误停止。 |
| 3 | 加载同目录环境脚本。 |
| 4 | 把文件描述符 9 绑定到仿真锁文件，后续子进程继承它。 |
| 5 | 尝试非阻塞锁；已有仿真时提示并退出。 |
| 6 | 自动编译插件；失败时把构建日志输出到标准错误并终止，不忽略缺失插件。 |
| 7 | 启动指定 launch 文件，转发 gui:=false 等参数。 |

<a id="module-14"></a>

## 14. 自动编译插件

源文件：[scripts/build_plugins.sh](../scripts/build_plugins.sh)（5 行）。

构建产物都放在 runtime，不修改系统插件。源码包不携带本机二进制，换机器后由启动脚本重新构建。

<!-- source: scripts/build_plugins.sh sha256: 7e500dc22d2579ff6e34cf9b835c584a6bd62120a8baf3c6388f3bde64fa4f30 -->

```bash
#!/usr/bin/env bash
set -e
GHOST_PLUGIN_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cmake -S "$GHOST_PLUGIN_ROOT/src/gazebo_plugins" -B "$GHOST_PLUGIN_ROOT/runtime/build/plugins" -DCMAKE_BUILD_TYPE=Release > "$GHOST_PLUGIN_ROOT/runtime/logs/plugin_build.log" 2>&1
cmake --build "$GHOST_PLUGIN_ROOT/runtime/build/plugins" -j2 >> "$GHOST_PLUGIN_ROOT/runtime/logs/plugin_build.log" 2>&1
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 使用 bash 执行。 |
| 2 | 任一步构建失败就停止。 |
| 3 | 根据脚本路径确定项目根目录。 |
| 4 | CMake -S 指源码目录，-B 指构建目录，Release 开启优化；标准输出和错误都写到构建日志。 |
| 5 | 执行增量构建，最多并行两个编译任务；追加日志而非覆盖配置阶段日志。 |

<a id="module-15"></a>

## 15. CMake 如何生成共享库

源文件：[src/gazebo_plugins/CMakeLists.txt](../src/gazebo_plugins/CMakeLists.txt)（6 行）。

生成的文件是 libghost_joint_feedback.so，与 SDF 中的 filename 一致。

<!-- source: src/gazebo_plugins/CMakeLists.txt sha256: abe740afc2ab3efd7c675dbcd9afbc77f384ef60722027aaa90a316727463404 -->

```cmake
cmake_minimum_required(VERSION 3.16)
project(ghost_arm_plugins)
find_package(ignition-gazebo6 REQUIRED)
add_library(ghost_joint_feedback SHARED joint_feedback.cc)
target_link_libraries(ghost_joint_feedback PRIVATE ignition-gazebo6::ignition-gazebo6)
set_property(TARGET ghost_joint_feedback PROPERTY CXX_STANDARD 17)
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 要求 CMake 至少为 3.16。 |
| 2 | 声明构建工程名称。 |
| 3 | 寻找 Fortress 对应 ignition-gazebo6 开发配置；REQUIRED 表示缺失就失败。 |
| 4 | 将 joint_feedback.cc 编译为共享库，而不是独立程序。 |
| 5 | 链接 Gazebo 的导入目标，由它带入相应头文件和依赖库。 |
| 6 | 该目标使用 C++17 标准。 |

<a id="module-16"></a>

## 16. 只打开 RViz

源文件：[scripts/view_ghost.sh](../scripts/view_ghost.sh)（4 行）。

RViz 订阅模型、TF 和图像，不计算碰撞或生成传感器。必须先有仿真后台，但不必打开 Gazebo GUI。

<!-- source: scripts/view_ghost.sh sha256: ea19230a38a9604a6b656ec3c1cdf37ec26f61bfa7dc1e8575e829ab81711b79 -->

```bash
#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
exec rviz2 -d "$GHOST_SIM_ROOT/config/ghost_camera.rviz" --ros-args -p use_sim_time:=true
```

| 源文件行号 | 逐句说明 |
| --- | --- |
| 1 | 使用 bash。 |
| 2 | 执行失败时停止。 |
| 3 | 使用项目统一环境，尤其需要相同 ROS_DOMAIN_ID。 |
| 4 | 加载底部相机专用 RViz 配置，并使用 /clock 的仿真时间。 |

## 其他配置、数学推导和验证入口

- [DH 参数与正运动学](DH参数与正运动学逐句讲解.md)：标准 DH 矩阵、工具坐标修正、FK 与仿真比较。数学示例角度不一定是当前抓取姿态。
- [解析逆运动学](逆运动学逐句讲解.md)：偏航、平面二连杆两分支、腕部补角与限位。这里求的是数学解，执行层再筛选球体路径。
- [底部相机接入记录](底部深度相机接入与逐句讲解.md)：光学坐标、内参校准、深度解码与反投影。当前窗口代码以本篇为准。
- [倾斜抓取与卡顿修正](倾斜抓取与相机卡顿修正.md)：故障原因和本机验证记录。
- [木块抓取操作](木块抓取操作与代码讲解.md)：接触、重量、松开与历史迭代记录。
- `config/arm_bridge.yaml`：六条 ROS Float64 到 Gazebo Double 的映射。原生话题中含 `/0/`，ROS 名称不用这个数字开头的路径片段。
- `config/ghost_camera.rviz`：底部图像话题、自动深度范围、5 FPS 显示；完整体素层默认关闭以降低渲染负载。
- `config/arm_gazebo.config`：Gazebo 视角与关节控制面板，属于 GUI 配置，不改变连杆或物理参数。
- `simulation/create_scene.py`：在已有世界中调用 `attach(ghost, urdf)` 与 `attach_wood(world)`，写出 SDF 和 URDF。更改结构后运行 `./ghost.sh build-model` 并重启仿真才生效。

### 参数与边界一览

| 项目 | 当前值或行为 |
| --- | --- |
| 抓取默认末端俯仰 | −60°；允许 −60°～0° |
| 抓取中心目标（球体坐标） | (0, 0, −0.46) m |
| 计划/实测球体间隙阈值 | 5 mm / 2 mm |
| 关节路径采样步长 | 最大单轴变化 0.01；离散检查，不是连续证明 |
| 手指最低角点离地目标 | 至少 8 mm；按倾斜几何计算 |
| 木块 | 4×4×6 cm，40 g，初始中心 (0.10,0,0.03) m |
| 木块重量 | −0.3924 N，单独作用；世界总体仍为零重力 |
| 物理步长 / 关节反馈 | 0.002 s / 50 Hz（均按仿真时间） |
| 相机 | 320×240，5 Hz，RGB-D 与内参按精确时间戳配对 |
| 渲染 | RViz 上限 5 FPS，独立相机绘图上限 10 FPS |
| release | 原地张开后自由落地，不是受控轻放 |
| 当前未实现 | 视觉识别闭环抓取、任意物体抓取、完整机械臂环境避障 |

### 检查命令

```bash
cd /home/ubuntu/ghost_arm
./ghost.sh test
./ghost.sh fk --live
./ghost.sh camera --pixel 160 100
source scripts/env.sh
python3 tests/integration/check_rgbd_stream.py
```

单元测试不要求启动仿真；后面三种测量需要仿真运行。单次像素测量只有深度有效时才会成功。历史实测记录：−60° 抓取抬升约 23.55 cm、运动连杆最小球体间隙约 2.20 cm；41 项单元测试通过。这些是已有验证记录，本次只更新文档，没有重新驱动机械臂。

### 常见报错怎样读

- “反馈中断”：查看具体是 joints、base 还是 block，以及多久没更新；可能是仿真暂停、通信中断或虚拟机调度卡顿。不是继续夹紧的信号。
- “没有避开球体的直接关节路径”：末端可达不代表整条机械臂可直接到达，不能绕过筛选强行执行。
- “关节运动超时”：实际位置没有在期限内到目标。抓着物块时不能用普通零位到达条件判断夹紧；手指积分已关闭，避免长时间夹持积累夹紧力。
- “STALE CAMERA”：独立窗口最近 2 秒没取得新同步帧，会隐藏旧图；仿真重启时缓存自动清空。

早期结构版本保留在 [历史讲解](Ghost_Arm代码逐句讲解_早期结构版本.md)，用于对照演变，不作为当前代码和指令依据。
