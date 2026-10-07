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
