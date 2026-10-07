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
