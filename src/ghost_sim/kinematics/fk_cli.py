"""Print offline DH FK or verify it against time-matched Gazebo poses."""
import argparse
import json
import time
import numpy as np
from ghost_sim.kinematics.forward import forward_kinematics, load_parameters


def pose_matrix(transform):
    """Convert a ROS Transform into a homogeneous transform."""
    t, q = transform.translation, transform.rotation
    x, y, z, w = np.array([q.x, q.y, q.z, q.w]) / np.linalg.norm([q.x, q.y, q.z, q.w])
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def live_sample():
    """Read only: never send joint or flight commands."""
    import rclpy
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    node = rclpy.create_node('ghost_arm_fk_check')
    states, poses = {}, {}
    stamp = lambda s: s.sec * 1000000000 + s.nanosec
    def joints(message):
        states[stamp(message.header.stamp)] = dict(zip(message.name, message.position))
        while len(states) > 2000:
            del states[next(iter(states))]
    def physical(message):
        for item in message.transforms:
            if item.child_frame_id in ('ghost/base_link', 'ghost/grasp_center'):
                poses.setdefault(stamp(item.header.stamp), {})[item.child_frame_id] = item
        while len(poses) > 200:
            del poses[next(iter(poses))]
    subscriptions = [
        node.create_subscription(JointState, '/joint_states', joints, 100),
        node.create_subscription(TFMessage, '/simulation/ground_truth/poses', physical, 100),
    ]
    names = [row['joint'] for row in load_parameters()['rows']]
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for key in sorted(states.keys() & poses.keys(), reverse=True):
                pair = poses[key]
                if len(pair) != 2 or not all(name in states[key] for name in names):
                    continue
                base, tip = pair['ghost/base_link'], pair['ghost/grasp_center']
                if base.header.frame_id != tip.header.frame_id:
                    raise RuntimeError('Gazebo link poses do not share a parent frame.')
                actual = np.linalg.inv(pose_matrix(base.transform)) @ pose_matrix(tip.transform)
                return [states[key][name] for name in names], actual, key
        raise RuntimeError('No time-matched joint/pose messages. Start Ghost Arm simulation first.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--joints', nargs=4, type=float, metavar=('YAW', 'SHOULDER', 'ELBOW', 'WRIST'))
    choice.add_argument('--live', action='store_true')
    args = parser.parse_args()
    actual = None
    if args.live:
        q, actual, timestamp = live_sample()
    else:
        q = args.joints
    calculated = forward_kinematics(q)
    report = {'parent_frame': 'base_link', 'child_frame': 'grasp_center',
              'joint_angles_rad': q, 'position_m': calculated[:3, 3].tolist(),
              'transform': calculated.tolist()}
    if actual is not None:
        distance = float(np.linalg.norm(calculated[:3, 3] - actual[:3, 3]))
        cosine = (np.trace(calculated[:3, :3].T @ actual[:3, :3]) - 1) / 2
        angle = float(np.arccos(np.clip(cosine, -1, 1)))
        report.update(simulation_stamp_ns=timestamp, gazebo_position_m=actual[:3, 3].tolist(),
                      position_error_m=distance, orientation_error_rad=angle,
                      passed=distance < 1e-4 and angle < 1e-3)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report.get('passed') is False:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
