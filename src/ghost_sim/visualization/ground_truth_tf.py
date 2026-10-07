"""Use Gazebo physical poses for the known-scene RViz demonstration, not SLAM."""
from copy import deepcopy
import rclpy
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init()
    node = rclpy.create_node('simulation_ground_truth_tf')
    broadcaster = TransformBroadcaster(node)

    def receive(message):
        by_child = {item.child_frame_id: item for item in message.transforms}
        if 'ghost' not in by_child or 'ghost/base_link' not in by_child:
            return
        model = deepcopy(by_child['ghost'])
        body = deepcopy(by_child['ghost/base_link'])
        # Preserve the simulator timestamp and full 3D rotation/translation.
        model.header.frame_id = 'simulation_reference'
        model.child_frame_id = 'simulation_ghost'
        body.header.frame_id = 'simulation_ghost'
        body.child_frame_id = 'base_link'
        broadcaster.sendTransform([model, body])

    subscription = node.create_subscription(
        TFMessage, '/simulation/ground_truth/poses', receive, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
