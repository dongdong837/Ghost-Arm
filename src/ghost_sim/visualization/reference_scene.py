"""Publish known simulation geometry, explicitly NOT a SLAM reconstruction."""
from ghost_sim.paths import WORLD
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
from ghost_sim.simulation.wood_block import SIZE


def main():
    rclpy.init()
    node = rclpy.create_node('simulation_reference_scene')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    scene_pub = node.create_publisher(MarkerArray, '/simulation/reference_scene', qos)
    world = ET.parse(WORLD).find('world')
    markers = MarkerArray()
    for model in world.findall('model'):
        if model.findtext('static') != 'true':
            continue
        pose = [float(v) for v in model.findtext('pose').split()]
        assert pose[3:] == [0, 0, 0], 'Reference rasterizer supports axis-aligned boxes only'
        visual = model.find('link/visual')
        size = [float(v) for v in visual.findtext('geometry/box/size').split()]
        color = [float(v) for v in visual.findtext('material/diffuse').split()]
        marker = Marker()
        marker.header.frame_id = 'simulation_reference'
        marker.ns = 'known_simulation_geometry'
        marker.id = len(markers.markers)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pose[:3]
        marker.pose.orientation.w = 1.0
        marker.scale.x, marker.scale.y, marker.scale.z = size
        marker.color.r, marker.color.g, marker.color.b = color[:3]
        marker.color.a = 0.04 if model.get('name') == 'ceiling' else (0.12 if model.get('name') in ('north','south','east','west') else 0.65)
        markers.markers.append(marker)
    wood = Marker()
    wood.header.frame_id = 'simulation_reference'
    wood.ns = 'dynamic_wood_block'
    wood.id = 0
    wood.type = Marker.CUBE
    wood.action = Marker.ADD
    wood.scale.x, wood.scale.y, wood.scale.z = SIZE
    wood.color.r, wood.color.g, wood.color.b, wood.color.a = .65, .35, .12, 1.
    def update_wood(message):
        for pose in message.transforms:
            if pose.child_frame_id == 'wood_block':
                wood.pose.position.x = pose.transform.translation.x
                wood.pose.position.y = pose.transform.translation.y
                wood.pose.position.z = pose.transform.translation.z
                wood.pose.orientation = pose.transform.rotation
                if not any(m.ns == wood.ns for m in markers.markers):
                    markers.markers.append(wood)
    wood_sub = node.create_subscription(TFMessage, '/model/wood_block/pose', update_wood, 10)
    def publish():
        now = node.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = now
        scene_pub.publish(markers)

    timer = node.create_timer(.2, publish)
    publish()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
