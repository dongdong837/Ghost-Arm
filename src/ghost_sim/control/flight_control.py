"""World-XYZ manual velocity control, attitude levelling and command watchdog."""
from ghost_sim.paths import WORLD
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage


def rotation(q):
    x,y,z,w = q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ghost_flight_control')
    output = node.create_publisher(Twist,'/model/ghost/cmd_vel',1)
    state = {'command':np.zeros(4),'received':0.,'pose':None,'pose_time':0.,'manual_time':0.,'nav_time':0.}
    obstacles = []
    for model in ET.parse(WORLD).findall('world/model'):
        if model.findtext('static') == 'true':
            centre = np.array(list(map(float,model.findtext('pose').split()))[:3])
            half = np.array(list(map(float,model.findtext('link/visual/geometry/box/size').split())))/2
            obstacles.append((centre,half))

    def receive(msg, navigation=False):
        now=time.monotonic()
        nonzero=max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.linear.z),abs(msg.angular.z))>1e-6
        if navigation:
            if now-state['manual_time']<.4:return
            state['nav_time']=now
        elif nonzero:
            state['manual_time']=now
        elif now-state['nav_time']<.25:
            return  # A keyboard's idle zeros must not overwrite autonomous flight.
        command = np.array([msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z])
        if not np.isfinite(command).all():
            command[:] = 0
        speed = np.linalg.norm(command[:3])
        if speed > .45:
            command[:3] *= .45/speed
        command[2] = np.clip(command[2],-.35,.35)
        command[3] = np.clip(command[3],-.8,.8)
        state.update(command=command,received=time.monotonic())

    def poses(msg):
        for pose in msg.transforms:
            if pose.child_frame_id == 'ghost':
                state.update(pose=pose.transform,pose_time=time.monotonic())
                break

    def tick():
        msg = Twist()
        now = time.monotonic()
        if state['pose'] is not None and now-state['pose_time'] < .5:
            pose = state['pose']; r = rotation(pose.rotation)
            command = state['command'].copy() if now-state['received'] < .4 else np.zeros(4)
            p = np.array([pose.translation.x,pose.translation.y,pose.translation.z])
            # Short look-ahead stop, not route planning. Geometry is axis-aligned.
            predicted = p + command[:3]*.3
            if any(np.linalg.norm(np.maximum(np.abs(predicted-c)-h,0)) < .33 for c,h in obstacles):
                command[:3] = 0
            body = r.T @ command[:3]  # Gazebo expects model-relative velocity.
            msg.linear.x,msg.linear.y,msg.linear.z = map(float,body)
            roll = math.atan2(r[2,1],r[2,2])
            pitch = math.asin(float(np.clip(-r[2,0],-1,1)))
            angular = r.T @ np.array([-2*roll,-2*pitch,command[3]])
            msg.angular.x,msg.angular.y,msg.angular.z = map(float,angular)
        output.publish(msg)

    cmd_sub = node.create_subscription(Twist,'/ghost/cmd_vel',receive,1)
    nav_sub = node.create_subscription(Twist,'/ghost/nav_cmd_vel',lambda msg:receive(msg,True),1)
    pose_sub = node.create_subscription(TFMessage,'/simulation/ground_truth/poses',poses,10)
    timer = node.create_timer(.05,tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            output.publish(Twist()); time.sleep(.02)
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__':
    main()
