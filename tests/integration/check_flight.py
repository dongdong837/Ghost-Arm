"""Integration check: moves ghost in a clear central room; stop keyboard first."""
from ghost_sim.paths import REPORTS
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('check_ghost_flight')
    pub = node.create_publisher(Twist,'/ghost/cmd_vel',1)
    state = {}; counts = {'rgb':0,'depth':0}
    def poses(msg):
        for t in msg.transforms:
            if t.child_frame_id == 'ghost':
                p,q = t.transform.translation,t.transform.rotation
                state['pose'] = [p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
    def camera(key):
        def receive(msg):
            if msg.width==320 and msg.height==240 and len(msg.data)>0:
                counts[key] += 1
        return receive
    subscriptions = [node.create_subscription(TFMessage,'/simulation/ground_truth/poses',poses,10),
                     node.create_subscription(Image,'/camera/image',camera('rgb'),qos_profile_sensor_data),
                     node.create_subscription(Image,'/camera/depth_image',camera('depth'),qos_profile_sensor_data)]
    def run(seconds,values=None):
        end = time.monotonic()+seconds
        next_command = 0.
        while time.monotonic()<end:
            if values is not None and time.monotonic()>=next_command:
                msg = Twist()
                msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z = map(float,values)
                pub.publish(msg); next_command = time.monotonic()+.05
            rclpy.spin_once(node,timeout_sec=.02)
    results = []
    try:
        run(3,[0,0,0,0])
        assert 'pose' in state, 'No simulation pose received'
        initial = state['pose'][:]
        assert math.hypot(*initial[:2])<.5 and .8<initial[2]<1.6, 'Run in the clear central starting area'
        run(1,[0,0,0,0]); p=state['pose'][:]
        assert math.dist(initial[:3],p[:3])<.02,'Ghost fell or drifted while hovering'
        for label,values,axis,sign in [('up',[0,0,.25,0],2,1),('down',[0,0,-.25,0],2,-1),
                                       ('x+',[.25,0,0,0],0,1),('x-',[-.25,0,0,0],0,-1),
                                       ('y+',[0,.25,0,0],1,1),('y-',[0,-.25,0,0],1,-1),
                                       ('turn',[0,0,0,.6],3,1),('world_x_after_turn',[.25,0,0,0],0,1),
                                       ('return_x',[-.25,0,0,0],0,-1)]:
            before = state['pose'][:]
            run(1.5,values); run(.2,[0,0,0,0])
            after = state['pose'][:]
            delta = [b-a for a,b in zip(before,after)]
            assert delta[axis]*sign>.1,(label,delta)
            if axis != 3:
                assert max(abs(delta[i]) for i in range(3) if i!=axis)<.04,(label,delta)
            results.append({'phase':label,'delta':delta})
        run(.5,[0,0,.2,0]); run(1.)  # stop publishing: flight node must stop on timeout.
        before=state['pose'][:]; run(1.)
        drift=math.dist(before[:3],state['pose'][:3])
        assert drift<.02,('Command watchdog did not stop',drift)
        run(7,[0,0,.35,0]); high=state['pose'][2]
        assert 2.35<high<2.72,('Ceiling clearance guard',high)
        run(.5,[0,0,0,0])
        deadline=time.monotonic()+12
        while state['pose'][2]>1.25 and time.monotonic()<deadline:
            run(.1,[0,0,-.25,0])
        run(.4,[0,0,0,0])
        assert 1.15<state['pose'][2]<1.3, 'Failed to return to viewing height'
        assert min(counts.values())>5,counts
        report={'result':'PASS','phases':results,'timeout_hover_drift_m':drift,
                'ceiling_stop_height_m':high,'camera_frames':counts,'final_pose':state['pose']}
        path=REPORTS/'flight_validation.json'
        path.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    finally:
        run(.3,[0,0,0,0])
        node.destroy_node(); rclpy.try_shutdown()

if __name__=='__main__':
    main()
