"""Publish one XYZ/yaw goal, monitor its matching status, cancel on Ctrl+C."""
import argparse,json,math,time
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


def main():
    parser=argparse.ArgumentParser(description='XYZ are metres; yaw is radians')
    for name in ['x','y','z']:parser.add_argument(name,type=float)
    parser.add_argument('--yaw',type=float,default=0.)
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in [args.x,args.y,args.z,args.yaw]):parser.error('坐标和朝向必须是有限数')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('ghost_goal_client')
    pub=node.create_publisher(PoseStamped,'/ghost/goal',1)
    cancel=node.create_client(Trigger,'/ghost/cancel_navigation')
    stamp=node.get_clock().now().to_msg();identifier=str(stamp.sec)+':'+str(stamp.nanosec)
    results=[]
    def receive(msg):
        data=json.loads(msg.data)
        if data['id']==identifier:
            results.append(data);print(json.dumps(data,ensure_ascii=False),flush=True)
    sub=node.create_subscription(String,'/ghost/navigation_status',receive,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sent=False;complete=False
    try:
        end=time.monotonic()+10
        while (pub.get_subscription_count()==0 or not cancel.service_is_ready()) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if pub.get_subscription_count()==0 or not cancel.service_is_ready():raise RuntimeError('请先启动幽灵仿真及 navigation3d 节点')
        # Allow the status subscriber to complete discovery before sending.
        ready=time.monotonic()+1
        while time.monotonic()<ready:rclpy.spin_once(node,timeout_sec=.05)
        goal=PoseStamped();goal.header.frame_id='simulation_reference';goal.header.stamp=stamp
        goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=args.x,args.y,args.z
        goal.pose.orientation.z=math.sin(args.yaw/2);goal.pose.orientation.w=math.cos(args.yaw/2)
        pub.publish(goal);sent=True;end=time.monotonic()+190
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
            if results and results[-1]['status'] in ['SUCCEEDED','FAILED','REJECTED','CANCELED']:
                complete=True;return 0 if results[-1]['status']=='SUCCEEDED' else 1
        raise RuntimeError('目标执行超时')
    except (KeyboardInterrupt,RuntimeError) as error:
        print(str(error) or '已中断，正在取消导航');return 1
    finally:
        if sent and not complete and cancel.service_is_ready():
            future=cancel.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':raise SystemExit(main())
