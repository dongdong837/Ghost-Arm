"""XYZ goal execution over a known static map; manual nonzero input cancels."""
from ghost_sim.paths import REPORTS
import json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Path as RosPath
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ghost_sim.navigation.planner3d import Planner


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('ghost_navigation3d');planner=Planner()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    velocity=node.create_publisher(Twist,'/ghost/nav_cmd_vel',1)
    pathpub=node.create_publisher(RosPath,'/ghost/path',qos)
    statuspub=node.create_publisher(String,'/ghost/navigation_status',qos)
    state={'pose':None,'pose_time':0.,'active':False,'id':'','trajectory':[]}
    def status(code,detail=''):
        report={'id':state['id'],'status':code,'detail':detail}
        if state['pose'] is not None:report['pose']=state['pose']
        statuspub.publish(String(data=json.dumps(report,ensure_ascii=False)))
        node.get_logger().info(code+': '+detail)
    def finish(code,detail=''):
        state['active']=False;velocity.publish(Twist());status(code,detail)
        if state.get('goal') is not None:
            report={k:state[k] for k in ['id','goal','trajectory','route']}
            report.update(status=code,detail=detail,arrival=state['pose'])
            report['position_error_m']=float(np.linalg.norm(np.array(state['pose'][:3])-state['goal'][:3])) if state['pose'] else None
            (REPORTS/'navigation3d_result.json').write_text(json.dumps(report,indent=2)+'\n')
    def poses(msg):
        for item in msg.transforms:
            if item.child_frame_id=='ghost':
                p,q=item.transform.translation,item.transform.rotation
                state['pose']=[p.x,p.y,p.z,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]
                state['pose_time']=time.monotonic();break
    def goal(msg):
        if state['active']:finish('CANCELED','被新目标替换')
        state['id']=str(msg.header.stamp.sec)+':'+str(msg.header.stamp.nanosec)
        state['goal']=None
        try:
            if msg.header.frame_id!='simulation_reference':raise ValueError('目标坐标系必须是 simulation_reference。')
            if state['pose'] is None or time.monotonic()-state['pose_time']>.5:raise ValueError('定位未就绪。')
            p,q=msg.pose.position,msg.pose.orientation
            if not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x)+abs(q.y)>1e-5 or abs(q.z*q.z+q.w*q.w-1)>1e-3:raise ValueError('只支持有效的水平 yaw 朝向。')
            target=[p.x,p.y,p.z];route=planner.plan(state['pose'][:3],target)
            yaw=2*math.atan2(q.z,q.w)
            state.update(goal=target+[yaw],route=route,index=1,active=True,trajectory=[],started=time.monotonic(),progress=time.monotonic(),anchor=state['pose'][:3])
            path=RosPath();path.header=msg.header
            for point in route:
                pose=PoseStamped();pose.header=msg.header;pose.pose.orientation.w=1.
                pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,point)
                path.poses.append(pose)
            pathpub.publish(path);status('ACTIVE',f'三维路线 {len(route)} 个路点')
        except ValueError as e:
            state['active']=False;velocity.publish(Twist());status('REJECTED',str(e))
    def cancel(request,response):
        if state['active']:finish('CANCELED','用户取消')
        else:velocity.publish(Twist())
        response.success=True;response.message='已停止导航';return response
    def manual(msg):
        if state['active'] and max(abs(msg.linear.x),abs(msg.linear.y),abs(msg.linear.z),abs(msg.angular.z))>1e-6:finish('CANCELED','手动控制接管')
    def tick():
        if not state['active']:return
        now=time.monotonic()
        if now-state['pose_time']>.5:return finish('FAILED','定位超时，已停止')
        current=np.array(state['pose'][:3]);state['trajectory'].append(state['pose'][:])
        if now-state['started']>180:return finish('FAILED','导航超过 180 秒')
        if np.linalg.norm(current-state['anchor'])>.08:state.update(anchor=current.tolist(),progress=now)
        if now-state['progress']>20:return finish('FAILED','长时间没有位移进展')
        route=state['route'];i=state['index'];target=np.array(route[i]);distance=np.linalg.norm(target-current)
        if distance<.045 and i<len(route)-1:
            # A tracking shortcut is allowed only after checking the whole new segment.
            if planner.segment_clear(current,route[i+1]):
                state['index']+=1;target=np.array(route[i+1]);distance=np.linalg.norm(target-current)
        if not planner.segment_clear(current,target):return finish('FAILED','实际位置到路点的连线不安全')
        msg=Twist();error=state['goal'][3]-state['pose'][3];error=math.atan2(math.sin(error),math.cos(error))
        final=state['index']==len(route)-1
        if final and distance<.06:
            if abs(error)<.10:return finish('SUCCEEDED','已到达三维目标并悬停')
        else:
            v=(target-current)*min(1.2,.28/max(distance,1e-9))
            msg.linear.x,msg.linear.y,msg.linear.z=map(float,v)
        msg.angular.z=float(np.clip(error,-.6,.6))
        velocity.publish(msg)
    subs=[node.create_subscription(TFMessage,'/simulation/ground_truth/poses',poses,10),
          node.create_subscription(PoseStamped,'/ghost/goal',goal,1),node.create_subscription(Twist,'/ghost/cmd_vel',manual,1)]
    service=node.create_service(Trigger,'/ghost/cancel_navigation',cancel)
    timer=node.create_timer(.05,tick,clock=Clock(clock_type=ClockType.STEADY_TIME));status('IDLE')
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if state['active']:finish('CANCELED','导航节点关闭')
        velocity.publish(Twist());node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
