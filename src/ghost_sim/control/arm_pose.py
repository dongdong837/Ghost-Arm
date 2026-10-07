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
