"""Read-only check of exact RGB-D synchronization and wall/simulation throughput."""
import json
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from tf2_msgs.msg import TFMessage
from ghost_sim.perception.rgbd_sync import RGBDSync
from ghost_sim.paths import REPORTS

rclpy.init()
node=rclpy.create_node('check_rgbd_stream')
sync=RGBDSync()
counts={}
stamps={}
pairs=[]
def receive(message,kind):
    header=message.transforms[0].header if kind=='poses' else message.header
    stamp=header.stamp.sec*1000000000+header.stamp.nanosec
    stamps.setdefault(kind,[]).append(stamp)
    counts[kind]=counts.get(kind,0)+1
    if kind in ('rgb','depth','info'):
        sync.push(kind,stamp,message)
        pair=sync.latest()
        if pair is not None:
            pairs.append(pair['depth'].header.stamp.sec+pair['depth'].header.stamp.nanosec*1e-9)
subscriptions=[]
for kind,topic,typ in [('rgb','/grasp_camera/image',Image),('depth','/grasp_camera/depth_image',Image),
                       ('info','/grasp_camera/camera_info',CameraInfo),('joints','/joint_states',JointState),
                       ('poses','/simulation/ground_truth/poses',TFMessage)]:
    subscriptions.append(node.create_subscription(typ,topic,lambda m,k=kind:receive(m,k),qos_profile_sensor_data))
start=time.monotonic()
while time.monotonic()-start<10:rclpy.spin_once(node,timeout_sec=.02)
elapsed=time.monotonic()-start
joint=stamps.get('joints',[])
report={'wall_seconds':elapsed,'counts':counts,'exact_rgbd_pairs':len(pairs),
        'matched_joint_pose_stamps':len(set(joint)&set(stamps.get('poses',[])))}
if len(joint)>1:
    delta=(joint[-1]-joint[0])*1e-9
    report.update(joint_sim_hz=(len(joint)-1)/delta,approx_real_time_factor=delta/elapsed)
report['passed']=len(pairs)>=3 and report['matched_joint_pose_stamps']>0 and 45<report.get('joint_sim_hz',0)<55
(REPORTS/'rgbd_stream_check.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
