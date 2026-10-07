"""View synchronized bottom RGB-D; click to measure a surface point in base_link."""
import argparse
import json
import time
import numpy as np
from ghost_sim.perception.depth_geometry import decode_depth, backproject
from ghost_sim.kinematics.fk_cli import pose_matrix
from ghost_sim.paths import REPORTS
from ghost_sim.perception.rgbd_sync import RGBDSync


def main():
    import rclpy
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from geometry_msgs.msg import PointStamped
    from tf2_ros import Buffer, TransformException
    from tf2_msgs.msg import TFMessage
    from cv_bridge import CvBridge
    import cv2
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pixel',nargs=2,type=int,metavar=('U','V'),help='measure once without opening a window')
    args=parser.parse_args()
    rclpy.init();node=rclpy.create_node('ghost_bottom_camera')
    buffer=Buffer()
    # The camera is fixed to base_link: only static extrinsics are needed.
    # Avoid queuing world/body TF and stale-time warnings after a world restart.
    def fixed_transforms(message):
        for transform in message.transforms:buffer.set_transform_static(transform,'robot_static_tf')
    listener=node.create_subscription(TFMessage,'/tf_static',fixed_transforms,
        QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    bridge=CvBridge();sync=RGBDSync();view={'frame':None,'received':0,'clicked':None,'text':'Click RGB to measure a surface point'}
    publisher=node.create_publisher(PointStamped,'/ghost/grasp_camera/selected_point',10)
    def key(header):return header.stamp.sec*1000000000+header.stamp.nanosec
    def receive(message,kind):
        previous=sync.resets
        sync.push(kind,key(message.header),message)
        if sync.resets!=previous:
            view.update(frame=None,received=0,clicked=None,text='Simulation restarted; waiting for RGB-D')
            node.get_logger().info('Simulation time reset: cleared RGB-D cache')
    def calibration(message):receive(message,'info')
    subscriptions=[node.create_subscription(Image,'/grasp_camera/'+topic,lambda m,k=k:receive(m,k),qos_profile_sensor_data)
                   for topic,k in [('image','rgb'),('depth_image','depth')]]
    subscriptions.append(node.create_subscription(CameraInfo,'/grasp_camera/camera_info',calibration,qos_profile_sensor_data))
    def measure(u,v):
        if view['frame'] is None or time.monotonic()-view['received']>2:
            raise ValueError('相机数据未就绪或已过期。')
        pair,cam,depth=view['frame']
        if any(abs(d)>1e-10 for d in cam.d):raise ValueError('当前工具只支持已校正、无畸变图像。')
        if cam.width!=depth.shape[1] or cam.height!=depth.shape[0]:raise ValueError('内参与图像尺寸不一致。')
        if pair['rgb'].header.frame_id!=pair['depth'].header.frame_id or cam.header.frame_id!=pair['depth'].header.frame_id:
            raise ValueError('RGB、深度与内参的坐标系不一致。')
        optical=backproject(depth,u,v,cam.k)
        tf=buffer.lookup_transform('base_link',cam.header.frame_id,Time.from_msg(pair['depth'].header.stamp),timeout=Duration(seconds=0))
        point=(pose_matrix(tf.transform)@np.r_[optical,1])[:3]
        result={'pixel':[u,v],'stamp_ns':key(pair['depth'].header),'depth_m':float(depth[v,u]),
                'optical_frame':cam.header.frame_id,'optical_position_m':optical.tolist(),
                'target_frame':'base_link','position_m':point.tolist(),
                'point_type':'visible_surface','motion_executed':False}
        selected=PointStamped();selected.header.stamp=pair['depth'].header.stamp;selected.header.frame_id='base_link'
        selected.point.x,selected.point.y,selected.point.z=map(float,point);publisher.publish(selected)
        (REPORTS/'bottom_camera_point.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
        view['clicked']=(u,v);view['text']='base_link XYZ: '+', '.join(f'{p:.3f}' for p in point)+' m'
        return result
    def click(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN and view['frame'] is not None:
            h,w=view['frame'][2].shape
            if x>=w:return
            try:measure(x,y)
            except (ValueError,TransformException) as error:
                view['text']='Invalid depth or TF; choose another pixel';print(str(error),flush=True)
    deadline=time.monotonic()+20
    window='Ghost Arm - Bottom RGB | Depth (click RGB, Q to quit)'
    next_draw=0.
    try:
        if args.pixel is None:
            cv2.namedWindow(window,cv2.WINDOW_AUTOSIZE);cv2.setMouseCallback(window,click)
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            pair=sync.latest()
            if pair is not None:
                cam=pair['info'];depth=decode_depth(pair['depth'])
                view['frame']=(pair,cam,depth);view['received']=time.monotonic()
            if args.pixel is not None:
                if view['frame'] is not None:
                    try:measure(*args.pixel);return
                    except TransformException:
                        pass
                if time.monotonic()>deadline:raise RuntimeError('等待同步图像或相机 TF 超时。')
                continue
            if time.monotonic()<next_draw:
                continue
            next_draw=time.monotonic()+.1
            if view['frame'] is not None:
                pair,cam,depth=view['frame'];rgb=bridge.imgmsg_to_cv2(pair['rgb'],desired_encoding='bgr8').copy()
                valid=np.isfinite(depth)&(depth>0)
                scaled=np.zeros(depth.shape,dtype=np.uint8)
                near,far=(np.percentile(depth[valid],[2,98]) if valid.any() else (0.,1.))
                far=max(float(far),float(near)+.05)
                scaled[valid]=(np.clip((depth[valid]-near)/(far-near),0,1)*255).astype(np.uint8)
                colored=cv2.applyColorMap(scaled,cv2.COLORMAP_TURBO);colored[~valid]=0
                if view['clicked'] is not None:cv2.drawMarker(rgb,view['clicked'],(0,255,0),cv2.MARKER_CROSS,12,1)
                canvas=np.hstack([rgb,colored]);canvas=cv2.copyMakeBorder(canvas,0,50,0,0,cv2.BORDER_CONSTANT)
                stale=time.monotonic()-view['received']>=2
                label=view['text'] if not stale else 'STALE CAMERA - image hidden; waiting for fresh RGB-D'
                if stale:canvas[:depth.shape[0],:]=0
                cv2.putText(canvas,f'Depth color range: {near:.2f} .. {far:.2f} m', (5,depth.shape[0]+40),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                cv2.putText(canvas,label,(5,depth.shape[0]+20),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
                cv2.imshow(window,canvas)
            else:
                waiting=np.zeros((290,640,3),dtype=np.uint8)
                cv2.putText(waiting,'Waiting for synchronized RGB-D...', (15,140),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                cv2.imshow(window,waiting)
            if cv2.waitKey(1)&0xff in (ord('q'),27):break
    finally:
        cv2.destroyAllWindows();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
