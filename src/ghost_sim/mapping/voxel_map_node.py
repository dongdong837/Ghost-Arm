"""Latched RViz layers and full XYZ/state point cloud for the static voxel map."""
import json
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2,PointField
from std_msgs.msg import String
from ghost_sim.mapping.voxel_map import save,FREE,CLEARANCE,OCCUPIED,index_of


def main():
    rclpy.init()
    node=rclpy.create_node('ghost_voxel_map')
    node.declare_parameter('slice_height',1.2)
    states,centres,meta=save()
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs={name:node.create_publisher(MarkerArray,'/ghost/map/'+name,qos)
          for name in ['obstacles','clearance','free_slice','free_volume']}
    info=node.create_publisher(String,'/ghost/map/metadata',qos)
    cloud_pub=node.create_publisher(PointCloud2,'/ghost/map/voxels',qos)
    colours={'obstacles':(1.,.3,.15,.5),'clearance':(1.,.75,.1,.12),
             'free_slice':(.1,1.,.65,.28),'free_volume':(.1,.8,1.,.12)}
    def layer(name,points,scale):
        m=Marker();m.header.frame_id=meta['frame'];m.header.stamp=node.get_clock().now().to_msg()
        m.ns=name;m.id=0;m.type=Marker.CUBE_LIST;m.action=Marker.ADD
        m.pose.orientation.w=1.
        m.scale.x,m.scale.y,m.scale.z=scale
        m.color.r,m.color.g,m.color.b,m.color.a=colours[name]
        m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])) for p in points]
        return MarkerArray(markers=[m])
    for name,label in [('obstacles',OCCUPIED),('clearance',CLEARANCE),('free_volume',FREE)]:
        # Full data, no sampling: display layers are optional to keep VMware responsive.
        pubs[name].publish(layer(name,centres[states==label],[.095]*3))
    info.publish(String(data=json.dumps(meta)))
    packed=np.empty(states.size,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('state','<u4')])
    flat=centres.reshape(-1,3)
    for i,key in enumerate(['x','y','z']):packed[key]=flat[:,i]
    packed['state']=states.ravel()
    cloud=PointCloud2();cloud.header.frame_id=meta['frame'];cloud.header.stamp=node.get_clock().now().to_msg()
    cloud.height=1;cloud.width=states.size;cloud.is_dense=True;cloud.is_bigendian=False
    cloud.fields=[PointField(name=k,offset=i*4,datatype=PointField.FLOAT32 if i<3 else PointField.UINT32,count=1)
                  for i,k in enumerate(['x','y','z','state'])]
    cloud.point_step=16;cloud.row_step=cloud.width*16;cloud.data=packed.tobytes()
    cloud_pub.publish(cloud)
    previous=[None]
    def update_slice():
        height=node.get_parameter('slice_height').value
        if previous[0]==height:return
        previous[0]=height
        # Select the containing voxel layer; invalid height clears the display.
        if np.isfinite(height) and 0<=height<3:
            index=int(index_of([0,0,height],meta)[2])
            points=centres[:,:,index][states[:,:,index]==FREE]
        else:
            points=[]
            node.get_logger().warning('slice_height must be in [0,3); slice cleared')
        pubs['free_slice'].publish(layer('free_slice',points,[.095,.095,.015]))
    timer=node.create_timer(.5,update_slice)
    update_slice()
    node.get_logger().info('Known 3D map ready: '+str(meta['counts']))
    try:rclpy.spin(node)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
