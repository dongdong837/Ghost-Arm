"""Ghost simulation, bridges, model display, camera and flight control."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ghost_sim.paths import WORLD, MODEL, CONFIG
from ghost_sim.simulation.tail_structure import JOINTS


def static_tf(name,parent,child,xyz,rpy=(0,0,0)):
    x,y,z = xyz
    roll,pitch,yaw = rpy
    return Node(package='tf2_ros',executable='static_transform_publisher',name=name,
                arguments=['--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),
                           '--pitch',str(pitch),'--yaw',str(yaw),'--frame-id',parent,'--child-frame-id',child])


def generate_launch_description():
    def gazebo(context):
        command = ['ign','gazebo','-r','-v','3','-s',str(WORLD)]
        processes = [ExecuteProcess(cmd=command,output='screen')]
        if LaunchConfiguration('gui').perform(context) == 'true':
            processes.append(TimerAction(period=3.0,actions=[ExecuteProcess(
                cmd=['ign','gazebo','-g','--gui-config',str(CONFIG/'arm_gazebo.config')],
                additional_env={'LIBGL_ALWAYS_SOFTWARE':'1','QT_OPENGL':'software','QSG_RENDER_LOOP':'basic'},
                output='screen')]))
        return processes

    actions = [DeclareLaunchArgument('gui',default_value='false'),OpaqueFunction(function=gazebo),
               Node(package='robot_state_publisher',executable='robot_state_publisher',
                    parameters=[{'robot_description':(MODEL).read_text(),'use_sim_time':True}])]
    for module in ['visualization.ground_truth_tf','visualization.reference_scene','visualization.cloud_frame','control.flight_control','mapping.voxel_map_node','navigation.navigation3d']:
        actions.append(ExecuteProcess(cmd=['python3','-m','ghost_sim.'+module,'--ros-args','-p','use_sim_time:=true'],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='ghost_bridge',
                        arguments=[
                            '/model/ghost/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                            '/model/ghost/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                            '/model/wood_block/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                            '/ghost/arm/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
                            '/grasp_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/grasp_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                            '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                        remappings=[('/ghost/arm/joint_states','/joint_states'),('/model/ghost/pose','/simulation/ground_truth/poses'),
                                    ('/camera/points','/camera/points_raw')],output='screen'))
    actions.append(Node(package='ros_gz_bridge',executable='parameter_bridge',name='arm_bridge',
                        parameters=[{'config_file':str(CONFIG/'arm_bridge.yaml')}],output='screen'))
    actions.extend([static_tf('camera_mount','base_link','camera_link',(.29,0,0)),
                    static_tf('camera_optical','camera_link','camera_optical_frame',(0,0,0),
                              (-1.57079632679,0,-1.57079632679))])
    return LaunchDescription(actions)
