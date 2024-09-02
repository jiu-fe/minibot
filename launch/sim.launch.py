import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name= 'minibot'
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Update the path to your custom world file
    world_file_path = '/home/yj/ros2_ws/install/minibot/share/minibot/worlds/empty.sdf'
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'
                )]), launch_arguments={'gz_args': world_file_path}.items()
    )

    spawn_model = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'),'launch','gz_spawn_model.launch.py'
                )]), launch_arguments={'topic': 'robot_description', 'name': 'minibot', 'z': '0.1'}.items()
    )

    # Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',  
        ],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_model,
        bridge
    ])