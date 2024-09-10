import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# Image Transport Republishers
# terminal command example: ros2 run image_transport republish raw compressed --ros-args -r in:=/camera/image_raw -r out/compressed:=/camera/image_raw/compressed
def image_transport_republisher(transport):
    return Node(
        package='image_transport',
        executable='republish',
        name=f'image_transport_republish_{transport}',
        arguments=['raw', transport],
        remappings=[
            ('in', '/camera/image_raw'),
            (f'out/{transport}', f'/camera/image_raw/{transport}'),
        ],
    )

def generate_launch_description():

    package_name= 'minibot'

    # Update the path to your custom world file
    world_file_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'playground.sdf')
    rviz_config_file_dir = os.path.join(get_package_share_directory(package_name), 'config', 'gz_sim_config.rviz')

    # robot_state_publisher node
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
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
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # Execute Image Transport Republishers
    image_transports = ['compressed','compressedDepth', 'theora', 'zstd' ]  # Add or remove formats as needed
    image_republishers = [image_transport_republisher(transport) for transport in image_transports]

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'),'launch','gz_sim.launch.py'
                )]), launch_arguments={'gz_args': f'-r {world_file_path}',}.items()
    )

    spawn_model = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'),'launch','gz_spawn_model.launch.py'
                )]), launch_arguments={'topic': 'robot_description', 'name': 'minibot', 'z': '0.1'}.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(rsp)
    ld.add_action(bridge)
    for republisher in image_republishers:
        ld.add_action(republisher)
    ld.add_action(GroupAction([gazebo, spawn_model]))
    ld.add_action(rviz2)

    # Generate the launch description and 
    return ld

    