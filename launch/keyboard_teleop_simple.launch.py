import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name= 'minibot'

    # twist_stamper node (Twist를 TwistStamped로 변환)
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="keyboard_twist_stamper",
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel'),  # Twist 입력 (직접 발행)
            ('cmd_vel_out', 'joy_vel'), # TwistStamped 출력
        ],
    )
        
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(twist_stamper_node)
   
    # Generate the launch description and 
    return ld 