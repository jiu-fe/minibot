import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name= 'minibot'

    # teleop_twist_keyboard node (Twist 메시지 발행)
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard_node',
        remappings=[('/cmd_vel','keyboard_vel')],  # Twist 메시지를 keyboard_vel로 발행
        output='screen'
    )   

    # twist_stamper node (Twist를 TwistStamped로 변환)
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="keyboard_twist_stamper",
        output='screen',
        remappings=[
            ('cmd_vel_in', 'keyboard_vel'),  # Twist 입력
            ('cmd_vel_out', 'joy_vel'),      # TwistStamped 출력
        ],
    )
        
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(teleop_keyboard_node)
    ld.add_action(twist_stamper_node)
   
    # Generate the launch description and 
    return ld 