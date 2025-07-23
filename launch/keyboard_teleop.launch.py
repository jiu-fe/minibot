import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name= 'minibot'

    # teleop_twist_keyboard node with remapping
    teleop_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard_node',
        remappings=[('/cmd_vel','joy_vel')],  # twist_mux 설정에 맞춰 리매핑
        output='screen'
    )   
        
    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(teleop_keyboard_node)
   
    # Generate the launch description and 
    return ld 