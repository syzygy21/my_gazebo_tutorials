from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from datetime import datetime
import os

def generate_launch_description():
    # Declare the record_data argument with default value of False
    record_data = DeclareLaunchArgument(
        'record_data',
        default_value='false',
        description='Flag to enable/disable rosbag recording'
    )

    # Create a timestamp for the bag file name
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    bag_file = f'walker_data_{timestamp}'

    # Configure rosbag record with topic exclusions
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_data')),
        cmd=['ros2', 'bag', 'record',
             '-o', bag_file,
             '-a',  # Record all topics
             '--exclude', '/camera/.*'],  # Exclude all camera topics
        output='screen'
    )

    # Launch the walker node
    walker_node = Node(
        package='walker',
        executable='robot_control',
        name='walker',
        output='screen'
    )

    return LaunchDescription([
        record_data,
        walker_node,
        rosbag_record
    ])
