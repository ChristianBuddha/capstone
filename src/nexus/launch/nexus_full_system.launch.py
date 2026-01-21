from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directories
    bno055_share_dir = get_package_share_directory('bno055')
    
    # Path to the BNO055 parameters file
    bno055_params_file = os.path.join(
        bno055_share_dir,
        'params',
        'bno055_params_i2c.yaml'
    )
    
    # Define nodes
    nexus_control = Node(
        package='nexus',
        executable='control_nexus',
        name='control_nexus'
    )
    
    nexus_wheel_encoder = Node(
        package='nexus',
        executable='wheel_encoder',
        name='wheel_encoder'
    )
    
    nexus_joy_to_cmd_vel = Node(
        package='nexus',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel'
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )
    
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[bno055_params_file]
    )
    
    # Import and launch sllidar
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    
    sllidar_share_dir = get_package_share_directory('sllidar_ros2')
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sllidar_share_dir,
                'launch',
                'sllidar_a2m8_launch.py'
            )
        )
    )
    
    # Create launch description
    return LaunchDescription([
        nexus_control,
        nexus_wheel_encoder,
        nexus_joy_to_cmd_vel,
        joy_node,
        bno055_node,
        sllidar_launch
    ])
