from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = False

    # -------------------------
    # BNO055 (그냥 ros2 run 그대로)
    # -------------------------
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[
            '/home/first/cap/capstone/src/bno055/bno055/params/bno055_params_i2c.yaml'
        ]
    )


    # -------------------------
    # Nexus
    # -------------------------
    nexus_control = Node(
        package='nexus',
        executable='control_nexus',
        name='control_nexus',
        output='screen'
    )

    nexus_wheel_encoder = Node(
        package='nexus',
        executable='wheel_encoder',
        name='wheel_encoder',
        output='screen'
    )

    nexus_joy_to_cmd_vel = Node(
        package='nexus',
        executable='joy_to_cmd_vel',
        name='joy_to_cmd_vel',
        output='screen'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # -------------------------
    # SL LiDAR
    # -------------------------
    sllidar_share_dir = get_package_share_directory('sllidar_ros2')

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sllidar_share_dir,
                'launch',
                'sllidar_a2m12_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/sllidar',
            'serial_baudrate': '256000',
            'frame_id': 'laser',
            'angle_compensate': 'true'
        }.items()
    )

    return LaunchDescription([
        joy_node,
        nexus_control,
        nexus_wheel_encoder,
        nexus_joy_to_cmd_vel,
        bno055_node,
        sllidar_launch
    ])

