"""
Main autonomy launch file for Szuflada V2 (ROS2 Jazzy).

Launches the UNIFIED core stack with a SINGLE MQTT connection:

    mqtt_bridge       -- ONE MQTT connection (ESP32 <-> ROS2)
    cmd_vel_bridge    -- /cmd_vel -> /motor_left, /motor_right (pure ROS2)
    odometry_node     -- /encoder_raw -> /odom + TF (pure ROS2)
    robot_description -- static TF frames (pure ROS2)
    depthimage_to_laserscan -- depth camera -> /scan

After launching this, run ONE behavior node in a separate terminal:
    ros2 run rover_autonomy rover_teleop
    ros2 run rover_autonomy obstacle_avoidance
    ros2 run rover_autonomy wall_follower
    ros2 run rover_autonomy visual_tracker
    ros2 run rover_autonomy move_distance --ros-args -p distance:=0.5

Usage:
    ros2 launch rover_autonomy autonomy_launch.py
    ros2 launch rover_autonomy autonomy_launch.py mqtt_broker:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch Arguments ────────────────────────────────────────────
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker', default_value='192.168.1.1',
        description='IP address of the MQTT broker')
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port', default_value='1883',
        description='MQTT broker port')

    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')

    # ── mqtt_bridge: THE SINGLE MQTT CONNECTION ─────────────────────
    mqtt_bridge = Node(
        package='rover_autonomy',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
        }]
    )

    # ── cmd_vel_bridge: /cmd_vel -> motor PWM (no MQTT) ─────────────
    cmd_vel_bridge = Node(
        package='rover_autonomy',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'max_pwm': 255,
            'min_pwm': 60,
            'max_linear_speed': 0.3,
            'wheel_base': 0.264,
            'motor_inversion': False,
            'timeout_sec': 0.5,
            'right_trim': 1.0,
        }]
    )

    # ── odometry_node: encoders -> /odom + TF (no MQTT) ────────────
    # Calibrated: 437 ticks/rev on both wheels, wheel diameter 0.05m
    odometry = Node(
        package='rover_autonomy',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{
            'wheel_diameter': 0.065,
            'wheel_base': 0.264,
            'encoder_ticks_per_rev_left': 437,
            'encoder_ticks_per_rev_right': 437,
        }]
    )

    # ── robot_description: static TF frames (already pure ROS2) ────
    robot_desc = Node(
        package='rover_autonomy',
        executable='robot_description',
        name='robot_description',
        output='screen',
        parameters=[{
            'ultrasonic_x': 0.10,
            'ultrasonic_y': 0.0,
            'ultrasonic_z': 0.05,
            'camera_x': 0.08,
            'camera_y': 0.0,
            'camera_z': 0.12,
        }]
    )

    # ── depthimage_to_laserscan: depth camera -> /scan ──────────────
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 5.0,
            'output_frame': 'laser_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan'),
        ]
    )

    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        mqtt_bridge,
        cmd_vel_bridge,
        odometry,
        robot_desc,
        depth_to_scan,
    ])
