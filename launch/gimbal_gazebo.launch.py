from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare('gimbal_demo')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'gimbal.xacro'])

    robot_description = Command(['xacro ', xacro_path])

    # 1. 로봇 모델 퍼블리셔
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']
        ),
        launch_arguments={'gz_args': '-s -r -v 4 empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'gimbal'],
        output='screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gimbal_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gimbal_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_state_spawner_delayed = TimerAction(
        period=3.0,
        actions=[joint_state_spawner]
    )

    gimbal_spawner_delayed = TimerAction(
        period=4.0,
        actions=[gimbal_spawner]
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner_delayed,
        gimbal_spawner_delayed,
        gz_bridge,
        foxglove_bridge, #
    ])
