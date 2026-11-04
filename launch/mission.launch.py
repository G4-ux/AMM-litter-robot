"""Launch the complete SLAM, navigation, perception, and collection mission."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('autonomous_mobile_manipulator')
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    slam_share = get_package_share_directory('slam_toolbox')
    nav2_share = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autonomous_roam = LaunchConfiguration('autonomous_roam')

    world = os.path.join(package_share, 'worlds', 'mission_world.sdf')
    robot_xacro = os.path.join(package_share, 'urdf', 'mobile_manipulator.urdf.xacro')
    robot_description = xacro.process_file(robot_xacro).toxml()
    slam_params = os.path.join(package_share, 'config', 'slam_toolbox.yaml')
    nav2_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(package_share, 'config', 'mission.rviz')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'autonomous_mobile_manipulator', '-x', '0.0', '-y', '0.0', '-z', '0.08'],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/model/autonomous_mobile_manipulator/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/mission_world/model/autonomous_mobile_manipulator/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/arm_waist/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm_shoulder/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm_elbow/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/arm_wrist/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/gripper/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/soda_can_red/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/soda_can_red/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/soda_can_blue/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/soda_can_blue/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_1/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_1/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_2/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_2/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_3/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/litter_3/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        remappings=[
            ('/model/autonomous_mobile_manipulator/tf', '/tf'),
            ('/world/mission_world/model/autonomous_mobile_manipulator/joint_state', '/joint_states'),
        ],
    )
    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=['/camera/image', '/camera/depth_image'],
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params, 'autostart': 'true'}.items(),
    )
    perception_and_mission = [
        Node(
            package='autonomous_mobile_manipulator', executable='odom_tf_broadcaster.py', output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        slam,
        navigation,
        Node(package='autonomous_mobile_manipulator', executable='litter_detector.py', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='autonomous_mobile_manipulator', executable='manipulator_controller.py', output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(
            package='autonomous_mobile_manipulator', executable='mission_manager.py', output='screen',
            parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(autonomous_roam),
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}], condition=IfCondition(use_rviz),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('autonomous_roam', default_value='true'),
        gazebo,
        state_publisher,
        spawn_robot,
        bridge,
        image_bridge,
        TimerAction(period=5.0, actions=perception_and_mission),
    ])
