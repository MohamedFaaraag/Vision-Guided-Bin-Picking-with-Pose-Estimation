"""
System Bringup Launch File
Starts the entire bin-picking pipeline with a single command:
  ros2 launch ur5e_bin_picking bringup.launch.py

Launches: UR5e driver, MoveIt2, Camera, Static TF, Detection, Transformation, BT Pipeline
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.56.101')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')

    # 1. UR5e Driver
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_robot_driver'), 'launch', 'ur_control.launch.py'])),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': robot_ip,
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',
            'initial_joint_controller': 'joint_trajectory_controller',
        }.items()
    )

    # 2. MoveIt2
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur5e_bin_picking_moveit_config'), 'launch', 'move_group.launch.py'])),
        launch_arguments={'ur_type': 'ur5e', 'launch_rviz': 'true'}.items()
    )

    # 3. Camera Driver (RealSense)
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
        }.items()
    )

    # 4. Static TF: Hand-eye calibration result (base_link -> camera_link)
    # REPLACE these values with your actual calibration output
    static_tf_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.45', '--y', '-0.12', '--z', '0.85',
            '--qx', '0.707', '--qy', '0.0', '--qz', '0.0', '--qw', '0.707',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    # 5. Object Detection Node (Phase 1)
    detector = Node(
        package='ur5e_bin_picking',
        executable='object_detector.py',
        name='object_detector',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('ur5e_bin_picking'), 'config', 'hsv_params.yaml'])]
    )

    # 6. Pose Transformer Node (Phase 2)
    transformer = Node(
        package='ur5e_bin_picking',
        executable='pose_transformer.py',
        name='pose_transformer',
        output='screen'
    )

    # 7. Behavior Tree Pipeline (Phase 5)
    bt_pipeline = Node(
        package='ur5e_bin_picking',
        executable='bt_pipeline',
        name='bt_pipeline',
        output='screen',
        parameters=[{
            'tree_file': PathJoinSubstitution([
                FindPackageShare('ur5e_bin_picking'), 'config', 'behavior_tree.xml'])
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.101'),
        DeclareLaunchArgument('use_fake_hardware', default_value='true'),
        ur_control,
        ur_moveit,
        camera,
        static_tf_cam,
        detector,
        transformer,
        bt_pipeline,
    ])
