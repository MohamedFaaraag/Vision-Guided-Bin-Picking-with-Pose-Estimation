"""
Simulation Launch File
Starts Gazebo with the bin-picking world + all pipeline nodes.
  ros2 launch ur5e_bin_picking simulation.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ur5e_bin_picking')

    # 1. Gazebo with UR5e simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur_simulation_gazebo'), 'launch', 'ur_sim_control.launch.py'])),
        launch_arguments={
            'ur_type': 'ur5e',
            'launch_rviz': 'false',
        }.items()
    )

    # 2. Spawn bin and objects into Gazebo
    spawn_bin = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'bin', '-file',
             PathJoinSubstitution([pkg_share, 'models', 'bin', 'model.sdf']),
             '-x', '0.4', '-y', '0.0', '-z', '0.0'],
        output='screen'
    )

    spawn_object1 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'object_0', '-file',
             PathJoinSubstitution([pkg_share, 'models', 'object', 'model.sdf']),
             '-x', '0.35', '-y', '0.05', '-z', '0.12'],
        output='screen'
    )

    spawn_object2 = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'object_1', '-file',
             PathJoinSubstitution([pkg_share, 'models', 'object', 'model.sdf']),
             '-x', '0.45', '-y', '-0.05', '-z', '0.12'],
        output='screen'
    )

    # 3. MoveIt2
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ur5e_bin_picking_moveit_config'), 'launch', 'move_group.launch.py'])),
        launch_arguments={'ur_type': 'ur5e', 'launch_rviz': 'true'}.items()
    )

    # 4. Static TF (same calibration - camera is in URDF for simulation)
    static_tf_cam = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.45', '--y', '-0.12', '--z', '0.85',
                   '--qx', '0.707', '--qy', '0.0', '--qz', '0.0', '--qw', '0.707',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link']
    )

    # 5. Detection + Transformation + Pipeline
    detector = Node(
        package='ur5e_bin_picking', executable='object_detector.py',
        name='object_detector', output='screen',
        parameters=[PathJoinSubstitution([pkg_share, 'config', 'hsv_params.yaml'])]
    )
    transformer = Node(
        package='ur5e_bin_picking', executable='pose_transformer.py',
        name='pose_transformer', output='screen'
    )
    bt_pipeline = Node(
        package='ur5e_bin_picking', executable='bt_pipeline',
        name='bt_pipeline', output='screen',
        parameters=[{'tree_file': PathJoinSubstitution([pkg_share, 'config', 'behavior_tree.xml'])}]
    )

    return LaunchDescription([
        gazebo_sim, spawn_bin, spawn_object1, spawn_object2,
        ur_moveit, static_tf_cam, detector, transformer, bt_pipeline,
    ])
