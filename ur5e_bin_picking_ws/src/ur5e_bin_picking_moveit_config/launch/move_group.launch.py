import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    moveit_config_pkg = FindPackageShare('ur5e_bin_picking_moveit_config')

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur5e_bin_picking_description'), 'urdf', 'workcell.urdf.xacro']),
        ' ur_type:=', ur_type, ' use_fake_hardware:=true',
    ])

    robot_description_semantic = PathJoinSubstitution([moveit_config_pkg, 'config', 'ur5e.srdf'])
    kinematics_yaml = PathJoinSubstitution([moveit_config_pkg, 'config', 'kinematics.yaml'])
    ompl_yaml = PathJoinSubstitution([moveit_config_pkg, 'config', 'ompl_planning.yaml'])
    controllers_yaml = PathJoinSubstitution([moveit_config_pkg, 'config', 'controllers.yaml'])

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic},
            kinematics_yaml,
            ompl_yaml,
            controllers_yaml,
            {'use_sim_time': True},
        ],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'robot_description_semantic': robot_description_semantic},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        move_group_node,
        rviz_node,
    ])
