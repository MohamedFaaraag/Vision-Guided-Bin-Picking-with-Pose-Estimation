from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([FindPackageShare('ur5e_bin_picking_description'), 'urdf', 'workcell.urdf.xacro']),
        ' ur_type:=', ur_type, ' use_fake_hardware:=true',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description_content}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2'),
    ])
