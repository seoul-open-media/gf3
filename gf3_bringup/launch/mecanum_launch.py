from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():


    can_interface_node = Node(
        package='ros2_can_interface',
        executable='can_interface',
        name='can_interface',
    )

    mecanum_control_node = Node(
        package='gf3_hardware',
        executable='mecanum_control',
        name='mecanum_control',
    )

    gf3_bringup = get_package_share_directory('gf3_bringup')
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/tf.launch.py'))

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("gf3_bringup"), "config", "gf3.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    nodes = [
        can_interface_node,
        rviz_node,
        mecanum_control_node,
        static_tf_launch,
    ]
    return LaunchDescription(nodes)

   
    