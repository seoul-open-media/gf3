from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():

    gf3_bringup = get_package_share_directory('gf3_bringup')
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/rplidar_launch.py'))
    mecanum_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/mecanum_launch.py'))
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/navigation_launch.py'))
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/slam_launch.py'))
    laser_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gf3_bringup + '/launch/laser_filter_launch.py'))
    nodes = [
        rplidar_launch,
        mecanum_launch,
        nav_launch,
        slam_launch,
        laser_filter_launch,
    ]
    return LaunchDescription(nodes)

   
    