from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

config_dir = get_package_share_directory("lidar_poly")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_poly',
            executable='determine_lpoly',
            name='determine_lpoly',
            parameters = [
                {'lidar_lane_width': 5.0},
                {'lidar_lane_length': 15.0},
                {'num_intervals': 10}
            ]
        ),
        Node(
            package='lidar_poly',
            executable='pose_service',
            name='pose_service',
            output='screen'
        ),
        Node(
            package='lidar_poly',
            executable='veh_info',
            name='veh_info',
            output='screen'
        ),
        Node(
            package='lidar_poly',
            executable='tf_broadcast',
            name='tf_broadcast',
            arguments = ['utm_local', '0', '0', '0', '0', '0', '0'],
            output='screen'
        ),
        Node(
            package='rviz2',
            namespace='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=["-d", config_dir + "/rviz/230803_rviz_stuff.rviz"]
        )
    ])