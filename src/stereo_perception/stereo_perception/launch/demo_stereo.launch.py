from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='stereo_perception', executable='file_image_publisher', output='screen'),
        Node(package='stereo_perception', executable='stereo_rectify_node', output='screen'),
        Node(package='stereo_perception', executable='depth_sgbm_node',   output='screen'),
        Node(package='stereo_perception', executable='pointcloud_node',   output='screen'),
    ])
