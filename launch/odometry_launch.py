import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_encoder_driver'
    encoder_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'encoder_params.yaml')
    odometry_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'odometry_params.yaml')
    
    return LaunchDescription([
        Node(
            package='kit_encoder_driver',
            executable='encoder_node',
            name='kit_encoders',
            parameters=[encoder_params_file],
        ),
        Node(
            package='kit_encoder_driver',
            executable='odometry_node',
            name='kit_odometry',
            parameters=[odometry_params_file]
        ),
        Node(
            package='kit_encoder_driver',
            executable='transformations_node',
            name='kit_transformada'
        )
    ])