import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'kit_encoder_driver'

    encoder_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'encoder_params.yaml')
    odometry_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'odometry_params.yaml')

    # O argumento 'debug' é declarado com o valor padrão de `false`
    debug_mode_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable or disable TF publishing for debugging'
    )

    # Esse arquivo launch recebe a configuração de `debug` para determinar se deve executar
    # o node `transformations_node`.

    # Nesse caso, o modo de debug significa que esse arquivo launch publicará a matriz de
    # transformação entre o eixo da odometria ("odom") e o eixo da base do robô ("base_link"). 
    # Isso permite debug facil através de ferramentas de visualização como rviz2, mas 
    # idealmente deve ser responsabilidade de um node de fusão de sensores, em outro pacote.
    debug = LaunchConfiguration('debug')
    
    return LaunchDescription([
        debug_mode_arg,
        Node(
            package='kit_encoder_driver',
            executable='encoder_node',
            name='kit_encoders',
            parameters=[encoder_params_file],
        ),
        Node(
            package='kit_encoder_driver',
            executable='wheel_odometry_node',
            name='kit_odometry',
            parameters=[odometry_params_file]
        ),
        Node(
            package='kit_encoder_driver',
            executable='transformations_node',
            name='kit_transform',
            condition=IfCondition(debug)
        )
    ])