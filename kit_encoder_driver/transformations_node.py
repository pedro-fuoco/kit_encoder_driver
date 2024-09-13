import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations

class TransformationNode(Node):
    def __init__(self):
        # Inicializa o node ROS 2 que calcula a odometria do Kit com base na leitura dos encoders
        super().__init__('transformation_node')

        # Cria o broadcaster das transformações
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Cria o subscritor para pegar as informações de odometria do Kit
        self.create_subscription(Odometry, 'kit/odom', self.odom_callback, 10)

        # Log para confirmar a inicialização
        self.get_logger().info('TransformationNode initialized')

    def odom_callback(self, msg):
        # Callback chamado quando há atualizações de odometria

        # Cria a mensagem da transformação
        transform = TransformStamped()

        # Popula o header e define os frames da transformação
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        # Salva a translação
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z

        # Salva a rotação
        transform.transform.rotation = msg.pose.pose.orientation

        # Publica a transformação
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)  # Inicializa o cliente ROS 2
    transformer_node = TransformationNode()  # Cria o node da transformação

    try:
        # Mantém o node ativo para continuar publicando as transformações
        rclpy.spin(transformer_node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        transformer_node.get_logger().info('Shutting down transform broadcaster...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        transformer_node.destroy_node() 
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()