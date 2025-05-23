from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import math
from tf_transformations import quaternion_from_euler

class WheelOdometryNode(Node):
    def __init__(self):
        # Inicializa o node ROS 2 que calcula a odometria do Kit com base na leitura dos encoders
        super().__init__('wheel_odometry_node')

        # Cria os subscritores para ler os "ticks" lidos através de cada um dos encoders

        self.create_subscription(Int16, 'kit/encoders/left/ticks', self.left_wheel_callback, 10)
        self.create_subscription(Int16, 'kit/encoders/right/ticks', self.right_wheel_callback, 10)

        # Cria o publicador para os dados de odometria
        # O nome do tópico segue os padrões encontrados em http://wiki.ros.org/ROS/Patterns/Conventions
        self.odom_publisher = self.create_publisher(Odometry, 'kit/wheel/odom', 10)
        
        # Declara os parâmetros e define os valores padrão

        # Frequencia de publicação desse nó, independente de atualizações nas informações dos encoders
        self.declare_parameter('sampling_frequency', 15.0)

        # Os encoders podem estar invertidos, vamos usar esse dado para corigir as distancias calculadas abaixo
        self.declare_parameter('right_wheel.ticks_inversed', False) 
        self.declare_parameter('left_wheel.ticks_inversed', True)  

        # Retirado da especificação da plataforma falcon  
        self.declare_parameter('robot_dimensions.wheel_radius', 0.0315) 
        self.declare_parameter('robot_dimensions.wheel_base', 0.15)    

        # Retirado da especificação dos encoders da RoboCore
        self.declare_parameter('encoders.ticks_per_revolution', 40)

        # Obtém os valores dos ROS params, caso o node tenha sido inicializado com valores customizados
        self.sampling_frequency = self.get_parameter('sampling_frequency').value
        self.wheel_radius = self.get_parameter('robot_dimensions.wheel_radius').value
        self.wheel_base = self.get_parameter('robot_dimensions.wheel_base').value
        self.ticks_per_revolution = self.get_parameter('encoders.ticks_per_revolution').value
        self.left_ticks_inversed = self.get_parameter('left_wheel.ticks_inversed').value
        self.right_ticks_inversed = self.get_parameter('right_wheel.ticks_inversed').value

        # Armazena os ticks anteriores para o cálculo da odometria
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        # Posição atual do robô
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Velocidade atual do robô
        self.last_time = self.get_clock().now()
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


        # Timer é criado para temporizar as publicações de odometria, indepentente da atualização da odometria
        self.timer = self.create_timer(1.0 / self.sampling_frequency, self.publish_odometry)

        # Log para confirmar a inicialização com os parâmetros configurados
        self.get_logger().info(
            'WheelOdometryNode initialized with '
            f'sampling_frequency: {self.sampling_frequency}, '
            f'wheel_radius: {self.wheel_radius}, '
            f'wheel_base: {self.wheel_base}, '
            f'ticks_per_revolution: {self.ticks_per_revolution}, '
            f'left_ticks_inversed: {self.left_ticks_inversed}, '
            f'right_ticks_inversed: {self.right_ticks_inversed}'
        )

    def left_wheel_callback(self, msg):
        # Callback para processar os ticks da roda esquerda
        self.update_odometry(msg.data, self.prev_right_ticks)  # Atualiza a odometria
        self.prev_left_ticks = msg.data  # Armazena o valor atual dos ticks

    def right_wheel_callback(self, msg):
        # Callback para processar os ticks da roda direita
        self.update_odometry(self.prev_left_ticks, msg.data)  # Atualiza a odometria
        self.prev_right_ticks = msg.data  # Armazena o valor atual dos ticks

    def update_odometry(self, left_ticks, right_ticks):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        # Calcula a distância percorrida por cada roda
        left_distance = (left_ticks - self.prev_left_ticks) * (2 * math.pi * self.wheel_radius / self.ticks_per_revolution)
        right_distance = (right_ticks - self.prev_right_ticks) * (2 * math.pi * self.wheel_radius / self.ticks_per_revolution)

        # Inverte as distancias calculadas se necessário
        if self.left_ticks_inversed:
            left_distance = -left_distance
        if self.right_ticks_inversed:
            right_distance = -right_distance

        # Calcula a distância média percorrida e a mudança de orientação
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        if dt == 0:
            self.linear_velocity = 0
            self.angular_velocity = 0
        else:
            self.linear_velocity = delta_distance / dt
            self.angular_velocity = delta_theta / dt

        # Atualiza a posição do robô
        self.x += delta_distance * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_distance * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta

        # Normaliza theta para mantê-lo no intervalo [-pi, pi]
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        self.last_time = current_time

    def publish_odometry(self):
        # Cria a mensagem de odometria
        odom_msg = Odometry()
        
        # Popula o header da mensagem ROS
        odom_msg.header.stamp = self.get_clock().now().to_msg()  
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Define a posição e orientação do robô
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Converter o theta (yaw do robo) em um quaternion
        quaternion = quaternion_from_euler(0, 0, self.theta)

        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # Publica os dados de odometria
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args) # Inicializa o cliente ROS 2
    wheel_odometry_node = WheelOdometryNode() # Cria o node do encoder

    try:
        # Mantém o node ativo para continuar calculando a odometria
        rclpy.spin(wheel_odometry_node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        wheel_odometry_node.get_logger().info('Shutting down odometry driver...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        wheel_odometry_node.destroy_node()
        rclpy.shutdown() 

# Ponto de entrada do script
if __name__ == '__main__':
    main()
