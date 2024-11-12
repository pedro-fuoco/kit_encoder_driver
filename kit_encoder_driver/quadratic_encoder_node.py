import RPi.GPIO as GPIO
from std_msgs.msg import Int16
import rclpy
from rclpy.node import Node


class QuadraticEncoderNode(Node):
    def __init__(self):
        # Inicializa o node ROS 2 que publica os sinais dos encoders de quadratura do Kit
        super().__init__('quadratic_encoder_node')
        
        # Cria os publicadores para enviar os "ticks" lidos através de cada um dos encoders

        # Os nomes dos tópicos seguem os padrões encontrados em http://wiki.ros.org/ROS/Patterns/Conventions
        # Vale ressaltar que os tipos utilizados são genericos uma vez que não há mensagens padrão ROS para lidar com informação de encoders
        # Isso acontece por conta da diferente natureza de cada tipo de encoder, o que impede uma padronização consiza

        # A fila dos topicos é limitada em 10 elementos... Isso será relevante caso os subscritores sejam mais lentos que os publicadores
        self.left_ticks_publisher = self.create_publisher(Int16, 'kit/encoders/left/ticks', 10)
        self.right_ticks_publisher = self.create_publisher(Int16, 'kit/encoders/right/ticks', 10)

        # Declara os ROS params com valores padrão
        self.declare_parameter('kit_encoders.right_wheel.GPIOs.channel_A', 23)
        self.declare_parameter('kit_encoders.right_wheel.GPIOs.channel_B', 22)
        self.declare_parameter('kit_encoders.left_wheel.GPIOs.channel_A', 24)
        self.declare_parameter('kit_encoders.left_wheel.GPIOs.channel_B', 25)

        # Obtém os valores dos ROS params, caso o node tenha sido inicializado com valores customizados
        self.enc_right_a = self.get_parameter('kit_encoders.right_wheel.GPIOs.channel_A').value
        self.enc_right_b = self.get_parameter('kit_encoders.right_wheel.GPIOs.channel_B').value
        self.enc_left_a = self.get_parameter('kit_encoders.left_wheel.GPIOs.channel_A').value
        self.enc_left_b = self.get_parameter('kit_encoders.left_wheel.GPIOs.channel_B').value

        # Loga os parâmetros iniciais dos encoders para o usuario
        self.get_logger().info(
            'Encoders initialized with ' 
            f'Right Channel A: {self.enc_right_a}, '
            f'Right Channel B: {self.enc_right_b}, ' 
            f'Left Channel A: {self.enc_left_a}, '
            f'Left Channel B: {self.enc_left_b}'
        )

        # Configura o modo de numeração dos pinos GPIO
        GPIO.setmode(GPIO.BCM)

        # Configura os pinos como entradas
        # Resistores de pull-up são configurados para que o nivel lógico dos pinos não flutue em caso de desconexão
        GPIO.setup(self.enc_left_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc_left_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc_right_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc_right_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Contadores de ticks para as rodas esquerda e direita
        self.left_ticks = 0
        self.right_ticks = 0

        # Armazena o estado anterior dos canais A e B
        # Essas variaveis serão utilizadas posteriormente para detectar a direção de rotação de cada encoder
        self.last_state_left_a = GPIO.input(self.enc_left_a)
        self.last_state_left_b = GPIO.input(self.enc_left_b)
        self.last_state_right_a = GPIO.input(self.enc_right_a)
        self.last_state_right_b = GPIO.input(self.enc_right_b)

        # Configura interrupções para detectar mudanças nos canais dos encoders
        # As interrupções são configuradas tanto para bordas de subida, quanto de decida
        GPIO.add_event_detect(self.enc_left_a, GPIO.BOTH, callback=self.left_wheel_tick)
        GPIO.add_event_detect(self.enc_left_b, GPIO.BOTH, callback=self.left_wheel_tick)
        GPIO.add_event_detect(self.enc_right_a, GPIO.BOTH, callback=self.right_wheel_tick)
        GPIO.add_event_detect(self.enc_right_b, GPIO.BOTH, callback=self.right_wheel_tick)


    def detect_direction(self, prev_A, prev_B, curr_A, curr_B):
        # Detecta a direção de rotação do encoder baseado nos estados anteriores e atuais dos canais A e B
        state = (prev_A << 1) | prev_B
        next_state = (curr_A << 1) | curr_B

        # Matriz de transição para detecção da direção com base na sequência quadratura
        # Sabemos a direção da rotação a depender de qual canal está "liderando" o outro
        # Mais informações podem ser encontradas no README
        transition = [
            [0, 1, -1, 0],
            [-1, 0, 0, 1],
            [1, 0, 0, -1],
            [0, -1, 1, 0]
        ]

        direction = transition[state][next_state]
        
        # Para cada estado da matriz de transição existem três possiveis estados
        # Caso tenhamos detectado o quarto caso, impossivel, podemos assumir que perdemos ticks
        # Isso pode acontecer caso os encoders detectem bordas de forma excessivamente agressiva
        # Nesse caso escolhemos avisar o usuario e manter a direção como zero, para evitar leituras incorretas
        if direction == 0 and state != next_state:
            self.get_logger().error('Encoders detected an invalid state! We are possibly losing ticks...')
        return direction

    def left_wheel_tick(self, channel):
        # Callback para detectar mudanças nos canais do encoder da roda esquerda
        curr_state_left_a = GPIO.input(self.enc_left_a)
        curr_state_left_b = GPIO.input(self.enc_left_b)

        # Detecta a direção do movimento com base nas mudanças dos canais
        direction = self.detect_direction(self.last_state_left_a, self.last_state_left_b,
                                          curr_state_left_a, curr_state_left_b)

        # Atualiza o estado anterior dos canais
        self.last_state_left_a = curr_state_left_a
        self.last_state_left_b = curr_state_left_b

        # Atualiza a contagem de ticks de acordo com a direção
        if direction == 1:
            self.left_ticks += 1
        elif direction == -1:
            self.left_ticks -= 1

        # Publica a contagem de ticks atualizada
        msg = Int16()
        msg.data = self.left_ticks
        self.left_ticks_publisher.publish(msg)

    def right_wheel_tick(self, channel):
        # Callback para detectar mudanças nos canais do encoder da roda direita, calcular a direção e atualizar a contagem de ticks
        curr_state_right_a = GPIO.input(self.enc_right_a)
        curr_state_right_b = GPIO.input(self.enc_right_b)

        # Detecta a direção do movimento com base nas mudanças dos canais
        direction = self.detect_direction(self.last_state_right_a, self.last_state_right_b,
                                          curr_state_right_a, curr_state_right_b)

        # Atualiza o estado anterior dos canais
        self.last_state_right_a = curr_state_right_a
        self.last_state_right_b = curr_state_right_b

        # Atualiza a contagem de ticks de acordo com a direção
        if direction == 1:
            self.right_ticks += 1
        elif direction == -1:
            self.right_ticks -= 1

        # Publica a contagem de ticks atualizada
        msg = Int16()
        msg.data = self.right_ticks
        self.right_ticks_publisher.publish(msg)
    
    def __del__(self):
        # Libera os recursos dos GPIOs caso esse objeto seja destruido
        # Seguindo principios de "Resource acquisition is initialization" (RAII)
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)  # Inicializa o cliente ROS 2
    quadratic_encoder_node = QuadraticEncoderNode()  # Cria o node do encoder

    try:
        # Mantém o node ativo para continuar capturando e publicando ticks
        rclpy.spin(quadratic_encoder_node)
    except KeyboardInterrupt:
        # Loga uma mensagem ao encerrar o node
        quadratic_encoder_node.get_logger().info('Shutting down encoder driver...')
    finally:
        # Destrói o node e finaliza o cliente ROS 2
        quadratic_encoder_node.destroy_node() 
        rclpy.shutdown()

# Ponto de entrada do script
if __name__ == '__main__':
    main()
