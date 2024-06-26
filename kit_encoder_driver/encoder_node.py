import RPi.GPIO as GPIO
from std_msgs.msg import Int16
import rclpy
from rclpy.node import Node


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        GPIO.setmode(GPIO.BCM)

        self.enc_right_a = 23  # GPIO pin A for right wheel encoder
        self.enc_right_b = 22  # GPIO pin B for right wheel encoder
        self.enc_left_a = 24   # GPIO pin A for left wheel encoder
        self.enc_left_b = 25   # GPIO pin B for left wheel encoder

        GPIO.setup(self.enc_left_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc_left_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.setup(self.enc_right_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.enc_right_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.left_ticks = 0
        self.last_state_left_a = GPIO.input(self.enc_left_a)
        self.last_state_left_b = GPIO.input(self.enc_left_b)

        self.right_ticks = 0
        self.last_state_right_a = GPIO.input(self.enc_right_a)
        self.last_state_right_b = GPIO.input(self.enc_right_b)

        GPIO.add_event_detect(self.enc_left_a, GPIO.BOTH,
                              callback=self.left_wheel_tick)
        GPIO.add_event_detect(self.enc_left_b, GPIO.BOTH,
                              callback=self.left_wheel_tick)

        GPIO.add_event_detect(self.enc_right_a, GPIO.BOTH,
                              callback=self.right_wheel_tick)
        GPIO.add_event_detect(self.enc_right_b, GPIO.BOTH,
                              callback=self.right_wheel_tick)

        self.left_ticks_publisher = self.create_publisher(
            Int16, 'left_wheel_ticks', 10)
        self.right_ticks_publisher = self.create_publisher(
            Int16, 'right_wheel_ticks', 10)

        # Limits of int16. Handle the overflow gracefully (TODO: Test to see if it is necessary)
        self.encoder_minimum = -32768
        self.encoder_maximum = 32767

    def detect_direction(self, prev_A, prev_B, curr_A, curr_B):
        # Previous state and next state as a binary pair of their respective (A,B) channels.
        state = (prev_A << 1) | prev_B
        next_state = (curr_A << 1) | curr_B

        # Since this is a quadrature encoder, we must find out which channel is leading the other to find out the direction of the movement
        # We can accomplish this with a transition matrix that accounts for the sequence of edges detected on each channel
        # Clockwise expected sequence: (0,0) -> (0,1) -> (1,1) -> (1,0) -> ...
        # Counter clockwise expected sequence: (0,0) -> (1,0) -> (1,1) -> (0,1) -> ...
        transition = [
            [0, 1, -1, 0],
            [-1, 0, 0, 1],
            [1, 0, 0, -1],
            [0, -1, 1, 0]
        ]

        direction = transition[state][next_state]

        return direction

    def left_wheel_tick(self, channel):
        curr_state_left_a = GPIO.input(self.enc_left_a)
        curr_state_left_b = GPIO.input(self.enc_left_b)

        direction = self.detect_direction(self.last_state_left_a, self.last_state_left_b,
                                          curr_state_left_a, curr_state_left_b)

        self.last_state_left_a = curr_state_left_a
        self.last_state_left_b = curr_state_left_b

        if direction == 1:
            if self.left_ticks == self.encoder_maximum:
                self.left_ticks = self.encoder_minimum
            else:
                self.left_ticks += 1
        elif direction == -1:
            if self.left_ticks == self.encoder_minimum:
                self.left_ticks = self.encoder_maximum
            else:
                self.left_ticks -= 1

        msg = Int16()
        msg.data = self.left_ticks
        self.left_ticks_publisher.publish(msg)

    def right_wheel_tick(self, channel):
        curr_state_right_a = GPIO.input(self.enc_right_a)
        curr_state_right_b = GPIO.input(self.enc_right_b)

        direction = self.detect_direction(self.last_state_right_a, self.last_state_right_b,
                                          curr_state_right_a, curr_state_right_b)

        self.last_state_right_a = curr_state_right_a
        self.last_state_right_b = curr_state_right_b

        if direction == 1:
            if self.right_ticks == self.encoder_maximum:
                self.right_ticks = self.encoder_minimum
            else:
                self.right_ticks += 1
        elif direction == -1:
            if self.right_ticks == self.encoder_minimum:
                self.right_ticks = self.encoder_maximum
            else:
                self.right_ticks -= 1

        msg = Int16()
        msg.data = self.right_ticks
        self.right_ticks_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    encoder_node = EncoderNode()
    try:
        rclpy.spin(encoder_node)
    except KeyboardInterrupt:
        pass
    finally:
        encoder_node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()


if __name__ == '__main__':
    main()
