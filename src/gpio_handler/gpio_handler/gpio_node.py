from numpy import c_
import rclpy
from rclpy.node import Node
from gpio_interface.msg import ServoCmd
from gpio_interface.srv import ServoInit, ServoSet
import lgpio

handle = lgpio.gpiochip_open(4)

class ServoHandler(Node):
    def __init__(self):
        super().__init__('gpio_node')
        # {servo_id: (gpio pin, pulse_min, pulse_max)}
        self.servos = {}

        self.servo_cmd = self.create_subscription(ServoCmd, '/servo/cmd', self.servo_set, 10)
        self.servo_initialize_srv = self.create_service(ServoInit, 'servo_init', self.servo_init)
        self.servo_cmd_srv = self.create_service(ServoSet, 'servo_set', self.servo_set_srv)

    def servo_init(self, init_request, response):
        servo_gpio = init_request.servo_gpio
        pulse_min = init_request.pulse_min
        pulse_max = init_request.pulse_max
        # init the servo and return the servo id
        if (servo_gpio in self.servos.values()):
            self.get_logger().error(f"Servo {servo_gpio} already initialized")
            response.servo_id = -1
            return response
        try:
            lgpio.gpio_claim_output(handle, servo_gpio)
            lgpio.tx_servo(
                handle=handle,
                gpio=servo_gpio,
                pulse_width=pulse_max-pulse_min,
                pulse_cycles=2)

            self.servos[self.servo_count] = servo_gpio
            self.get_logger().info(f"Servo {self.servo_count} initialized")
            response.servo_id = len(self.servos)-1
            return response
        except Exception as e:
            self.get_logger().error(f"Error initializing servo {self.servo_count} on GPIO:{servo_gpio}: {e}")
            response.servo_id = -1
            return response

    def servo_set(self, cmd_msg):
        servo_id = cmd_msg.servo_id
        angle = cmd_msg.servo_angle
        if (servo_id not in self.servos.keys()):
            self.get_logger().error(f"Servo {servo_id} not initialized")
            return False
        try:
            lgpio.tx_servo(
                handle=self.handle,
                gpio=self.servos[servo_id][0],
                pulse_width=self.get_pulse(
                    angle=angle,
                    pulse_min=self.servos[servo_id][1],
                    pulse_max=self.servos[servo_id][2]),
                pulse_cycles=2)
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting servo {servo_id}: {e}")
            return False

    def servo_set_srv(self, cmd_msg_request, response):
        response.success = self.servo_set(cmd_msg_request)
        return response

    def cmd_callback(self, cmd_msg):
        servo_id = cmd_msg.servo_id
        angle = cmd_msg.servo_angle
        if (servo_id not in self.servos.keys()):
            self.get_logger().error(f"Servo {servo_id} not initialized")
            return -1
        try:
            lgpio.tx_servo(
                handle=handle,
                gpio=self.servos[servo_id][0],
                pulse_width=self.get_pulse(
                    angle=angle,
                    pulse_min=self.servos[servo_id][1],
                    pulse_max=self.servos[servo_id][2]),
                pulse_cycles=2)
        except Exception as e:
            self.get_logger().error(f"Error setting servo {servo_id} on GPIO:{self.servos[servo_id]}: {e}")
            return -1

    # Helpers:
    def get_pulse(self, angle, pulse_min, pulse_max):
        step = (pulse_max - pulse_min) / 180
        if (angle < 0 or angle > 180):
            self.get_logger().error(f"Error setting the angle {angle}. Angle not in range 0-180deg")
        pulse = pulse_min + (angle * step)
        return int(pulse)

def main(args=None):
    rclpy.init(args=args)
    servo_handler = ServoHandler()
    rclpy.spin(servo_handler)

    servo_handler.destroy_node()
    rclpy.shutdown()
    lgpio.gpiochip_close(handle)

if __name__ == '__main__':
    main()
