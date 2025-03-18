from numpy import c_
import rclpy
from rclpy.node import Node
from gpio_interface.msg import ServoCmd, GpioCmd
from gpio_interface.srv import ServoInit, ServoSet, GpioInit, GpioSet
import lgpio

handle = lgpio.gpiochip_open(4)
class GpioHandler(Node):
    def __init__(self):
        super().__init__('gpio_node')
        # {servo_id: (gpio pin, pulse_min, pulse_max)}
        self.servos = {}

        # Other GPIO pins for simple output control:
        self.pins = {}

        # Servo control
        self.servo_cmd = self.create_subscription(ServoCmd, '/gpio/servo/cmd', self.servo_set_callback, 10)
        self.servo_initialize_srv = self.create_service(ServoInit, 'servo_init', self.servo_init)
        self.servo_cmd_srv = self.create_service(ServoSet, 'servo_set', self.servo_set_srv)

        # Other GPIO pin control
        self.gpio_cmd = self.create_subscription(GpioCmd, '/gpio/pins/cmd', self.gpio_set_callback, 10)
        self.gpio_initialize_srv = self.create_service(GpioInit, 'gpio_init', self.gpio_init)
        self.gpio_cmd_srv = self.create_service(GpioSet, 'gpio_set', self.gpio_set_srv)

        self.get_logger().info("GPIO handler node initialized")

    # Servo control funcs
    def servo_init(self, init_request, response):
        servo_gpio = init_request.servo_gpio
        pulse_min = init_request.servo_pulse_min
        pulse_max = init_request.servo_pulse_max
        servo_range = init_request.servo_range
        # init the servo and return the servo id
        for servo_id, servo in self.servos.items():
            if servo[0] == servo_gpio:
                self.get_logger().error(f"Servo GPIO {servo_gpio} already initialized")
                response.servo_id = servo_id
                return response
        try:
            lgpio.gpio_claim_output(handle, servo_gpio)
            lgpio.tx_servo(
                handle=handle,
                gpio=servo_gpio,
                pulse_width=pulse_max-pulse_min,
                pulse_cycles=10)

            self.servos[len(self.servos)] = (servo_gpio, pulse_min, pulse_max, servo_range)
            self.get_logger().info(f"Servo {len(self.servos)} initialized")
            response.servo_id = len(self.servos)-1
            return response
        except Exception as e:
            self.get_logger().error(f"Error initializing servo {len(self.servos)} on GPIO:{servo_gpio}: {e}")
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
                handle=handle,
                gpio=self.servos[servo_id][0],
                pulse_width=self.get_pulse(
                    angle=angle,
                    pulse_min=self.servos[servo_id][1],
                    pulse_max=self.servos[servo_id][2],
                    servo_range=self.servos[servo_id][3]),
                pulse_cycles=10)
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting servo {servo_id}: {e}")
            return False

    def servo_set_callback(self, cmd_msg):
        self.servo_set(cmd_msg)

    def servo_set_srv(self, cmd_msg_request, response):
        response.success = self.servo_set(cmd_msg_request)
        return response

    # GPIO control funcs
    def gpio_set(self, cmd_msg):
        if (cmd_msg.gpio_id not in self.pins):
            self.get_logger().error(f"GPIO {cmd_msg.gpio_id} not initialized")
            return False
        try:
            lgpio.gpio_write(handle, self.pins[cmd_msg.gpio_id], cmd_msg.value)
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting GPIO {cmd_msg.gpio_id}: {e}")
            return False

    def gpio_set_callback(self, cmd_message):
        self.gpio_set(cmd_message)

    def gpio_init(self, request, response):
        gpio_pin = request.gpio_pin
        if gpio_pin in self.pins:
            self.get_logger().error(f"GPIO {gpio_pin} already initialized")
            response.gpio_id = -1
            return response

        try:
            lgpio.gpio_claim_output(handle, gpio_pin)
            self.pins[len(self.pins)] = gpio_pin
            self.get_logger().info(f"GPIO {gpio_pin} initialized")
            response.gpio_id = len(self.pins)-1
            return response
        except Exception as e:
            self.get_logger().error(f"Error initializing GPIO {gpio_pin}: {e}")
            response.gpio_id = -1
            return response

    def gpio_set_srv(self, cmd_msg_request, response):
        response.success = self.gpio_set(cmd_msg_request)
        return response

    # Helpers:
    def get_pulse(self, angle, pulse_min, pulse_max, servo_range):
        step = (pulse_max - pulse_min) / servo_range
        if (angle < 0 or angle > servo_range):
            self.get_logger().error(f"Error setting the angle {angle}. Angle not in range 0-180deg")
        pulse = pulse_min + (angle * step)
        return int(pulse)

def main(args=None):
    rclpy.init(args=args)
    servo_handler = GpioHandler()
    rclpy.spin(servo_handler)

    servo_handler.destroy_node()
    rclpy.shutdown()
    lgpio.gpiochip_close(handle)

if __name__ == '__main__':
    main()
