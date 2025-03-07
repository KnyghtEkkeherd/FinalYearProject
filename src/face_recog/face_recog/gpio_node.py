from numpy import c_
import rclpy
from rclpy.node import Node
from gpio_interface.msg import ServoCmd
from gpio_interface.srv import ServoInit, ServoSet
import lgpio

class ServoHandler(Node):
    def __init__(self):
        super().__init__('gpio_node')
        # {servo_id: gpio pin}
        self.servo_gpios = {}
        self.handle = lgpio.gpiochip_open(4)
        self.servo_pwm_min = 500
        self.servo_pwm_max = 2500
        self.servo_count = 0
        self.servo_pwm_step = (self.servo_pwm_max - self.servo_pwm_min) / 180

        self.servo_cmd = self.create_subscription(ServoCmd, '/servo/cmd', self.servo_set, 10)
        self.servo_initialize_srv = self.create_service(ServoInit, 'servo_init', self.servo_init)
        self.servo_cmd_srv = self.create_service(ServoSet, 'servo_cmd', self.servo_set_srv)

    def __del__(self):
        try:
            lgpio.gpiochip_close(self.handle)
            self.get_logger().info("GPIO chip closed successfully")
        except Exception as e:
            self.get_logger().error(f"Error closing GPIO chip: {e}")

    def servo_init(self, init_request, response):
        servo_gpio = init_request.servo_gpio
        # init the servo and return the servo id
        if (servo_gpio in self.servo_gpios.values()):
            self.get_logger().error(f"Servo {servo_gpio} already initialized")
            response.servo_id = -1
        try:
            lgpio.gpio_claim_output(self.handle, servo_gpio)
            lgpio.tx_servo(self.handle, servo_gpio, 0)
            self.servo_gpios[self.servo_count] = servo_gpio
            self.get_logger().info(f"Servo {self.servo_count} initialized")
            response.servo_id = self.servo_count
            self.servo_count += 1
        except Exception as e:
            self.get_logger().error(f"Error initializing servo {self.servo_count} on GPIO:{servo_gpio}: {e}")

    def servo_set(self, cmd_msg):
        servo_id = cmd_msg.servo_id
        angle = cmd_msg.servo_angle
        if (servo_id not in self.servo_gpios.keys()):
            self.get_logger().error(f"Servo {servo_id} not initialized")
            return False
        try:
            lgpio.tx_servo(self.handle, self.servo_gpios[servo_id], self.servo_pwm_min + self.servo_pwm_step*angle)
            return True
        except Exception as e:
            self.get_logger().error(f"Error setting servo {servo_id}: {e}")

    def servo_set_srv(self, cmd_msg_request, response):
        response.success = self.servo_set(cmd_msg_request)
        return response


    def cmd_callback(self, cmd_msg):
        servo_id = cmd_msg.servo_id
        angle = cmd_msg.servo_angle
        if (servo_id not in self.servo_gpios.keys()):
            self.get_logger().error(f"Servo {servo_id} not initialized")
            return -1
        try:
            lgpio.tx_servo(self.handle, self.servo_gpios[servo_id], self.servo_pwm_min + self.servo_pwm_step*angle)
        except Exception as e:
            self.get_logger().error(f"Error setting servo {servo_id} on GPIO:{self.servo_gpios[servo_id]}: {e}")

def main(args=None):
    rclpy.init(args=args)
    servo_handler = ServoHandler()
    rclpy.spin(servo_handler)

    servo_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
