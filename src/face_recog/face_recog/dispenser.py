import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpio_interface.srv import ServoInit, ServoSet
import sys

class Dispenser(Node):
    def __init__(self):
        super().__init__('dispenser_node')
        self._person_subscriber = self.create_subscription(
            String,
            '/recognized_person',
            self.person_subscriber_cb,
            10
        )
        # servo id list
        self.servos = []
        self.servo_init_cli = self.create_client(ServoInit, 'servo_init')
        self.servo_set_cli = self.create_client(ServoSet, 'servo_set')
        
        while not self.servo_init_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.servo_init_req = ServoInit.Request()

        while not self.servo_set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.servo_set_req = ServoSet.Request()

        # Initialize the servos -- Change the GPIOs (12 and 13) if needed!!!
        # TODO: Fix the return type from the result message using sys
        self.servos.append(self.send_init_servo_req(13).result())
        self.servos.append(self.send_init_servo_req(12).result())
        for servo_id in self.servos:
            self.get_logger().info(f"Initialized servo ID {servo_id}")

    def person_subscriber_cb(self, message):
        pass

    def send_init_servo_req(
        self,
        servo_gpio,
        servo_pulse_min=500,
        servo_pulse_max=2500,
        servo_range=180
    ):
        self.servo_init_req.servo_gpio = servo_gpio
        self.servo_init_req.servo_pulse_min = servo_pulse_min
        self.servo_init_req.servo_pulse_max = servo_pulse_max
        self.servo_init_req.servo_range = servo_range
        self.get_logger().info(f"Sent initializing request for GPIO {servo_gpio}")
        return self.servo_init_cli.call_async(self.servo_init_req)

    def send_set_servo_req(
        self,
        servo_id,
        servo_angle
    ):
        self.servo_set_req.servo_id = servo_id
        self.servo_set_req.servo_angle = servo_angle
        self.get_logger().info(f"Sent {servo_angle}deg request for Servo {servo_id}")
        return self.servo_set_cli.call_async(self.servo_set_req)

    def dispense_medicine(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    dispenser = Dispenser()
    rclpy.spin(dispenser)
    dispenser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
