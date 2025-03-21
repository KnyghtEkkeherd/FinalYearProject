import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpio_interface.srv import ServoInit, ServoSet
import sys
import yaml
import time

class Dispenser(Node):
    def __init__(self, yaml_file='/home/gyattbot/FinalYearProject/src/medicine_dispenser/medicine_dispenser/medicines.yaml'):
        super().__init__('dispenser_node')
        self._person_subscriber = self.create_subscription(
            String,
            '/recognized_person',
            self.person_subscriber_cb,
            10
        )
        # medicine command file
        self.medicine_data = self.load_medicine_data(yaml_file)
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

        # Initialize the servos -- Change the GPIOs (12 and 13) if needed
        # and/or change servo parameters
        self.servos.append(self.send_init_servo_req(
                               servo_gpio = 13,
                               servo_pulse_min = 1000,
                               servo_pulse_max = 2000,
                               servo_range = 180
                           ))
        self.servos.append(self.send_init_servo_req(
                               servo_gpio = 12,
                               servo_pulse_min = 500,
                               servo_pulse_max = 2500,
                               servo_range = 180
                           ))

    def load_medicine_data(self, yaml_file):
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def get_servo_commands(self, medicine_name):
        for medicine in self.medicine_data['medicines']:
            if medicine['name'] == medicine_name:
                return medicine['servo_commands']
        self.get_logger().error(f"No medicine with name: {medicine_name}")
        return None

    def person_subscriber_cb(self, message):
        pass

    def send_init_servo_req(
        self,
        servo_gpio,
        servo_pulse_min=1000,
        servo_pulse_max=2000,
        servo_range=180
    ):
        self.servo_init_req.servo_gpio = servo_gpio
        self.servo_init_req.servo_pulse_min = servo_pulse_min
        self.servo_init_req.servo_pulse_max = servo_pulse_max
        self.servo_init_req.servo_range = servo_range

        self.get_logger().info(f"Sent initializing request for GPIO {servo_gpio}")
        future = self.servo_init_cli.call_async(self.servo_init_req)
        while not future.done():
            self.get_logger().info("Waiting for the response from the GPIO handler")
            rclpy.spin_once(self)

        self.get_logger().info(f"Initialized servo {future.result().servo_id} on GPIO {servo_gpio}")
        return future.result().servo_id

    def send_set_servo_req(
        self,
        servo_id,
        servo_angle
    ):
        if servo_id not in range(len(self.servos)) or servo_id == -1:
            self.get_logger().error(f"Error sending set request to Servo {servo_id}. Invalid servo ID.")
            return

        self.servo_set_req.servo_id = servo_id
        self.servo_set_req.servo_angle = servo_angle
        self.get_logger().info(f"Sent {servo_angle}deg request for Servo {servo_id}")

        future = self.servo_set_cli.call_async(self.servo_set_req)

        while not future.done():
            self.get_logger().info("Waiting for the response from the GPIO handler")
            rclpy.spin_once(self)

        self.get_logger().info(f"Request to set servo {servo_id} to {servo_angle}deg successful")
        return future.result().success

    def dispense_medicine(self, medicine_name):
        self.get_logger().info(f"Dispensing: {medicine_name}")
        for servo in self.get_servo_commands(medicine_name):
            for angle in servo['angle_sequence']:
                self.send_set_servo_req(servo_id=servo['servo_id'], servo_angle=angle)
                time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    dispenser = Dispenser()
    time.sleep(10)
    dispenser.dispense_medicine('medicine1')
    rclpy.spin(dispenser)
    dispenser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
