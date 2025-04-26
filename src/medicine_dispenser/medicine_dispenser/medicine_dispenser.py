import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpio_interface.srv import ServoInit, ServoSet
import yaml

class Dispenser(Node):
    def __init__(self, yaml_file='/home/gyattbot/FinalYearProject/src/medicine_dispenser/medicine_dispenser/medicines.yaml'):
        super().__init__('dispenser_node')
        self.recognized_names = []
        self._person_subscriber = self.create_subscription(
            String,
            '/recognized_person',
            self.person_subscriber_cb,
            10
        )
        # Load medicine command file
        self.medicine_data = self.load_medicine_data(yaml_file)
        # create a list that will hold servo IDs (None until initialized)
        self.servos = [None, None]
        
        # create service clients
        self.servo_init_cli = self.create_client(ServoInit, 'servo_init')
        self.servo_set_cli = self.create_client(ServoSet, 'servo_set')

        # Wait (blocking) until services are available
        while not self.servo_init_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo_init service...")
        while not self.servo_set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo_set service...")

        # Asynchronously initialize servos
        self.send_init_servo_req_async(
            servo_gpio=13, 
            servo_pulse_min=1000,
            servo_pulse_max=2020,
            servo_range=180,
            servo_index=0   # this will be servo0 (bottom/big)
        )
        self.send_init_servo_req_async(
            servo_gpio=12, 
            servo_pulse_min=500,
            servo_pulse_max=2500,
            servo_range=180,
            servo_index=1   # this will be servo1 (top/small)
        )

        # Create a timer to check if both servos have been initialized.
        self.init_pose_timer = self.create_timer(1.0, self.try_set_initial_pose)

    def load_medicine_data(self, yaml_file):
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)

    def person_subscriber_cb(self, message):
        recognized_person = message.data
        self.get_logger().info(f"Recognized person: {recognized_person}")
        self.recognized_names.append(recognized_person)
        if len(self.recognized_names) < 5:
            return 

        # Count the occurrences
        name_counts = {}
        for name in self.recognized_names:
            name_counts[name] = name_counts.get(name, 0) + 1
        most_frequent_name = max(name_counts, key=name_counts.get)
        most_frequent_name_count = name_counts[most_frequent_name]
        self.get_logger().info(f"Most frequent recognized person: {most_frequent_name} ({most_frequent_name_count} times)")
        
        if most_frequent_name_count < 3:
            self.get_logger().warning(f"{most_frequent_name} was mentioned only {most_frequent_name_count} times; no meds dispensed.")
            self.recognized_names = []
            return

        if most_frequent_name == "Armaan":
            medicine_name = "medicine1"
        elif most_frequent_name == "Wiktor":
            medicine_name = "medicine3"
        else:
            self.get_logger().warning(f"Womp womp {most_frequent_name} has no meds in the db")
            self.recognized_names = []
            return

        self.dispense_medicine(medicine_name)
        self.recognized_names = []

    def send_init_servo_req_async(self, servo_gpio, servo_pulse_min=1000, servo_pulse_max=2000, servo_range=180, servo_index=0):
        req = ServoInit.Request()
        req.servo_gpio = servo_gpio
        req.servo_pulse_min = servo_pulse_min
        req.servo_pulse_max = servo_pulse_max
        req.servo_range = servo_range

        self.get_logger().info(f"Initializing servo on GPIO {servo_gpio}")
        future = self.servo_init_cli.call_async(req)
        # Capture servo_gpio and servo_index in the callback lambda
        future.add_done_callback(lambda fut: self.init_servo_callback(fut, servo_gpio, servo_index))

    def init_servo_callback(self, future, servo_gpio, servo_index):
        try:
            result = future.result()
            self.servos[servo_index] = result.servo_id
            self.get_logger().info(f"Initialized servo {result.servo_id} on GPIO {servo_gpio}")
        except Exception as e:
            self.get_logger().error(f"Servo init call failed for GPIO {servo_gpio}: {e}")

    def try_set_initial_pose(self):
        # Once both servos are initialized, set their initial pose and cancel the timer.
        if self.servos[0] is not None and self.servos[1] is not None:
            self.get_logger().info("Both servos initialized, setting initial pose.")
            self.send_set_servo_req_async(servo_id=0, servo_angle=135)  # servo0: bottom/big (180-45)
            self.send_set_servo_req_async(servo_id=1, servo_angle=45)   # servo1: top/small
            self.init_pose_timer.cancel()

    def send_set_servo_req_async(self, servo_id, servo_angle, done_callback=None):
        if servo_id not in range(len(self.servos)) or servo_id == -1:
            self.get_logger().error(f"Invalid servo id: {servo_id}.")
            if done_callback:
                done_callback(False)
            return
        req = ServoSet.Request()
        req.servo_id = servo_id
        req.servo_angle = servo_angle
        self.get_logger().info(f"Requesting servo {servo_id} to set to {servo_angle}°")
        future = self.servo_set_cli.call_async(req)
        future.add_done_callback(lambda fut: self.set_servo_callback(fut, servo_id, servo_angle, done_callback))

    def set_servo_callback(self, future, servo_id, servo_angle, done_callback):
        try:
            result = future.result()
            self.get_logger().info(f"Servo {servo_id} successfully set to {servo_angle}°")
            if done_callback:
                done_callback(True)
        except Exception as e:
            self.get_logger().error(f"Failed to set servo {servo_id} to {servo_angle}°: {e}")
            if done_callback:
                done_callback(False)

    def get_servo_commands(self, medicine_name):
        for medicine in self.medicine_data['medicines']:
            if medicine['name'] == medicine_name:
                return medicine['servo_commands']
        self.get_logger().error(f"No medicine with name: {medicine_name}")
        return None

    def dispense_medicine(self, medicine_name):
        self.get_logger().info(f"Dispensing medicine: {medicine_name}")
        commands = self.get_servo_commands(medicine_name)
        if commands is None:
            return
        # Flatten the list into a list of (servo_id, angle) tuples:
        self.command_list = []
        for servo in commands:
            for angle in servo['angle_sequence']:
                self.command_list.append((servo['servo_id'], angle))
        # Start the asynchronous chain with the first command.
        self.execute_servo_commands(0)

    def execute_servo_commands(self, index):
        if index >= len(self.command_list):
            self.get_logger().info("Completed dispensing commands.")
            return
        servo_id, angle = self.command_list[index]
        # Start the asynchronous servo set and wait for it to finish.
        self.send_set_servo_req_async(
            servo_id, angle,
            done_callback=lambda success: self.after_command(index)
        )

    def after_command(self, index):
        self.get_logger().info("Command executed; waiting 2 seconds before the next command.")
        # Create a one-shot timer for 2 seconds before executing the next command.
        def timer_callback():
            self.execute_servo_commands(index + 1)
            timer.cancel()  # cancel the one-shot timer once fired
        timer = self.create_timer(2.0, timer_callback)

def main(args=None):
    rclpy.init(args=args)
    dispenser = Dispenser()
    # The node can now process both subscriptions and asynchronous service callbacks.
    rclpy.spin(dispenser)
    dispenser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()