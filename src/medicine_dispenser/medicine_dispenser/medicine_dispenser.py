import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpio_interface.srv import ServoInit, ServoSet
import yaml
import time

class Dispenser(Node):
    def __init__(self,
                 yaml_file='/home/gyattbot/FinalYearProject/src/medicine_dispenser/medicine_dispenser/medicines.yaml'):
        super().__init__('dispenser_node')
        self.get_logger().info("Initializing dispenser node...")

        # Initialize recognized names and prepare a member to store dispensing decision.
        self.recognized_names = []
        self.last_medicine = None

        # Subscribe to recognized persons.
        self._person_subscriber = self.create_subscription(
            String,
            '/recognized_person',
            self.person_subscriber_cb,
            10
        )
        self.get_logger().info("Subscribed to /recognized_person topic.")

        # Load medicine configuration data.
        self.medicine_data = self.load_medicine_data(yaml_file)
        self.get_logger().info("Medicine configuration loaded.")

        # Setup servo-related items.
        self.servos = []
        self.servo_init_cli = self.create_client(ServoInit, 'servo_init')
        self.servo_set_cli = self.create_client(ServoSet, 'servo_set')

        # Wait for servo_init service.
        while not self.servo_init_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo_init service to become available...")
        self.servo_init_req = ServoInit.Request()

        # Wait for servo_set service.
        while not self.servo_set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for servo_set service to become available...")
        self.servo_set_req = ServoSet.Request()

        # Initialize the servos -- change the GPIOs (13 and 12) if needed.
        self.get_logger().info("Initializing servos...")
        self.servos.append(self.send_init_servo_req(  # servo0 = bottom/big
            servo_gpio=13,
            servo_pulse_min=1000,
            servo_pulse_max=2020,
            servo_range=180
        ))
        self.servos.append(self.send_init_servo_req(  # servo1 = top/small
            servo_gpio=12,
            servo_pulse_min=500,
            servo_pulse_max=2500,
            servo_range=180
        ))
        # Set the servos to an initial pose.
        self.get_logger().info("Setting servos to initial positions...")
        self.send_set_servo_req(
            servo_id=0,
            servo_angle=135   # 180-45=135
        )
        self.send_set_servo_req(
            servo_id=1,
            servo_angle=45
        )
        self.get_logger().info("Initialization of dispenser is complete.")

    def load_medicine_data(self, yaml_file):
        self.get_logger().info(f"Loading medicine data from: {yaml_file}")
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
        return data

    def get_servo_commands(self, medicine_name):
        for medicine in self.medicine_data['medicines']:
            if medicine['name'] == medicine_name:
                self.get_logger().info(f"Found servo commands for medicine: {medicine_name}")
                return medicine['servo_commands']
        self.get_logger().error(f"No medicine with name: {medicine_name}")
        return None

    def person_subscriber_cb(self, message):
        """Process recognized person messages and decide on the medicine to dispense."""
        recognized_person = message.data
        self.get_logger().info(f"Received recognized person: {recognized_person}")
        self.recognized_names.append(recognized_person)
        self.get_logger().info(f"Collected {len(self.recognized_names)} recognized names so far.")

        # Return early if not enough names collected.
        if len(self.recognized_names) < 5:
            return None

        # Count occurrences.
        name_counts = {}
        for name in self.recognized_names:
            name_counts[name] = name_counts.get(name, 0) + 1

        most_frequent_name = max(name_counts, key=name_counts.get)
        most_frequent_name_count = name_counts[most_frequent_name]
        self.get_logger().info(
            f"Most frequent recognized person is {most_frequent_name} with {most_frequent_name_count} occurrences."
        )

        # Determine if medicine should be dispensed.
        if most_frequent_name_count < 3:
            self.get_logger().warning(
                f"{most_frequent_name} was mentioned only {most_frequent_name_count} times; no medicine will be dispensed."
            )
            self.recognized_names = []
            self.last_medicine = None
            return None

        # Map recognized person to a medicine from the database.
        if most_frequent_name == "Armaan":
            medicine_name = "medicine1"
            self.get_logger().info("Person Armaan detected. Medicine 'medicine1' selected.")
        elif most_frequent_name == "Wiktor":
            medicine_name = "medicine3"
            self.get_logger().info("Person Wiktor detected. Medicine 'medicine3' selected.")
        else:
            self.get_logger().warning(f"No medicine mapping for person {most_frequent_name}. No dispensing will occur.")
            self.recognized_names = []
            self.last_medicine = None
            return None

        # Reset recognized names for future detections.
        self.recognized_names = []
        self.last_medicine = medicine_name
        self.get_logger().info(f"Medicine decision completed: {medicine_name}")
        return medicine_name

    def send_init_servo_req(self, servo_gpio, servo_pulse_min=1000, servo_pulse_max=2000, servo_range=180):
        self.servo_init_req.servo_gpio = servo_gpio
        self.servo_init_req.servo_pulse_min = servo_pulse_min
        self.servo_init_req.servo_pulse_max = servo_pulse_max
        self.servo_init_req.servo_range = servo_range

        self.get_logger().info(f"Sending initialization request for servo on GPIO {servo_gpio}.")
        future = self.servo_init_cli.call_async(self.servo_init_req)
        while not future.done():
            self.get_logger().info("Waiting for response from gpio_handler for initialization...")
            rclpy.spin_once(self)
        servo_id = future.result().servo_id
        self.get_logger().info(f"Servo initialized: servo ID {servo_id} on GPIO {servo_gpio}")
        return servo_id

    def send_set_servo_req(self, servo_id, servo_angle):
        if servo_id not in range(len(self.servos)) or servo_id == -1:
            self.get_logger().error(f"Invalid servo ID: {servo_id}. Cannot send set request.")
            return

        self.servo_set_req.servo_id = servo_id
        self.servo_set_req.servo_angle = servo_angle
        self.get_logger().info(f"Sending set request: Servo {servo_id} to {servo_angle} degrees.")

        future = self.servo_set_cli.call_async(self.servo_set_req)
        while not future.done():
            self.get_logger().info("Waiting for response from gpio_handler for setting servo...")
            rclpy.spin_once(self)
        self.get_logger().info(f"Servo {servo_id} successfully set to {servo_angle} degrees.")
        return future.result().success

    def dispense_medicine(self, medicine_name):
        """Dispense the selected medicine if a valid name is provided.
        
        If medicine_name is None the method logs that the dispensing is aborted.
        """
        if medicine_name is None:
            self.get_logger().info("No valid medicine identified. Dispensing aborted.")
            return

        self.get_logger().info(f"Starting dispensing process for {medicine_name}...")
        servo_commands = self.get_servo_commands(medicine_name)
        if servo_commands is None:
            self.get_logger().error("No servo commands available for the requested medicine. Aborting dispensing.")
            return

        for servo in servo_commands:
            self.get_logger().info(f"Processing servo {servo['servo_id']} commands...")
            for angle in servo['angle_sequence']:
                self.get_logger().info(
                    f"Setting servo {servo['servo_id']} to {angle} degrees."
                )
                self.send_set_servo_req(servo_id=servo['servo_id'], servo_angle=angle)
                self.get_logger().info("Waiting 2 seconds before next command...")
                time.sleep(2)
        self.get_logger().info(f"Dispensing of {medicine_name} complete.")

def main(args=None):
    rclpy.init(args=args)
    dispenser = Dispenser()

    dispenser.get_logger().info("Dispenser node started. Waiting to collect recognized-person messages...")
    
    # Continuously check for 5 names before making a decision.
    while not dispenser.last_medicine:
        rclpy.spin_once(dispenser, timeout_sec=0.1)
    
    dispenser.get_logger().info(f"Medicine identified for dispensing: {dispenser.last_medicine}")
    dispenser.dispense_medicine(dispenser.last_medicine)
    
    dispenser.get_logger().info("Shutting down dispenser node now.")
    dispenser.destroy_node()