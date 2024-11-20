import rclpy
from rclpy.node import Node
import math
from serial_motor_demo_msgs.msg import MotorCommand
from serial_motor_demo_msgs.msg import MotorVels
from serial_motor_demo_msgs.msg import EncoderVals

class MotorTerminal(Node):

    def __init__(self):
        super().__init__('motor_terminal')

        self.publisher = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.speed_sub = self.create_subscription(MotorVels, 'motor_vels', self.motor_vel_callback, 10)
        self.encoder_sub = self.create_subscription(EncoderVals, 'encoder_vals', self.encoder_val_callback, 10)

        self.pwm_mode = True  # Default mode
        self.max_rev_sec = 255  # Default max rev/sec

    def show_values(self):
        print(f"Motor 1: {self.m1}, Motor 2: {self.m2}")

    def send_motor_once(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        if self.pwm_mode:
            msg.mot_1_req_rad_sec = float(self.m1)
            msg.mot_2_req_rad_sec = float(self.m2)
        else:
            msg.mot_1_req_rad_sec = float(self.m1 * 2 * math.pi)
            msg.mot_2_req_rad_sec = float(self.m2 * 2 * math.pi)

        self.publisher.publish(msg)

    def stop_motors(self):
        msg = MotorCommand()
        msg.is_pwm = self.pwm_mode
        msg.mot_1_req_rad_sec = 0.0
        msg.mot_2_req_rad_sec = 0.0
        self.publisher.publish(msg)

    def set_mode(self, new_mode):
        self.pwm_mode = new_mode
        mode_str = "PWM" if self.pwm_mode else "Feedback"
        print(f"Current Mode: {mode_str}")

    def motor_vel_callback(self, motor_vels):
        mot_1_spd_rev_sec = motor_vels.mot_1_rad_sec / (2 * math.pi)
        mot_2_spd_rev_sec = motor_vels.mot_2_rad_sec / (2 * math.pi)
        print(f"Speed - Motor 1: {mot_1_spd_rev_sec:.2f} rev/s, Motor 2: {mot_2_spd_rev_sec:.2f} rev/s")

    def encoder_val_callback(self, encoder_vals):
        print(f"Encoders - Motor 1: {encoder_vals.mot_1_enc_val}, Motor 2: {encoder_vals.mot_2_enc_val}")

    def update_scale_limits(self):
        print(f"Max Rev/sec set to: {self.max_rev_sec}")

    def run_terminal_interface(self):
        while True:
            print("\nOptions:")
            print("1. Set Motor 1 Speed")
            print("2. Set Motor 2 Speed")
            print("3. Send Motor Command")
            print("4. Stop Motors")
            print("5. Switch Mode")
            print("6. Exit")

            choice = input("Choose an option: ")

            if choice == '1':
                self.m1 = float(input("Enter speed for Motor 1 (-255 to 255): "))
            elif choice == '2':
                self.m2 = float(input("Enter speed for Motor 2 (-255 to 255): "))
            elif choice == '3':
                self.send_motor_once()
            elif choice == '4':
                self.stop_motors()
            elif choice == '5':
                self.set_mode(not self.pwm_mode)
            elif choice == '6':
                break
            else:
                print("Invalid choice. Please try again.")

def main(args=None):
    rclpy.init(args=args)

    motor_terminal = MotorTerminal()

    rate = motor_terminal.create_rate(20)
    try:
        motor_terminal.run_terminal_interface()
        while rclpy.ok():
            rclpy.spin_once(motor_terminal)
    finally:
        motor_terminal.destroy_node()
        rclpy.shutdown()
