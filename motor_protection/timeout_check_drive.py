import rclpy, can
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from robot_interfaces.msg import CanCommand

from rclpy.qos import qos_profile_sensor_data
from myactuator_lib import Motor as MyActuatorMotor



class TimeoutCheck(Node):

    def __init__(self):
        super().__init__('timeout_check')

        # Subscriber to the cmd_vel topic
        self.drive_sub = self.create_subscription(Twist, '/cmd_vel', self.drive_callback, qos_profile_sensor_data)

        # Publisher to the CAN message topic
        self.can_pub = self.create_publisher(CanCommand, 'outgoing_can_commands', 10)

        # Timer for timeout
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.got_drive = False

        # Set up Drive Motors
        self.front_left_motor   = MyActuatorMotor(0x144)
        self.back_left_motor    = MyActuatorMotor(0x143)
        self.front_right_motor  = MyActuatorMotor(0x142)
        self.back_right_motor   = MyActuatorMotor(0x141)

    # Callback for Drive Subscription
    def drive_callback(self, drive_message):
        self.got_drive = True

    # Callback for timer
    def timer_callback(self):
        
        # If we have not recieved a drive message in 0.5 seconds stop it
        if not self.got_drive:
            self.send_can_message(self.front_left_motor.Speed_Closed_loop_Control_Command(0.0))
            self.send_can_message(self.front_right_motor.Speed_Closed_loop_Control_Command(0.0))
            self.send_can_message(self.back_left_motor.Speed_Closed_loop_Control_Command(0.0))
            self.send_can_message(self.back_right_motor.Speed_Closed_loop_Control_Command(0.0))

        # Reset for next go around
        self.got_drive = False

    def send_can_message(self, can_command: can.Message):

        can_outgoing_ros_message = CanCommand()
        can_outgoing_ros_message.arbitration_id = can_command.arbitration_id
        can_outgoing_ros_message.is_extended_id = can_command.is_extended_id
        can_outgoing_ros_message.byte_0 = can_command.data[0]
        can_outgoing_ros_message.byte_1 = can_command.data[1]
        can_outgoing_ros_message.byte_2 = can_command.data[2]
        can_outgoing_ros_message.byte_3 = can_command.data[3]
        can_outgoing_ros_message.byte_4 = can_command.data[4]
        can_outgoing_ros_message.byte_5 = can_command.data[5]
        can_outgoing_ros_message.byte_6 = can_command.data[6]
        can_outgoing_ros_message.byte_7 = can_command.data[7]

        self.can_pub.publish(can_outgoing_ros_message)

def main(args=None):
    rclpy.init(args=args)

    timeout_check = TimeoutCheck()

    rclpy.spin(timeout_check)

    timeout_check.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()