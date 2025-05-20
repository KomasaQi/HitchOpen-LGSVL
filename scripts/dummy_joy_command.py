import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from race_msgs.msg import VehicleManualControlCommand
import argparse
from environs import Env

CAR_NUMBER = 7

env = Env()
env.read_env("race.env")


class MinimalPublisher(Node):
    def __init__(self, suffix, topic):
        super().__init__("publish_dummy_joystick_control" + suffix)
        self.declare_parameter("joystick_emergency", False)
        self.declare_parameter("publish_command", True)
        self.declare_parameter("take_control", False)
        self.declare_parameter("limit_auto_throttle", False)
        self.declare_parameter("use_manual_cmd", False)
        self.declare_parameter("control_type", 0)
        self.publisher_ = self.create_publisher(
            VehicleManualControlCommand, topic, qos_profile_sensor_data
        )
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = VehicleManualControlCommand()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.header.vehicle_number = CAR_NUMBER
        msg.vehicle_control_command.lon_control_type = self.get_parameter("control_type").value
        msg.vehicle_control_command.emergency_stop_cmd = self.get_parameter(
            "joystick_emergency"
        ).value
        msg.take_control = self.get_parameter("take_control").value
        msg.limit_auto_throttle = self.get_parameter("limit_auto_throttle").value
        msg.use_manual_cmd = self.get_parameter("use_manual_cmd").value
        if self.get_parameter("publish_command").value:
            self.publisher_.publish(msg)


def main(args=None):
    if env.bool("LAUNCH_JOYSTICK"):
        print("Dummy joystick is not needed when using real joystick")
        return

    rclpy.init()
    minimal_publisher = MinimalPublisher(args.suffix, args.topic)
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--suffix", "-s", default="", type=str)
    parser.add_argument("--topic", "-t", default="/joystick/control_command", type=str)
    args = parser.parse_args()
    main(args=args)
