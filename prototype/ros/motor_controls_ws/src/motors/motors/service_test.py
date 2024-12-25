import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64

from service_interfaces.srv import LogMessage

import time


class SensorInstallation(Node):

    def __init__(self):
        super().__init__('SensorInstallationService')
        self.srv = self.create_service(LogMessage, '/logger_service', self.install_sensor)

        # self.publisher_ = self.create_publisher(Float64, '/gripper_velocity', 1)

        self.gripper_velocity = 0.1
        self.sleep_time = 5


    def install_sensor(self, request, response):
        message_to_publish = Float64()
        message_to_publish.data = self.gripper_velocity

        # self.publisher_.publish(message_to_publish)

        self.get_logger().info('gonna wait 5 seconds now')

        time.sleep(self.sleep_time)

        self.get_logger().info('done waiting bruh')

        response.success = True
        response.message = "Worked"

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = SensorInstallation()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()