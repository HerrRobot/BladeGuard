import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64

from service_interfaces.srv import LogMessage
from service_interfaces.srv import ManualAction

import time
import random


class SensorInstallation(Node):

    def __init__(self):
        super().__init__('SensorInstallationService')
        # self.srv = self.create_service(LogMessage, '/logger_service', self.install_sensor)

        # self.publisher_ = self.create_publisher(Float64, '/gripper_velocity', 1)

        self.manual_action_server = self.create_service(ManualAction, '/manual_action_service', self.manual_action_callback)

        self.gripper_velocity = 0.1
        self.sleep_time = 5


    def manual_action_callback(self, request, response):
        self.get_logger().info(f'Got action {request.action_name}. Waiting 5 seconds')

        time.sleep(5)

        response.success = True
        response.action_name = request.action_name
        response.response_id = request.request_id

        randVal = random.random()

        self.get_logger().info(f'Randval = {randVal}')

        if randVal < 0.2:
            response.success = False
        elif randVal < 0.4:
            time.sleep(15)

        return response



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