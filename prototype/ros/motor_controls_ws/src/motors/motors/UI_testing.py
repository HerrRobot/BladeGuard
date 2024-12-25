import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from service_interfaces.srv import LogMessage

from datetime import datetime

class UI_TestingNode(Node):
    def __init__(self):
        super().__init__("UI_TestingNode")
        self._publisher = self.create_publisher(String, "/distance_sensors", 10)

        self.left = 10.0
        self.right = 12.5
        self.ceiling = 17.2
        self.root = 0.2

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.publishLogMessage)

        self.client = self.create_client(LogMessage, '/logger_service')

        self.messageID = 0


    def publishSensorData(self):
        message = String()

        self.left -= 0.1
        self.right -= 0.1
        self.ceiling -= 0.1
        self.root += 0.2

        message.data = f"{self.left};{self.right};{self.ceiling};{self.root}"
        self._publisher.publish(message)


    def publishLogMessage(self):
        request = LogMessage.Request()

        request.timestamp = datetime.now().strftime('%H:%M:%S')
        request.message = f"Log message #{self.messageID}"
        self.messageID += 1

        self.client.call_async(request)

        


def main(args=None):

   rclpy.init()

   my_node = UI_TestingNode()

   try:
      rclpy.spin(my_node)

   except KeyboardInterrupt:
        my_node.destroy_node()

        rclpy.shutdown()
        return


if __name__ == '__main__':

   main()
