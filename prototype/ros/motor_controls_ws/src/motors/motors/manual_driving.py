import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from motors.arduino_serial import arduino # singleton


class ManualDrivingNode(Node):
    def __init__(self):
        super().__init__("ManualDrivingNode")
        self._subscription = self.create_subscription(String, "/UI_clamp_dispenser_controlls", self.myListenerCallback, 1)


    def myListenerCallback(self, message):
        # self.get_logger().info(f'RECEIVED: {message.data}')

        arduino.write(bytes(message, 'ascii'))


def main(args=None):

   rclpy.init()

   my_node = ManualDrivingNode()

   try:
      rclpy.spin(my_node)

   except KeyboardInterrupt:
        my_node.destroy_node()

        rclpy.shutdown()
        return


if __name__ == '__main__':

   main()
