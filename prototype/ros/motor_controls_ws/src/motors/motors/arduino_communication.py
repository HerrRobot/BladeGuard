import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from motors.arduino_serial import arduino # singleton


class ServoController(Node):
    def __init__(self):
        super().__init__("ClampControllerNode")
        self._subscription = self.create_subscription(String, "/UI_clamp_dispenser_controlls", self.myListenerCallback, 1)


    def myListenerCallback(self, message):
        self.get_logger().info(f'RECEIVED: {message.data}')

        if message.data == "MIN":
            arduino.write(bytes("MIN", 'utf-8'))
            # self.get_logger().info(f'ARDUINO RESPONSE: {arduino.readline()}')
        elif message.data == "MAX":
            arduino.write(bytes("MAX", 'utf-8'))
            # self.get_logger().info(f'ARDUINO RESPONSE: {arduino.readline()}')


def main(args=None):

   rclpy.init()

   my_node = ServoController()

   try:
      rclpy.spin(my_node)

   except KeyboardInterrupt:
        my_node.destroy_node()

        rclpy.shutdown()
        return


if __name__ == '__main__':

   main()
