import rclpy

from rclpy.node import Node
from std_msgs.msg import String

from motors.arduino_serial import arduino # singleton

class ServoProgress(Node):
    def __init__(self):
        super().__init__("ServoProgressNode")
        self.publisher_ = self.create_publisher(String, "/servo_angle", 1)

        self.timer = self.create_timer(0.01, self.read_data_callback)
        self.get_logger().info("INITIALISED")



    def read_data_callback(self):
        self.get_logger().info(f'WAITING: {arduino.in_waiting}')
        if arduino.in_waiting > 0:
            arduino_data = arduino.readline().decode()

            # self.get_logger().info(f'GOOD ARDUINO MESSAGE: {arduino_data}')

            split_data = arduino_data.split(':')

            if split_data[0] == "SERVO_ANGLE":
                message = String()
                message.data = split_data[1]
                self.publisher_.publish(message)
            else:
                self.get_logger().info(f'BOGUS ARDUINO MESSAGE: {arduino_data}')


def main(args=None):

   rclpy.init()

   my_node = ServoProgress()

   try:
      rclpy.spin(my_node)

   except KeyboardInterrupt:
        my_node.destroy_node()

        rclpy.shutdown()
        return

   # rclpy.init(args=args)

   # minimal_publisher = MinimalPublisher()

   # rclpy.spin(minimal_publisher)

   # minimal_publisher.destroy_node()

   # rclpy.shutdown()


if __name__ == '__main__':

   main()
