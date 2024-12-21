import rclpy

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

import serial

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.5)

executor_timeout = 0.1

class ArduinoWriter(Node):
    """
    Listens to the '/write_arduino_message' topic and writes those messages to the Arduino via serial connection
    """
    def __init__(self):
        super().__init__("ArduinoWriterNode")
        self._subscription = self.create_subscription(String, "/write_arduino_message", self.writeTopicMessageToArduino, 100)


    def writeTopicMessageToArduino(self, message):
        """
        Writes the message to the Arduino via serial connection
        """
        command = message.data

        arduino.write(bytes(command, 'ascii'))


class ArduinoMessageDistributor(Node):
    """
    Reads Arduino serial messages and distributes them to the correct topics
    """
    def __init__(self):
        super().__init__("ArduinoMessageDistributorNode")
        self._publisher = self.create_publisher(String, "/publish_arduino_message", 100)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.distributeArduinoMessage)

    
    def distributeArduinoMessage(self):
        """
        Reads a single message and publishes it to the correct topic
        """
        if(arduino.in_waiting > 0):
            arduino_data = arduino.readline().decode()

            message_to_publish = String()
            message_to_publish.data = arduino_data

            self._publisher.publish(message_to_publish)
    
    


def main(args=None):

    rclpy.init()

    arduino_writer_node = ArduinoWriter()
    arduino_distributor_node = ArduinoMessageDistributor()

    executor = SingleThreadedExecutor()
    executor.add_node(arduino_writer_node)
    executor.add_node(arduino_distributor_node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=executor_timeout)
    finally:
        arduino_writer_node.destroy_node()
        arduino_distributor_node.destroy_node()

        rclpy.shutdown()
        return



if __name__ == '__main__':

   main()
