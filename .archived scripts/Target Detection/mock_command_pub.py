#Created on: 070421
#Author:
#Description: Provide specific commands to turret

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float64MultiArray

class CommandNode(Node):

    def __init__(self):
        super().__init__('com_node')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'com_node', 10)
        self.heat_subscription = self.create_subscription(Float64MultiArray, 'heat_array', self.callback, 10)
        self.heat_subscription  # prevent unused variable warning

    def callback(self, heat_array):
        coordinates = get_bright_loc(heat_array.data)
        self.get_logger().info(f'Coordinates: X:{coordinates[0]}, Y:{coordinates[1]}')
        comm_array = Int8MultiArray()
        comm_array.data = command(coordinates)
        self.publisher_.publish(comm_array)
        self.get_logger().info(
            f'Direction: X:{comm_array.data[0]}, Y:{comm_array.data[1]}, Active:{comm_array.data[2]}')
