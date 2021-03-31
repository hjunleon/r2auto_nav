# !usr/bin/python

"""
Created on Wed Mar 31, 2021
Sensor: Newwing AMG8833 IR 8x8 Thermal Imager Array
Author: @coldyoungguy

Copy from r2auto_nav/Target Detection/command_test_sub.py in git
"""

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import numpy as np


class CommandSub(Node):

    def __init__(self):
        super().__init__('com_sub')
        self.heat_subscription = self.create_subscription(Int8MultiArray, 'com_node', self.callback, 10)
        self.heat_subscription

    def callback(self, comm_array):
        self.get_logger().info(
            f'Direction: X:{comm_array.data[0]}, Y:{comm_array.data[1]}, Active:{comm_array.data[2]}')


def main(args=None):
    rclpy.init(args=args)
    command_sub = CommandSub()
    rclpy.spin(command_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()