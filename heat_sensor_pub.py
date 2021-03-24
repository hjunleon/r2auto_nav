# !usr/bin/python

"""
Created on Wed Mar 24, 2021
Sensor: Newwing AMG8833 IR 8x8 Thermal Imager Array
Author: @coldyoungguy

References:
https://makersportal.com/blog/2018/1/25/heat-mapping-with-a-64-pixel-infrared-sensor-and-raspberry-pi
http://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
"""

### ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

### RPi Imports
import sys
import os.path

# this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep
import time
import matplotlib as mpl

mpl.use('tkagg')  # to enable real-time plotting in Raspberry Pi
import matplotlib.pyplot as plt
import numpy as np

sensor = Adafruit_AMG88xx()
# wait for AMG to boot
sleep(0.1)

# preallocating variables
norm_pix = []
cal_vec = []
kk = 0
cal_size = 10  # size of calibration
cal_pix = []
time_prev = time.time()  # time for analyzing time between plot updates


class HeatNode(Node):

    def __init__(self):
        super().__init__('heat_array')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'heat_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int8MultiArray()
        msg.data = [0, 0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % str(msg.data))


def main(args=None):

    rclpy.init(args=args)
    heat_node = HeatNode()
    rclpy.spin(heat_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heat_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
