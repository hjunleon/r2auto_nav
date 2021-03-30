# !usr/bin/python

"""
Created on Wed Mar 25, 2021
Sensor: Newwing AMG8833 IR 8x8 Thermal Imager Array
Author: @coldyoungguy

References:
https://makersportal.com/blog/2018/1/25/heat-mapping-with-a-64-pixel-infrared-sensor-and-raspberry-pi
http://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
"""

### ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float64MultiArray

### RPi Imports
import time
import busio
import board
import adafruit_amg88xx
import numpy as np
import cv2

i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

# wait for AMG to boot
time.sleep(0.1)


class HeatArray(Node):

    def __init__(self):
        super().__init__('heat_array')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'heat_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = Float64MultiArray()
        resized_array = cv2.resize(np.array(amg.pixels), (64, 64), interpolation=cv2.INTER_LANCZOS4)
        msg.data = np.reshape(resized_array, 4096).tolist()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')


def main(args=None):

    rclpy.init(args=args)
    heat_array = HeatArray()
    rclpy.spin(heat_array)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heat_array.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
