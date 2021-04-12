"""
Created on Wed Mar 31, 2021
Sensor: thermal test publisher
Author: @coldyoungguy

Copy from r2auto_nav/Target Detection/thermal_test.py in git
"""

### ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

### RPi Imports
import time
import busio
import board
import adafruit_amg88xx
import numpy as np
i2c = busio.I2C(board.SCL, board.SDA)
amg = adafruit_amg88xx.AMG88XX(i2c)

# wait for AMG to boot
time.sleep(0.1)


class HeatArray(Node):

    def __init__(self):
        super().__init__('heat_array')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'heat_ori', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = Float32MultiArray()
        msg.data = np.reshape(amg.pixels, 64).tolist()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing...')


def main(args=None):
    rclpy.init(args=args)
    heat_array = HeatArray()
    rclpy.spin(heat_array)

    # Destroy the node explicitly
    heat_array.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
