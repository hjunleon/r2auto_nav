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
from std_msgs.msg import Int8MultiArray, Float32MultiArray

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

# pre-allocating variables (for interpolation)
norm_pix = []
cal_vec = []
kk = 0
cal_size = 10  # size of calibration
cal_pix = []
time_prev = time.time()  # time for analyzing time between plot updates

# temperature threshold
temp_thres = 30


class HeatArray(Node):

    def __init__(self):
        super().__init__('heat_array')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'heat_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = Float32MultiArray()
        msg.data = amg.pixels
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


class CommandNode(Node):

    def __init__(self):
        super().__init__('com_array')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'com_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = Int8MultiArray()
        msg.data = command()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def bright_loc():
    amg_array = np.array(amg.pixels)
    resized_array = cv2.resize(amg_array, (64, 64), interpolation=cv2.INTER_LANCZOS4)
    coord = [0, 0, 0]  # x, y, on or off
    for r in range(amg_array.shape[0]):
        for c in range(amg_array.shape[1]):
            if amg_array[r, c] > highest_temp:
                highest_temp = amg_array[r, c]
                coord[0], coord[1], coord[2] = r, c, 1
    return coord


def command():
    # -1 == left, down
    # 0 == stop motion
    # 1 == right, up
    coord = bright_loc()
    comm = bright_loc()
    if coord[0] == 3 or coord[0] == 4:
        comm[0] = 0
    elif coord[0] < 3:
        comm[0] = -1
    else:
        comm[0] = 1

    if coord[1] == 3 or coord[1] == 4:
        comm[1] = 0
    elif coord[1] < 3:
        comm[1] = -1
    else:
        comm[1] = 1

    return comm


def main(args=None):
    try:
        rclpy.init(args=args)
        heat_array = HeatArray()
        comm_node = CommandNode
        rclpy.spin(heat_array)
        rclpy.spin(comm_node)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        heat_array.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        print("CTRL-C: Program Stopping via Keyboard Interrupt...")

    finally:
        print("Exiting Loop")


if __name__ == '__main__':
    main()
