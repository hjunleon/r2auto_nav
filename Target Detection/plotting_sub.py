# !usr/bin/python

"""
Created on Wed Mar 25, 2021
Author: @coldyoungguy

References:
https://makersportal.com/blog/2018/1/25/heat-mapping-with-a-64-pixel-infrared-sensor-and-raspberry-pi
http://docs.ros.org/en/rolling/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
"""

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

# Plotting imports
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

mpl.use('tkagg')  # to enable real-time plotting in Raspberry Pi

plt.ion()


class Plotter(Node):

    def __init__(self):
        super().__init__('plotter')
        self.subscription = self.create_subscription(Float32MultiArray, 'heat_array', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, array):
        heat_array = np.reshape(array, (8, 8))
        plt.imshow(heat_array, cmap='magma')  # viridis, plasma, inferno, magma, cividis
        plt.colorbar()
        # plt.clim(1, 8)
        plt.show()
        # self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()
    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
