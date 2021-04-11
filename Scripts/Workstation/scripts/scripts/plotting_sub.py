# !usr/bin/python

"""
Created on Wed Mar 31, 2021
Author: @coldyoungguy

Copy from r2auto_nav/Target Detection/plotting_sub.py in git
"""

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Plotting imports
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

resolution = 64

class Plotter(Node):

    def __init__(self):
        super().__init__('plotter')
        self.subscription = self.create_subscription(Float64MultiArray, 'heat_array', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, array):
        heat_array = np.reshape(array.data, (resolution, resolution))
        heat_img = Image.fromarray(np.uint8(heat_array), 'L')
        graph = plt.imshow(heat_img, cmap=plt.cm.hot)  # viridis, plasma, inferno, magma, cividis
        plt.clim(20, 60)
        plt.draw_all()
        self.get_logger().info('Receiving')
        plt.pause(0.00000000001)


def main(args=None):
    rclpy.init(args=args)
    plotter = Plotter()

    plt.ion()
    # plt.colorbar()
    plt.show()

    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
