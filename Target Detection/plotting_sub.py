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
from std_msgs.msg import Float64MultiArray

# Plotting imports
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


class Plotter(Node):

    def __init__(self):
        super().__init__('plotter')
        self.subscription = self.create_subscription(Float64MultiArray, 'heat_array', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, array):
        heat_array = np.reshape(array.data, (64, 64))
        heat_img = Image.fromarray(np.uint8(heat_array), 'L')
        graph = plt.imshow(heat_img, cmap=plt.cm.hot)  # viridis, plasma, inferno, magma, cividis
        # plt.colorbar(graph)
        plt.clim(20, 60)
        plt.draw_all()
        print('Receiving...')
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
