# !usr/bin/python

"""
Created on Wed Mar 31, 2021
Sensor: Newwing AMG8833 IR 8x8 Thermal Imager Array
Author: @coldyoungguy

Copy from r2auto_nav/Target Detection/command_pub2.py in git
"""

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np

# temperature threshold
temp_thres = 30
h_angle_limit = 21.8 # at 100cm away, horizontal view is 40cm
v_angle_limit = 26.6 # at 100cm away, vertical view is 50cm
resolution = 64
angle_thres = 5


class CommandNode(Node):

    def __init__(self):
        super().__init__('com_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'com_node', 10)
        self.heat_subscription = self.create_subscription(Float64MultiArray, 'heat_array', self.callback, 10)
        self.heat_subscription  # prevent unused variable warning

    def callback(self, heat_array):
        coordinates = get_bright_loc(heat_array.data)
        self.get_logger().info(f'Coordinates: X:{coordinates[0]}, Y:{coordinates[1]}')
        comm_array = Float32MultiArray()
        comm_array.data = command(coordinates)
        self.publisher_.publish(comm_array)
        self.get_logger().info(
            f'Direction: X:{comm_array.data[0]}, Y:{comm_array.data[1]}, Active:{comm_array.data[2]}')


# def get_bright_loc(array):
#     coord = [0, 0, 0]  # scripts, y, on or off
#     highest_temp = temp_thres
#     for r in range(array.shape[0]):
#         for c in range(array.shape[1]):
#             if array[r, c] > highest_temp:
#                 highest_temp = array[r, c]
#                 if abs(coord[0] - r) > radius_thres and abs(coord[1] - c) >= radius_thres:
#                     coord[0], coord[1], coord[2] = r, c, 1
#     return coord


def get_bright_loc(array):
    heat_img = np.reshape(array, (resolution, resolution))
    coord = [resolution // 2, resolution // 2, 0]  # scripts, y, on or off
    i, j = np.unravel_index(heat_img.argmax(), heat_img.shape)
    if heat_img[i, j] > temp_thres:
        coord = [j, i, 1]
    return coord


def command(coord):
    # -1 == left, down
    # 0 == stop
    # 1 == right, up

    command_list = [0.0, 0.0, float(coord[2])]
    x_hori_diff, y_vert_diff = coord[0] - (resolution // 2), coord[1] - (resolution // 2)
    x_angle_diff, y_angle_diff = h_angle_limit * (x_hori_diff / (resolution / 2)), v_angle_limit * (y_vert_diff / (resolution / 2))

    if abs(x_angle_diff) >= angle_thres:
        command_list[0] = float(x_angle_diff)
    else:
        command_list[0] = 0.0

    if abs(y_angle_diff) >= angle_thres:
        command_list[1] = float(y_angle_diff)
    else:
        command_list[1] = 0.0

    return command_list


def main(args=None):
    rclpy.init(args=args)
    command_pub = CommandNode()
    rclpy.spin(command_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
