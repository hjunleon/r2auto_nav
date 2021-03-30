# !usr/bin/python

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float32MultiArray

# temperature threshold
temp_thres = 30
radius_thres = 5
command_list = [0, 0, 0]


class CommandNode(Node):

    def __init__(self):
        super().__init__('com_array')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'com_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = Int8MultiArray()
        msg.data = command_list
        self.publisher_.publish(msg)
        self.get_logger().info('Movement: "%s"' % msg.data)


class HeatArraySub(Node):

    def __init__(self):
        super().__init__('heat_array_sub')
        self.subscription = self.create_subscription(Float32MultiArray, 'heat_array', self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.coordinates = [0, 0, 0]

    def callback(self, heat_array):
        self.coordinates = get_bright_loc(heat_array.data)
        self.get_logger().info('Position "%s"' % self.coordinates)
        command(self.coordinates)


def get_bright_loc(array):
    coord = [0, 0, 0]  # x, y, on or off
    highest_temp = temp_thres
    for r in range(array.shape[0]):
        for c in range(array.shape[1]):
            if array[r, c] > highest_temp:
                highest_temp = array[r, c]
                if abs(coord[0] - r) > radius_thres and abs(coord[1] - c) >= radius_thres:
                    coord[0], coord[1], coord[2] = r, c, 1
    return coord


def command(coordinates):
    # -1 == left, down
    # 0 == stop
    # 1 == right, up

    coord = coordinates
    global command_list
    command_list = coord[::]

    if coord[0] == 3 or coord[0] == 4:
        command_list[0] = 0
    elif coord[0] < 3:
        command_list[0] = -1
    else:
        command_list[0] = 1

    if coord[1] == 3 or coord[1] == 4:
        command_list[1] = 0
    elif coord[1] < 3:
        command_list[1] = -1
    else:
        command_list[1] = 1


def main(args=None):
    rclpy.init(args=args)
    heat_array_sub = HeatArraySub()
    command_pub = CommandNode()

    rclpy.spin(heat_array_sub)
    rclpy.spin(command_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heat_array_sub.destroy_node()
    command_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
