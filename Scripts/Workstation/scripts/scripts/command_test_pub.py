import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CommandNode(Node):

    def __init__(self):
        super().__init__('com_node')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'com_node', 10)
        self.array = Float32MultiArray()

    def readKey(self):
        command = [20.0, 10.0, 1.0]
        try:
            while True:
                _ = input('Test: [20.0, 10.0, 1.0]\nEnter to continue...')
                self.array.data = command
                self.publisher_.publish(self.array)
        except Exception as e:
            print(e)

        finally:
            self.publisher_.publish(self.array)


def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()

    command_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
