import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import keyboard


class CommandNode(Node):

    def __init__(self):
        super().__init__('com_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'com_node', 10)
        self.array = Int8MultiArray()

    def readKey(self):
        command = [0.0, 0.0, 1.0]
        try:
            while True:
                print('W A S D for movement')
                key = keyboard.read_key()
                if key == 'w':
                    command[1] = -10.0
                if key == 'a':
                    command[0] = -10.0
                if key == 's':
                    command[1] = 10.0
                if key == 'd':
                    command[0] = 10.0

                self.array.data = command
                self.publisher_.publish(self.array)
                command = [0.0, 0.0, 1.0]
        except Exception as e:
            print(e)

        finally:
            self.publisher_.publish(self.array)


def main(args=None):
    rclpy.init(args=args)
    command_node = CommandNode()
    command_node.readKey()

    command_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
