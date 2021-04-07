import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray, Float64MultiArray

class CommandNode(Node):

    def __init__(self):
        super().__init__('com_node')
        self.publisher_ = self.create_publisher(Int8MultiArray, 'com_node', 10)
        self.publisher_  # prevent unused variable warning

    def readKey(self):
        command = [0, 0, 1]
        try:
            while True:
                key = str(input('w a s d for movement'))
                if key == 'w':
                    command[1] = 1
                if key == 'a':
                    command[0] = -1
                if key == 's':
                    command[1] = -1
                if key == 'd':
                    command[0] = 1

                self.publisher_.publish(command)
                command = [0, 0, 1]
        except Exception as e:
            print(e)

        finally:
            self.publisher_.publish(command)

def main(args=None):

    rclpy.init(args=args)
    command_node = CommandNode()
    command_node.readKey()

    command_node.destroy_node()

    rclpy.shutdown()

if __name == '__main__':
    main()