import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

# from std_msgs.msg import String
from time import sleep
import RPI.GPIO as GPIO

#pins
DCT_1 = 10 #top firing motor pin 2
DCB_1 = 11 #bottom firing motor pin 2
DCT_2 = 9 #top firing motor pin 2
DCB_2 = 8 #bottom firing motor pin 2

#constants
SPR = 200 #steps per revolution, 360/1.8
delay = 0.005 / 32 #156 microseconds
CW = 1 #clockwise
CCW = 0 #counter clockwise
complete = 0 #turns to 1 when firing is complete, controlled by a timer
yaw = 0 #keep track of relative position of top layer

#setting up pins
GPIO.setmode(BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(DCT_1, GPIO.OUT)
GPIO.setup(DCB_1, GPIO.OUT)
GPIO.setup(DCT_2, GPIO.OUT)
GPIO.setup(DCB_2, GPIO.OUT)

#power on dc motors when target is sighted, stop powering when target has been shot
def fire(array):
    speed = 100 #0 - 100
    if array[2] == 1 and complete == 0: #1 for target found
        GPIO.output(DCT_1, HIGH)
        GPIO.output(DCB_1, HIGH)
        GPIO.output(DCT_2, LOW)
        GPIO.output(DCB_2, LOW)

    if complete == 1:
        shoot_motors.ChangeDutyCycle(0)
        GPIO.output(DCT_2, LOW)
        GPIO.output(DCB_2, LOW)

# com_array = [0,0] #scripts and y coordinate
def move_x(array):
    if array[0] == -1:
        GPIO.output(DIR, CCW) # counter clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
        yaw -= 1
    elif array[0] == 1:
        GPIO.output(DIR, CW) #clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
        yaw += 1
    else:
        continue #do nothing

    #when shooting is complete, move everything back to origin
    if complete == 1:
        if yaw < 0:
            GPIO.output(DIR, CW)
        elif yaw > 0:
            GPIO.output(DIR, CCW)
        for steps in range(yaw):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)

class Firing_Sys(Node):

    def __init__(self):
        super().__init__('firing_mechanism')
        self.subscription = self.create_subscription(
            Int8MultiArray, #need to setup topic
            'com_node',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, com_array):
        fire(com_array.data)
        move_x(com_array.data)
        move_y(com_array.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
