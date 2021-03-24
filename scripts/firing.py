#references
#https://github.com/binbash12/raspberrypi-stepper/commit/854b3b2670adaf8eb166469dd42b9713b1417157
#https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

#to add
#GPIO.cleanup, pwm.stop(), to do this, need to check if the balls have been fired
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

# from std_msgs.msg import String
from time import sleep
import RPI.GPIO as GPIO

#pins
DIR = 22 #direction GPIO pin
STEP = 27 #step GPIO pin
PWM = 18 #servo pwm pin

#constants
SPR = 200 #steps per revolution, 360/1.8
delay = 0.005 / 32
CW = 1 #clockwise
CCW = 0 #counter clockwise

#setting up pins
GPIO.setmode(BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

pwm = GPIO.PWM(PWM, 50) #pwm pin at 50hz
pwm.start(0)

heat_array = [0,0] #x and y coordinate
def move_x():
    if heat_array[0] == -1:
        GPIO.output(DIR, CCW) # counter clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    elif heat_array[0] == 1:
        GPIO.output(DIR, CW) #clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    else:
        continue #do nothing

def move_y():
    duty = angle / 18 + 2 #duty cycle

    if heat_array[1] == -1:  #decrease angle by 1
        if angle != 0:
            angle = angel - 1
    elif heat_array[1] == 1: #increase angle by 1
        if angle != 180:
            angle = angle + 1
    else:
        continue #do nothing

    GPIO.output(PWM, True) #turn on pwm pin

    if prev_angle != angle: #move servo if there is a change in angle
        pwm.ChangeDutyCycle(duty)
        sleep(1)

    prev_angle = angle #check whats the prev angle


class Firing_Sys(Node):

    def __init__(self):
        super().__init__('firing_mechanism')
        self.subscription = self.create_subscription(
            Int8MultiArray, #need to setup topic
            'com_array',
            10)
        self.subscription  # prevent unused variable warning

    def callback(self):



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    while(True): #while true for now, change to while balls have not been shot
        move_x()
        move_y()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
