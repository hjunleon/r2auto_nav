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
Servo_PWM = 18 #servo pwm pin
DC_PWM= 12 #top and bottom firing pwm pins
DCT_2 = 9 #top firing motor pin 2
DCB_2 = 8 #bottom firing motor pin 2

#constants
SPR = 200 #steps per revolution, 360/1.8
delay = 0.005 / 32
CW = 1 #clockwise
CCW = 0 #counter clockwise

#setting up pins
GPIO.setmode(BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(Servo_PWM, GPIO.OUT)
GPIO.setup(DC_PWM, GPIO.OUT)
GPIO.setup(DCT_2, GPIO.OUT)
GPIO.setup(DCB_2, GPIO.OUT)

#max rpi pwm frequency is 8khz
servo = GPIO.PWM(Servo_PWM, 50) #servo pwm pin at 50hz
shoot_motors = GPIO.PWM(DC_PWM, 1000) #firing motors pwm pin at 1000 hz
servo.start(0)

# com_array = [0,0] #x and y coordinate
def move_x(array):
    if array[0] == -1:
        GPIO.output(DIR, CCW) # counter clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    elif array[0] == 1:
        GPIO.output(DIR, CW) #clockwise direction
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)
    else:
        continue #do nothing

def move_y(array):
    duty = angle / 18 + 2 #duty cycle

    if array[1] == -1:  #decrease angle by 1
        if angle != 0:
            angle = angle - 1
    elif array[1] == 1: #increase angle by 1
        if angle != 180:
            angle = angle + 1
    else:
        continue #do nothing

    GPIO.output(Servo_PWM, True) #turn on pwm pin

    if prev_angle != angle: #move servo if there is a change in angle
        servo.ChangeDutyCycle(duty)
        sleep(1)

    prev_angle = angle #check whats the prev angle

def fire(array):
    if array[2] == 1: #1 for target found
        GPIO.output(DCT_2, LOW)
        GPIO.output(DCB_2, LOW)
    return


class Firing_Sys(Node):

    def __init__(self):
        super().__init__('firing_mechanism')
        self.subscription = self.create_subscription(
            Int8MultiArray, #need to setup topic
            'com_array',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, com_array):
        #while true for now, change to while balls have not been shot
        move_x(com_array.data)
        move_y(com_array.data)
        fire(com_array.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)



    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
