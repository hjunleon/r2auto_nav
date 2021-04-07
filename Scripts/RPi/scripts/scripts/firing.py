#Created on: 24/03/21
#Author: jasshanK
#Description: Precise control of firing system using a stepper, servo and 2 dc motors. Data is being sent from a thermal imaging sensor
#references
#https://github.com/binbash12/raspberrypi-stepper/commit/854b3b2670adaf8eb166469dd42b9713b1417157
#https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

#to add
#GPIO.cleanup, pwm.stop(), to do this, need to check if the balls have been fired
#create timer to tell when shooting has stopped from first retrieval of target detection
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import time

# from std_msgs.msg import String
from time import sleep
import RPi.GPIO as GPIO

#pins based on BCM
DIR = 22 #direction GPIO pin
STEP = 27 #step GPIO pin
Tilt_PWM = 18 #servo pwm pin
Loading_PWM= 12 #Loading servo pwm pibn
DCT_1 = 10 #top firing motor pin 2
DCB_1 = 11 #bottom firing motor pin 2
DCT_2 = 9 #top firing motor pin 2
DCB_2 = 8 #bottom firing motor pin 2

#constants
SPR = 200 #steps per revolution, 360/1.8
delay = 0.005 / 32
CW = 1 #clockwise
CCW = 0 #counter clockwise
complete = 0 #turns to 1 when firing is complete, controlled by a timer
yaw = 0 #keep track of relative position of top layer
angle = 97.2 #from 97.2 to 112.5 degrees
prev_angle = 97.2
timer_time = 40


#setting up pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(Tilt_PWM, GPIO.OUT)
GPIO.setup(Loading_PWM, GPIO.OUT)
GPIO.setup(DCT_1, GPIO.OUT)
GPIO.setup(DCB_1, GPIO.OUT)
GPIO.setup(DCT_2, GPIO.OUT)
GPIO.setup(DCB_2, GPIO.OUT)

#max rpi pwm frequency is 8khz
tilt = GPIO.PWM(Tilt_PWM, 50) #servo pwm pin at 50hz
loading = GPIO.PWM(Loading_PWM, 50) #firing motors pwm pin at 1000 hz
tilt.start(7.9)
loading.start(8.75)

# com_array = [0,0] #x and y coordinate
def move_x(array):
    if yaw < 200:
        if array[0] == -1:
            GPIO.output(DIR, CW) # counter clockwise direction
            print("Pan left\n")
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)
            yaw -= 1
        elif array[0] == 1:
            GPIO.output(DIR, CCW) #clockwise direction
            print("Pan right\n")
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)
            yaw += 1
    else:
        print("Exceeded pan range, returning to origin\n")
        if yaw < 0:
            GPIO.output(DIR, CCW)
        elif yaw > 0:
            GPIO.output(DIR, CW)
        for steps in range(yaw):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)

    #when shooting is complete, move everything back to origin
    if complete == 1:
        print("Shooting complete, pan returning to origin\n")
        if yaw < 0:
            GPIO.output(DIR, CCW)
        elif yaw > 0:
            GPIO.output(DIR, CW)
        for steps in range(yaw):
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)


def move_y(array):
    duty = angle / 18 + 2.5 #duty cycle

    if array[1] == -1:  #decrease angle by 1
        if angle > 97:
            print("Tilt down\n")
            angle -= 1
    elif array[1] == 1: #increase angle by 1
        if angle < 113:
            print("Tilt up\n")
            angle += 1

    GPIO.output(Tilt_PWM, True) #turn on pwm pin

    if prev_angle != angle: #move servo if there is a change in angle
        servo.ChangeDutyCycle(duty)
        sleep(1)

    prev_angle = angle #check whats the prev angle

    #if complete, move back to origin
    if complete == 1:
        print("Firing complete, tilt return to origin\n")
        servo.ChangeDutyCycle(7.9)

#power on dc motors when target is sighted, stop powering when target has been shot
def fire(array):
    if array[2] == 1 and complete == 0: #1 for target found
        GPIO.output(DCT_1, GPIO.HIGH)
        GPIO.output(DCB_1, GPIO.HIGH)
        GPIO.output(DCT_2, GPIO.LOW)
        GPIO.output(DCB_2, GPIO.LOW)

    if complete == 1:
        print("Firing complete, motors whining down\n")
        GPIO.output(DCT_1, GPIO.LOW)
        GPIO.output(DCB_1, GPIO.LOW)
        GPIO.output(DCT_2, GPIO.LOW)
        GPIO.output(DCB_2, GPIO.LOW)

#if 40 seconds have passed, complete  = 1
def timer(array):
    if array[2] == 1 and complete == 0:
        start_time = time.time()
        while (time.time() - start_time) < timer_time:
            return
        complete = 1
    return complete

#loading of balls using servo motor
def load(com_array):
    #needs to be stopping at ball at neutral(?)<---confirm this with rest
    #servo arm goes back and moves forward in a certain timing range to push balls forward
    if array[0] == 0 and array[1] == 0:
        for i in range(3):
            print("Loading ball\n")
            loading.ChangeDutyCycle(2.92)
            sleep(1)
            loading.ChangeDutyCycle(7.9)
            sleep(1)

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
        move_x(com_array.data)
        move_y(com_array.data)
        fire(com_array.data)
        load(com_array.data)
        timer(com_array.data)

def main(args=None):
    rclpy.init(args=args)

    firing_sys = Firing_Sys()

    rclpy.spin(firing_sys)

    firing_sys.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
