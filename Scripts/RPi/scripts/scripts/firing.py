# Created on: 24/03/21
# Author: jasshanK
# Description: Precise control of firing system using a stepper, servo and 2 dc motors. Data is being sent from a thermal imaging sensor
# references
# https://github.com/binbash12/raspberrypi-stepper/commit/854b3b2670adaf8eb166469dd42b9713b1417157
# https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# to add
# GPIO.cleanup, pwm.stop(), to do this, need to check if the balls have been fired
# create timer to tell when shooting has stopped from first retrieval of target detection
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

# from std_msgs.msg import String
from time import sleep
import RPi.GPIO as GPIO

# turn off warnings
GPIO.setwarnings(False)

# pins based on BCM
DIR = 22  # direction GPIO pin
STEP = 27  # step GPIO pin
STEPPER_EN = 6 # stepper motor enable pin
Tilt_PWM = 18  # servo pwm pin
Loading_PWM = 12  # Loading servo pwm pin
M1 = 26  # firing motors pin
M_PWM = 13  # bottom firing motor pin 2

# constants
global complete
complete = 0  # turns to 1 when firing is complete, controlled by a timer


# setting up pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(STEPPER_EN, GPIO.OUT)
GPIO.setup(Tilt_PWM, GPIO.OUT)
GPIO.setup(Loading_PWM, GPIO.OUT)
GPIO.setup(M1, GPIO.OUT)
GPIO.setup(M_PWM, GPIO.OUT)

# max rpi pwm frequency is 8khz
tilt = GPIO.PWM(Tilt_PWM, 50)  # servo pwm pin at 50hz
loading = GPIO.PWM(Loading_PWM, 50)  # firing motors pwm pin at 1000 hz
motor = GPIO.PWM(M_PWM, 1000)  # motor pwm at 1000 hz
tilt.start(7.9)
loading.start(8.75)
motor.start(0)


# com_array = [0,0] #x and y coordinate
def move_x(array):
    yaw = 0  # keep track of relative position of top layer\
    CW = 0  # clockwise
    CCW = 1  # counter clockwise
    delay = 0.005 / 32
    GPIO.output(STEPPER_EN, GPIO.LOW)
    if yaw < 200:
        if array[0] == -1:
            GPIO.output(DIR, CW)
            print("Pan left\n")
            for i in range(2):
                GPIO.output(STEP, GPIO.HIGH)
                sleep(delay)
                GPIO.output(STEP, GPIO.LOW)
                sleep(delay)
                yaw -= 1
        elif array[0] == 1:
            GPIO.output(DIR, CCW)
            print("Pan right\n")
            for i in range(2):
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
        while yaw != 0:
            GPIO.output(STEP, GPIO.HIGH)
            sleep(delay)
            GPIO.output(STEP, GPIO.LOW)
            sleep(delay)
            yaw -= 1

    #turn off stepper
    if yaw == 0 and complete == 1:
        GPIO.output(STEPPER_EN, GPIO.HIGH)

    # when shooting is complete, move everything back to origin
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
    angle = 97.2  # from 97.2 to 112.5 degrees
    prev_angle = 97.2
    duty = angle / 18 + 2.5  # duty cycle

    if array[1] == -1:  # decrease angle by 1
        if angle > 97:
            print("Tilt down\n")
            angle -= 1
    elif array[1] == 1:  # increase angle by 1
        if angle < 113:
            print("Tilt up\n")
            angle += 1

    GPIO.output(Tilt_PWM, True)  # turn on pwm pin

    if prev_angle != angle:  # move servo if there is a change in angle
        tilt.ChangeDutyCycle(duty)
        sleep(1)

    prev_angle = angle  # check whats the prev angle

    # if complete, move back to origin
    if complete == 1:
        print("Firing complete, tilt return to origin\n")
        tilt.ChangeDutyCycle(7.9)


# power on dc motors when target is sighted, stop powering when target has been shot
def fire(array):
    if array[0] == 0 and array[1] == 0 and complete == 0:  # 1 for target found
        GPIO.output(M1, GPIO.HIGH)
        motor.ChangeDutyCycle(30)

    if complete == 1:
        print("Firing complete, motors whining down\n")
        GPIO.output(M1, GPIO.LOW)
        motor.ChangeDutyCycle(0)


# if 40 seconds have passed, complete  = 1
# def timer(array):
#     if array[2] == 1 and complete == 0:
#         start_time = time.time()
#         while (time.time() - start_time) < timer_time:
#             return
#         complete = 1
#     return complete


# loading of balls using servo motor
def load(array):
    # needs to be stopping at ball at neutral(?)<---confirm this with rest
    # servo arm goes back and moves forward in a certain timing range to push balls forward
    if array[0] == 0 and array[1] == 0:
        GPIO.output(Loading_PWM, True)
        for i in range(4):
            print("Loading ball\n")
            loading.ChangeDutyCycle(2.8)
            sleep(1)
            loading.ChangeDutyCycle(7.9)
            sleep(1)
            if i == 3:
                complete = 1
                GPIO.output(Loading_PWM, False)


class Firing_Sys(Node):
    def __init__(self):
        super().__init__('firing_mechanism')
        self.subscription = self.create_subscription(
            Int8MultiArray,  # need to setup topic
            'com_node',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

    def callback(self, com_array):
        move_x(com_array.data)
        move_y(com_array.data)
        fire(com_array.data)
        load(com_array.data)


def main(args=None):
    print("Actuation initialised\n")
    rclpy.init(args=args)

    firing_sys = Firing_Sys()

    rclpy.spin(firing_sys)

    firing_sys.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        GPIO.cleanup()
        tilt.stop()
        loading.stop()
        motor.stop()
