#Created on: 24/03/21
#Author: jasshanK
#Description: Stepper motor test
#references
#https://github.com/binbash12/raspberrypi-stepper/commit/854b3b2670adaf8eb166469dd42b9713b1417157
#https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
#https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

# from std_msgs.msg import String
from time import sleep
import RPi.GPIO as GPIO

#pins
DIR = 22 #direction GPIO pin
STEP = 27 #step GPIO pin
Servo_PWM = 18 #servo pwm pin

#constants
SPR = 200 #steps per revolution, 360/1.8
delay = 0.005 / 32
CW = 1 #clockwise
CCW = 0 #counter clockwise
complete = 0 #turns to 1 when firing is complete, controlled by a timer
yaw = 0 #keep track of relative position of top layer

end = 0

def test():
    GPIO.output(DIR,CCW)
    GPIO.output(STEP, GPIO.HIGH)
    sleep(delay)

try:
    for i in range(5):
        test()
    GPIO.output(STEP, GPIO.LOW)
    sleep(delay)
except KeyboardInterrupt:
    GPIO.cleanup()



# # com_array = [0,0] #x and y coordinate
# def move_x(array):
#     if array[0] == -1:
#         GPIO.output(DIR, CCW) # counter clockwise direction
#         GPIO.output(STEP, GPIO.HIGH)
#         sleep(delay)
#         GPIO.output(STEP, GPIO.LOW)
#         sleep(delay)
#         yaw -= 1
#     elif array[0] == 1:
#         GPIO.output(DIR, CW) #clockwise direction
#         GPIO.output(STEP, GPIO.HIGH)
#         sleep(delay)
#         GPIO.output(STEP, GPIO.LOW)
#         sleep(delay)
#         yaw += 1
#     else:
#         continue #do nothing
#
#     #when shooting is complete, move everything back to origin
#     if complete == 1:
#         if yaw < 0:
#             GPIO.output(DIR, CW)
#         elif yaw > 0:
#             GPIO.output(DIR, CCW)
#         for steps in range(yaw):
#             GPIO.output(STEP, GPIO.HIGH)
#             sleep(delay)
#             GPIO.output(STEP, GPIO.LOW)
#             sleep(delay)
