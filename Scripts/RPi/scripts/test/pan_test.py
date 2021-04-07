#Created on: 070421
#Author: jasshanK
#Description: Testing pan mechanism of turret

from time import sleep
import RPi.GPIO as GPIO

DIR = 22 #direction GPIO pin
STEP = 27 #step GPIO pin

delay = 0.005 / 32
CW = 1 #clockwise
CCW = 0 #counter clockwise
yaw = 0 #keep track of relative position of top layer

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)

def pan(direction):
    #direction of pan
    if direction.lower() == "l":
        GPIO.output(DIR, CW)
    elif direction.lower() == "r":
        GPIO.output(DIR, CCW)

    #stepper moves one rotation
    for i in range(step - 1):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)

while True:
    direction = input("Please input direction, l or r. Press enter to continue in previous direction: \n")
    pan(direction)
