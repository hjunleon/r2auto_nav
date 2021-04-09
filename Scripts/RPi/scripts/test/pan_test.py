# Created on: 070421
# Author: jasshanK
# Description: Testing pan mechanism of turret

from time import sleep
import RPi.GPIO as GPIO

EN = 17
DIR = 22  # direction GPIO pin
STEP = 27  # step GPIO pin

step = 200
delay = 3000 * (10 ** -6)
CW = 1  # clockwise
CCW = 0  # counter clockwise
yaw = 0  # keep track of relative position of top layer

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.setup(EN, GPIO.OUT)
GPIO.output(EN, GPIO.LOW)

def pan(direction):
    # direction of pan
    if direction.lower() == "l":
        GPIO.output(DIR, CW)
    elif direction.lower() == "r":
        GPIO.output(DIR, CCW)

    # stepper moves one rotation
    for i in range(step - 1):
        GPIO.output(STEP, GPIO.HIGH)
        sleep(delay)
        GPIO.output(STEP, GPIO.LOW)
        sleep(delay)


try:
    while True:
        direction = input("Please input direction, l or r. Press enter to continue in previous direction: \n")
        pan(direction)
        sleep(delay)
except Exception as e:
    print(e)
    GPIO.cleanup()

finally:
    GPIO.cleanup()

