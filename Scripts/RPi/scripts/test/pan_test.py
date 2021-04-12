# Created on: 070421
# Author: jasshanK
# Description: Testing pan mechanism of turret

from time import sleep
import pigpio

EN = 6
DIR = 22  # direction GPIO pin
STEP = 27  # step GPIO pin

step = 100
delay = 3000 * (10 ** -6)
CW = 1  # clockwise
CCW = 0  # counter clockwise
yaw = 0  # keep track of relative position of top layer

pi = pigpio.pi()
pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.set_mode(EN, pigpio.OUTPUT)


def pan(direction):
    # direction of pan
    if direction.lower() == "l":
        pi.write(DIR, CW)
    elif direction.lower() == "r":
        pi.write(DIR, CCW)

    pi.write(EN, 0)

    # stepper moves one rotation
    for i in range(step - 1):
        pi.write(STEP, 1)
        sleep(delay)
        pi.write(STEP, 0)
        sleep(delay)

    pi.write(EN, 1)


try:
    while True:
        direction = input("Please input direction, l or r. Press enter to continue in previous direction: \n")
        pan(direction)
        sleep(delay)
except Exception as e:
    print(e)
    pi.stop()

finally:
    pi.write(EN, 1)
    pi.stop()

