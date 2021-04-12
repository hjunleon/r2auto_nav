#Created on: 070421
#Author: jasshanK
#Description: Testing tilt mechanism of turret

import RPi.GPIO as GPIO
from time import sleep

Tilt_PWM = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(Tilt_PWM, GPIO.OUT)

tilt = GPIO.PWM(Tilt_PWM, 50) #servo pwm pin at 50hz
tilt.start(7.9)


def rotate(angle):
    angle = float(angle)
    duty = angle / 18 + 2.5

    if 97 < angle < 113:
        tilt.ChangeDutyCycle(duty)
        sleep(1)
    else:
        print("Angle not withing range, please try again\n")

try:
    while True:
        angle = input("Input angle between 98 and 112: \n")
        GPIO.output(Tilt_PWM, True)
        rotate(angle)
except Exception as e:
    print(e)
    GPIO.cleanup()
finally:
    GPIO.cleanup()
