#Created on: 070421
#Author: jasshanK
#Description: Testing tilt mechanism of turret

import RPi.GPIO as GPIO

Tilt_PWM = 18 #servo pwm pin

GPIO.setmode(GPIO.BCM)
GPIO.setup(Tilt_PWM, GPIO.OUT)

tilt = GPIO.PWM(Tilt_PWM, 50) #servo pwm pin at 50hz
tilt.start(7.9)


def tilt(angle):
    angle = float(angle)
    duty = angle / 18 + 2.5

    if angle > 97 and angle < 113:
        tilt.ChangeDutyCycle(duty)
        sleep(1)
    else:
        print("Angle not withing range, please try again\n")

while True:
    angle = input("Input angle between 98 and 112: \n")
    tilt(angle)
