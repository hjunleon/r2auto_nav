#Created on: 070421
#Author: jasshanK
#Description: Testing firing and loading mechanism of turret

from time import sleep
import RPI.GPIO as GPIO

Loading_PWM= 12 #Loading servo pwm pibn
DCT_1 = 10 #top firing motor pin 2
DCB_1 = 11 #bottom firing motor pin 2
DCT_2 = 9 #top firing motor pin 2
DCB_2 = 8 #bottom firing motor pin 2

GPIO.setup(Loading_PWM, GPIO.OUT)
GPIO.setup(DCT_1, GPIO.OUT)
GPIO.setup(DCB_1, GPIO.OUT)
GPIO.setup(DCT_2, GPIO.OUT)
GPIO.setup(DCB_2, GPIO.OUT)

#checks for f or s
def fire(start):
    if start == 'y':
        GPIO.output(DCT_1, GPIO.HIGH)
        GPIO.output(DCB_1, GPIO.HIGH)
        GPIO.output(DCT_2, GPIO.LOW)
        GPIO.output(DCB_2, GPIO.LOW)
    elif start == 'n':
        GPIO.output(DCT_1, GPIO.LOW)
        GPIO.output(DCB_1, GPIO.LOW)
        GPIO.output(DCT_2, GPIO.LOW)
        GPIO.output(DCB_2, GPIO.LOW)

#check for y or n
def load(release):
    if release == "y":
        loading.ChangeDutyCycle(2.92)
        sleep(1)
        loading.ChangeDutyCycle(7.9)
        sleep(1)

while True:
    start = input("Start motors? y/n/enter to skip: \n")
    release = input("Release ball? y/n: \n")
    fire(start)
    load(release)
