#Created on: 070421
#Author: jasshanK
#Description: Testing firing and loading mechanism of turret

from time import sleep
import RPi.GPIO as GPIO

Loading_PWM= 12 #Loading servo pwm pibn

GPIO.setmode(GPIO.BCM)
GPIO.setup(Loading_PWM, GPIO.OUT)

loading = GPIO.PWM(Loading_PWM, 50) #firing motors pwm pin at 1000 hz
loading.start(8.75)

#check for y or n
def load(release):
    if release == "y":
        GPIO.output(Loading_PWM, True)
        loading.ChangeDutyCycle(2.92)
        sleep(1)
        loading.ChangeDutyCycle(7.9)
        sleep(1)
        GPIO.output(Loading_PWM, True)
try:
    while True:
        release = input("Release ball? y/n: \n")
        load(release)
except Exception as e:
    print(e)
    GPIO.cleanup()
finally:
    GPIO.cleanup()