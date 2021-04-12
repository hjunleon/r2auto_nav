# Created on: 070421
# Author: jasshanK
# Description: Testing firing and loading mechanism of turret

from time import sleep
import pigpio

Loading_PWM = 12  # Loading servo pwm pibn

pi = pigpio.pi()
pi.set_mode(Loading_PWM, pigpio.OUTPUT)

pi.set_servo_pulsewidth(Loading_PWM, 700)  # firing motors pwm pin at 1000 hz


# check for y or n
def load(release):
    if release == "y":
        pi.set_servo_pulsewidth(Loading_PWM, 1800)
        sleep(0.5)
        pi.set_servo_pulsewidth(Loading_PWM, 700)
        sleep(0.5)


try:
    while True:
        release = input("Release ball? y/n: \n")
        load(release)
except Exception as e:
    print(e)
    pi.stop()
finally:
    pi.stop()
