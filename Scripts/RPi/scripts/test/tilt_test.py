# Created on: 070421
# Author: jasshanK
# Description: Testing tilt mechanism of turret

import pigpio
from time import sleep


Tilt_PWM = 18
pi = pigpio.pi()
pi.set_mode(Tilt_PWM, pigpio.OUTPUT)

# tilt = GPIO.PWM(Tilt_PWM, 50)  # servo pwm pin at 50hz
# tilt.start(7.9)

# Enable/Disable Exceptions:
pigpio.exceptions = True

print('Starting at: ', pi.set_servo_pulsewidth(Tilt_PWM, 1650))
sleep(0.25)


def rotate(angle):
    # angle = float(angle)
    # pulse_width = 1500 + ((angle - 90) * 10)

    # 1650 to 1850
    if 1650 <= angle <= 1850:
        print("setting to: ", pi.set_servo_pulsewidth(Tilt_PWM, angle))
        sleep(1)
        print("set to: ", pi.get_servo_pulsewidth(Tilt_PWM))
        sleep(1)
    else:
        print('Out of range')


try:
    while True:
        angle = input("Range: 1650 and 1850\n--Input pulse width: ")
        rotate(int(angle))
except Exception as e:
    print(e)
    pi.stop()
finally:
    pi.stop()
