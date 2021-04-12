from time import sleep
import pigpio as pig

M1 = 26
PWM = 13

motor = pig.pi()
motor.set_mode(M1, pig.OUTPUT)
motor.set_mode(PWM, pig.OUTPUT)

motor.set_PWM_dutycycle(13, 0) # motor off, 0 - 255

def fire(begin):
    if begin == 'y':
        motor.write(M1, 1)
        motor.set_PWM_dutycycle(13, 180) # motor on

    elif begin == 'n':
        motor.write(M1, 0)
        motor.set_PWM_dutycycle(13, 0) # motor off


try:
    while True:
        start = input("On motors: y/n")
        fire(start)
        sleep(1)
except Exception as e:
    print(e)
    motor.write(M1, 0)
    motor.set_PWM_dutycycle(13, 0)
    motor.stop()
finally:
    motor.write(M1, 0)
    motor.set_PWM_dutycycle(13, 0)
    motor.stop()
