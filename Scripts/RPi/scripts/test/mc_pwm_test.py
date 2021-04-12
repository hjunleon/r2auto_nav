from time import sleep
import RPi.GPIO as GPIO

M1 = 26
PWM = 13

GPIO.setmode(GPIO.BCM)
GPIO.setup(M1, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

speed = GPIO.PWM(PWM, 1000) #motor pwm at 1000 hz
speed.start(0)


def fire(start):
    if start == 'y':
        GPIO.output(M1, GPIO.HIGH)
        speed.ChangeDutyCycle(50)

    elif start == 'n':
        GPIO.output(M1, GPIO.LOW)
        speed.ChangeDutyCycle(0)


try:
    while True:
        start = input("On motors: y/n")
        fire(start)
        sleep(1)
except Exception as e:
    print(e)
    GPIO.output(M1, GPIO.LOW)
    GPIO.cleanup()
finally:
    GPIO.output(M1, GPIO.LOW)
    GPIO.cleanup()
