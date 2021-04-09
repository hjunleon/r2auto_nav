from time import sleep
import RPi.GPIO as GPIO

# EN = 13
M1 = 26
# M2 = 19

GPIO.setmode(GPIO.BCM)
GPIO.setup(M1, GPIO.OUT)
# GPIO.setup(M2, GPIO.OUT)
# GPIO.setup(VDD, GPIO.OUT)
# GPIO.output(VDD, GPIO.HIGH)


def fire(start):
    if start == 'y':
        GPIO.output(M1, GPIO.HIGH)
        # GPIO.output(M2, GPIO.HIGH)

    elif start == 'n':
        GPIO.output(M1, GPIO.LOW)
        # GPIO.output(M2, GPIO.LOW)


try:
    while True:
        # GPIO.output(VDD, GPIO.HIGH)
        start = input("On motors: y/n")
        fire(start)
        sleep(1)
except Exception as e:
    print(e)
    GPIO.output(M1, GPIO.LOW)
    # GPIO.output(M2, GPIO.LOW)
    GPIO.cleanup()
finally:
    GPIO.output(M1, GPIO.LOW)
    # GPIO.output(M2, GPIO.LOW)
    GPIO.cleanup()
