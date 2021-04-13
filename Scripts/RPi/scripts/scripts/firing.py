# Created on: 24/03/21
# Author: jasshanK
# Description: Precise control of firing system using a stepper, servo and 2 dc motors. Data is being sent from a thermal imaging sensor
# references
# https://github.com/binbash12/raspberrypi-stepper/commit/854b3b2670adaf8eb166469dd42b9713b1417157
# https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# https://www.instructables.com/Servo-Motor-Control-With-Raspberry-Pi/

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from time import sleep
import pigpio

# pins based on BCM
DIR = 22  # direction GPIO pin
STEP = 27  # step GPIO pin
STEPPER_EN = 6  # stepper motor enable pin
Tilt_PWM = 18  # servo pwm pin
Loading_PWM = 12  # Loading servo pwm pin
M1 = 26  # firing motors pin
M_PWM = 13  # bottom firing motor pin 2

# setting up pins
pi = pigpio.pi()
pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)
pi.set_mode(STEPPER_EN, pigpio.OUTPUT)
pi.set_mode(Tilt_PWM, pigpio.OUTPUT)
pi.set_mode(Loading_PWM, pigpio.OUTPUT)
pi.set_mode(M1, pigpio.OUTPUT)
pi.set_mode(M_PWM, pigpio.OUTPUT)

# setting up PWM
pi.set_PWM_frequency(M_PWM, 1000)  # motor pwm at 1000 hz
pi.set_servo_pulsewidth(Tilt_PWM, 1650)  # upwards: 1510, flat: 1840
pi.set_servo_pulsewidth(Loading_PWM, 700)  # forward: 700, backwards: 1800
pi.set_PWM_dutycycle(M_PWM, 0)
pi.write(STEPPER_EN, 1)


class FiringSys(Node):
    def __init__(self):
        super().__init__('firing_mechanism')
        # constants to mark completion
        self.servo_done = 0  # servo complete
        self.stepper_done = 0  # stepper complete
        self.dc_done = 0  # firing complete
        self.loading_done = 0
        self.done = 'Searching'  # firing complete AND returned to origin
        self.yaw = 0  # keep track of relative position of top layer
        self.prev_pulse = 1650

        # sub 1 for x axis
        self.sub1 = self.create_subscription(
            Float32MultiArray,  # topic
            'com_node',
            self.callback_x,
            1)
        self.sub1  # prevent unused variable warning

        # sub 2 for y_axis, fire and load
        self.sub2 = self.create_subscription(
            Float32MultiArray,  # topic
            'com_node',
            self.callback_y,
            1)
        self.sub2  # prevent unused variable warning

        # pub for completed actuation
        self.pub = self.create_publisher(
            String,
            'fire_done',
            10)

    def callback_x(self, com_array):
        self.move_x(com_array.data)
        print(f"Complete: {self.dc_done}")
        msg = String()
        msg.data = self.done
        self.pub.publish(msg)
        self.get_logger().info(f'Done: {self.done}')

    def callback_y(self, com_array):
        self.move_y(com_array.data)
        self.fire(com_array.data)
        self.load(com_array.data)

    # com_array = [0,0] #x and y coordinate, negative: left, down, positive: right, up
    def move_x(self, array):
        sleep(1)  # pseudo debouncing

        step_limit = 100  # max number of steps
        teeth_scale = 2
        cw = 0  # clockwise
        ccw = 1  # counter clockwise
        delay = 3000 * (10 ** -6)

        # pi.write(STEPPER_EN, 0)  # start stepper

        if step_limit > self.yaw > -step_limit and self.dc_done == 0:
            if array[0] < 0:
                pi.write(STEPPER_EN, 0)  # start stepper
                pi.write(DIR, cw)
                print("Pan left")
                for i in range(abs(int(array[0] // 1.8)) * teeth_scale):
                    # check if will exceed limit, allows for more precision
                    # if step_limit > self.yaw > -step_limit:
                    print("turning left")
                    pi.write(STEP, 1)
                    sleep(delay)
                    pi.write(STEP, 0)
                    sleep(delay)
                    self.yaw -= 1
                    # else:
                    #     continue
            elif array[0] > 0:
                pi.write(STEPPER_EN, 0)  # start stepper
                pi.write(DIR, ccw)
                print("Pan right")
                for i in range(abs(int(array[0] // 1.8)) * teeth_scale):
                    # check if will exceed limit, allows for more precision
                    # if step_limit > self.yaw > -step_limit:
                    print("turning right")
                    pi.write(STEP, 1)
                    sleep(delay)
                    pi.write(STEP, 0)
                    sleep(delay)
                    self.yaw += 1
                    # else:
                    #     continue
            print(f"Yaw: {self.yaw}")

        # in case yaw exceeds recommended range
        elif -step_limit > self.yaw > step_limit and self.dc_done == 0:
            print("Exceeded pan range, returning to origin")
            while self.yaw:
                # barrel on the right
                if self.yaw > 0:
                    pi.write(DIR, cw)
                    self.yaw -= 1
                # barrel on the left
                elif self.yaw < 0:
                    pi.write(DIR, ccw)
                    self.yaw += 1
                pi.write(STEP, 1)
                sleep(delay)
                pi.write(STEP, 0)
                sleep(delay)
            print(f"Yaw: {self.yaw}")

        # when shooting is complete, move everything back to origin
        if self.dc_done == 1:
            print("Shooting complete, pan returning to origin")
            self.stepper_done = 1
            if self.yaw > 0:
                pi.write(DIR, cw)
                self.yaw -= 1
            elif self.yaw < 0:
                pi.write(DIR, ccw)
                self.yaw += 1
            while self.yaw:
                # pi.write(STEP, 1)
                # sleep(delay)
                # pi.write(STEP, 0)
                # sleep(delay)
                if self.yaw > 0:
                    self.yaw -= 1
                elif self.yaw < 0:
                    self.yaw += 1
                print(f"Yaw @ origin: {self.yaw}")

        # turn off stepper
        if self.yaw == 0 and self.dc_done == 1:
            pi.write(STEPPER_EN, 1)
            self.done = 'Annihilated'

    def move_y(self, array):
        sleep(1)  # pseudo debouncing

        flat_angle = 1850
        max_angle = 1510
        pulse = self.prev_pulse
        if array[1] != 0:
            pulse = flat_angle - abs(10 * (array[1])) # formula to translate angle to pulse width

        if self.dc_done == 0 and self.servo_done == 0:
            pi.set_servo_pulsewidth(Tilt_PWM, int(pulse))
            self.servo_done = 1
            self.prev_pulse = pulse
            sleep(0.5)

        if array[2] == 1:
            self.done = "Detected"

    # power on dc motors when target is sighted, stop powering when target has been shot
    def fire(self, array):
        cw = 0  # clockwise
        ccw = 1  # counter clockwise
        delay = 3000 * (10 ** -6)

        if array[0] == 0 and array[1] == 0 and array[2] == 1 and self.dc_done == 0:  # 1 for target found
            cur_pulse = pi.get_servo_pulsewidth(Tilt_PWM)
            print(f"cur pulse: {cur_pulse}")
            pi.write(M1, 1)
            pi.set_PWM_dutycycle(M_PWM, 125)  # motor on
            # pi.set_servo_pulsewidth(Tilt_PWM, cur_pulse)  # force servo up

            # vibration hotfix
            pi.write(DIR, cw)
            for i in range(2):
                pi.write(STEP, 1)
                sleep(delay)
                pi.write(STEP, 0)
                sleep(delay)
            pi.write(DIR, ccw)
            for i in range(2):
                pi.write(STEP, 1)
                sleep(delay)
                pi.write(STEP, 0)
                sleep(delay)

        if self.loading_done == 1:
            print("Firing complete, motors whining down")
            pi.write(M1, 0)
            pi.set_PWM_dutycycle(M_PWM, 0)  # motor off
            self.dc_done = 1

    # loading of balls using servo motor
    def load(self, array):
        if array[0] == 0 and array[1] == 0 and array[2] == 1 and self.dc_done == 0:
            for i in range(5):
                print("Loading ball")
                pi.set_servo_pulsewidth(Loading_PWM, 1800)
                sleep(0.2)
                pi.set_servo_pulsewidth(Loading_PWM, 700)
                sleep(0.2)
                if i == 4:
                    self.loading_done = 1


def main(args=None):
    print("Actuation initialised")
    rclpy.init(args=args)

    firing_sys = FiringSys()
    rclpy.spin(firing_sys)

    firing_sys.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pi.write(STEPPER_EN, 1)
        pi.stop()
    finally:
        pi.stop()