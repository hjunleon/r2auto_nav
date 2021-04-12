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

# from std_msgs.msg import String
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
pi.set_servo_pulsewidth(Tilt_PWM, 1650)  # upwards: 1650, flat: 1840
pi.set_servo_pulsewidth(Loading_PWM, 700)  # forward: 700, backwards: 1800
pi.set_PWM_dutycycle(M_PWM, 0)
pi.write(STEPPER_EN, 1)


class FiringSys(Node):
    def __init__(self):
        super().__init__('firing_mechanism')
        # constants to mark completion
        self.complete = 0  # firing complete
        self.done = 'not done'  # firing complete AND returned to origin
        self.yaw = 0  # keep track of relative position of top layer

        # sub 1 for x axis
        self.subscription = self.create_subscription(
            Float32MultiArray,  # topic
            'com_node',
            self.callback_x,
            10)
        self.subscription  # prevent unused variable warning

        # sub 2 for y_axis, fire and load
        self.subscription_ = self.create_subscription(
            Float32MultiArray,  # topic
            'com_node',
            self.callback_y,
            10)
        self.subscription_  # prevent unused variable warning

        # pub for completed actuation
        self.publisher_ = self.create_publisher(
            String,
            'fire_done',
            10)

    def callback_x(self, com_array):
        self.move_x(com_array.data)
        print(f"Complete: {self.complete}")
        msg = String()
        msg.data = self.done
        self.publisher_.publish(msg)
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

        pi.write(STEPPER_EN, 0)  # start stepper

        if step_limit > yaw > -step_limit and self.complete == 0:
            if array[0] < 0:
                pi.write(DIR, cw)
                print("Pan left\n")
                for i in range(int(array[0] // 1.8) * teeth_scale):
                    # print("turning left")
                    pi.write(STEP, 1)
                    sleep(delay)
                    pi.write(STEP, 0)
                    sleep(delay)
                    yaw -= 1
            elif array[0] > 0:
                pi.write(DIR, ccw)
                print("Pan right\n")
                for i in range(int(array[0] // 1.8) * teeth_scale):
                    # print("turning right")
                    pi.write(STEP, 1)
                    sleep(delay)
                    pi.write(STEP, 0)
                    sleep(delay)
                    yaw += 1
            print(f"Yaw: {yaw}")

        # in case yaw exceeds recommended range
        elif -step_limit > yaw > step_limit and self.complete == 0:
            print("Exceeded pan range, returning to origin\n")
            while yaw:
                # barrel on the right
                if yaw > 0:
                    pi.write(DIR, cw)
                    yaw -= 1
                # barrel on the left
                elif yaw < 0:
                    pi.write(DIR, ccw)
                    yaw += 1
                pi.write(STEP, 1)
                sleep(delay)
                pi.write(STEP, 0)
                sleep(delay)
            print(f"Yaw: {yaw}")

        # when shooting is complete, move everything back to origin
        if self.complete == 1:
            print("Shooting complete, pan returning to origin\n")
            if yaw > 0:
                pi.write(DIR, cw)
                yaw -= 1
            elif yaw < 0:
                pi.write(DIR, ccw)
                yaw += 1
            while yaw:
                pi.write(STEP, 1)
                sleep(delay)
                pi.write(STEP, 0)
                sleep(delay)
                if yaw > 0:
                    yaw -= 1
                elif yaw < 0:
                    yaw += 1
                print(f"Yaw @ origin: {yaw}")

        # turn off stepper
        if yaw == 0 and self.complete == 1:
            pi.write(STEPPER_EN, 1)
            self.done = 'done'

    def move_y(self, array):
        sleep(1)  # pseudo debouncing

        flat_angle = 1850
        pulse = flat_angle - 10 * (array[1])  # formula to translate angle to pulse width

        if self.complete == 0:
            pi.set_servo_pulsewidth(Tilt_PWM, int(pulse))
            sleep(0.5)

        if self.complete == 1:
            pi.set_servo_pulsewidth(Tilt_PWM, flat_angle)
            sleep(0.5)

    # power on dc motors when target is sighted, stop powering when target has been shot
    def fire(self, array):
        if array[0] == 0 and array[1] == 0 and array[2] == 1 and self.complete == 0:  # 1 for target found
            cur_pulse = pi.get_servo_pulsewidth(Tilt_PWM)
            print(f"cur pulse: {cur_pulse}")
            pi.set_servo_pulsewidth(Tilt_PWM, cur_pulse)
            pi.write(M1, 1)
            pi.set_PWM_dutycycle(M_PWM, 180)  # motor on

        if self.complete == 1:
            print("Firing complete, motors whining down\n")
            pi.write(M1, 0)
            pi.set_PWM_dutycycle(M_PWM, 0)  # motor off

    # loading of balls using servo motor
    def load(self, array):
        # needs to be stopping at ball at neutral(?)<---confirm this with rest
        # servo arm goes back and moves forward in a certain timing range to push balls forward
        if array[0] == 0 and array[1] == 0 and array[2] == 1 and self.complete == 0:
            for i in range(4):
                print("Loading ball\n")
                pi.set_servo_pulsewidth(Loading_PWM, 1800)
                sleep(0.5)
                pi.set_servo_pulsewidth(Loading_PWM, 700)
                sleep(0.5)
                if i == 3:
                    self.complete = 1


def main(args=None):
    print("Actuation initialised\n")
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
        pi.stop()
    finally:
        pi.stop()

### TRASH ###
# if 40 seconds have passed, self.complete  = 1
# def timer(array):
#     if array[2] == 1 and self.complete == 0:
#         start_time = time.time()
#         while (time.time() - start_time) < timer_time:
#             return
#         self.complete = 1
#     return self.complete\

# def move_y(array):
#     global self.complete
#     angle = 97.2  # from 97.2 to 112.5 degrees
#     prev_angle = 97.2
#     duty = angle / 18 + 2.5  # duty cycle
#
#     if array[1] == -1:  # decrease angle by 1
#         if angle > 97:
#             print("Tilt down\n")
#             angle -= 1
#     elif array[1] == 1:  # increase angle by 1
#         if angle < 113:
#             print("Tilt up\n")
#             angle += 1
#
#     pi.write(Tilt_PWM, True)  # turn on pwm pin
#
#     if prev_angle != angle:  # move servo if there is a change in angle
#         tilt.ChangeDutyCycle(duty)
#         sleep(1)
#
#     prev_angle = angle  # check whats the prev angle
#
#     # if self.complete, move back to origin
#     if self.complete == 1:
#         print("Firing self.complete, tilt return to origin\n")
#         tilt.ChangeDutyCycle(7.9)

# move_x(com_array.data)
# move_y(com_array.data)
# fire(com_array.data)
# load(com_array.data)

# class FiringDone(Node):
#     def __init__(self):
#         super().__init__('fire_done')
#         self.publisher_ = self.create_publisher(
#             String,
#             'fire done',
#             10)
#         timer_period = 0.2  # seconds
#         self.timer = self.create_timer(timer_period, self.callback)
#
#     def callback(self):
#         msg = String()
#         msg.data = done
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Done: {done}')

# timer_period = 0.2  # seconds
# self.timer_x = self.create_timer(timer_period, self.callback_x)
# self.timer_y = self.create_timer(timer_period, self.callback_y)

# # global constants
# self.complete = 0  # turns to 1 when firing is self.complete, controlled by loading of balls
# done = 'not done'  # turns to 1 when actuation is reset
