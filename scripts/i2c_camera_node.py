#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from smbus2 import SMBus
import amg8833_i2c
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import time

# RPi Channel 1
channel = 1
# AMG8833 address and registers
address = 0x48
reg_config = 0x01
reg_conversion = 0x00

bus = SMBus(channel)

# Config value:
# - Single conversion
# - A0 input
# - 4.096V reference
config = [0x83, 0xC3]

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=0)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        #i2c stuffs
        bus.write_i2c_data(address, reg_config, config)

