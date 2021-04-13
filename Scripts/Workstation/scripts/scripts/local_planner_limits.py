#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 22:52:38 2021

@author: facestomperx
"""
class LocalPlannerLimits():
    #by right should just parse a yaml file but nahhhhhhhh
    def __init__(self, *args, **kwargs):
        self.max_vel_x = 0.16 #0.22 seems too fast after all
        self.max_vel_y = 0.0
        self.min_vel_x = -0.16  #-self.max_vel_x#0.0 the bot can reverse
        self.min_vel_y = 0.0
        self.max_vel_theta = 1.0 #0.75 / 2
        self.min_vel_theta = -1.0 #/ 2 #Given how velocity iterator runs, it's best to use this
        self.max_speed_xy = 0.16  #0.22
        self.min_x_velocity_threshold = 0.001
        self.acc_lim_x = 2.5
        self.acc_lim_y = 0.0
        self.acc_lim_theta = 3.2 #/ 2 3.2
        self.decel_lim_x = -self.acc_lim_x
        self.decel_lim_y = -self.acc_lim_y
        self.decel_lim_theta = -self.acc_lim_theta
        self.sim_time = 1.7 #not sure what this is
        self.vx_samples = 20
        self.vy_samples = 5
        self.vtheta_samples = 20
        self.xy_gooal_tolerance = 0.2
        self.transform_tolerance = 0.2 #doubt will use this
        self.robot_length = 0.21
        self.robot_width = 0.16
        self.min_vel_trans = -1.0 #this is assuming can move diagonal. Srsly navstack people need to be clearer
        self.max_vel_trans = -1.0#self.max_speed_xy
        #self.min_vel_
        
    
    def getRobotDimmensions(self):
        return self.robot_length, self.robot_width
    
    def getAccLimits(self):
        #print("Getting acceleration limits")
        return [self.acc_lim_x, self.acc_lim_y, self.acc_lim_theta]
    
    def getMaxVelTheta(self):
        return self.max_vel_theta
    
    def getMinVelTheta(self):
        return self.min_vel_theta
    
    def getMaxVelX(self):
        return self.max_vel_x
    
    def getMaxVelY(self):
        return self.max_vel_y
    
    def getMinVelX(self):
        return self.min_vel_x
    
    def getMinVelY(self):
        return self.min_vel_y
    
    
    def getMinVelTrans(self):
        return self.min_vel_trans
    
    def getMaxVelTrans(self):
        return self.max_vel_trans