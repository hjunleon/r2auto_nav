#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 20:12:15 2021

@author: facestomperx
"""

import utils
import trajectory
import numpy as np
import math
import VelocityIterator

from local_planner_limits import LocalPlannerLimits

"""
 * generates trajectories based on equi-distant discretisation of the degrees of freedom.
 * This is supposed to be a simple and robust implementation of the TrajectorySampleGenerator
 * interface, more efficient implementations are thinkable.
 *
 * This can be used for both dwa and trajectory rollout approaches.
 * As an example, assuming these values:
 * sim_time = 1s, sim_period=200ms, dt = 200ms,
 * vsamples_x=5,
 * acc_limit_x = 1m/s^2, vel_x=0 (robot at rest, values just for easy calculations)
 * dwa_planner will sample max-x-velocities from 0m/s to 0.2m/s.
 * trajectory rollout approach will sample max-x-velocities 0m/s up to 1m/s
 * trajectory rollout approach does so respecting the acceleration limit, so it gradually increases velocity
"""


#Personal note, this generator is instantiated everytime new transformed plan comes
#Trajectory generator is x,y and thetha, velocity iterator is either x, y or theta


"""
   * @param pos current robot position
   * @param vel current robot velocity
   * @param limits Current velocity limits
   * @param vsamples: in how many samples to divide the given dimension
   * @param use_acceleration_limits: if true use physical model, else idealized robot model
   * @param additional_samples (deprecated): Additional velocity samples to generate individual trajectories from.
   * @param discretize_by_time if true, the trajectory is split according in chunks of the same duration, else of same length
"""



class SimpleTrajectoryGenearator():
    def __init__(self, *args, **kwargs):
        #assume position and velocity at start is 0 default
        self._pos = kwargs.get("pos",np.zeros(3,dtype=float))
        #if(self._pos != []):
        #    cur_rot = self._pos.orientation
        #    roll, pitch, yaw = utils.euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        #    self._pos = [self._pos.position.x,self._pos.position.y,yaw]
        self._vel = kwargs.get("vel",np.zeros(3,dtype=float))
        #if(self._vel != []):
        #    self._vel = [self._vel.linear.x,self._vel.linear.y,self._vel.angular.z]
        self._limits = kwargs.get("limits",LocalPlannerLimits())
        self._next_sample_index = 0
        self._discretise_by_time = kwargs.get("discretise_by_time",False)
        self._sim_time = kwargs.get("sim_time",0.0)
        self._sim_granularity = kwargs.get("sim_granularity",0.0)
        self._angular_sim_granularity = kwargs.get("angular_sim_granularity",0.0)
        self._use_dwa = kwargs.get("use_dwa",True)
        self._continued_acceleration = not self._use_dwa#not self._use_dwa, True
        self._sim_period = kwargs.get("sim_period",0.125)
        
        self._windowPlanPoints = kwargs.get("windowPlanPoints",5)
        
        self._sample_params = []
        
        vsamples = kwargs.get("vsamples", [10,10,10])
        
        #vsamples = kwargs.get("vsamples","maybe a zero vecotr? Idk how this works yet")
        accel_limits = self._limits.getAccLimits()
        
        max_vel_th = self._limits.getMaxVelTheta()
        min_vel_th = -1.0 * max_vel_th
        min_vel_x = self._limits.getMinVelX()
        #min_vel_y = self._limits.getMinVelY()
        max_vel_x = self._limits.getMaxVelX()
        #max_vel_y = self._limits.getMaxVelY()
         
        #if sampling number is zero in any dimmension, we don't generate samples generically
        
        """
        JL implementation: I moved the intatntiation of the generator to dwa's init. 
        Reason being: turtlebot within most time frames can stop and reach max speed even within 0.25s?
        
        """
        
        if (vsamples[0] * vsamples[1] * vsamples[2] > 0):
            
            max_vel = np.zeros(3,dtype=float)
            min_vel = np.zeros(3, dtype=float)
            if (not self._use_dwa):
                """
                no point overshooting the goal, may break robot behaviour. So 
                limit velocities to those that do not overshoot in the sim_time... oh that's why
                """
                max_vel[0] = min(max_vel_x, self._vel[0] + accel_limits[0] * self._sim_period)
            else:
                """
                    Using DWA, dont need accelearte beyond the first step. Just sample within 
                    velocities within reach in sim_period
                """
                #print(self._vel)
                max_vel[0] = min(max_vel_x, self._vel[0] + accel_limits[0] * self._sim_period)
                max_vel[1] = 0#min(max_vel_y, self._vel[1] + accel_limits[1] * self._sim_period)
                max_vel[2] = min(max_vel_th, self._vel[2] + accel_limits[2] * self._sim_period)
                
                min_vel[0] = max(min_vel_x, self._vel[0] - accel_limits[0] * self._sim_period)
                min_vel[1] = 0#max(min_vel_y, self._vel[1] - accel_limits[1] * self._sim_period)
                min_vel[2] = max(min_vel_th, self._vel[2] - accel_limits[2] * self._sim_period)
            
            """"
            VelocittyIterator
            """
            #print("VelocittyIterator")
            #print("Min_vel: ", min_vel)
            #print("Max_vel: ", max_vel)
            x_it = VelocityIterator.VelocityIterator(min_vel[0], max_vel[0], vsamples[0])
            y_it = VelocityIterator.VelocityIterator(min_vel[1], max_vel[1], vsamples[1])
            th_it = VelocityIterator.VelocityIterator(min_vel[2], max_vel[2], vsamples[2])
            #print("VelocityIterators Ready")
            #https://stackoverflow.com/questions/53824618/appending-a-numpy-array-to-a-list-strange-happenings
            for x in iter(x_it):
                vel_sample = np.zeros(3, dtype=float)
                vel_sample[0] = x
                for y in iter(y_it):
                    vel_sample[1] = y
                    for th in iter(th_it):
                        vel_sample[2] = th
                        #print(vel_sample)
                        self._sample_params.append(vel_sample.copy())
                    th_it.resetIdx()
                y_it.resetIdx()
        print("Number of possible trajectories: ", len(self._sample_params)) #honestly dk why is called sample params
        #print("Current velocity sample index: ", self._next_sample_index)
        #print(self._sample_params)
        return 
    
    
        
    
    def __iter__(self):
        return self
    def __next__(self):
        return self.nextTrajectory()
    
    def updateState(self, *args, **kwargs):
        self._pos = kwargs.get("pos",np.array([]))
        if(self._pos != []):
            cur_rot = self._pos.orientation
            roll, pitch, yaw = utils.euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
            self._pos = [self._pos.position.x,self._pos.position.y,yaw]
        self._vel = kwargs.get("vel",np.array([]))
        if(self._vel != []):
            self._vel = [self._vel.linear.x,self._vel.linear.y,self._vel.angular.z]
            
        self._windowPlanPoints = kwargs.get("windowPlanPoints",5)    
        self.resetIterator()
    
    def resetIterator(self):
        self._next_sample_index = 0
        #print("Current velocity sample index: ", self._next_sample_index)
    
    def hasMoreTrajectories(self):
        #print("Has more trajectories? ", self._next_sample_index,len(self._sample_params))
        return self._next_sample_index < len(self._sample_params)
    
    def nextTrajectory(self):
        result = None
        if self.hasMoreTrajectories():
            #print("self._sample_params[self._next_sample_index]")
            #print(self._sample_params[self._next_sample_index])
            result = self.generateTrajectory(self._pos, self._vel, self._sample_params[self._next_sample_index])
            self._next_sample_index += 1
        else:
            raise StopIteration
        return result
    """
     @param pos current position of robot
     @param vel desired velocity for sampling 
    """
    
    def generateTrajectory(self, pos, vel, sample_target_vel):
        #print("Generating the "+ str(self._next_sample_index) +" trajectory out of " + str(len(self._sample_params)))
        #print(pos)
        #print("sample_target_vel: ", sample_target_vel)
        v_mag = utils.hypotenuseLen(sample_target_vel[0], sample_target_vel[1])
        epsilon = 10**-4
        resTraj = trajectory.Trajectory(cost=-1.0)
        resTraj.resetPoints()
            
        #print(v_mag)
        
        if ((self._limits.getMinVelTrans() >= 0 and v_mag + epsilon < self._limits.getMinVelTrans())and
            (self._limits.getMinVelTheta() >= 0 and math.fabs(sample_target_vel[2]) + epsilon < self._limits.getMinVelTheta())
            ):
            print("Robot not moviing at minimum linear / angular speed")
            return False
        
        if (self._limits.getMaxVelTrans() >= 0 and v_mag - epsilon > self._limits.getMaxVelTrans()):
            print("Exceeded max diagonla (x+y) translational velocity")
            return False
        num_steps = None #to deermine the number of time steps based on how detailed our time info is
        
        #print(self._sim_time)
        #print(self._sim_granularity)
        
        if (self._discretise_by_time):
            num_steps = math.ceil(self._sim_time / self._sim_granularity)
            #print(num_steps)
        else:
            # compute number of steps we must take along this trajectory to be "safe"
            sim_time_distance = v_mag * self._sim_time
            sim_time_angle = math.fabs(sample_target_vel[2]) * self._sim_time
            num_steps = math.ceil(max(sim_time_distance / self._sim_granularity,
                                  sim_time_angle / self._angular_sim_granularity))      
        
        if (num_steps == 0):
            print("num_steps 0")
            return False
        
        #trying out same number of points as window plan, seems to work hahaha By right they should have the same, and controlled by variables
        #maybe navstack samples only some part of the transformed plan
        num_steps = self._windowPlanPoints
        
        #compute a single timestep
        dt = self._sim_time / num_steps
        resTraj._time_delta = dt
        loop_velocity = np.zeros(3)
        
        #print("my dt")
        #print(dt)
        
        #my understanding is, if the robot is moving, then try to converge towards velocity
        #otherwise stationary then just go in that direction
        if (self._continued_acceleration):
            loop_velocity = self.computeNewVelocities(sample_target_vel, vel, self._limits.getAccLimits(), dt)
        else:
            loop_velocity = sample_target_vel
        #print("loop_velocity: ",loop_velocity)
        resTraj._xv = loop_velocity[0]
        resTraj._yv = loop_velocity[1]
        resTraj._thetav = loop_velocity[2]
        #print(resTraj)
        #Now simulate the trajectory and check for collisions, updating costs
        for i in range(num_steps):
            resTraj.addPoint(pos[0],pos[1],pos[2])
            if (self._continued_acceleration):
                loop_velocity = self.computeNewVelocities(sample_target_vel, loop_velocity, self._limits.getAccLimits(), dt)
            
            pos = self.computeNewPositions(pos, loop_velocity, dt)
        #print("loop_velocity: ",loop_velocity)
        return resTraj
    
    def computeNewPositions(self, pos, vel, dt):
        new_pos = np.zeros(3, dtype=float)
        new_pos[0] = pos[0] + (vel[0] * math.cos(pos[2]) + vel[1] * math.cos(utils.M_PI_2 + pos[2])) * dt;
        new_pos[1] = pos[1] + (vel[0] * math.sin(pos[2]) + vel[1] * math.sin(utils.M_PI_2 + pos[2])) * dt;
        new_pos[2] = pos[2] + vel[2] * dt;
        return new_pos;
    """
        Change Vel using acceleration limits to converge towards sample_target_vel
        returns a numpy vector of size 3
        
    """
    def computeNewVelocities(self, sample_target_vel, current_vel, accel_limits, dt):
        #print("computeNewVelocities")
        new_vel = np.zeros(3,dtype=float)
        for i in range(3):
            if (sample_target_vel[i] > current_vel[i]):
                new_vel[i] = min(sample_target_vel[i], (current_vel[i] + accel_limits[i] * dt))
            else:
                new_vel[i] = max(sample_target_vel[i], current_vel[i] + accel_limits[i] * dt)
        return new_vel
        