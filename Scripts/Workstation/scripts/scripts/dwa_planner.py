#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 10:57:03 2021

@author: facestomperx
"""
#http://wiki.ros.org/tf x/y/z offset in meters and yaw/pitch/roll in radians
#The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value. 


from trajectoryScorer import trajectoryScorer
from simplete_trajectory_generator import SimpleTrajectoryGenearator
import utils
from geometry_msgs.msg import Pose,Twist
from local_planner_limits import LocalPlannerLimits

from ObstacleCostFunction import ObstacleCostFunction
from PathDistCostFunction import PathDistCostFunction
from GoalDistCostFunction import GoalDistCostFunction

import math
from GlobalPlan import GlobalPlan,TransformedPlan

import numpy as np
import time

NUM_OF_SAMPLE_TRAJECTORIES = 8 # 10,15,8 

#https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/dwa_local_planner/cfg/DWAPlanner.cfg
class DWAPlanner():
    def __init__(self, *args, **kwargs):
        self.robot_limits = kwargs.get("limits", LocalPlannerLimits()) #I hardcoded the turtlebot's params
        
        #instantiate the cost functions
        self.obstacleCost = ObstacleCostFunction(scale=1.0)#ObstacleCostFunction(scaling_speed = 1.0)
        self.pathDist = PathDistCostFunction(scale=1.0)#PathDistCostFunction(scaling_speed = 1.0)
        self.goalDist = GoalDistCostFunction(scale=1.0)#GoalDistCostFunction(scaling_speed = 1.0)
        listOfCostFuncs = [
            self.obstacleCost,
            self.pathDist,
            self.goalDist
        ]
        
        self._cost_funcs = kwargs.get("cost_funcs",listOfCostFuncs)
        self._trajectoryScorers = trajectoryScorer(self._cost_funcs, NUM_OF_SAMPLE_TRAJECTORIES ** 3)
        self.xy_goal_tolerance = 0.08 * 1.5#0.21 * 1.25 #* 1.25#0.10fatter_edges
        self.recovery_method = kwargs.get("recovery_method","reverse_current_trajectory")
        self.currentCmdVelocity = None
        
        self.trajStack = []
        
        #print("Instantiating trajectory generator")
        trajGen = SimpleTrajectoryGenearator(
            sim_time = 1.7, #derive this value from the average tf callback interval EDIT: 1.0 too slow. Acutally may bbe too fast cuz my given architecture works such that it favours equidistant points. So it favours faster moving points
            sim_period = 0.125, #sim_period is the controller frequency
            sim_granularity = 0.5,#1.0,#0.5,#0.025,0.123
            angular_sim_granularity = 0.5,#1.0,#0.5,#0.1,
            discretise_by_time = True,
            vsamples = [NUM_OF_SAMPLE_TRAJECTORIES,1,NUM_OF_SAMPLE_TRAJECTORIES] #NUM_OF_SAMPLE_TRAJECTORIES * 2
        )
        if (trajGen == None):
            print("Traj gen not created properly")
        #else:
        #    print("trajGen is generated")
        
        self.trajGenerator = trajGen
        
        self.lidarRange = None
        
    def updateCostmap(self, costmap2D):
        self._costmap = costmap2D
    """
        Given the current state of the robot, find the best trajectory and drive commands
    """
    def findBestPath(self, robot_pose, robot_vel, global_plan):
      #  print("FINDING BEST PATH")
     #   print("TRANSFORMED PLAN: ")
        #print(len(global_plan.getAllPoses()))
        if (len(global_plan.getAllPoses()) == 0):
            return None,"Error: transformed plan is empty"
        self.pathDist.updateGlobalPlan(global_plan)
        self.goalDist.updateGlobalPlan(global_plan)
        #print(self._costmap)
        self.obstacleCost.updateCostMap(self._costmap)
        
        
        
        
        self.trajGenerator.updateState(
            pos = robot_pose,
            vel = robot_vel,
            windowPlanPoints = len(global_plan.getAllPoses()),
        )
        trajGen = self.trajGenerator
        
        
        
        cmd_velocities = Pose()
        #pos = np.array([])
        #goal_pose = global_plan.getCurrentGoalPose()    #[len(global_plan) - 1]
        #limits = self.robot_limits
        
        bestTraj = self._trajectoryScorers.findBestTrajectory(trajGen)
        if (bestTraj == None or bestTraj.getCost() < 0):
            cmd_velocities = None#self.recover(None, None, None)
            
        else:
            #cmd_velocities.position.x = bestTraj._xv
            #cmd_velocities.position.y = bestTraj._yv
            
            cmd_velocities.position.x, cmd_velocities.position.y, yaw = bestTraj.getVelocities()
            
            # then convert orientation to quarternion
            quat = utils.quaternion_from_euler(0,0,yaw)
            #print("Quat: ", quat)
            cmd_velocities.orientation.x = quat[0]
            cmd_velocities.orientation.y = quat[1]
            cmd_velocities.orientation.z = quat[2]
            cmd_velocities.orientation.w = quat[3]
            #self.trajStackPush(cmd_velocities)    
            self.currentCmdVelocity = cmd_velocities
        return bestTraj, cmd_velocities
        
    
    def trajStackPush(self, cmd_vel):
        #next time just do the conversion from pose to twist here
        self.trajStack.append(cmd_vel)
    
    def trajStackPop(self):
        if(len(self.trajStack) == 0):
            return None
        return self.trajStack.pop()
    
    
    
    """
    So this is my modular part that converts the trajectory from best path to Twist message
    """
    def convertTrajectoryToTwist(self, drive_cmds):
        cmd_vel = Twist()
        cmd_vel.linear.x = drive_cmds.position.x
        cmd_vel.linear.y = drive_cmds.position.y
        cmd_vel.angular.z = utils.getYaw(drive_cmds.orientation)
        return cmd_vel
        
    """
    This is where the code might diverge from navstack le. Should I prune my global plan into 
    windows where each goal pose is the theoretical maximum number of ppoints that can be covered in that
    time frame. Actually you know what forget it I figured i will stick to some part of the navstack
    
    EDIT: somehow this function works such that its distance threshold is just he half the larger dimmension
    of the map, so I'm not sure if this will fk things up cuz unlikely anything is pruned
    
    Navstack's definition of pruning is by removing the point from the linkedList, but in my case i using a 
    global plan object with some index management 
    """
    def transformGlobalPlan(self, robot_pose, global_plan, costmap, current_transformed_plan = None):
        #print("transformGlobalPlan")
        if costmap == None:
            costmap = self._costmap
        costmap_h, costmap_w = self._costmap.getGridShape()
        
        #max vel * SIM granularity
        dist_threshold = 0.0 #self.robot_limits.getMaxVelX() * 1.0 #because i wanna it to change direction at most twice a second       #max(costmap_h * costmap.getMapResolution() / 2.0 , costmap_w * costmap.getMapResolution() / 2.0 )
        localised_plan_section = []
        #incrementDistThresh = self.robot_limits.getMaxVelX() / 2 #* 1.25
        #while(len(localised_plan_section) == 0): #having just one is stupid, cuz that's just the origin possibly? Even then, where should it go
        dist_threshold  = 0.40#0.30
        
        #print("Distance threshold: ", dist_threshold) #Bear in mind, the distance threshold honestly should be longer. Because we dont want this cock to be too short until same index
        
        
        
        #sq_dist_threshold = dist_threshold ** 2 
        sq_dist = 0
        if (current_transformed_plan != None):
            global_plan_idx = current_transformed_plan.getGlobalStartIdx()
        else:
            global_plan_idx = 0#global_plan.getCurrentIndex()
        #print(global_plan.getPointsLength(), global_plan_idx)
        localised_plan_section = []
        
        #get the roughly closest point to the path (im assuming it follows the path well enough)
        #get the roug
        nearestPoint = global_plan.getByIdx(global_plan_idx).pose
        nearestDist = float('inf')
        nearestIdx = global_plan_idx
        
        while(global_plan_idx < global_plan.getPointsLength()):
            #print("Finding nearest point: ", sq_dist, sq_dist_threshold)
            cur_global_plan_pose = global_plan.getByIdx(global_plan_idx).pose
            x_diff = robot_pose.position.x - cur_global_plan_pose.position.x
            y_diff = robot_pose.position.y - cur_global_plan_pose.position.y
            sq_dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
            #localised_plan_section.append(global_plan.getByIdx(global_plan_idx))
            if (nearestDist > sq_dist):
                nearestDist = sq_dist
                nearestPoint = cur_global_plan_pose
                nearestIdx = global_plan_idx
            #print("Current distance: ", sq_dist)
            if (sq_dist >= dist_threshold):
               # print("Furthest point in sim period is :")
                #print(global_plan_idx)
                break
            global_plan_idx += 1
            global_plan.setCurrentIndex(global_plan_idx)
    
        localised_plan_section = global_plan.getAllPosestamps()[nearestIdx:global_plan_idx]
        """
                
        while(global_plan_idx < global_plan.getPointsLength() and sq_dist <= dist_threshold):
            #print("Creating transformed path")
            print("Distance away: ", sq_dist)
            localised_plan_section.append(global_plan.getByIdx(global_plan_idx))
            cur_global_plan_pose = global_plan.getByIdx(global_plan_idx).pose
            x_diff = robot_pose.position.x - cur_global_plan_pose.position.x
            y_diff = robot_pose.position.y - cur_global_plan_pose.position.y
            sq_dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
            
            global_plan_idx += 1
            global_plan.setCurrentIndex(global_plan_idx)
        """
        #print("Distance away: ", sq_dist)
        #print("Current goal pose is ")
        #print(globaself.dwa_planner.lidarRangel_plan.getByIdx(global_plan_idx).pose)
        #print(global_plan_idx, global_plan.getCurrentIndex())
        
        #print("Windowed plan length: ", len(localised_plan_section),nearestIdx,global_plan_idx)
        local_plan = TransformedPlan(localised_plan_section,nearestIdx,global_plan_idx) #since this is transformed, just get finalGoalPose
        return local_plan
    
    """
    Not as straight forward as you think. Grid cell points are pretty exact, and there's a chance that
    trying to hit that point requires the bot to turn around in circles. So we need a function from local planner
    to say "oh im roughly there, so let's deem it reached"
    """
    
    def isGoalReached(self, robot_pos, costmap, goal_point):
        #goal_x = goal_pose.position.x
        #goal_y = goal_pose.position.y
        #goal_th = utils.getYaw(goal_pose.orientation)
        #print(goal_pose)
        if (utils.getEuclideanDistance(goal_point,robot_pos) <= self.xy_goal_tolerance):
            #by right navstack checks if orientation matches, but I personally think it's no necessary because we aren't a car
            # Cars/vehicles need care about the orientation, so i guess this depends on the complexity of global planner too
            return True
        
        return False
    
    
    def updatePlanAndLocalCosts(self):
        return NotImplementedError()
    
    
    
    def recover(self, lidar, robotPose, cmd_vel_publisher):
        #print("Recovering")
        cmd_velocities = Twist()#self.currentCmdVelocity
        #print(lidar)
        lidar = self.lidarRange
        #if(lidar == None):
          
        #print("Have to recover abit first")
        lri = lidar.nonzero()
        #print('Distances: %s' % str(lidar))
        #if (len(lri) > 0):
            
        if(self.recovery_method == 'reverse_current_trajectory'):
            #print("Reverse")
            #print("traj stack length: ", len(self.trajStack))
            previousVel = self.trajStackPop()
            if(previousVel == None):
                return self.recoverByLidar()
                #return "isRecovered"
            #print("Previous vel")
            #print(previousVel)
            cmd_velocities.linear.x = -previousVel.position.x
            cmd_velocities.linear.y = 0.0
            cmd_velocities.angular.z = utils.getYaw(cmd_velocities.orientation)
            
        elif (lri == None or len(lri) == 0):
            #wait for lidar data
            cmd_velocities.linear.x = 0.0
            cmd_velocities.linear.y = 0.0
            cmd_velocities.angular.z = 0.0
        else:
            cmd_velocities = self.recoverByLidar()
            #return "isRecovered"
      
        return cmd_velocities
    
    def recoverByLidar(self):
        #print("do some rotate and move away hahaha")
        #lr2i = np.nanargmax(lidar)
        #lidar = self.lidarRange
        lrShortest = np.nanargmin(self.lidarRange)
        #print('Picked direction: %d %f m' % (lrShortest, self.lidarRange[lrShortest]))
        #robotYaw = utils.getYaw(robotPose.orientation)
        
        lidarCmd = None
        #print("Adjusting rotation")
        
        if(lrShortest > 45 and lrShortest < 325):
            if ((lrShortest < 90)):
                return utils.rotateOnTheSpot(lrShortest)
            elif (lrShortest < 180):
                return utils.rotateOnTheSpot(lrShortest - 180 )
            elif (lrShortest < 270):
                return utils.rotateOnTheSpot( 180 - lrShortest)
            else:
                return utils.rotateOnTheSpot(-lrShortest)
        """
        if(not ((lrShortest > 350 and lrShortest < 10) or (lrShortest > 170 and lrShortest < 190))):
            #if(lrShortest <= 270 and lrShortest >= 90):
            lrShortest = np.nanargmin(self.lidarRange)
            return utils.rotateOnTheSpot()
        """
       # print("Adjusting Distance away")
        
        if(self.lidarRange[lrShortest] < 0.5):   #0.2 dpends on a star map and costmap
            print(self.lidarRange[lrShortest])
            if (lrShortest > 270 or lrShortest < 90):
                #linearDir = "goBack"
                return utils.moveBackCmdVel()
                    
            else:
                #linearDir = "goForward"
                return utils.moveForwardCmdVel()
        else:
            lidarCmd = "isRecovered"
            
        #print('After position: %d %f m' % (lrShortest, self.lidarRange[lrShortest]))    
        
        return lidarCmd