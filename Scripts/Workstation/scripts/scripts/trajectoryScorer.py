#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr  4 12:28:28 2021

@author: facestomperx
"""

#https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/base_local_planner/src/simple_scored_sampling_planner.cpp

import trajectory
class trajectoryScorer():
    def __init__(self, cost_funcs, max_samples):
        #print("Initialising trajectory scorer")
        self.cost_funcs = cost_funcs
        self.max_samples = max_samples
        #maybe i need the windowed global plan? Apparnetly the
        # cost functions have the global plan and costsmaps
    #returns the cost of this trajectory
    def scoreTrajectory(self, traj)->float:
        #print("Scoring")
        traj_cost = 0
        for (idx, cost_func) in enumerate(self.cost_funcs):
            if (cost_func.getScale() == 0):
                continue
            cost = cost_func.scoreTrajectory(traj)
            if (cost < 0):
                #discard it
                traj_cost = cost
                break
            if (cost != 0):
                cost *= cost_func.getScale()
            traj_cost += cost
          
        #print("Final traj cost: ", traj_cost)
        return traj_cost

    #return the best traj and cost and all explored trajectories, and their costs
    def findBestTrajectory(self, traj_generator):
       # print("GOING TO FIND THE BEST TRAJECTORY NOW")
        best_traj_cost = -1
        cur_traj_cost = -1
        valid_traj_count = 0
        allExplored = []
        bestTraj = None
        traj_count = 0
        for traj in traj_generator:
            #print("This is the ", traj_count," trajectory")
            #print(traj.getVelocities())
            #print(traj)
            traj_count += 1
            cur_traj_cost = self.scoreTrajectory(traj)
            traj.setCost(cur_traj_cost)
            allExplored.append(traj)
            if(cur_traj_cost >= 0):
                valid_traj_count += 1
                if (best_traj_cost < 0 or best_traj_cost > cur_traj_cost):
                    bestTraj = traj
                    best_traj_cost = cur_traj_cost
            if (self.max_samples > 0 and traj_count > self.max_samples):
                break
        
        if(bestTraj == None):
            print("Best Traj None??")
        #else:
        #    print("Best traj cost: ", bestTraj.getCost())
        #    print("Best traj points", bestTraj.getAllPoints())
        #    print("Best traj getVelocities: ", bestTraj.getVelocities())
        return bestTraj
            
        