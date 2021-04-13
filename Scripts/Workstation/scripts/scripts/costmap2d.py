#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 11:09:49 2021

@author: facestomperx
"""
"""
    A class to store the occupancy grid from occupancy grid node
    Then got some methods to abstract it i guess
"""

import numpy as np
import utils
from LineIterator import LineIterator
import rclpy
from local_planner_limits import LocalPlannerLimits
import math

from geometry_msgs.msg import Point

OBSTACLE = 1

class costmap2d():
    def __init__(self, occupancy_grid, binned_grid, footprint_spec = None):
        self.occupancy_grid = occupancy_grid
        self.binned_grid = binned_grid
        self.proccessed_occ = self.reshapeOccupancyGrid()
        self.footprint_spec = footprint_spec #just fk it, navstack too complicated. A series of points describing edges of footprint or radius  for circular shape
        self.worldFootprint = None
        self.mapFootprint = None
        radius = 0.08 #safer? 0.12,0.125, 0.11, 0.125,0.13
        self.worldBaseFootprint = self.makeFootprintFromRadius(radius)
        #print(self.worldBaseFootprint[0])
        self.mapBaseFootprint = self.formMapBaseFootprint(self.worldBaseFootprint)
        self.robotRadiusInCells = round(radius / self.getMapResolution()) #frankly idk whether to be safe than sorry
        #print("Initialising costmap")
        #print(self.mapBaseFootprint) #math.ceil
        #print("Robot base: ", self.robotRadiusInCells)
        
    def formMapBaseFootprint(self,worldCoords):
        #temp = self.footprintWorldToMap(worldCoords)
        #zeroPoint = Point()
        #zeroPoint.x = zeroPoint.y = 0.0
        toBeRet = set()
        for x in worldCoords:
            newCoord = (int(x.x / self.getMapResolution()),int(x.y / self.getMapResolution()))
            if newCoord not in toBeRet:
                toBeRet.add(newCoord)
            
        return toBeRet
        
    def reshapeOccupancyGrid(self):
        self.h = self.getMapCellHeight()
        self.w = self.getMapCellWidth()
        return np.uint8(np.array(self.occupancy_grid.data).reshape(self.h,self.w))
        
    """
    Returns the shape of the grid
    """
    def getGridShape(self):
        return self.h, self.w
    
    def updateOccupancyGrid(self, occupancy_grid):
        self.occupancy_grid = occupancy_grid
        self.reshapeOccupancyGrid()
        
    def getOccupancyGrid(self):
        return self.occupancy_grid
    
    def getRaw2dGrid(self):
        return self.proccessed_occ
    
    def getBinnedGrid(self):
        return self.binned_grid
    
    
    def getMapOrigin(self):
        return self.getOccupancyGrid().info.origin.position
    
    def getMapResolution(self):
        return self.getOccupancyGrid().info.resolution
    
    def getMapCellWidth(self):
        return self.getOccupancyGrid().info.width
    
    def getMapCellHeight(self):
        return self.getOccupancyGrid().info.height
    
    def getCost(self, row, col):
        return self.binned_grid[row][col]
    
    #https://github.com/ros-planning/navigation/blob/9964398cf590100a20549ab4d16e71b10d16fcb7/costmap_2d/include/costmap_2d/costmap_2d_ros.h
    def getRobotFootprint(self):
        return self.footprint_spec
    
    def footPrintCostRadius(self, robot_pos):
        #print("Robot base: ", self.robotRadiusInCells)
        cur_pos = utils.worldToMap(robot_pos, self.getMapOrigin(), self.getMapResolution())
        #print(cur_pos)
        grid = self.getBinnedGrid()  #dont use raw grid for this
        #print(self.mapBaseFootprint)
        for x in self.mapBaseFootprint:
            temp = [0,0]
            temp[0] = x[0] + cur_pos[0]
            temp[1] = x[1] + cur_pos[1]
            if(temp[0] >= len(grid) or temp[0] < 0 or temp[1] >=  len(grid[0]) or temp[1] < 0):
                #print("Invalid point")
                return -3.0
                
            if (grid[temp[0]][temp[1]] != 0):
                #print("Invalid point")
                return -3.0
        return 0.0
            
    """
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space
    """
    def footPrintCost(self, robot_pose, footprint_points):
        #if (footprint_points == None):
        #    footprint_points = self.mapBaseFootprint
        cell = utils.worldToMap(robot_pose, self.getMapOrigin(), self.getMapResolution())
        if (not cell):
            return -3.0
        line_cost = 0.0
        footprint_cost = 0.0
        
        for i in range(len(footprint_points) - 1):
            # get first point
            p1 = utils.worldToMap(footprint_points[i], self.getMapOrigin(), self.getMapResolution())
            if (p1 == None):
                return -3.0
            # get last ppoint
            pLast = utils.worldToMap(footprint_points[i+1], self.getMapOrigin(), self.getMapResolution())
            if (pLast == None):
                return -3.0
            # form a line and see if any point along line is affected
            line_cost = self.lineCost(p1,pLast)
            footprint_cost = max(line_cost, footprint_cost)
            
            if (line_cost < 0):
                return line_cost
        
        # get first point
        p1 = utils.worldToMap(footprint_points[-1], self.getMapOrigin(), self.getMapResolution())
        if (p1 == None):
            return -3.0
        # get last ppoint
        pLast = utils.worldToMap(footprint_points[0], self.getMapOrigin(), self.getMapResolution())
        if (pLast == None):
            return -3.0
        # form a line and see if any point along line is affected
        line_cost = self.lineCost(p1,pLast)
        footprint_cost = max(line_cost, footprint_cost)
        
        if (line_cost < 0):
            return line_cost
        
        
        return footprint_cost
    
    def lineCost(self,p1,p2):
        line_cost = 0.0
        point_cost = -1.0
        line_it = LineIterator(p1,p2)
        while(line_it.isValid()):
            point_cost = self.pointCost(line_it.getX(),line_it.getY())
            if (point_cost < 0):
                return point_cost
            if(line_cost < point_cost):
                line_cost = point_cost
            line_it.advance()
        return line_cost
        
    def pointCost(self, row, col):
        cost = self.getCost(row, col)
        if (cost > 0):
            return -1
        return cost
    
    def getRobotPoseCell(self):
        return self._robot_cell_pose
    
    def setRobotPoseCell(self, robot_pose_pos):
        self._robot_cell_pose = utils.worldToMap(robot_pose_pos, self.getMapOrigin(),self.getMapResolution())
        
    """
    Specifically for a robot with a specific number of sides (im hardcoding now)
    and with a certain length and width (regular polygons)
    """
    #https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/base_local_planner/include/base_local_planner/footprint_helper.h
    #https://github.com/ros-planning/navigation/blob/4a3d261daa4e7eafa40bf7e4505f8aa8678d7bd7/costmap_2d/src/footprint.cpp
    def footprintVertices(self, robot_pos, robot_orientation, sides = 4):
        self.setRobotPoseCell(robot_pos)
        robot_height, robot_width = self.LocalPlannerLimits.getRobotDimmensions()
        cur_robot_cell = self.getRobotPoseCell()
        cur_robot_orientation = robot_orientation
        
        footprint = [[],[],[],[]]
        return footprint
        
    """Given current position and rotation, then return the oriented xy values of the footprint
        robot_pos and robot_rot should come direct from tf frame, so xyz and quaternion
    """
    def transformedFootprint(self, robot_pos, robot_rot):
        #build the oriented footprint at a given location
        oriented_footprint = []
        theta = utils.getYaw(robot_rot)
        x = robot_pos.x
        y = robot_pos.y
        footprint_spec = self.footprint_spec
        cos_th = math.cos(theta)
        sin_th = math.sin(theta);
        for i in range(len(self.footprint_spec)):
          new_pt = Point();
          new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
          new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
          oriented_footprint.append(new_pt);
        return oriented_footprint
    
    
    """The xy circumference from the 0,0 point, requires further translation into world coords. Now local"""
    def makeFootprintFromRadius(self,radius = 0.1):
        points = []
        N = 32
        for i in range(N):
            angle = i * 2 * utils.M_PI_2 / N
            newPt = Point()
            newPt.x = math.cos(angle) * radius
            newPt.y = math.sin(angle) * radius
            points.append(newPt)
            
        return points
    
    def translateToWorldRobotPose(self, robot_pos, robot_orientation, footprint_list):
        trans_x = robot_pos.x
        trans_y = robot_pos.y
        translated_footprint = []
        #check if it does make copies or edits original
        for x in footprint_list:
            point = x
            point.x += trans_x
            point.y += trans_y
            translated_footprint.append(point) 
        self.worldFootprint = translated_footprint
        return translated_footprint
    
    def footprintWorldToMap(self, worldFootprint):
        mapFootprint = set()
        
        for point in worldFootprint:
            print(point)
            cell = utils.worldToMap(point, self.getMapOrigin(),self.getMapResolution())
            if cell not in mapFootprint:
                mapFootprint.add(cell)
                
        return mapFootprint
    
    def getMapBaseFootprint(self): #only local to the robot itself, so like a circle centered on origin
        return self.mapBaseFootprint
    def getRobotMapFootprint(self):
        return self.mapFootprint
    
    """
    Check if coordinate tuple is within the bounds of the map and not an obstacle
    If it isn't returns a valid coordinate to path towards 
    """
    def isCellCoordValid(self, coord):
        h, w = self.getGridShape()
        #print("Checking cell: ", coord," in ", h, w)
        #print("Cell cost: ", self.getCost(coord[0], coord[1]) )
        if (self.isCellCoordExceedBoundary(coord) == True and self.getCost(coord[0], coord[1]) > 0): #obstacle any number larger than 0
            return False
        return True
    """
    Given a point obj, which has x,y and z i think? Convert worldToMap then call isCellCoordValid
    """    
    def isPointValid(self, point):
        #print("isPointValid")
        cell = self.worldToMap(point)
        return self.isCellCoordValid(cell)
    
    def worldToMap(self, point):
        return utils.worldToMap(point, self.getMapOrigin(),self.getMapResolution())
        
    def isCellCoordExceedBoundary(self, coord):
        #print(coord, "vs", self.getMapCellHeight(),self.getMapCellWidth())
        if (coord[0] < 0 or coord[0] >= self.getMapCellHeight()):
            return True
        elif (coord[1] < 0 or coord[1] >= self.getMapCellWidth()):
            return True
        return False
    
    """
    Returns a set of cells at a certain dist from a provided cell in costmap
    """
    
    def generateSurroundingCells(self, cell, dimmensions, genType="square"):
        if (genType == "square"):
            seenCells = set(cell)
            queueOfCells = [cell]
            movements = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
            while(len(queueOfCells) > 0 and len(seenCells) < dimmensions[0] * dimmensions[1]):
                curCell = queueOfCells.pop()
                for move in movements:
                    evalCell = curCell + move
                    if (evalCell not in seenCells and self.isCellCoordValid(evalCell)):
                        seenCells.add(evalCell)
            return seenCells
            
            
        else:
            #convert to world coords, generate circle, convert circle points into cells
            return []
