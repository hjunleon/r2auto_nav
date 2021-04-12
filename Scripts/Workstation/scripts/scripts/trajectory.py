#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 20:24:56 2021

@author: facestomperx
"""


import numpy as np
from types import SimpleNamespace

#https://realpython.com/python-pass-by-reference/
"""
Furthermore, since Python already uses pointers behind the scenes, there would be no additional performance benefits even if it were able to pass arguments by reference.
Aim to write single-purpose functions that return one value, then (re)assign that value to variables,

ALTERNATIVELY
pass in an object,dictionary or list
"""


#https://stackoverflow.com/questions/1098549/proper-way-to-use-kwargs-in-python
"""
        * @brief  Constructs a trajectory
       * @param xv The x velocity used to seed the trajectory
       * @param yv The y velocity used to seed the trajectory
       * @param thetav The theta velocity used to seed the trajectory
       * @param num_pts The expected number of points for a trajectory  

"""
class Trajectory():
    def __init__(self, *args, **kwargs):
        #print("Creating traj instance")
        self._xv = kwargs.get("xv",0.0)
        self._yv = kwargs.get("yv",0.0)
        self._thetav = kwargs.get("thetav",0.0)
        self._cost = kwargs.get("cost",-1.0)
        self._time_delta = kwargs.get("time_delta",0.0)
        self._x_pts = kwargs.get("x_pts", None)
        self._y_pts = kwargs.get("y_pts", None)
        self._th_pts = kwargs.get("th_pts", None)
    
    def getCost(self):
        #print("getting trajectory cost??")
        return self._cost
    
    def setCost(self, cost):
        self._cost = cost
    
    def getVelocities(self):
        return self._xv, self._yv, self._thetav
    
    def getPoint(self, index, pointObj):        
        pointObj.x = self._x_pts[index]
        pointObj.y = self._y_pts[index]
        pointObj.th = self._th_pts[index]
        
    def getAllPoints(self):
        points = []
        for i in range(self.getPointsListLength()):
            point = SimpleNamespace()
            point.x = self._x_pts[i]
            point.y = self._y_pts[i]
            points.append(point)
        return points
        
    def setPoint(self, index, pointObj):
        self._x_pts[index] = pointObj["x"]
        self._y_pts[index] = pointObj["y"]
        self._th_pts[index] = pointObj["th"]

    def resetPoints(self):
        self._x_pts = np.array([])
        self._y_pts = np.array([])
        self._th_pts = np.array([])
        
    def getPointsListLength(self):
        return len(self._x_pts)
    
    def getEndpoint(self, pointObj):
        lastIndex = self.getPointsListLength() - 1
        pointObj["x"] = self._x_pts[lastIndex]
        pointObj["y"] = self._y_pts[lastIndex]
        pointObj["th"] = self._th_pts[lastIndex]

    def addPoint(self, x, y, th):
        #print("ADDING POINT")
        #print(x, y , th)
        #print(self._x_pts)
        #print(len(self._x_pts))
        self._x_pts = np.append(self._x_pts,x)
        self._y_pts = np.append(self._y_pts,y)
        self._th_pts = np.append(self._th_pts,th)
        #self._x_pts.append(x)
        #self._y_pts.append(y)
        #self._th_pts.append(th)