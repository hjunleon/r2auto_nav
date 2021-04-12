#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 11 11:38:59 2021

@author: facestomperx
"""
for frontiers in frontierCells:
            startPos = mBotPos
            endPos = tuple(frontiers)
            #print("Cost: ",costmap.getCost(endPos[0], endPos[1]))
            """
            if (not costmap.isCellCoordValid(endPos) or costmap.getCost(endPos[0], endPos[1]) > 0):
                print("Sadly invalid path? Maybe can find somewhere nearer")
                continue
            """
            """
            endPose = utils.mapToWorld2(endPos, costmap.getMapOrigin(), costmap.getMapResolution())
            inc_x = endPose[0] - cur_pos.x
            inc_y = endPose[1] - cur_pos.y
            angleBetween = math.atan2(inc_y, inc_x) - yaw # this works
            print("Bot yaw: ", np.degrees(yaw))
            print("angle between points??"  , angleBetween, np.degrees(angleBetween))
            """
            print("Forming path from ", startPos, endPos)    
            print(costmap.getGridShape())
            path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
            if(path == "invalid start"):
                break
            if (path == None):
                print("A star invalid path")
                continue
            #print(mBotPos)
            #print(path)
            #print("Path length: ", len(path))
            if shortestPath == None or len(path) < len(shortestPath):
                shortestPath = path
                shortestEnd = endPos