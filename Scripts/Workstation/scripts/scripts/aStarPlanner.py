#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 10:49:53 2021

@author: facestomperx
"""

import utils

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        #print("INIT")
        self.parent = parent
        self.position = position
        #print(position)
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def checkFootprintObstructed(footprint, cur_pos, grid):
    #displaced_print = []
    #print("Checking footprint")
    for x in footprint:
        #print(x)
        temp = [0,0]
        temp[0] = x[0] + cur_pos[0]
        temp[1] = x[1] + cur_pos[1]
        #print("traslated footprint: ", temp)
        if(temp[0] >= len(grid) or temp[0] < 0 or temp[1] >=  len(grid[0]) or temp[1] < 0):
            #print("Footprint exceeded bounds", temp)
            return True
        
        
        if (grid[temp[0]][temp[1]] != 0):
            #print("Footprint obstrcuted. I wonder where... ", temp)
            return True
    return False
    
def distanceBetween2Cells(cell1,cell2):
    return utils.getEuDist4Cells(cell1,cell2)

def astar(occupancy_grid, start, end, aStarConfig):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    
    # check what is occupied and not
   # occ_val = aStarConfig["occupied"]
   # vac_val = aStarConfig["vacant"]
    footprint = None
    footprintRadius = 1
    if ("footprint" in aStarConfig):
        footprint = aStarConfig["footprint"]
        footprintRadius = aStarConfig["footprintRadius"]
    else:
        footprint = [(0,0)]    
    #print("A star footprint: ", footprint)
   # currentFootprintDisplacement = (0,0) #x,y displacements
    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    open_set = set()
    closed_set = set()

    # Add the start node
    open_list.append(start_node)
    open_set.add(start)
    
    
    
    #Before looping, check whether this bot is even in a ssafe place
    if (checkFootprintObstructed(footprint, start, occupancy_grid)):
        print("Obstructed from the start!")
        return "invalid start"
        print(footprint)
        footprint = [(0,0)]    
    
    
    # Loop until you find the end, 
    # TODO: set threshold for how many times a nearby goal cell can be visited
    while len(open_list) > 0 and len(open_set) > 0:
        #print(len(open_list))
        # Get the current nodedist_threshold
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        open_set.remove(current_node.position)
        closed_list.append(current_node)
        closed_set.add(current_node.position)
        
        #print("End Cell: ", end_node.position)
        #print(current_node.position)
        #print("Bot radius: ", footprintRadius)
        distBetweenCells = distanceBetween2Cells(current_node.position,end_node.position)
        #print("Distance between cells: ", distBetweenCells)
        #print(len(open_list))
        #print("Map height vs width: ", len(occupancy_grid),len(occupancy_grid[0]))
        # Found the goal
        if current_node == end_node or distBetweenCells <= footprintRadius:
            break
            

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            
            
            # Make sure within range
            if node_position[0] > (len(occupancy_grid) - 1) or node_position[0] < 0 or node_position[1] > (len(occupancy_grid[len(occupancy_grid)-1]) -1) or node_position[1] < 0:
                continue
            #print("Position valid")
            # Make sure walkable terrain
            if checkFootprintObstructed(footprint, node_position, occupancy_grid):
                continue
            #print("Footprint valid")
            """
            if occupancy_grid[node_position[0]][node_position[1]] != 0:
                continue
            """
            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            if child.position in closed_set:
                continue
            """
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            """    
                
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            if child.position in open_set:
                continue
            for open_node in open_list:
                if child.g > open_node.g: #child == open_node and 
                    continue

            # Add the child to the open list
            #print("Adding child: ",child.position)
            open_list.append(child)
            open_set.add(child.position)
    
    if(len(open_list) == 0):
        print("Turns out all the open list ran out, maybe didnt reach a path after all")
        return None #prompts recovery
    
    if(len(open_list) == 1 and open_list[0].position == start):
        print("Only found origin, activate recovery")
        return None
    
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1] # Return reversed path
    
    #return "No path??"