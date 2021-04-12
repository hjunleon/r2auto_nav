#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 20:15:37 2021

@author: facestomperx
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import cv2
import random as rng
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from aStarPlanner import astar

import matplotlib.pyplot as plt
from PIL import Image

import scipy.stats





from GlobalPlan import GlobalPlan
from costmap2d import costmap2d
import utils
from getFrontierAndCentroids import getfrontier, binMap, widenBin 
import dwa_planner
from RobotMovement import OdomRobotMovement, TfRobotMovement

#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  #ok so there's no such lib lmao

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


#from getFilter import getfrontier 

# constants
rotatechange = 0.1
speedchange = 0.05

occ_bins = [-1, 0, 50,100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

#bot footprint in terms of cells
footprint_spec = {
        "width":4,
        "length": 5,
    }

OCCUPIED = 1

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


class bokuNoNav(Node):
    def __init__(self):
        super().__init__('boku_no_nav')
        self.cmd_publisher_ = self.create_publisher(Twist,'cmd_vel',12)  #original 10
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
        self.tfTimeInterval = 0
        self.dwa_planner = dwa_planner.DWAPlanner()
        self.OdomRobotMovement = OdomRobotMovement()
        self.TfRobotMovement = TfRobotMovement()
        self.currentTfFrame = None
        self.currentOdom = None
        self.finalCmd = None
        
        
        self.RobotState = ["Mapping","Evaluating"]
        """might make this timers into another class"""
        self.localPlannerTime = 0
        self.windowed_plan = None
        
        #recovery
        self.isRobotRecovering = False
        self.lidarDistance = None
        self.forwardAngles = [-20,20]
        self.backwardAngles = [160,200]
        
        
        
        #navstack
        #self._action_client = ActionClient(self, MoveBaseAction, 'move_base')
        
        
        
        
        self.currentFrontierGoal = None
        self.currentEndCell = (0, 0)
        self.goal_publisher_ = self.create_publisher(PoseStamped,'goal_pose',10) 
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        #self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
        #                                               'amcl_pose', self.poseCallback, pose_qos)
        
        self.RobotState = "NavInitialised"
    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m') 
        
    #callbacks
        
    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')
        #print(msg)
        
        if (self.OdomRobotMovement.hasReference):
            #print(self.OdomRobotMovement.getVelocity())
            timeDiff = self.OdomRobotMovement.getTimeDiff(msg)
            if timeDiff != 0:
                #print("A future ODOM")
                self.tfTimeInterval = (self.tfTimeInterval + timeDiff) / 2
            #print("Average time: ", self.tfTimeInterval)
        
        self.OdomRobotMovement.updateFromOdomMsg(msg)
        
        #odom_position = msg.pose.pose.position
        #orientation_quat =  msg.pose.pose.orientation
        #self.get_logger().info('Odom Trans: %f, %f' % (odom_position .x, odom_position .y))
        #self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        #cell = utils.worldToMap(coords, world_origin, resolution)
        
        # got this hacky way
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            #tf_vel = self.tfBuffer.lookupTwistFull()
            #https://answers.ros.org/question/256354/does-tftransformlistenerlookuptransform-return-quaternion-position-or-translation-and-rotation/
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        #print(trans)
        self.currentTfFrame = trans
        return

    def occ_callback(self, msg):
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        if(self.currentTfFrame == None):
            print("currentTfFrame still None")
            return 
        
        robot_vel = self.OdomRobotMovement.getVelocity()
        x = robot_vel.linear.x
        y = robot_vel.linear.y
        xy_vel = math.sqrt(x ** 2 + y ** 2) 
        if(not (robot_vel != None and xy_vel < 0.5 and robot_vel.angular.z < 0.15)):
            print("Robot fast enough, so technically can continue planning")
            print(xy_vel)
            return
        #binning grid
        grid_edges = binMap(msg)
        astar_grid_edges = widenBin(grid_edges,1) #for A star to not hug so tightly 
        costmap = costmap2d(msg, astar_grid_edges, footprint_spec) #grid_edges, fatter_edges
        
        if (not (self.RobotState == "NavInitialised" or self.RobotState == "NavCompleted")):
            return
        #if (len(self.RobotState) == 2 and self.RobotState[0] == "Mapping" and self.RobotState[1] == "Travelling" or self.RobotState[1] == "LocalPlan"):
        #    return
        
        
        
        #we dont want the bot flying around while planning
        #self.stopbot()
        #time.sleep(0.25) # If a star computes fast rnough and got no chance to stop
        
        
        
        
        cur_pos = self.currentTfFrame.transform.translation
        cur_rot = self.currentTfFrame.transform.rotation
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        
        robot_pose = utils.createRobotPose(cur_pos,cur_rot)
        
        
        
        
        #trying to pass a fattened grid
        frontierImg, frontierCells, frontierWorldPts,mBotPos, grid_edges, fatter_edges = getfrontier(msg,cur_pos,grid_edges)
       # print("FRONTIERS")
       # print(frontierCells)
     #   print(len(frontierCells))
        
     #   print("Bot footprint: ", costmap.getMapBaseFootprint())   #should update my map
       
        
        binary_grid_edges = []
        for row in fatter_edges: #grid_edges, fatter_edges
            temp = []         
            for col in row:
                if col == 255:
                    temp.append(1)
                else:
                    temp.append(0)
            binary_grid_edges.append(temp)
        
        """
        print(frontierCells)
        for row in binary_grid_edges:
            print(row)
        """
        
        occdata = np.array(msg.data)
        #print(occdata.shape)
        #checkOcc = np.uint8(occdata.reshape(msg.info.height,msg.info.width))
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
       # iwidth = msg.info.width
        #iheight = msg.info.height
        #total_bins = iwidth * iheight
        #odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        
        img = Image.fromarray(frontierImg) #grid_edges, frontierImg, fatter_edges
        # show the image using grayscale map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        plt.pause(0.00000000001)
        print(frontierCells)
        
        if (len(frontierCells) == 0): #(frontierCells[0][0] == 0 and frontierCells[0][1] == 0) ):
            self.evaluateCompletion()
            return
        
        
        shortestPath = None
        shortestEnd = None
        
        
        
        shortestPathIdx = 0
        shortestDistance = float('inf')
        #print(frontierWorldPts)
        #return
        
        startPos = mBotPos
        endPos = tuple(frontierCells[shortestPathIdx])
        for idx,frontier in enumerate(frontierWorldPts):
            dist = utils.getRoughEuclideanDistance(cur_pos, frontier)
            endPos = tuple(frontierCells[shortestPathIdx])
            print("dist: ", dist)
            print("Last time cell: ", self.currentEndCell)
            if (utils.areCellsEqual(startPos , endPos)):
                print("endPos and startpos equal")
                continue
            if (utils.areCellsEqual(endPos, self.currentEndCell)):
                print("endPos and self.currentEndCell equal")
                continue
            if dist < shortestDistance:
                shortestDistance = dist
                shortestPathIdx = idx
          
        startPos = mBotPos
        endPos = tuple(frontierCells[shortestPathIdx])
        endPosWorld = frontierWorldPts[shortestPathIdx]
        #path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
        print(startPos, endPos, self.currentEndCell)
        
        if (utils.areCellsEqual(endPos, self.currentEndCell)):
            print("SAME CELLS STILL???")
            return
        
        
        tryGoal = PoseStamped()
        tryGoal.header.stamp.sec = 0
        tryGoal.header.frame_id = 'map'
        tryGoal.pose.position = endPosWorld
        #if (tryGoal == self.currentFrontierGoal):
            
        
        self.currentEndCell = endPos
        self.currentFrontierGoal = tryGoal
        print("Publishing ", tryGoal)
        self.goal_publisher_.publish(tryGoal)
        
        
        
        return
        if(shortestPath == None):
            print("Maybe all the A star stuff fked, so oops")
            self.recover()
            return
            #self.evaluateCompletion()
        
        if(shortestPath == "invalid start" or shortestPath == None):
            print("A star invalid path")
            self.recover()
            return
            
        print("Shortest path is: ", shortestPath[0], shortestPath[len(shortestPath) - 1], len(shortestPath))
        print(shortestEnd)
        print(shortestPath)
        poses = utils.convertPlanToPoses(shortestPath, costmap.getMapOrigin(), costmap.getMapResolution())
        #print("Poses length: ", len(poses))
        global_plan = GlobalPlan(poses)
        self.global_plan = global_plan
        goal = poses[len(poses) - 1].pose.position
        nextPoint = poses[1].pose.position
        
        
        #endPose = utils.mapToWorld2(endPos, costmap.getMapOrigin(), costmap.getMapResolution())
        inc_x = nextPoint.x - cur_pos.x
        inc_y = nextPoint.y - cur_pos.y
        angleBetween = math.atan2(inc_y, inc_x) - yaw # this works
        print("Bot yaw: ", np.degrees(yaw))
        print("angle between points??"  , angleBetween, np.degrees(angleBetween))
        return
        

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        print("min dist: ", min(self.laser_range))
        #np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan
      
    def getRobotPose(self):
        cur_pos = self.currentTfFrame.transform.translation
        cur_rot = self.currentTfFrame.transform.rotation 
        return utils.createRobotPose(cur_pos,cur_rot)    
    
    def evaluateCompletion(self):
        print("Yay we're done??")
        self.RobotState = "fullyComplete"
    
    
    def recover(self):
        #if (not self.isRobotRecovering):
        self.isRobotRecovering = True
        bestCmd = self.dwa_planner.recover(self.laser_range, self.getRobotPose(), self.cmd_publisher_)
        if (bestCmd == "isRecovered"):
            self.isRobotRecovering = False
            return
        
        print("Current cmd")
        print(self.finalCmd)
        self.finalCmd = self.dwa_planner.convertTrajectoryToTwist(bestCmd)
        
        
        print("Recovery cmd: ")
        print(self.finalCmd)
        self.cmd_publisher_.publish(self.finalCmd)
    
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.cmd_publisher_.publish(twist)
        
        
    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            #self.pick_direction()
            """
            #https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/msg/GoalStatus.msg
            
            
            self.info_msg("Waiting for 'NavigateToPose' action server")
            while not self.action_client.wait_for_server(timeout_sec=1.0):
                self.info_msg("'NavigateToPose' action server not available, waiting...")
                self.RobotState = "NavConnectionFailed"
                continue
            self.RobotState = "NavInitialised"
            """
            
                
            while rclpy.ok():
                
                #print(self.RobotState)
                
                # allow the callback functions to run
                if(self.RobotState == "fullyComplete"):
                    return
                rclpy.spin_once(self)
                """
                if(self.currentFrontierGoal == None):
                    continue
                self.RobotState = "Travelling"
                goal_msg = NavigateToPose.Goal()
                #print(self.currentFrontierGoal.pose)
                print(goal_msg)
                goal_msg.pose = self.currentFrontierGoal
                
                self.info_msg('Sending goal request...')
                send_goal_future = self.action_client.send_goal_async(goal_msg)
        
                rclpy.spin_until_future_complete(self, send_goal_future)
                goal_handle = send_goal_future.result()
        
                if not goal_handle.accepted:
                    self.error_msg('Goal rejected')
                    continue
        
                self.info_msg('Goal accepted')
                get_result_future = goal_handle.get_result_async()
        
                self.info_msg("Waiting for 'NavigateToPose' action to complete")
                rclpy.spin_until_future_complete(self, get_result_future)
                status = get_result_future.result().status
                if status != GoalStatus.STATUS_SUCCEEDED:
                    self.info_msg('Goal failed with status code: {0}'.format(status))
                else:    
                    #return False
                    #if status != GoalStatus.STATUS_ABORTED:
                        
                    self.info_msg('Goal succeded!')
                self.RobotState = "NavCompleted"
                """

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()




def main(args=None):
    rclpy.init(args=args)

    auto_nav = bokuNoNav()
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()