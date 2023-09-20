#!/usr/bin/env python
import numpy as np
from matplotlib import pyplot as plt
from sklearn import datasets, linear_model
import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import std_msgs.msg
import tf 
import tf2_ros
import tf2_py as tf2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from laser_geometry import LaserProjection
from std_msgs.msg import Float64

class State:
    MOVING = 0
    BLOCKED_LEFT = 1
    BLOCKED_RIGHT = 2
    DONE = 3 
    LOST = 4 #confused
    DISABLED = 5
    UNEXPECTED = 6 #confused
    MOVEMENT_PAUSED = 7
    END_OF_SHELF = 8


class BathNvg(): 
    def __init__(self): 
        self.pcPub = rospy.Publisher ("/laserPointCloud", PointCloud2, queue_size =1) 
        self.laserSub = rospy.Subscriber("/merged_cloud" , PointCloud2, self.laserCallback)
        self.odom = rospy.Subscriber("/odometry/filtered" , Odometry, self.distanceCallback)
        self.cmdPub = rospy.Publisher("/bath/cmd_vel", Twist, queue_size=10)

        rospy.set_param("/bath/linear_vel_y", 0.005)
        rospy.set_param("/bath/distance_2_wall", 1.5)
        rospy.set_param("/bath/enabled_functionality", 0)
        rospy.set_param("/bath/shelf_length", 1.0)
        rospy.set_param("/bath/movement_pause", 0)
        self.start_posX = 0
        self.start_posY = 0
        self.first_run = True
        self.distance_moved = 0
        self.target_reached = False

        self.state = State.DISABLED

        # DEBUGGING
        self.previous_state = State.DONE
        self.helper_counter = 0


    def distanceCallback(self, data):
        if rospy.get_param("/bath/enabled_functionality"):
            if self.first_run:
                self.start_posX = data.pose.pose.position.x
                self.start_posY = data.pose.pose.position.y
                self.target_reached = False
                self.first_run = False

            acctual_posX = data.pose.pose.position.x
            acctual_posY = data.pose.pose.position.y
            self.distance_moved = np.sqrt((acctual_posX-self.start_posX)**2 + (acctual_posY-self.start_posY)**2)
            
            target_shelf_length = rospy.get_param("/bath/shelf_length")
            if self.distance_moved > target_shelf_length:
                rospy.set_param("/bath/enabled_functionality", 0)
                self.state = State.DONE

                self.first_run = True
                self.distance_moved = 0
                self.target_reached = True


    def laserCallback(self, data):
        if rospy.get_param("/bath/enabled_functionality"):
            # Info about read points structure:
            # Point(x=0.5640362501144409, y=0.2720063626766205, z=-0.054999999701976776, intensity=0.0, index=1)
            #pts_list = pc2.read_points_list(cloud_out)   # "not efficient" http://wiki.ros.org/laser_geometry example 3.1
            #print(pts_list)
            #print(pts_list[1])
            #print(len(pts_list))
            
            point_generator = pc2.read_points(data)

            self.targetDistance2Wall = rospy.get_param("/bath/distance_2_wall")		# in meters
            noiseLevel = 0.25			# in meters +/-
            x_wall = []
            y_wall = []
            z_wall = []

            targetDistance2Obstacle = 0.8
            x_obstacle_left = []
            y_obstacle_left = []
            x_obstacle_right = []
            y_obstacle_right = []
        
            target_shelf_length = rospy.get_param("/bath/shelf_length")
            for point in point_generator:
                x = point[0]
                y = point[1]
                z = point[2]
                intensity = point[3]
                #index = point[4]
                
                # Locating wall feature
                if (x < (self.targetDistance2Wall+noiseLevel)) and (x > (self.targetDistance2Wall-noiseLevel)) and (y < 1.2) and (y > -1.2):
                    x_wall.append(x)
                    y_wall.append(y)
                    z_wall.append(z)
                    # print(len(x_wall))
                
                # obstacle left
                footprint = 0.7
                if (x < footprint) and (x > -footprint) and (y < targetDistance2Obstacle) and (y > 0.4):
                    x_obstacle_left.append(x)
                    y_obstacle_left.append(y)
                    # print(x, ",", y, ",", z, ",", intensity, ",", index)
                    distance = math.sqrt(x ** 2 + y ** 2)
                    # print(distance)

                # obstacle right
                if (x < footprint) and (x > -footprint) and (y > -targetDistance2Obstacle) and (y < -0.4):
                    x_obstacle_right.append(x)
                    y_obstacle_right.append(y)
                    # print(x, ",", y, ",", z, ",", intensity, ",", index)
                    distance = math.sqrt(x ** 2 + y ** 2)
                    # print(distance)
            
            #header
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'base_link'
            #create pcl from points
            data = np.array([x_wall,y_wall,z_wall]).T
            scaled_polygon_pcl = pc2.create_cloud_xyz32(header, data)
            self.pcPub.publish(scaled_polygon_pcl)

            skip = False
            direction = rospy.get_param("/bath/linear_vel_y")
            if rospy.get_param("/bath/movement_pause"):
                self.state = State.MOVEMENT_PAUSED
                skip = True

            if len(x_wall) < 40:
                self.state = State.LOST
                skip = True

            if len(x_obstacle_left) > 0 and direction > 0.0:
                self.state = State.BLOCKED_LEFT
                skip = True

            if len(x_obstacle_right) > 0 and direction < 0.0:
                self.state = State.BLOCKED_RIGHT
                skip = True

            if not (min(y_wall) < 0.0) and (max(y_wall) > 0.0):
                self.state = State.END_OF_SHELF
                skip = True

            if not skip:
                distance, angle = self.fit_line(y_wall, x_wall, True)
                self.state = State.MOVING
                self.speed_controller(distance, angle)

        # print for debuging 
        if self.helper_counter == 100 or self.previous_state != self.state:
            print(self.state)
            self.helper_counter = 0
            self.previous_state = self.state
        self.helper_counter += 1
        
    def speed_controller(self, distance, angle):
        velocity_msg = Twist()
        target_distance = self.targetDistance2Wall
        target_angle = 0.0
        delta_distance = target_distance - distance
        delta_angle = target_angle - angle
        kp_distance = -0.5 #-0.5
        kp_angle = 0.5
        linear_vel_x = kp_distance * delta_distance
        angular_vel = kp_angle * delta_angle
        velocity_msg.linear.x = linear_vel_x
        velocity_msg.linear.y = rospy.get_param("/bath/linear_vel_y")
        velocity_msg.angular.z = angular_vel
        self.cmdPub.publish(velocity_msg)
        
    def fit_line(self, X, y, ransac):
        pts_cnt = len(X)
        X = np.array(X).reshape(-1, 1)
        y = np.array(y).reshape(-1, 1)

        if ransac:
            # Robustly fit linear model with RANSAC algorithm
            ransac = linear_model.RANSACRegressor()
            ransac.fit(X, y)
            inlier_mask = ransac.inlier_mask_
            outlier_mask = np.logical_not(inlier_mask)

            # Predict data of estimated models
            line_X = np.arange(X.min(), X.max())[:, np.newaxis]
            line_y_ransac = ransac.predict(line_X)

            coef = ransac.estimator_.coef_
            intercept = ransac.estimator_.intercept_
        
        else:
            # Fit line using all data
            lr = linear_model.LinearRegression()
            lr.fit(X, y)
        
            # Predict data of estimated models
            line_X = np.arange(X.min(), X.max())[:, np.newaxis]
            line_y = lr.predict(line_X)

            coef = lr.coef_
            intercept = lr.intercept_
        
        return intercept , coef
        

if __name__ == '__main__':
    rospy.init_node("BathroomNvg")
    l2pc = BathNvg()
    rospy.spin()