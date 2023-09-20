#!/usr/bin/env python3

#***********************************

# Property of Hochschule Darmstadt
# Team Project Summer Semester 2023

#***********************************

# Import necessary libraries
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from actionlib_msgs.msg import GoalID

import rospy
from visualization_msgs.msg import Marker


class navigation_on_map():
   def __init__(self):
      self.cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
      self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

      #define a client to send goal requests to the move_base server through a SimpleActionClient
      self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
      #wait for the action server to come up
      self.move_base_client.wait_for_server()


   def move2pose(self, x_goal,y_goal,theta_goal):
      # validate pose
      # TODO

      # build msg
      goal = MoveBaseGoal()
      goal.target_pose.header.frame_id = "map"
      goal.target_pose.header.stamp = rospy.Time.now()
      goal.target_pose.pose.position =  Point(x_goal,y_goal,0)
      quaternion = quaternion_from_euler(0,0, theta_goal) 
      goal.target_pose.pose.orientation.x = quaternion[0]
      goal.target_pose.pose.orientation.y = quaternion[1]
      goal.target_pose.pose.orientation.z = quaternion[2]
      goal.target_pose.pose.orientation.w = quaternion[3]

      # validate amcl pose
      # TODO

      # publish target marker to rviz
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.header.stamp = rospy.Time.now()
      # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
      marker.type = 0
      marker.id = 0
      # Set the scale of the marker
      marker.scale.x = 1.0
      marker.scale.y = 0.10
      marker.scale.z = 0.10
      # Set the color
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 0.0
      marker.color.a = 1.0
      # Set the pose of the marker
      marker.pose.position.x = x_goal
      marker.pose.position.y = y_goal
      marker.pose.position.z = 0.1
      marker.pose.orientation.x = quaternion[0]
      marker.pose.orientation.y = quaternion[1]
      marker.pose.orientation.z = quaternion[2]
      marker.pose.orientation.w = quaternion[3]
      self.marker_pub.publish(marker)

      # move 2 pose (blocking)
      self.move_base_client.send_goal(goal)
      self.move_base_client.wait_for_result()

       # calculate deviation from target pose

      # return with result
      if(self.move_base_client.get_state() ==  GoalStatus.SUCCEEDED):
         rospy.loginfo("Reached the destination")
         return True
      else:
         rospy.loginfo("The robot failed to reach the destination")
         return False
      
   def cancel(self):
      # This doesnt work at al? at least when i envoke ctrl + c in terminal goal is beeing move to (function is called)
      print("Called cleanup")
      self.move_base_client.cancel_all_goals()
      self.move_base_client.cancel_goal()

      cancel_msg = GoalID()
      self.cancel_pub.publish(cancel_msg)      
        
   # Uncomment below to run the script for standalone purpose
# if __name__ == '__main__':
#    rospy.init_node('autonomous_navigation')
#    auto_nav = navigation_on_map()
#    auto_nav.move2pose(-0.4663,-2.0768, -2.8801)
#    print("move fast")
#    rospy.sleep(5)
#    auto_nav.move2pose(-0.4663,-2.0768, -2.8801)
#    # rospy.spin()






        
         
          
           




