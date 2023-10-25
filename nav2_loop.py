#!/usr/bin/env python3
#This script moves the TB3 between goal pos1 and goal pos2 endlessly.
# JB - 25 Oct 2023

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator:BasicNavigator, position_x, position_y, rotation_z):
  q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = position_x
  goal_pose.pose.position.y = position_y
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = q_x
  goal_pose.pose.orientation.y = q_y
  goal_pose.pose.orientation.z = q_z
  goal_pose.pose.orientation.w = q_w
  return goal_pose

def main():
  #--init
  rclpy.init()
  nav = BasicNavigator()
  nav.waitUntilNav2Active()

  while True:
  #send to goal
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 3.14)
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
  
  #Goto pose
    nav.goToPose(goal_pose1)
    while not nav.isTaskComplete():
      feedback = nav.getFeedback()
      #print(feedback)
    print(nav.getResult())

    nav.goToPose(goal_pose2)
    while not nav.isTaskComplete():
      feedback = nav.getFeedback()
      #print(feedback)
    print(nav.getResult())

  #-- shutdown
  rclpy.shutdown()  

if __name__=='__main__':
  main()
