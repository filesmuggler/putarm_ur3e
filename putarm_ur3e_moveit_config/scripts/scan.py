#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Krzysztof Stezala


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf
import time

from geometry_msgs.msg import PoseStamped


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

   
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    
    planning_frame = move_group.get_planning_frame()
    print ("============ Planning frame: %s" % planning_frame)

    
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print ("============ Printing robot state")
    print (robot.get_current_state())
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def degrees_to_radians(self,degrees):
      radians = 2*pi*degrees/360
      return radians

  def go_to_joint_state_ur3e(self, angle_vector):
    move_group = self.move_group
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = angle_vector[0]
    joint_goal[1] = angle_vector[1]
    joint_goal[2] = angle_vector[2]
    joint_goal[3] = angle_vector[3]
    joint_goal[4] = angle_vector[4]
    joint_goal[5] = angle_vector[5]

    move_group.go(joint_goal, wait=True)

    move_group.stop()

    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def set_quat_from_axis_angle(self,angle,axis):
    s = sin(angle/2)
    x = axis[0] * s
    y = axis[1] * s
    z = axis[2] * s
    w = cos(angle/2)
    return [x,y,z,w]

  
  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


def main():
  try:
    tutorial = MoveGroupPythonIntefaceTutorial()
    print("============ Robot go to scan position ...")
    angle_vector = [-45,-55,-96,-70,55,-150]
    radian_vector = [0,0,0,0,0,0]
    for i in range(len(angle_vector)):
        radian_vector[i] = tutorial.degrees_to_radians(angle_vector[i])
    tutorial.go_to_joint_state_ur3e(radian_vector)
    angle_vector = [0,-55,-96,-70,55,-150]
    radian_vector = [0,0,0,0,0,0]
    for i in range(len(angle_vector)):
        radian_vector[i] = tutorial.degrees_to_radians(angle_vector[i])
    tutorial.go_to_joint_state_ur3e(radian_vector)
    angle_vector = [-45,-55,-96,-70,55,-150]
    radian_vector = [0,0,0,0,0,0]
    for i in range(len(angle_vector)):
        radian_vector[i] = tutorial.degrees_to_radians(angle_vector[i])
    tutorial.go_to_joint_state_ur3e(radian_vector)
    angle_vector = [-90,-55,-96,-70,55,-150]
    radian_vector = [0,0,0,0,0,0]
    for i in range(len(angle_vector)):
        radian_vector[i] = tutorial.degrees_to_radians(angle_vector[i])
    tutorial.go_to_joint_state_ur3e(radian_vector)
    angle_vector = [-45,-55,-96,-70,55,-150]
    radian_vector = [0,0,0,0,0,0]
    for i in range(len(angle_vector)):
        radian_vector[i] = tutorial.degrees_to_radians(angle_vector[i])
    tutorial.go_to_joint_state_ur3e(radian_vector)
    print ("============ Robot arm in default position ...")
    

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
