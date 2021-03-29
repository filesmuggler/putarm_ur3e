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

from putarm_ur3e_utils.srv import GoHome,GoHomeResponse
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

class GotoHome(object):
    def __init__(self):
        super(GotoHome,self).__init__()
        rospy.init_node('go_home_server')
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.planning_group_name = "manipulator"

        self.planning_move_group = moveit_commander.MoveGroupCommander(self.planning_group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.planning_frame = self.planning_move_group.get_planning_frame()
        self.eef_link = self.planning_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        s = rospy.Service('go_home_server', GoHome, self.go_home)
        rospy.loginfo("Ready to go home")
        rospy.spin()

    def set_quat_from_axis_angle(self,angle,axis):
        s = sin(angle/2)
        x = axis[0] * s
        y = axis[1] * s
        z = axis[2] * s
        w = cos(angle/2)
        return [x,y,z,w]

    def degrees_to_radians(self,degrees):
        radians = 2*pi*degrees/360
        return radians

    def go_home(self, req):
        move_group = self.planning_move_group
        angle_vector_deg = req.angle_vector
        radian_vector = [0,0,0,0,0,0]
        for i in range(len(angle_vector_deg)):
            radian_vector[i] = self.degrees_to_radians(angle_vector_deg[i])

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = radian_vector[0]
        joint_goal[1] = radian_vector[1]
        joint_goal[2] = radian_vector[2]
        joint_goal[3] = radian_vector[3]
        joint_goal[4] = radian_vector[4]
        joint_goal[5] = radian_vector[5]
      
        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        all_close(joint_goal, current_joints, 0.01)
        return GoHomeResponse("executed go to home position")

def main():
    try:
       go_home_server = GotoHome()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

# rosservice call /go_home_server "angle_vector: [-45,-55,-96,-70,55,-150]"