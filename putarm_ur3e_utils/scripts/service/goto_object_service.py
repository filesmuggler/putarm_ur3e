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

from putarm_ur3e_utils.srv import GoToObj,GoToObjResponse
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import tf

import numpy as np


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


class GotoObject(object):
    def __init__(self):
        super(GotoObject,self).__init__()
        rospy.init_node('goto_object_server')
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
        
        s = rospy.Service('goto_object_server', GoToObj, self.goto_object)
        rospy.loginfo("Ready to goto")
        rospy.spin()

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0       # jump_threshold
                                        )         

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.planning_move_group
        move_group.execute(plan, wait=True)

    def goto_object(self,req):
        goal_position = req.object_coordinates.position
        #goal_position.z -= 0.05

        current_pose = self.planning_move_group.get_current_pose().pose
        current_position = current_pose.position

        no_samples = 5

        x_linspace = np.linspace(current_position.x,goal_position.x,num=no_samples)[1:]
        y_linspace = np.linspace(current_position.y,goal_position.y,num=no_samples)[1:]
        z_linspace = np.linspace(current_position.z,goal_position.z,num=no_samples)[1:]

        waypoints = []
        new_pose = geometry_msgs.msg.Pose()
        
        for i in range(no_samples-1):
            new_pose.position.x = x_linspace[i]
            new_pose.position.y = y_linspace[i]
            new_pose.position.z = z_linspace[i]
            new_pose.orientation = req.object_coordinates.orientation
            waypoints.append(copy.deepcopy(new_pose))       

        (plan, fraction) = self.planning_move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    avoid_collisions=True)        
        self.planning_move_group.execute(plan,wait=True)

        self.planning_move_group.stop()

        #current_joints = self.planning_move_group.get_current_joint_values()
        #all_close(joint_goal, current_joints, 0.01)

        return GoToObjResponse("executed go to object position")

def main():
    try:
       goto_object_server = GotoObject()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()

# rosservice call /goto_object_service "object_coordinates:
#   position:
#     x: 0.20
#     y: 0.69
#     z: 0.23
#   orientation:
#     x: 0.0
#     y: 0.9999997
#     z: 0.0
#     w: 0.0007963"
