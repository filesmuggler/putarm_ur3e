#!/usr/bin/env python3
from putarm_ur3e_moveit_config.srv import GoToObj,GoToObjResponse
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
        
        self.moveit_commander.roscpp_initialize(sys.argv)
        self.robot = self.moveit_commander.RobotCommander()
        self.scene = self.moveit_commander.PlanningSceneInterface()

        self.planning_group_name = "manipulator"

        self.planning_move_group = self.moveit_commander.MoveGroupCommander(self.planning_group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.planning_frame = self.planning_move_group.get_planning_frame()
        self.eef_link = self.planning_move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        s = rospy.Service('goto_object_service', GoToObj, self.goto_object)
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
        goal_position = req.pose.position
        #goal_position.z -= 0.05

        current_pose = self.planning_move_group.get_current_pose().pose
        current_position = current_pose.position

        no_samples = 50

        x_linspace = np.linspace(current_position.x,goal_position.x,num=no_samples)
        y_linspace = np.linspace(current_position.y,goal_position.y,num=no_samples)
        z_linspace = np.linspace(current_position.z,goal_position.z,num=no_samples)

        waypoints = []
        new_pose = current_pose
        new_pose.orientation = req.pose.orientation

        for i in range(no_samples):
            new_pose.position.x = x_linspace[i]
            new_pose.position.y = y_linspace[i]
            new_pose.position.z = z_linspace[i]
            waypoints.append(copy.deepcopy(new_pose))

        (plan, fraction) = self.planning_move_group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0,         # jump_threshold
                                    avoid_collisions=True)        
        output = self.planning_move_group.execute(plan,wait=True)

        return GoToObjResponse(output)
