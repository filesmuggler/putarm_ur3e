#!/usr/bin/env python3

from putarm_ur3e_utils.srv import GrabObj, GrabObjResponse
from putarm_ur3e_utils.srv import GoHome, GoHomeResponse
from putarm_ur3e_utils.srv import GoToObj,GoToObjResponse
import sys
import copy
import rospy
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from ur_msgs.srv import SetIO
import geometry_msgs

import numpy as np
import rosservice

grab_prox = rospy.ServiceProxy("/grab_object_server", GrabObj)
home_prox = rospy.ServiceProxy("/go_home_server", GoHome)
goto_prox = rospy.ServiceProxy("/goto_object_server",GoToObj)

pose_1 = geometry_msgs.msg.Pose()
pose_1.position.x = 0.1
pose_1.position.y = 0.79
pose_1.position.z = 0.50
pose_1.orientation.x = 0.0
pose_1.orientation.y = 0.9999997
pose_1.orientation.z = 0.0
pose_1.orientation.w = 0.0007963

pose_1_b = geometry_msgs.msg.Pose()
pose_1_b.position.x = 0.1
pose_1_b.position.y = 0.79
pose_1_b.position.z = 0.42
pose_1_b.orientation.x = 0.0
pose_1_b.orientation.y = 0.9999997
pose_1_b.orientation.z = 0.0
pose_1_b.orientation.w = 0.0007963

pose_2 = geometry_msgs.msg.Pose()
pose_2.position.x = -0.1
pose_2.position.y = 0.79
pose_2.position.z = 0.50
pose_2.orientation.x = 0.0
pose_2.orientation.y = 0.9999997
pose_2.orientation.z = 0.0
pose_2.orientation.w = 0.0007963

pose_2_b = geometry_msgs.msg.Pose()
pose_2_b.position.x = -0.1
pose_2_b.position.y = 0.79
pose_2_b.position.z = 0.42
pose_2_b.orientation.x = 0.0
pose_2_b.orientation.y = 0.9999997
pose_2_b.orientation.z = 0.0
pose_2_b.orientation.w = 0.0007963

wait_delay = 0.2

def main():
    grab_prox(pins=[16,17],states=[0,0])
    home_prox(angle_vector=[-45,-55,-96,-70,55,-150])
    while(1):        
        home_prox(angle_vector=[-45,-55,-96,-70,55,-150])

        goto_prox(object_coordinates=pose_1)
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_1_b)
        grab_prox(pins=[16,17],states=[1,1])
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_1)
        rospy.sleep(wait_delay)

        goto_prox(object_coordinates=pose_2)
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_2_b)
        grab_prox(pins=[16,17],states=[0,0])
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_2)
        rospy.sleep(wait_delay)

        home_prox(angle_vector=[-45,-55,-96,-70,55,-150])

        goto_prox(object_coordinates=pose_2)
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_2_b)
        grab_prox(pins=[16,17],states=[1,1])
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_2)
        rospy.sleep(wait_delay)

        goto_prox(object_coordinates=pose_1)
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_1_b)
        grab_prox(pins=[16,17],states=[0,0])
        rospy.sleep(wait_delay)
        goto_prox(object_coordinates=pose_1)
        rospy.sleep(wait_delay)


if __name__ == "__main__":
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
