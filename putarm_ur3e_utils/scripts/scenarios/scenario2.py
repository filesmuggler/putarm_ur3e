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
import message_filters
import tf

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
    rospy.init_node('scenario2')
    home_prox(angle_vector=[-60,-70,-90,-80,76,-152])
    grab_prox(pins=[16,17],states=[0,0])
    rospy.sleep(wait_delay)
    listener = tf.TransformListener()
    bottle_pose = geometry_msgs.msg.Pose()
    rate = rospy.Rate(10.0)
    counter = 30
    while counter>0:
        try:
            (trans,rot) = listener.lookupTransform('/world', '/bottle', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #print(trans,rot)
        bottle_pose.position.x = trans[0]
        bottle_pose.position.y = trans[1]
        bottle_pose.position.z = trans[2]

        bottle_pose.orientation.x = rot[0]
        bottle_pose.orientation.y = rot[1]
        bottle_pose.orientation.z = rot[2]
        bottle_pose.orientation.w = rot[3]
        counter = counter - 1

        rate.sleep()

    print(bottle_pose)
    grab_prox(pins=[16,17],states=[0,0])
    home_prox(angle_vector=[-60,-70,-90,-80,76,-152])
    rospy.sleep(wait_delay)
    bottle_pose.position.z += 0.17
    goto_prox(object_coordinates=bottle_pose)
    rospy.sleep(wait_delay)
    bottle_pose.position.z -= 0.05
    goto_prox(object_coordinates=bottle_pose)
    grab_prox(pins=[16,17],states=[1,1])
    rospy.sleep(2*wait_delay)
    bottle_pose.position.z += 0.05
    goto_prox(object_coordinates=bottle_pose)
    rospy.sleep(wait_delay)
    home_prox(angle_vector=[-60,-70,-90,-80,76,-152])
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
