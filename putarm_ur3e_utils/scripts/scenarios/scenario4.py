#!/usr/bin/env python3

from putarm_ur3e_utils.srv import GrabObj, GrabObjResponse
from putarm_ur3e_utils.srv import GoHome, GoHomeResponse
from putarm_ur3e_utils.srv import GoToObj,GoToObjResponse
from lssn.srv import *
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

import tf2_ros
import tf2_geometry_msgs 
from tf.transformations import quaternion_from_euler

import numpy as np
import rosservice

def main():
    try:
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) #tf buffer length
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        tf_transform = tf_buffer.lookup_transform("world",
                                    "dumb_link_2", #source frame
                                    rospy.Time(0), #get the tf at first available time
                                    rospy.Duration(4.0))
        pose_transformed = tf2_geometry_msgs.do_transform_pose(bottle_pose, tf_transform)  
        print(pose_transformed)

    except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
        print("tf not working")

if __name__ == "__main__":
    main()

