#!/usr/bin/env python3

from putarm_ur3e_utils.srv import GrabObj, GrabObjResponse
import sys
import copy
import rospy
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from ur_msgs.srv import SetIO

import numpy as np
import rosservice


class GrabObject:
    def __init__(self):
        super(GrabObject,self).__init__()
        #
        s = rospy.Service('grab_object_server', GrabObj, self.grab_object)
        rospy.loginfo("Ready to grab")
        rospy.spin()

    def grab_object(self,req):
        pins = req.pins
        if len(pins) >2:
            return GrabObjResponse("wrong lenght of pins. no action executed")
        for p in pins:
            if p != 16 and p != 17:
              return GrabObjResponse("wrong pins. no action executed")
        states = req.states
        for s in states:
            if s != 1 and s !=0:
              return GrabObjResponse("wrong state values. no action executed")
        print(pins)
        print(states)
        set_io_prox = rospy.ServiceProxy("/ur_hardware_interface/set_io", SetIO)
        out = set_io_prox(1,pins[0],states[0])
        print(out)
        out = set_io_prox(1,pins[1],states[1])
        print(out)
        #rosservice.call_service("/ur_hardware_interface/set_io",[1,pins[0],states[0]])
        #rosservice.call_service("/ur_hardware_interface/set_io",[1,pins[1],states[1]])

        return GrabObjResponse("action executed")



def main():
    try:
        rospy.init_node('grab_object_server')
        gb = GrabObject()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
