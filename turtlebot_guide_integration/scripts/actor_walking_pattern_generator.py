#!/usr/bin/env python

import rospy
from nav_msgs.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('move_base/make_plan')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
