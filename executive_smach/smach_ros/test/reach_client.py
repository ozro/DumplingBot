#!/usr/bin/env python

import sys
import rospy
from smach_ros.srv import *

def reach_table_client():
    rospy.wait_for_service('reach_table')
    try:
        reach_table = rospy.ServiceProxy('reach_table', Reach_Table)
        resp1 = reach_table()
        return 42;
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    reach_table_client()
