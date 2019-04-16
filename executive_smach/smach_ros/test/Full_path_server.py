#!/usr/bin/env python
from smach_ros.srv import *
import rospy

def Find_Full_Path(request):
    return full_pathResponse([0,1,2])

def Find_Full_Path_server():
    rospy.init_node('full_path_server')
    s = rospy.Service('Full_Path', full_path, Find_Full_Path)
    rospy.spin()

if __name__ == "__main__":
    Find_Full_Path_server()