#!/usr/bin/env python
import roslib; roslib.load_manifest('smach_ros')
import rospy
from smach_ros.srv import *
import time
import smach
import smach_ros

cur_path = []
reached = False
# define state Foo
class Base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Loading','Ready'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Base')
        rospy.wait_for_service('Full_Path')
        try:
            full_path_service = rospy.ServiceProxy('Full_Path', full_path)
            resp = full_path_service()
            global cur_path
            cur_path = list(resp.path)
            print(cur_path)
            return "Ready"
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        


# define state Bar
class InTransit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reached','wait'])

    def execute(self, userdata):
        rospy.loginfo('Executing state InTransit')
        global reached
        if reached == True:
            reached = False
            return 'reached'
        else:
            time.sleep(0.5)
            return 'wait'

class AtTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done','finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AtTable')
        global cur_path
        print(cur_path[0])
        if cur_path[0]==1:
            return 'finished'
        else:
            cur_path.pop(0)
            return 'Done'    

def Reach_Table(request):
    global reached
    reached = True
    return reach_tableResponse()

def reach_table_server():
    #rospy.init_node('reach_table_server')
    s = rospy.Service('reach_table', reach_table, Reach_Table)
    rospy.spin()



def main():
    rospy.init_node('smach_example_state_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Base', Base(), 
                               transitions={'Loading':'Base', 'Ready':'InTransit'})
        smach.StateMachine.add('InTransit', InTransit(), 
                               transitions={'reached':'AtTable', 'wait':'InTransit'})
        smach.StateMachine.add('AtTable', AtTable(), 
                               transitions={'Done':'InTransit','finished':'outcome4'})
    reach_table_server()
    # Execute SMACH plan
    outcome = sm.execute()
    
    



if __name__ == '__main__':
    main()