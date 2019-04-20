#!/usr/bin/env python
import roslib; roslib.load_manifest('smach_ros')
import rospy
from smach_ros.srv import *
import time
import smach
import smach_ros
from std_msgs.msg import String
from smach_ros.msg import *
cur_path = []
global reached 
reached = False
global ready_to_go 
ready_to_go= False
global transit_count
# define state Foo
global attable_count
pub = rospy.Publisher('robot_state', robot_state, queue_size=10)
class Base(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Loading','Ready'])

    def execute(self, userdata):
        cur_state = robot_state()
        cur_state.state = "AtBase"
        rospy.loginfo('Executing state Base')
        rospy.wait_for_service('Full_Path')
        try:
            full_path_service = rospy.ServiceProxy('Full_Path', full_path)
            resp = full_path_service()
            global cur_path
            cur_path = list(resp.path)
            cur_state.next_goal = cur_path[0]
            pub.publish(cur_state)
            #print(cur_path)
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
        global transit_count
        transit_count = 0        
        rospy.Subscriber("Transit_status", String, self.callback)
        cur_state = robot_state()
        cur_state.state = "InTransit"
        cur_state.next_goal = cur_path[0]
        if reached == True:
            transit_count = 1
            reached = False
            return 'reached'
        else:
            pub.publish(cur_state)
            time.sleep(0.1)
            return 'wait'
    def callback(self,data):
        global reached
        global transit_count
        if data.data == "Done" and transit_count ==0:
            reached = True

class AtTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait','Done','finished'])

    def execute(self, userdata):
        rospy.loginfo('Executing state AtTable')
        rospy.Subscriber("serve_status", String, self.callback)
        global attable_count
        global reached
        global ready_to_go
        global cur_path
        cur_state = robot_state()
        cur_state.state = "AtTable"
        if len(cur_path) >1:
            cur_state.next_goal = cur_path[0]
        else:
            cur_path.next_goal = -1
        pub.publish(cur_state)
        attable_count = 0
        if ready_to_go == False:
            time.sleep(0.1)
            return 'wait'
        attable_count = 1
        ready_to_go = False
        print(cur_path[0])
        if cur_path[0]==1:
            return 'finished'
        else:

            cur_path.pop(0)
            return 'Done'
    def callback(self,data):
        global ready_to_go
        if data.data == "Done" and attable_count == 0:
            ready_to_go = True    


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
                               transitions={'wait':'AtTable','Done':'InTransit','finished':'outcome4'})
    #reach_table_server()
    # Execute SMACH plan
    outcome = sm.execute()
    
    



if __name__ == '__main__':
    main()