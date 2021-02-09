#!/usr/bin/env python

import roslib; roslib.load_manifest('uros_controller')
import rospy
import smach
from smach import StateMachine
import smach_ros
from smach_ros import IntrospectionServer, SimpleActionState
import actionlib
from std_msgs.msg import Empty, Int8, Int32
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from uros_controller.msg import Uros
from tf.transformations import quaternion_from_euler
import time
import sys

from signal import signal, SIGINT
from sys import exit

#Real warehouse
waypoint_start = [3.5,6.2]
waypoint_start_rpy = [0,0,3.14/2]
waypoint_humidity = [1.9,1.0]
waypoint_humidity_rpy = [0, 0, 0]
waypoint_door_inside = [14.5,1.5]
waypoint_door_inside_rpy = [0, 0, 0]
waypoint_door_outside = [18.5,1.5]
waypoint_door_outside_rpy = [0, 0, 0]
waypoint_light = [32,3]
waypoint_light_rpy = [0, 0, 0]

#Simplified for tests
waypoint_start = [2.0,1.0]
waypoint_start_rpy = [0, 0, 0]
waypoint_humidity = [2.1,1.0]
waypoint_humidity_rpy = [0, 0, 0]
waypoint_door_inside = [2.2,1.0]
waypoint_door_inside_rpy = [0, 0, 0]
waypoint_door_outside = [2.3,1.0]
waypoint_door_outside_rpy = [0, 0, 0]
waypoint_light = [2.4,1.0]
waypoint_light_rpy = [0, 0, 0]

global_wait_on_start = True
global_test_assume_goal_reached = True


def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.
    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).
    The "answer" return value is one of "yes" or "no".
    """
    valid = {"yes":True,   "y":True,  "ye":True,
             "no":False,     "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")

def clear_costmap():
    #rosservice call move_base/clear_costmaps
    rospy.wait_for_service('move_base/clear_costmaps')
    try:
       ret = rospy.ServiceProxy('move_base/clear_costmaps', Empty)
       ret()
       print('Costmap cleared before task')
    except rospy.ServiceException as e:
       print("Service call failed: %s"%e)


class MoveBaseClient:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()

    def send_goal(self, goal):
        self.client.send_goal(goal)
        
    def wait_for_result(self):
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            print("Something is wrong")
            return self.client.get_result()
        else:
            print("Got result form move_base")
            return self.client.get_result()
    
    def get_state(self) :
        return self.client.get_state()
    
    def get_status_text(self) :
        return self.client.get_goal_status_text()

class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # Your state initialization goes here
        
    def execute(self, userdata):
        # Your state execution goes here
        # get UWB position  wait to receive
        # mission 1 or mission 2
        #clear_costmap()
        if global_wait_on_start:
            if not query_yes_no("Start mission?"):
                return 'failure'
        return 'next'
        
class NAVIGATE_TO_HUMIDITY_SENSOR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init

    def execute(self, userdata):
        ## Navigate to the humidity sensor getting the pos information
        if global_wait_on_start:
            if not query_yes_no("Go to humidity sensor?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            print("Go to humidity sensor!")
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_humidity[0]
            goal.target_pose.pose.position.y = waypoint_humidity[1]
            quat = quaternion_from_euler(waypoint_humidity_rpy[0], waypoint_humidity_rpy[1], waypoint_humidity_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <= 3:
                print("Got humidity position: " + str(state))
                break
            else:
                print("Humidity position error: " + str(state))
                if query_yes_no("Repeat state?"):
                    continue
                else:
                    return 'failure'
        print("Move_base result: " + movebase.get_status_text())
        
        return 'next'

class GET_HUMIDITY_SENSOR_VAL(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'], output_keys=['temperature','humidity'])
        # State init depending on the mission

    def execute(self, userdata):
        # Read the vlalue from the 6lowpan channels humidity sensor
        if global_wait_on_start:
            if not query_yes_no("Read humidity sensor?"):
                return 'failure'
        timeout = 60
        timeout_start = time.time()
        userdata.temperature = "reading..."
        userdata.humidity = "reading..."
        while True:
            try:
                print("Wait for humidity measurement")
                humidity = rospy.wait_for_message("/humidity", Int32, timeout=5)
                temperature = rospy.wait_for_message("/temperature", Int32, timeout=5)
                userdata.temperature = str(temperature.data)
                userdata.humidity = str(humidity.data)
                print("Got humidity sensor readings: humidity: " + str(humidity) + ", temperature: " + str(temperature))
                return 'next'
            except rospy.ROSException:
                print("Can not get humidity readings - timeout")
            if time.time() > (timeout_start + timeout):
                print("Humidity sensor timeout - failure")
                if query_yes_no("Wait more time?"):
                    timeout_start = time.time()
                    continue
                else:
                    return 'failure'
        
        if got_result == 0:
            return 'failure'
        
        print("Got humidity sensor readings: humidity: " + str(humidity) + ", temperature: " + str(temperature))
        return 'next'

class NAVIGATE_TO_DOOR_OPENER_IN(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init etc...

    def execute(self, userdata):
        # Read the value
        if global_wait_on_start:
            if not query_yes_no("Go to the door inside warehouse?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_door_inside[0]
            goal.target_pose.pose.position.y = waypoint_door_inside[1]
            quat = quaternion_from_euler(waypoint_door_inside_rpy[0], waypoint_door_inside_rpy[1], waypoint_door_inside_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <= 3:
                print("Got door opener position: " + str(state))
                break
            else:
                print("Door opener error: " + str(state))
                if query_yes_no("Wait more time?"):
                    continue
                else:
                    return 'failure'
            print("Move_base result: " + movebase.get_status_text())
        
        return 'next'

class NAVIGATE_TO_DOOR_OPENER_IN_TO_OPEN(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init etc...

    def execute(self, userdata):
        # Read the value
        if global_wait_on_start:
            if not query_yes_no("Go to the door inside warehouse?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_door_inside[0]
            goal.target_pose.pose.position.y = waypoint_door_inside[1]
            quat = quaternion_from_euler(waypoint_door_inside_rpy[0], waypoint_door_inside_rpy[1], waypoint_door_inside_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <= 3:
                print("Got door opener position: " + str(state))
                break
            else:
                print("Door opener error: " + str(state))
                if query_yes_no("Wait more time?"):
                    continue
                else:
                    return 'failure'
            print("Move_base result: " + movebase.get_status_text())
        
        return 'next'

class NAVIGATE_TO_DOOR_OPENER_OUT_BEFORE_DOOR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init etc...

    def execute(self, userdata):
        # read position
        if global_wait_on_start:
            if not query_yes_no("Go to the door outside warehouse?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_door_outside[0]
            goal.target_pose.pose.position.y = waypoint_door_outside[1]
            quat = quaternion_from_euler(waypoint_door_outside_rpy[0], waypoint_door_outside_rpy[1], waypoint_door_outside_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <=3:
                print("Got door opener position outside: " + str(state))
                break
            else:
                print("Door opener outside error: " + str(state))
                if query_yes_no("Repeat state?"):
                    continue
                else:
                    return 'failure'
        print("Move_base result: " + movebase.get_status_text())
        
        return 'next'

class ACTIVATE_DOOR_OPENER(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'], output_keys=['opener_status'])
        # State init etc...

    def execute(self, userdata):
        # Write the value to open the door.
        if global_wait_on_start:
            if not query_yes_no("Open the door?"):
                return 'failure'
        print("Open door - send topic to uROS")
        pub = rospy.Publisher('/opener_cmd', Int8, queue_size=1)
        status = 0
        opener_started = 0
        timeout = 60
        timeout_start = time.time()
        userdata.opener_status = "reading..."
        print("Wait for door opener status")
        while status != Int8(0) or opener_started != 1:
            if opener_started != 1:
                #try to send topic to door opener until got status == running
                print("Door opener - trying to send command open")
                connections = pub.get_num_connections()
                if connections > 0:
                    pub.publish(1)
                    print("Door opener - sent command open")

            try:
                status = rospy.wait_for_message("/opener_status", Int8, timeout=5)
                print("Got door opener status: " + str(status) + ", opener started? :" + str(opener_started))
                if status != Int8(0):
                    opener_started = 1
                    userdata.opener_status = "opening..."
            except rospy.ROSException:
                print("Can not get door opener status - timeout, try to wait for next message")
            
            if time.time() > (timeout_start + timeout):
                print("Door open timeout - failure")
                if query_yes_no("Wait more time?"):
                    timeout_start = time.time()
                    continue
                else:
                    return 'failure'
        
        print("Door opened!")
        userdata.opener_status = "opened"
        return 'next'

class INACTIVATE_DOOR_OPENER(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'], output_keys=['opener_status'])
        # State init etc...

    def execute(self, userdata):
        # Write the value to open the door.
        if global_wait_on_start:
            if not query_yes_no("Close the door?"):
                return 'failure'
        print("Open door - send topic to uROS")
        pub = rospy.Publisher('/opener_cmd', Int8, queue_size=1)
        status = 0
        opener_started = 0
        timeout = 60
        timeout_start = time.time()
        userdata.opener_status = "reading..."
        print("Wait for door opener status")
        while status != Int8(0) or opener_started != 1:
            if opener_started != 1:
                #try to send topic to door opener until got status == running
                print("Door opener - trying to send command close")
                connections = pub.get_num_connections()
                if connections > 0:
                    pub.publish(2)
                    print("Door opener - sent command close")

            try:
                status = rospy.wait_for_message("/opener_status", Int8, timeout=5)
                print("Got door opener status: " + str(status) + ", opener started? :" + str(opener_started))
                if status != Int8(0):
                    opener_started = 1
                    userdata.opener_status = "closing..."
            except rospy.ROSException:
                print("Can not get door opener status - timeout, try to wait for next message")
            
            if time.time() > (timeout_start + timeout):
                print("Door open timeout - failure")
                if query_yes_no("Wait more time?"):
                    timeout_start = time.time()
                    continue
                else:
                    return 'failure'
        
        print("Door closed!")
        userdata.opener_status = "closed"
        return 'next'

class READ_LIDAR(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'], output_keys=['lidar_distance'])
        # State init depending on the mission

    def execute(self, userdata):
        # Read the lidar value to check if there is an obstacle or not.
        if global_wait_on_start:
            if not query_yes_no("Read distance sensor?"):
                return 'failure'
        timeout = 60
        min_dist = 100 #cm
        timeout_start = time.time()
        userdata.lidar_distance = "reading..."
        while True:
            try:
                print("Wait for distance measurement")
                distance = rospy.wait_for_message("/distance", Int32, timeout=5)
                print("Measured distance:" + str(distance.data))
                userdata.lidar_distance = str(distance.data) + "cm"
                if(distance.data < min_dist):
                    if not query_yes_no("Probably detected obstacle. Continue?"):
                        timeout_start = time.time()
                    else:
                        print("Area is not clear, but accepted by user!")
                        return 'next'
                else:
                    print("Area is clear")
                    return 'next'
            except rospy.ROSException:
                print("Can not get distnace readings - timeout")
            if time.time() > (timeout_start + timeout):
                print("Distance sensor timeout - failure")
                if query_yes_no("Wait more time?"):
                    timeout_start = time.time()
                    continue
                else:
                    return 'failure'

class NAVIGATE_TO_DOOR_OPENER_OUT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init etc...

    def execute(self, userdata):
        # read position
        if global_wait_on_start:
            if not query_yes_no("Go to the door outside warehouse?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_door_outside[0]
            goal.target_pose.pose.position.y = waypoint_door_outside[1]
            quat = quaternion_from_euler(waypoint_door_outside_rpy[0], waypoint_door_outside_rpy[1], waypoint_door_outside_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <=3:
                print("Got door opener position outside: " + str(state))
                break
            else:
                print("Door opener outside error: " + str(state))
                if query_yes_no("Repeat state?"):
                    continue
                else:
                    return 'failure'
        print("Move_base result: " + movebase.get_status_text())
        
        return 'next'


class NAVIGATE_TO_START_POS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init

    def execute(self, userdata):
        # read position
        return 'next'

class NAVIGATE_TO_LIGHT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'])
        # State init

    def execute(self, userdata):
        # read position
        if global_wait_on_start:
            if not query_yes_no("Go to light?"):
                return 'failure'
        while True:
            if global_test_assume_goal_reached:
                print("Assumed that goal is reached - only for test purposes")
                return 'next'
            clear_costmap()
            print("Wait for GPS fix")
            time.sleep(10.0) #to got GPS fix
            print("Go to lamp!")
            movebase = MoveBaseClient()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = waypoint_light[0]
            goal.target_pose.pose.position.y = waypoint_light[1]
            quat = quaternion_from_euler(waypoint_light_rpy[0], waypoint_light_rpy[1], waypoint_light_rpy[2])
            goal.target_pose.pose.orientation.x = quat[0]
            goal.target_pose.pose.orientation.y = quat[1]
            goal.target_pose.pose.orientation.z = quat[2]
            goal.target_pose.pose.orientation.w = quat[3]
            movebase.send_goal(goal)
            movebase.wait_for_result()
            state = movebase.get_state()
            if state <= 3:
                print("Got light position: " + str(state))
                break
            else:
                print("Humidity light error: " + str(state))
                if query_yes_no("Repeat state?"):
                    continue
                else:
                    return 'failure'
        print("Move_base result: " + movebase.get_status_text())
        
        return 'next'

class ACTIVATE_LIGHT(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next','failure'], output_keys=['final_effector_status'])

    def execute(self, userdata):
        if global_wait_on_start:
            if not query_yes_no("Switch on the light?"):
                return 'failure'
        print("Light - send topic to uROS")
        pub = rospy.Publisher('/final_effector_cmd', Int8, queue_size=1)
        status = 0
        timeout = 60
        timeout_start = time.time()
        userdata.final_effector_status = "reading..."
        print("Wait for final effector status")
        while status != Int8(1):
            connections = pub.get_num_connections()
            if connections > 0:
                pub.publish(1)
                print("Final effector - sent command switch on")

            try:
                status = rospy.wait_for_message("/final_effector_status", Int8, timeout=5)
                print("Got final effector status: " + str(status))
            except rospy.ROSException:
                print("Can not get final effector status - timeout, try to wait for next message")
            
            if time.time() > (timeout_start + timeout):
                print("Final effector timeout - failure")
                if query_yes_no("Wait more time?"):
                    timeout_start = time.time()
                    continue
                else:
                    return 'failure'
        
        print("Final effector switched on!")
        userdata.final_effector_status = "switched on"
        return 'next'

class SYSTEM_FINISH(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        # init finish system
        #

    def execute(self, userdata):
        # Show a green light mission finished or something
        print("Mission 2 completed!!!!") 
        print('Press CTRL-C to exit.')
        return 'next'

class SYSTEM_FAILURE(smach.State):
    def  __init__(self):
        smach.State.__init__(self, outcomes=['next'])
        # error init status.

    def execute(self, userdata):
        #Stop motor, Error display previous state, var etc....
        print("System failure. Stopped!")
        print('Press CTRL-C to exit.')
        return 'next'

def main():
    rospy.init_node('smach_node')
    rospy.loginfo("Starting SmachNode.")
    # movebase = MoveBaseClient()
    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = 2
    # goal.target_pose.pose.position.y = -1
    # goal.target_pose.pose.orientation.w = 1.0
    # movebase.send_goal(goal)
    # result = movebase.wait_for_result()
    # print(result)
    sm = smach.StateMachine(outcomes=['FINISH', 'ERROR'])
    # Execute SMACH plan
    with sm:
        smach.StateMachine.add('IDLE',
            IDLE(),
            transitions={'next':'NAVIGATE_TO_DOOR_OPENER_IN_TO_OPEN', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_DOOR_OPENER_IN_TO_OPEN',
            NAVIGATE_TO_DOOR_OPENER_IN_TO_OPEN(),
            transitions={'next':'ACTIVATE_DOOR_OPENER', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('ACTIVATE_DOOR_OPENER',
            ACTIVATE_DOOR_OPENER(),
            transitions={'next':'NAVIGATE_TO_DOOR_OPENER_OUT', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_DOOR_OPENER_OUT',
            NAVIGATE_TO_DOOR_OPENER_OUT(),
            transitions={'next':'READ_LIDAR', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('READ_LIDAR',
            READ_LIDAR(),
            transitions={'next':'NAVIGATE_TO_LIGHT', 'failure':'SYSTEM_FAILURE'})
                
        smach.StateMachine.add('NAVIGATE_TO_LIGHT',
            NAVIGATE_TO_LIGHT(),
            transitions={'next':'ACTIVATE_LIGHT', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('ACTIVATE_LIGHT',
                ACTIVATE_LIGHT(),
                transitions={'next':'NAVIGATE_TO_DOOR_OPENER_OUT_BEFORE_DOOR', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_DOOR_OPENER_OUT_BEFORE_DOOR',
            NAVIGATE_TO_DOOR_OPENER_OUT_BEFORE_DOOR(),
            transitions={'next':'NAVIGATE_TO_DOOR_OPENER_IN', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_DOOR_OPENER_IN',
            NAVIGATE_TO_DOOR_OPENER_IN_TO_OPEN(),
            transitions={'next':'INACTIVATE_DOOR_OPENER', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('INACTIVATE_DOOR_OPENER',
            INACTIVATE_DOOR_OPENER(),
            transitions={'next':'NAVIGATE_TO_HUMIDITY_SENSOR', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_HUMIDITY_SENSOR',
            NAVIGATE_TO_HUMIDITY_SENSOR(),
            transitions={'next':'GET_HUMIDITY_SENSOR_VAL', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('GET_HUMIDITY_SENSOR_VAL',
            GET_HUMIDITY_SENSOR_VAL(),
            transitions={'next':'NAVIGATE_TO_START_POS', 'failure':'SYSTEM_FAILURE'})

        smach.StateMachine.add('NAVIGATE_TO_START_POS',
            NAVIGATE_TO_START_POS(),
            transitions={'next':'SYSTEM_FINISH', 'failure':'SYSTEM_FAILURE'})
        
        smach.StateMachine.add('SYSTEM_FINISH', SYSTEM_FINISH(),
                transitions={'next':'FINISH'})

        smach.StateMachine.add('SYSTEM_FAILURE', SYSTEM_FAILURE(),
                transitions={'next':'ERROR'})
    
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/UROS_MISSION2')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    signal(SIGINT, handler)
    print('Running. Press CTRL-C to exit.')
    main()
