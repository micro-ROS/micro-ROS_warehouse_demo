#! /usr/bin/env python

import rospy
import threading
import traceback
 
from actionlib_msgs.msg import GoalStatus
 
from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle
 
 
def nop_cb(goal_handle):
    pass
 
 
 
class UrosActionServer:    
    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):

        self.new_goal = False
        self.preempt_request = False
        self.new_goal_preempt_request = False

        self.execute_callback = execute_cb
        self.goal_callback = None
        self.preempt_callback = None

        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()

        # since the internal_goal/preempt_callbacks are invoked from the
        # ActionServer while holding the self.action_server.lock
        # self.lock must always be locked after the action server lock
        # to avoid an inconsistent lock acquisition order
        self.lock = threading.RLock()

        self.execute_condition = threading.Condition(self.lock)

        self.current_goal = ServerGoalHandle()
        self.next_goal = ServerGoalHandle()

        if self.execute_callback:
            self.execute_thread = threading.Thread(None, self.executeLoop)
            self.execute_thread.start()
        else:
            self.execute_thread = None

        # create the action server
        self.action_server = ActionServer(name, ActionSpec, self.internal_goal_callback, self.internal_preempt_callback, auto_start)

    def __del__(self):
        if hasattr(self, 'execute_callback') and self.execute_callback:
            with self.terminate_mutex:
                self.need_to_terminate = True

            assert(self.execute_thread)
            self.execute_thread.join()

    
    def accept_new_goal(self):
        with self.action_server.lock, self.lock:
            if not self.new_goal or not self.next_goal.get_goal():
                rospy.logerr("Attempting to accept the next goal when a new goal is not available")
                return None

            # check if we need to send a preempted message for the goal that we're currently pursuing
            if self.is_active() and self.current_goal.get_goal() and self.current_goal != self.next_goal:
                self.current_goal.set_canceled(None, "This goal was canceled because another goal was received by the simple action server")

            rospy.logdebug("Accepting a new goal")

            # accept the next goal
            self.current_goal = self.next_goal
            self.new_goal = False

            # set preempt to request to equal the preempt state of the new goal
            self.preempt_request = self.new_goal_preempt_request
            self.new_goal_preempt_request = False

            # set the status of the current goal to be active
            self.current_goal.set_accepted("This goal has been accepted by the simple action server")

            return self.current_goal.get_goal()

    
    def is_new_goal_available(self):
        return self.new_goal

    
    def is_preempt_requested(self):
        return self.preempt_request

    
    def is_active(self):
        if not self.current_goal.get_goal():
            return False

        status = self.current_goal.get_goal_status().status
        return status == GoalStatus.ACTIVE or status == GoalStatus.PREEMPTING

    
    def set_succeeded(self, result=None, text=""):
        with self.action_server.lock, self.lock:
            if not result:
                result = self.get_default_result()
            self.current_goal.set_succeeded(result, text)

    
    def set_aborted(self, result=None, text=""):
        with self.action_server.lock, self.lock:
            if not result:
                result = self.get_default_result()
            self.current_goal.set_aborted(result, text)

    
    def publish_feedback(self, feedback):
        self.current_goal.publish_feedback(feedback)

    def get_default_result(self):
        return self.action_server.ActionResultType()

    
    def set_preempted(self, result=None, text=""):
        if not result:
            result = self.get_default_result()
        with self.action_server.lock, self.lock:
            rospy.logdebug("Setting the current goal as canceled")
            self.current_goal.set_canceled(result, text)

    
    def register_goal_callback(self, cb):
        if self.execute_callback:
            rospy.logwarn("Cannot call UrosActionServer.register_goal_callback() because an executeCallback exists. Not going to register it.")
        else:
            self.goal_callback = cb

    
    def register_preempt_callback(self, cb):
        self.preempt_callback = cb

    
    def start(self):
        self.action_server.start()

    
    def internal_goal_callback(self, goal):
        self.execute_condition.acquire()

        try:
            rospy.logdebug("A new goal %shas been recieved by the single goal action server", goal.get_goal_id().id)

            # check that the timestamp is past that of the current goal and the next goal
            if((not self.current_goal.get_goal() or goal.get_goal_id().stamp >= self.current_goal.get_goal_id().stamp)
               and (not self.next_goal.get_goal() or goal.get_goal_id().stamp >= self.next_goal.get_goal_id().stamp)):
                # if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
                if(self.next_goal.get_goal() and (not self.current_goal.get_goal() or self.next_goal != self.current_goal)):
                    self.next_goal.set_canceled(None, "This goal was canceled because another goal was received by the simple action server")

                self.next_goal = goal
                self.new_goal = True
                self.new_goal_preempt_request = False

                # if the server is active, we'll want to call the preempt callback for the current goal
                if(self.is_active()):
                    self.preempt_request = True
                    # if the user has registered a preempt callback, we'll call it now
                    if(self.preempt_callback):
                        self.preempt_callback()

                # if the user has defined a goal callback, we'll call it now
                if self.goal_callback:
                    self.goal_callback()

                # Trigger runLoop to call execute()
                self.execute_condition.notify()
                self.execute_condition.release()
            else:
                # the goal requested has already been preempted by a different goal, so we're not going to execute it
                goal.set_canceled(None, "This goal was canceled because another goal was received by the simple action server")
                self.execute_condition.release()
        except Exception as e:
            rospy.logerr("UrosActionServer.internal_goal_callback - exception %s", str(e))
            self.execute_condition.release()

    
    def internal_preempt_callback(self, preempt):
        with self.lock:
            rospy.logdebug("A preempt has been received by the UrosActionServer")

            # if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
            if(preempt == self.current_goal):
                rospy.logdebug("Setting preempt_request bit for the current goal to TRUE and invoking callback")
                self.preempt_request = True

                # if the user has registered a preempt callback, we'll call it now
                if(self.preempt_callback):
                    self.preempt_callback()
            # if the preempt applies to the next goal, we'll set the preempt bit for that
            elif(preempt == self.next_goal):
                rospy.logdebug("Setting preempt request bit for the next goal to TRUE")
                self.new_goal_preempt_request = True

    
    def executeLoop(self):
        loop_duration = rospy.Duration.from_sec(.1)

        while (not rospy.is_shutdown()):
            with self.terminate_mutex:
                if (self.need_to_terminate):
                    break

            # the following checks (is_active, is_new_goal_available)
            # are performed without locking
            # the worst thing that might happen in case of a race
            # condition is a warning/error message on the console
            if (self.is_active()):
                rospy.logerr("Should never reach this code with an active goal")
                return

            if (self.is_new_goal_available()):
                # accept_new_goal() is performing its own locking
                goal = self.accept_new_goal()
                if not self.execute_callback:
                    rospy.logerr("execute_callback_ must exist. This is a bug in UrosActionServer")
                    return
                try:
                    self.execute_callback(goal)
                    if self.is_active():
                        rospy.logwarn("Your executeCallback did not set the goal to a terminal status.  " +
                                      "This is a bug in your ActionServer implementation. Fix your code!  " +
                                      "For now, the ActionServer will set this goal to aborted")
                        self.set_aborted(None, "No terminal state was set.")
                except Exception as ex:
                    rospy.logerr("Exception in your execute callback: %s\n%s", str(ex),
                                 traceback.format_exc())
                    self.set_aborted(None, "Exception in execute callback: %s" % str(ex))
            with self.execute_condition:
                self.execute_condition.wait(loop_duration.to_sec())

if __name__ == "__main__":
    server = UrosActionServer()
    rospy.spin()