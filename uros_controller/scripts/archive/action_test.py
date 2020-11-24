#! /usr/bin/env python

import roslib
roslib.load_manifest('uros_controller')
import rospy
import actionlib

from uros_controller.msg import GetHumidityAction

class GetHumidity:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('get_humidity', GetHumidityAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here
    print("Execute goal:" + str(goal.humidity_id))
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('action_test')
  server = GetHumidity()
  rospy.spin()