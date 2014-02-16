#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

class Starter:
    def __init__(self):
        self._start = -1
        rospy.Subscriber("start", Int64, self.callback)

    def callback(self, val):
      self._start = val.data
        
    def getValue(self):
      return self._start

    def reset(self):
      self._start = -1

