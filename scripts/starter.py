#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64

class Starter:
    def __init__(self):
        print "Starter"
        self._start = 0
#        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("start", Int64, self.callback)

    def callback(self, val):
      rospy.loginfo(rospy.get_name() + ": I heard %f" % val.data)
      self._start = val.data
        
    def getValue(self):
      return self._start

    def reset(self):
      self._start = 0

