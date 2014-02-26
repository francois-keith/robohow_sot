#!/usr/bin/env python
import roslib
import rospy
import sys

from sensor_msgs.msg import Joy
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped
roslib.load_manifest('robohow_sot')


"""
"""
class tf2transform:

  pub = None
  child = None
  def __init__(self, name):
    # Create the node
    rospy.init_node('selector', anonymous=True)

    self.pub   = rospy.Publisher(name, TransformStamped)
    self.child = "/"+name #rospy.get_param("/child")

    # Subscribe to the constraint publisher
    rospy.Subscriber("/tf", tfMessage, self.callback)
    rospy.spin()

  def callback(self, data):
    for t in data.transforms:
      if t.child_frame_id == self.child:
        ts = TransformStamped()
        ts.transform = t.transform
        self.pub.publish(ts)
        
# Start the listener
if __name__ == '__main__':
  if len(sys.argv) < 2:
      print("usage: my_node.py arg1")
  else:
    tf2transform(sys.argv[1])

