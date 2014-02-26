#!/usr/bin/env python
## Common Tools for the robohow > sot bridge
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib

from robohow_common_msgs.msg  import ConstraintCommand

## Tools
""" convert a vector3 to a string """
def vector3ToStr(vec):
    st = "(%f, %f, %f)" % (vec.x, vec.y, vec.z)
    return st;

""" Convert a vector of double into a string"""
def vectorToStr(vec):
    rospy.loginfo(rospy.get_name() + ": I heard %d" % len(vec))
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

""" convert a vector3 to a string """
def vectorToStr(vec):
    rospy.loginfo(rospy.get_name() + ": I heard %d" % len(vec))
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

""" run an instruction """
def runCommandProxy(proxy, instruction):
#    rospy.loginfo ("run instruction: " + instruction)
    result = proxy (instruction)
#    rospy.loginfo ("stdout: " + result.stdout)
#    rospy.loginfo ("stderr: " + result.stderr)


"""
Send parameters to the constraint
Takes a ConstraintCommand as an input.
"""
def parameterizeContraint(proxy, c):
  if c.controller_id == '':
    return

  instruction = "setTaskGoal(robot, '"+c.controller_id+"', " +\
    vectorToStr(c.pos_lo) + ", " + vectorToStr(c.pos_hi) + ", " +\
    "'" + c.selec + "'"   + ", " + vectorToStr(c.gain) + ")"
  runCommandProxy(proxy, instruction)


