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
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

""" convert a vector3 to a string """
def vectorToStr(vec):
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

""" Regroup a list of python instructions as a single one """ 
def regroupCommands(instructionList):
    instruction = instructionList[0]
    for elmt in instructionList[1:]:
      if elmt != "":
        instruction = instruction + "  ;  " + elmt
    return instruction

""" run an instruction """
def runCommandProxy(proxy, instruction):
    if not instruction == "":
      rospy.logdebug ("run instruction: \"%s\"", instruction)
    result = proxy (instruction)
    #rospy.loginfo ("stdout: \"%s\"", result.stdout)
    if not result.stderr == "":
      rospy.loginfo ("stderr: \"%s\"", result.stderr)


"""
Send parameters to the constraint
Takes a ConstraintCommand as an input.
"""
def parameterizeContraint(c):
  if c.controller_id == '':
    return ""

  instruction = "setTaskGoal(robot, '"+c.controller_id+"', " +\
    vectorToStr(c.pos_lo) + ", " + vectorToStr(c.pos_hi) + ", " +\
    "'" + c.selec + "'"   + ", " + vectorToStr(c.gain) + ")"
  return instruction

