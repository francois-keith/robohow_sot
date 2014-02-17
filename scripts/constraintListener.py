#!/usr/bin/env python
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature

roslib.load_manifest('robohow_sot')

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge')
from dynamic_graph_bridge.srv import RunCommand


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

""" run an instruction """
def runCommand(proxy, instruction):
#    rospy.logdebug ("run instruction: " + instruction)
    result = proxy (instruction)
#    rospy.loginfo ("stdout: " + result.stdout)
#    rospy.loginfo ("stderr: " + result.stderr)

""" Send parameters to the constraint"""
def parameterizeContraint(proxy, c):
  instruction = "setTaskGoal(robot, '"+c.controller_id+"', " +\
    vectorToStr(c.pos_lo) + ", " + vectorToStr(c.pos_hi) + ", " +\
    "'" + c.selec + "'" + ")"
  runCommand(proxy, instruction)



""" create the feature """
def createFeature(proxy, feat):
  if feat.type == 0: # LINE
    rospy.loginfo ("TODO LINE feature not handled")
  elif feat.type == 1: # PLANE
    instruction = "createExpression(robot, PlaneElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "normal = "   + vector3ToStr(feat.direction) +","\
      "position = " + vector3ToStr(feat.position ) +"))"
    runCommand(proxy, instruction)
  elif feat.type == 2: # POINT
    instruction = "createExpression(robot, PointElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "position = " + vector3ToStr(feat.position)  +"))"
    runCommand(proxy, instruction)
  elif feat.type == 3: # VERSOR
    instruction = "createExpression(robot, VersorElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "versor = "   + vector3ToStr(feat.direction) +"))"
    runCommand(proxy, instruction)
  else:
   rospy.loginfo ("not handled")


""" create the task, that will be stored in the robot database. """
def createConstraint(proxy, constr):
  bounds = "lowerBound = " + vectorToStr(constr.command.pos_lo)+", "+\
           "upperBound = " + vectorToStr(constr.command.pos_hi)
  instruction = "createTask(robot,'" + constr.name + "', '" + constr.tool_feature.name + "', " +\
                "'" + constr.world_feature.name+"', '"+constr.function+"', " +\
                bounds  +")"
  runCommand(proxy, instruction)

  # push the task in the solver
  #instruction = "solver.push(robot.tasks['"+command.name+"'])"
  #runCommand(proxy, instruction)

"""
Build the set of constraints (aka stack) and the attached parameterization.
Update the stack of tasks.
"""
def convertContraintToCommands(proxy, constraints):
  runCommand(proxy, "superviser.clear()")
  for c in constraints:
    # if the type of the task is other, we assume that the task 
    #  has been created in the SoT / does not depend on expression-graph system.
    if c.function != 'other':    
      rospy.loginfo(": Working the constraint %s, %s" % (c.name, c.function))
      createFeature(proxy, c.tool_feature)
      createFeature(proxy, c.world_feature)
      createConstraint(proxy, c)
      parameterizeContraint(proxy, c.command)

    # Push the stack into the desired stack
    runCommand(proxy, "superviser.push('" + c.name +"')")

  # Actualize the current solver.
  runCommand(proxy, "superviser.update()")

#uint8 ANGLE=0			# computes the angle between the two features
#uint8 DISTANCE=1	# computes the distance between the two features
#uint8 POSITION=2	# compute the difference of position between the two features
#uint8 function  # name of the constraint function ('perpendicular', etc.)



"""
ConstraintListener listens to the constraints message sent, and converts
them into commands sent to the stack of tasks through the run_command interface.
"""
class ConstraintListener:

    run_command = None

    def __init__(self):
        # Create the node
        rospy.init_node('listener', anonymous=True)

        # Wait for the run_command service to be started
        rospy.loginfo("\n " + rospy.get_name() + " waiting for run_command")
        rospy.wait_for_service ('run_command')
        rospy.loginfo(rospy.get_name() + "run_command obtained")
        self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

        # Subscribe to the constraint publisher
        rospy.Subscriber("/constraint_config", ConstraintConfig, self.callback)
        rospy.spin()

    def callback(self, data):
        # convert the constrain and the send the corresponding command
        convertContraintToCommands(self.run_command, data.constraints)

# Start the listener
if __name__ == '__main__':
    ConstraintListener ()

