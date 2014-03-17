#!/usr/bin/env python
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature

roslib.load_manifest('robohow_sot')
from robohow_sot.utils import *

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge_msgs')
from dynamic_graph_bridge_msgs.srv import RunCommand


""" create the feature """
def createFeature(feat):
  instruction = ""
  if feat.type == 0: # LINE
    instruction = "createExpression(robot, LineElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "normal = "   + vector3ToStr(feat.direction) +","\
      "position = " + vector3ToStr(feat.position ) +"))"
  elif feat.type == 1: # PLANE
    instruction = "createExpression(robot, PlaneElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "normal = "   + vector3ToStr(feat.direction) +","\
      "position = " + vector3ToStr(feat.position ) +"))"
  elif feat.type == 2: # POINT
    instruction = "createExpression(robot, PointElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "position = " + vector3ToStr(feat.position)  +"))"
  elif feat.type == 3: # VERSOR
    instruction = "createExpression(robot, VersorElement('"+feat.name+"', robot, '"+feat.frame_id+"', "+\
      "versor = "   + vector3ToStr(feat.direction) +"))"
  else:
   rospy.loginfo ("not handled")

  return instruction

""" 
Create the task, that will be stored in the robot task database.
"""
def createConstraint(constr):
  if constr.function == 0:
    func = 'angle'
  elif constr.function == 1:
    func = 'distance'
  elif constr.function == 2:
    func = 'position'
  else:
    rospy.loginfo(rospy.get_name() + ": The constraint %s has a unknown type %s" % (c.name, c.function))

  bounds = "lowerBound = " + vectorToStr(constr.command.pos_lo)+", "+\
           "upperBound = " + vectorToStr(constr.command.pos_hi)
  instruction = "createTask(robot,'" + constr.name + "', '" + constr.tool_feature.name + "', " +\
                "'" + constr.world_feature.name+"', '"+ func +"', " +\
                bounds  +")"
  return instruction


"""
Build the set of python commands corresponding to a constraints message.
returns a list of instructions.
"""
def convertContraintToCommands(constraints):
  instructions = ["superviser.clear()"]
  for c in constraints:
    # if the type of the task is other, we assume that the task 
    #  has been created in the SoT / does not depend on expression-graph system.
    if c.function != 3:    
      instructions.append(createFeature(c.tool_feature))
      instructions.append(createFeature(c.world_feature))
      instructions.append(createConstraint(c))

    # Parameterize the constraint
    instructions.append(parameterizeContraint(c.command))

    # Push the stack into the desired stack
    instructions.append("superviser.push('" + c.name +"')")

  # Actualize the current solver.
  instructions.append("superviser.update()")

  return instructions

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
        rospy.loginfo(rospy.get_name() + " waiting for run_command")
        rospy.wait_for_service ('run_command')
        rospy.sleep(1)
        rospy.loginfo(rospy.get_name() + " run_command obtained")
        self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

        # Subscribe to the constraint publisher
        rospy.Subscriber("/constraint_config", ConstraintConfig, self.callback)
        rospy.spin()

    def callback(self, data):
        # convert the constrain and the send the corresponding command
        instructionList = convertContraintToCommands(data.constraints)
        if instructionList != []:
          instruction = regroupCommands(instructionList)
          runCommandProxy(self.run_command, instruction)


# Start the listener
if __name__ == '__main__':
    ConstraintListener ()

