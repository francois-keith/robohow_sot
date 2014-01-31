#!/usr/bin/env python
import roslib
roslib.load_manifest('constraint_msgs')
import rospy
import actionlib

from constraint_msgs.msg import ConstraintConfig, Constraint, Feature

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge')
#roslib.load_manifest('openhrp_bridge')
from dynamic_graph_bridge.srv import RunCommand

""" convert a vector3 to a string """
def vector3ToStr(vec):
    st = "(%f, %f, %f)" % (vec.x, vec.y, vec.z)
    return st;

""" run an inscruction """
def runCommand(proxy, instruction):
    rospy.loginfo ("run instruction: " + instruction)
    result = proxy (instruction)
    rospy.loginfo ("stdout: " + result.stdout)
    rospy.loginfo ("stderr: " + result.stderr)


""" create the task, push it into the sot """
def createConstraint(proxy, command):
  instruction = "createTask(robot,'" + command.name + "', '" + command.tool_feature.name + "', " +\
                "'" + command.world_feature.name+"', '"+command.function+"', " +\
                "lowerBound = (0), upperBound  = (0))"
  runCommand(proxy, instruction)

  # push the task in the solver
  instruction = "solver.push(robot.tasks['"+command.name+"'])"
  runCommand(proxy, instruction)


def convertContraintToCommands(proxy, c):
  rospy.loginfo(": Working the constraint %s" % (c.controller_id))
#     if(lowerBound == upperBound):
#      feature.reference.value = lowerBound
#    else:
#      feature.reference.value = 0
#      task.referenceInf.value = lowerBound
#      task.referenceSup.value = upperBound
#  instruction = "robot.features['"+ command.name + "'].reference.value = 
  runCommand(proxy, instruction)

# Allows to modify the state of a constraint online
# string identifier for the controller used
# string controller_id

### hash identifier for this motion
##int64 movement_id
#float64[] pos_lo	# lower bound
#float64[] pos_hi	# upper bound
#float64[] gain    # exponential gain (1/3 parameters)



"""
ConstraintListener listens to the constraints message sent, and converts
them into commands sent to the stack of tasks run_command interface.
"""
class Listener:

    run_command = None

    def __init__(self):
        # Create the node
        rospy.init_node('listener', anonymous=True)

        # Wait for the run_command service to be started
        rospy.wait_for_service ('run_command')
        rospy.loginfo(rospy.get_name() + "run_command obtained")
        self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

        # import the minimum set of python packages in the sot
        runCommand(self.run_command, "from dynamic_graph import plug")
        runCommand(self.run_command, "from dynamic_graph.sot.core import *")
        runCommand(self.run_command, "from dynamic_graph.sot.expression_graph.expression_graph import *")
        runCommand(self.run_command, "from dynamic_graph.sot.dyninv import  TaskInequality")

        # Subscribe to the constraint publisher
        rospy.Subscriber("/constraint_config", ConstraintConfig, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.controller_id)

        # convert the constrain and the send the corresponding command
        convertContraintCommandToPython(self.run_command, data.constraints)

# Start the listener
if __name__ == '__main__':
    talker = Listener ()

