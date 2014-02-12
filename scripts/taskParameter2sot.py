#!/usr/bin/env python
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib

from robohow_common_msgs.msg  import ConstraintCommand

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge')
#roslib.load_manifest('openhrp_bridge')
from dynamic_graph_bridge.srv import RunCommand


## Common


""" convert a vector3 to a string """
def vectorToStr(vec):
    rospy.loginfo(rospy.get_name() + ": I heard %d" % len(vec))
    st = '('
    for i in range(0, len(vec)):
      s = "%f, " % vec[i]
      st = st + s
    st = st + ')'
    return st

""" run an inscruction """
def runCommand(proxy, instruction):
    rospy.loginfo ("run instruction: " + instruction)
    result = proxy (instruction)
#    rospy.loginfo ("stdout: " + result.stdout)
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


def parameterizeContraint(proxy, c):
  rospy.loginfo(": Working Beta the constraint %s" % (c.controller_id))
  instruction = "setTaskGoal(robot, '"+c.controller_id+"', " +\
    vectorToStr(c.pos_lo) + ", " + vectorToStr(c.pos_hi) + ", " +\
    "'" + c.selec + "'" + ")"
  runCommand(proxy, instruction)


## End common



"""
ConstraintListener listens to the constraints message sent, and converts
them into commands sent to the stack of tasks run_command interface.
"""
class ConstraintConfigListener:

    run_command = None

    def __init__(self):
        # Create the node
        rospy.init_node('constraint_config', anonymous=True)

        # Wait for the run_command service to be started
        rospy.loginfo("\n " + rospy.get_name() + " waiting for run_command")
        rospy.wait_for_service ('run_command')
        rospy.loginfo(rospy.get_name() + "run_command obtained")
        self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

        # import the minimum set of python packages in the sot
        runCommand(self.run_command, "from dynamic_graph import plug")
        runCommand(self.run_command, "from dynamic_graph.sot.core import *")
        runCommand(self.run_command, "from dynamic_graph.sot.expression_graph.expression_graph import *")
        runCommand(self.run_command, "from dynamic_graph.sot.dyninv import  TaskInequality")

        # Subscribe to the constraint publisher
        rospy.Subscriber("/constraint_command", ConstraintCommand, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_name() + ": I heard %s" % data.controller_id)

        # convert the constrain and the send the corresponding command
        parameterizeContraint(self.run_command, data)

# Start the listener
if __name__ == '__main__':
    talker = ConstraintConfigListener ()

