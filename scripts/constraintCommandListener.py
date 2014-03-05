#!/usr/bin/env python
import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib

roslib.load_manifest('robohow_sot')
from robohow_sot.utils import *

from robohow_common_msgs.msg  import ConstraintCommand

roslib.load_manifest('dynamic_graph_actionlib')
from dynamic_graph_actionlib.msg import *

roslib.load_manifest ('dynamic_graph_bridge_msgs')
from dynamic_graph_bridge_msgs.srv import RunCommand

"""
ConstraintListener listens to the constraints message sent, and converts
them into python commands sent to the stack of tasks through run_command interface.
"""
class ConstraintConfigListener:

    run_command = None

    def __init__(self):
        # Create the node
        rospy.init_node('constraint_config', anonymous=True)

        # Wait for the run_command service to be started
        rospy.loginfo("\n " + rospy.get_name() + " waiting for run_command")
        rospy.wait_for_service ('run_command')
        rospy.sleep(1)
        rospy.loginfo(rospy.get_name() + "run_command obtained")
        self.run_command = rospy.ServiceProxy ('run_command', RunCommand)

        # Subscribe to the constraint publisher
        rospy.Subscriber("/constraint_command", ConstraintCommand, self.callback)
        rospy.spin()

    def callback(self, data):
        # convert the constrain and the send the corresponding command
        instructionList = [parameterizeContraint(data)]
        if instructionList != []:
          instruction = regroupCommands(instructionList)
          runCommandProxy(self.run_command, instruction)

# Start the listener
if __name__ == '__main__':
    configListener = ConstraintConfigListener ()

