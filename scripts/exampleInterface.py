#!/usr/bin/python

# SoT Example Interface - only pouring task, PR2 ONLY
import roslib #This load ROS lib
import rospy #This load Python bindings

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature, ConstraintCommand
from geometry_msgs.msg import Vector3 #Geometric msg, Vector3
from numpy import radians #radians facilities

from std_srvs.srv import Empty, EmptyResponse

# REMINDER ENUM for pubParam (parameters of the task)
# Please check Constraint.msg definition
ANGLE=0
DISTANCE=1
POSITION=2
OTHER=3

################################# Create the tasks for the pouring task

# ---- Define the features corresponding the manipulation of the bottle ---
# The first vector indicates the position of the feature wrt. the reference frame (here, the 'ground'). 
# The second is the axis. its meaning depends on the type of Feature.
ground_z =  Feature('ground_z', 'ground', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,0,1))
#                 ^position       ^versor

#NOTE: Here, the reference frame of 'cup' and 'bung' are well known
cup   = Feature('cup', 'cup', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))

bung   = Feature('bung', 'bung', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))
bung_x = Feature('bung_x', 'bung', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(1,0,0))

ground_plane = Feature('ground_plane', 'ground', Feature.PLANE,
                  Vector3(0,0,0), Vector3(0,0,1))
 
r_gripper_y = Feature('r_gripper_y', 'r_gripper', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,1,0))

# define the constraints.....
constraints = {} # dictionnary of constraint
parameters  = {} # dictionnary of constraint parameters

# The following tasks are not needed/mandatory to be defined,
# since the SoT Bridge takes care of adding them automatically for the PR2
# However, they are here for sake of completeness
constraints['taskcontact'] = Constraint('taskcontact', OTHER, None, None, None)
constraints['taskbase'] = Constraint('taskbase', OTHER, None, None, None)
constraints['taskJL'] = Constraint('taskJL', OTHER, None, None, None)
constraints['weight'] = Constraint('weight', OTHER, None, None, None)

### Define the constraints with initial parameters.

## ANGLE POURING
# In this example, we have only this task to be accomplished.
# These lines defines the constraints, numbers are hardcoded for testing
# Constrain the rotation of the bottle for the pouring task : 
# 90* => the Z axis of the world and the Z axis of the bottle are colinear
#  0* => the bottle is horizontal

# first param is the name
# second is a dummy int that is never used (TODO: remove?)
# Third and Fourth are the task lower and upper constraints values. Leave empty if you do not want to change those bounds
# Fifth ('') is the selector. Only used for the position task: allows to define the axes that will be considered (X,Y,Z)
# Sixth [] is the gain (aka the velocity at which the task will be realized). Empty for the default value.
parameters['angle_pouring'] = ConstraintCommand(\
  'angle_pouring', 0, [radians(90)], [radians(90)], '', [])
constraints['angle_pouring'] = Constraint('angle_pouring', ANGLE, bung_x, ground_z, parameters['angle_pouring'])

# Constrain the rotation of the gripper to keep the hand horizontal 
parameters['angle_gripperY_in_ground_plane'] = ConstraintCommand(\
  'angle_gripperY_in_ground_plane', 0, [radians(0)], [radians(0)], '', [])
constraints['angle_gripperY_in_ground_plane'] = Constraint('angle_gripperY_in_ground_plane',  ANGLE,  ground_plane, r_gripper_y, parameters['angle_gripperY_in_ground_plane'])

# ---- TASKS corresponding the manipulation of the bottle ---
#######################################################
## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
  'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_bung_Z'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])

#######################################################
## position of the bottle above the target.
## inequality task: we want the bottle to be above the recipient
parameters['position_bung_XY'] = ConstraintCommand(\
  'position_bung_XY', 0, [-0.025,-0.025], [ 0.025, 0.025], '011', [])
constraints['position_bung_XY'] = Constraint('position_bung_XY', POSITION, cup, bung, parameters['position_bung_XY'])



parameters['taskright-wrist']  = ConstraintCommand('taskright-wrist', 0, [], [], '', [2])
constraints['taskright-wrist'] = Constraint('taskright-wrist', OTHER, None, None, parameters['taskright-wrist'])



""" Move the element at the end of the list. """
def moveLast(list, x):
  if x in list:
    list.remove(x)
    list.append(x)

""" NOTE: A LIST OF TASK IS SENT EVERYTIME, with ONLY the TASK constraints to be solved!!
Internally, the SoT bridge takes care of actually remove all the task previously inserted and not
needed anymore (hence, not in the list """

""" Our main class """
class ExamplePouringSOT:
  pubStack = None      # publisher for the constraint
  pubParam = None      # publisher for the constrainCommand

  ### Define the stack that will be sent to the robot.
  stack = []

  """ Constructor """
  def __init__(self, pubStack, pubParam):
    self.stepIndex = 0
    self.pubStack = pubStack
    self.pubParam = pubParam
    # NOTE: REMOVE THE FOLLOW (CALL RESET
    self.reset()

  """ reinitialize the cram """
  def reset(self):
    rospy.loginfo ("Reset")

    # initial state of the stack for the pr2
    self.stack = []
    self.stack.append(constraints['taskcontact'])
    self.stack.append(constraints['taskbase'])
    self.stack.append(constraints['taskJL'])
    self.stack.append(constraints['taskright-wrist'])
    self.stack.append(constraints['weight'])

    self.stepIndex = 0
    self.pubStack.publish(ConstraintConfig('pouring', self.stack))

    return EmptyResponse()
 
  # Executing this function will add the pouring task to the SoT
  def _addPouringTask(self):
    rospy.loginfo ("Step: Start pouring")
    self.stack.remove(constraints['taskright-wrist'])
    self.stack.append(constraints['position_bung_Z'])
    self.stack.append(constraints['position_bung_XY'])
    self.stack.append(constraints['angle_pouring'])
    self.stack.append(constraints['angle_gripperY_in_ground_plane'])
    moveLast(self.stack, constraints['weight'])

  # Executing this function will make the pouring motion
  def _executePouringTask(self):
    rospy.loginfo ("Step: And more")
    parameters['angle_pouring'].pos_lo = [radians(115)] # NOTE: these values are the one IMPOSED for the pouring
    parameters['angle_pouring'].pos_hi = [radians(115)] # NOTE: these values are the one IMPOSED for the pouring
    self.pubParam.publish(parameters['angle_pouring'])

  # bringingObjBack
  def _bringingObjBack(self):
    rospy.loginfo ("Bringing object back")
    parameters['angle_pouring'].pos_lo = [radians(90)] # NOTE: these values are the one IMPOSED for the pouring
    parameters['angle_pouring'].pos_hi = [radians(90)] # NOTE: these values are the one IMPOSED for the pouring
    self.pubParam.publish(parameters['angle_pouring'])


  """ run a step """
  def step(self):
    if(self.stepIndex == 0):
      print "... adding Pouring Task to SoT ..."
      self._addPouringTask()
      self.pubStack.publish(ConstraintConfig('pouring', self.stack))
    elif(self.stepIndex == 1):
      print "... Executing Pouring Task to SoT ..."
      self._executePouringTask()
      self.pubStack.publish(ConstraintConfig('pouring', self.stack))
    elif(self.stepIndex == 2):
      print "... Object Back after Pouring to SoT ..."
      self._bringingObjBack()
      self.pubStack.publish(ConstraintConfig('pouring', self.stack))
    print "step ", self.stepIndex

    # increase step number
    self.stepIndex = self.stepIndex + 1
    return EmptyResponse()

if __name__ == '__main__':
  ## OK, let's go!
  rospy.init_node('constraint_config')
  pubStack = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
  pubParam = rospy.Publisher('/constraint_command', ConstraintCommand, latch=True)

  d=ExamplePouringSOT(pubStack, pubParam)

  stepperSrv = rospy.Service('cram_run_step', Empty, lambda req: d.step())
  resetSrv   = rospy.Service('cram_reset', Empty, lambda req: d.reset())

  # Dummy step by step validation
  rospy.spin()

