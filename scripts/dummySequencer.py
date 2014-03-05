#!/usr/bin/python

# Constraint configuration for pancake flipping.
# A feature-based configuration, imitating the cylinder/orientation setup.

import roslib
roslib.load_manifest('robohow_common_msgs')
roslib.load_manifest('pr2_controllers_msgs') # gripper msgs
import rospy
import actionlib #for the gripper

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature, ConstraintCommand
from geometry_msgs.msg import Vector3
from numpy import radians
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *


from std_srvs.srv import Empty, EmptyResponse


# Reminder: object types.
ANGLE=0
DISTANCE=1
POSITION=2
OTHER=3

################################# Create the tasks for the pouring task

# ---- Define the features corresponding the manipulation of the bottle ---
ground_x =  Feature('ground_x', 'ground', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(1,0,0))
ground_z =  Feature('ground_z', 'ground', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,0,1))
ground_plane =  Feature('ground_plane', 'ground', Feature.PLANE,
                  Vector3(0,0,0), Vector3(0,0,1))

cup   = Feature('cup', 'cup', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))

bung   = Feature('bung', 'bung', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))
bung_x = Feature('bung_x', 'bung', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(1,0,0))

ground_plane = Feature('ground_plane', 'ground', Feature.PLANE,
                  Vector3(0,0,0), Vector3(0,0,1))

# Name of the feature
l_gripper    = Feature('l_gripper', 'l_gripper', Feature.POINT,
                   Vector3(0,0,0), Vector3(0,0,0))
l_gripper_z  = Feature('l_gripper_z', 'l_gripper', Feature.VERSOR,
                   Vector3(0,0,0), Vector3(0,1,0))

# 
r_gripper   = Feature('r_gripper', 'r_gripper', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))
r_gripper_y = Feature('r_gripper_y', 'r_gripper', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,1,0))
r_gripper_z = Feature('r_gripper_z', 'r_gripper', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,0,1))
r_gripper_uz  = Feature('r_gripper_uz', 'r_gripper', Feature.VERSOR,
                   Vector3(0,0,0), Vector3(0,0,-1))

bottle   = Feature('bottle', 'bottle', Feature.POINT,
                  Vector3(0,0,0), Vector3(0,0,0))
bottle_z = Feature('bottle_z', 'bottle', Feature.VERSOR,
             Vector3(0,0,0), Vector3(0,0,1))


# define the constraints.....
constraints = {} # dictionnary of constraint
parameters  = {} # dictionnary of constraint parameters

# Additional tasks, not defined using XPgraph- Features
constraints['robot_task_com']         = Constraint('robot_task_com', OTHER, None, None, None)
constraints['robot_task_left-ankle']  = Constraint('robot_task_left-ankle', OTHER, None, None, None)
constraints['robot_task_right-ankle'] = Constraint('robot_task_right-ankle', OTHER, None, None, None)
constraints['robot_task_position']    = Constraint('robot_task_position', OTHER, None, None, None)

constraints['taskcontact'] = Constraint('taskcontact', OTHER, None, None, None)
constraints['taskbase'] = Constraint('taskbase', OTHER, None, None, None)
constraints['taskJL'] = Constraint('taskJL', OTHER, None, None, None)
constraints['weight'] = Constraint('weight', OTHER, None, None, None)

parameters['taskright-wrist']  = ConstraintCommand('taskright-wrist', 0, [], [], '', [1])
constraints['taskright-wrist'] = Constraint('taskright-wrist', OTHER, None, None, parameters['taskright-wrist'])

### Define the constraints with initial parameters.
# angle_gripperZ_bottleZ: the gripper is oriented with the Z axis of the bottle
parameters['angle_gripperZ_bottleZ']  = ConstraintCommand(\
  'angle_gripperZ_bottleZ', 0, [radians(180)], [radians(180)], '', [0.5])
constraints['angle_gripperZ_bottleZ'] = Constraint ('angle_gripperZ_bottleZ', ANGLE, r_gripper_uz, bottle_z, parameters['angle_gripperZ_bottleZ'] )

# position_gripper_bottle: the gripper is at the same height as the can.
parameters['position_gripper_bottle']  = ConstraintCommand(\
  'position_gripper_bottle', 0, [0.1, 0, 0], [0.1, 0, 0], '111', [0.5])
constraints['position_gripper_bottle'] = Constraint ('position_gripper_bottle', POSITION, r_gripper, bottle, parameters['position_gripper_bottle'])


# Constrain the rotation of the bottle for the pouring task : 
# 90* => the Z axis of the world and the Z axis of the bottle are colinear
#  0* => the bottle is horizontal
parameters['angle_pouring'] = ConstraintCommand(\
  'angle_pouring', 0, [radians(90)], [radians(90)], '', [])
constraints['angle_pouring'] = Constraint('angle_pouring', ANGLE, bung_x, ground_z, parameters['angle_pouring'])


# Constrain the rotation of the gripper to keep the hand horizontal 
parameters['angle_gripperY_in_ground_plane'] = angle_gripperY_in_ground_plane_Param = ConstraintCommand(\
  'angle_gripperY_in_ground_plane', 0, [radians(0)], [radians(0)], '', [])
constraints['angle_gripperY_in_ground_plane'] = Constraint('angle_gripperY_in_ground_plane',  ANGLE,  ground_plane, r_gripper_y, parameters['angle_gripperY_in_ground_plane'])

# Distance bottle / r_hand
parameters['distance_bottle_gripper'] = ConstraintCommand(\
  'distance_bottle_gripper', 0, [radians(0)], [radians(0)], '', [])
constraints['distance_bottle_gripper'] = Constraint('distance_bottle_gripper', DISTANCE, r_gripper, bottle, parameters['distance_bottle_gripper'])


# ---- TASKS corresponding the manipulation of the bottle ---
################################ #######################
## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
   'position_bung_Z', 0, [0.0], [0.0], '100', [])
  #'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_Z_bung'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])

#######################################################
parameters['position_rg_XY'] = ConstraintCommand(\
  'position_rg_XY', 0, [0.02], [100], '', [])
constraints['position_rg_XY'] = Constraint('position_rg_XY', DISTANCE, cup, r_gripper, parameters['position_rg_XY'])


#######################################################
## position of the bottle above the target.
## inequality task: we want the bottle to be above the recipient
parameters['position_bung_XY'] = ConstraintCommand(\
  'position_bung_XY', 0, [-0.025,-0.025], [ 0.025, 0.025], '011', [])
constraints['position_bung_XY'] = Constraint('position_bung_XY', POSITION, cup, bung, parameters['position_bung_XY'])

# ---- TASKS corresponding the manipulation of the bottle ---
################################ #######################
## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
   'position_bung_Z', 0, [0.0], [0.0], '100', [])
  #'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_bung_Z'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])

#
#parameters['tips']  = ConstraintCommand('tips', 0, [2.5], [2.5], '', [])
#constraints['tips'] = Constraint('tips', ANGLE, ground_x, r_gripper_y, parameters['tips'] )


##     ################################ #######################
##     ## height of the bottle above the target
##     self.addExpressions(PointElement('cup', robot, 'cup'))
##     self.addExpressions(PointElement('bung', robot, 'bung'))
##     (self.tasks['position_bung_Z'], self.features['position_bung_Z']) = \
##       createTaskAndFeaturePointToPoint('position_bung_Z', \
##         self.expressions['bung'], self.expressions['cup'])
##     self.features['position_bung_Z'].selec.value ='100' #TODO
##     self.features['position_bung_Z'].reference.value = (-0.05,)

#cup    = Feature('cup', 'cup', Feature.POINT,
#                   Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0))
#bung   = Feature('bung', 'bung', Feature.POINT,
#                   Vector3(0,0,0), Vector3(0,0,0), Vector3(0,0,0))



""" remove the key from the list, only if it is in the list"""
def safeRemove(l, key):
 if key in l:
  l.remove(key)

""" Reduce the index of the element x by 1 in the list. """
def up (list, x):
  if x in list:
    val = list.index(x)
    if val > 0:
      list.remove(x)
      list.insert(val-1,x)

""" Move the element at the end of the list. """
def moveLast(list, x):
  if x in list:
    list.remove(x)
    list.append(x)



""" A simple sequencer. """
class DummySequencer:
  criticalTask = None  # critical task: the
  stepIndex = 0       # current step
  pubStack = None      # publisher for the constraint
  pubParam = None      # publisher for the constrainCommand
  gripperCall = None   # Gripper actionlib calls

  ### Define the stack that will be sent to the robot.
  stack = []

  """ Constructor """
  def __init__(self, pubStack, pubParam):
    self.pubStack = pubStack
    self.pubParam = pubParam

    self.stepList = [] 
    self.stepList.append(lambda:self.reset())
    self.stepList.append(lambda:self._step0())
    self.stepList.append(lambda:self._step1())
    self.stepList.append(lambda:self._step2())
    self.stepList.append(lambda:self._step2a())
    self.stepList.append(lambda:self._step3())
    self.stepList.append(lambda:self._step4())
    self.stepList.append(lambda:self._step5())
    self.stepList.append(lambda:self._step6())
    self.stepList.append(lambda:self._step7())
    self.stepList.append(lambda:self._step8())
    self.stepList.append(lambda:self._step9())

    #self.createGripper()
    #self.reset()

  """ create the action gripper """
  def createGripper(self):
    self.gripperCall = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',Pr2GripperCommandAction)
    print "Waiting for the gripper server"
    self.gripperCall.wait_for_server()


  """ open the gripper through action """
  def openGripper(self):
    if self.gripperCall != None :
      self.gripperCall.send_goal(Pr2GripperCommandGoal(\
        Pr2GripperCommand(position = 0.3, max_effort = -1)))
      self.gripperCall.wait_for_result()
#      result = self.gripperCall.get_result()
      rospy.loginfo ("I have finished the gripper opening")


  """ close the gripper through action """
  def closeGripper(self):
    if self.gripperCall != None :
      self.gripperCall.send_goal(Pr2GripperCommandGoal(\
          Pr2GripperCommand(position = 0.05, max_effort = 50)))
      self.gripperCall.wait_for_result()
#      result = self.gripperCall.get_result()
      rospy.loginfo ("I have finished the gripper closing")


  """ reinitialize the cram """
  def reset(self):
    rospy.loginfo ("Reset")

    # Add the basic tasks for humanoid/mobile robot
    robot = rospy.get_param("robot")
    isHumanoid = rospy.get_param("humanoid")
    if robot.lower() == "hrp4" or robot.lower() == "romeo":
      isHumanoid = True

    if isHumanoid == True:
      self.stack = []
      self.stack.append(constraints['robot_task_com'])
      self.stack.append(constraints['robot_task_left-ankle'])
      self.stack.append(constraints['robot_task_right-ankle'])
    else:
      self.stack = []
      self.stack.append(constraints['taskcontact'])
      self.stack.append(constraints['taskbase'])
      self.stack.append(constraints['taskJL'])
      self.stack.append(constraints['weight'])
    #TODO #stack.append(constraints['robot_task_position'])

    self.stepIndex = 0
    self.pubStack.publish(ConstraintConfig('test', self.stack))

    self.openGripper()
    
    return EmptyResponse()



  # graps
  def _step0(self):
    rospy.loginfo ("going in front of the bottle")
    safeRemove(self.stack, constraints['robot_task_position'])
    self.stack.append(constraints['position_gripper_bottle'])
#    stack.append(constraints['angle_gripperZ_bottleZ'])


    self.criticalTask = 'position_gripper_bottle'
    pubStack.publish(ConstraintConfig('test', self.stack))

  def _step1(self):
    rospy.loginfo ("Step: Add gripper task")   
    #fk self.solver.push(self.r_gripper_angle.task)
    #fk self.r_gripper_angle.featureDes.errorIN.value = (1,0)
    
    # update the criticalTask
    #fk self.criticalTask = self.r_gripper_angle.task
#self.tasks['angle_pouring']

  # close the gripper
  def _step2(self):
    rospy.loginfo ("Step: Going to the bottle")
    # Add a task to go to the bottle
    self.stack.append(constraints['taskright-wrist'])
#    self.stack.append(constraints['distance_bottle_gripper'])
    #self.solver.push(self.robot.tasks[''])
    safeRemove(self.stack, constraints['position_gripper_bottle'])
    #self.solver.remove(self.robot.tasks['position_gripper_bottle'])
    self.criticalTask = 'distance_bottle_gripper'

  # bent the bottle a little
  def _step2a(self):
    rospy.loginfo ("Step: Grasping")
    #fk self.r_gripper_angle.featureDes.errorIN.value = (1,0.4)
#    self.r_gripper_angle.close()
    # update the criticalTask
    #fk self.criticalTask = self.r_gripper_angle.task

    # replace the task controlling the orientation of the bottle by the pouring one.
    safeRemove(self.stack, constraints['angle_gripperZ_bottleZ'])
#    self.solver.remove(self.tasks['distance-gripperX_bottleX'])
    self.closeGripper()


    #todo: estimate the position of the bottle neck
 
  # go above the glass.
  def _step3(self):
    rospy.loginfo ("Step: Start pouring")
    safeRemove(self.stack, constraints['taskright-wrist'])
    self.stack.append(constraints['position_bung_Z'])
    self.stack.append(constraints['position_bung_XY'])
    self.stack.append(constraints['angle_pouring'])

    self.stack.append(constraints['angle_gripperY_in_ground_plane'])
#    self.stack.append(constraints['tips'])

    self.criticalTask = 'position_bung_Z'

  # pour a little
  def _step4(self):
    rospy.loginfo ("Step: Pouring more")
    parameters['angle_pouring'].pos_lo = [radians(100)]
    parameters['angle_pouring'].pos_hi = [radians(100)]
    self.pubParam.publish(parameters['angle_pouring'])
    self.criticalTask = 'angle_pouring'

  # pour ...
  def _step5(self):
    rospy.loginfo ("Step: And more")
    parameters['angle_pouring'].pos_lo = [radians(115)]
    parameters['angle_pouring'].pos_hi = [radians(115)]
    self.pubParam.publish(parameters['angle_pouring'])
    self.criticalTask = 'angle_pouring'

  def _step6(self):
    rospy.loginfo ("Step: going to initial position")
    
    #TODO safeRemove(stack, constraints['FoV'])
    safeRemove(self.stack, constraints['position_bung_Z'])
    safeRemove(self.stack, constraints['position_bung_XY'])
    safeRemove(self.stack, constraints['angle_gripperY_in_ground_plane'])
    safeRemove(self.stack, constraints['angle_pouring'])

#    parameters['angle_pouring'].pos_lo = [radians(85)]
#    parameters['angle_pouring'].pos_hi = [radians(85)]
#    self.pubParam.publish(parameters['angle_pouring'])
    #self.criticalTask = 'taskRH'
    
    self.stack.append(constraints['taskright-wrist'])



  def _step7(self):
#    self.stack.append(constraints['distance_bottle_gripper'])
#    self.stack.append(constraints['angle_gripperZ_bottleZ'])
    criticalTask = 'distance_bottle_gripper'

  def _step8(self):
    #r_gripper_angle.featureDes.errorIN.value = (1,0)
    criticalTask = 'r_gripper_angle.task'

  def _step9(self):
    #solver.remove(self.r_gripper_angle.task)
    criticalTask = None


  """ run a step """
  def step(self):
    if self.stepIndex < len(self.stepList):
      self.stepList[self.stepIndex]()
      moveLast(self.stack, constraints['weight'])
      self.pubStack.publish(ConstraintConfig('test', self.stack))
      print "step ", self.stepIndex

      # increase step number
      self.stepIndex = self.stepIndex + 1
    else:
      print "No more things to do."
      print "You can reset the demonstration using the cram_reset service "
      
    return EmptyResponse()



if __name__ == '__main__':
  ## OK, let's go!
  rospy.init_node('constraint_config')
  pubStack = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
  pubParam = rospy.Publisher('/constraint_command', ConstraintCommand, latch=True)
  rospy.sleep(5)
  d=DummySequencer(pubStack, pubParam)

  stepperSrv = rospy.Service('cram_run_step', Empty, lambda req: d.step())
  resetSrv   = rospy.Service('cram_reset', Empty, lambda req: d.reset())
  openSrv    = rospy.Service('fk_open', Empty, lambda req: d.openGripper())
  closeSrv   = rospy.Service('fk_close', Empty, lambda req: d.closeGripper())

  # Dummy step by step validation
  rospy.spin()

