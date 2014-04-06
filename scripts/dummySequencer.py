#!/usr/bin/python
# Constraint configuration for pouring task
# Requires the following rosparams:
#  robot:      indicates the name of the robot.
#  kinematics: indicates if it is only a kinematic simulation

import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy
import actionlib #for the gripper

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature, ConstraintCommand
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3
from numpy import radians
from actionlib_msgs.msg import *

roslib.load_manifest('pr2_controllers_msgs') # gripper msgs
from pr2_controllers_msgs.msg import *

from std_srvs.srv import Empty, EmptyResponse

# Reminder: object types.
OTHER=0
ANGLE=1
DISTANCE=2
POSITION=3
POINTING_AT=4

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
bung_z = Feature('bung_z', 'bung', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,0,1))

ground_plane = Feature('ground_plane', 'ground', Feature.PLANE,
                  Vector3(0,0,0), Vector3(0,0,1))

# Name of the feature
l_gripper    = Feature('l_gripper', 'l_gripper', Feature.POINT,
                   Vector3(0,0,0), Vector3(0,0,0))
l_gripper_y = Feature('l_gripper_y', 'l_gripper', Feature.VERSOR,
                  Vector3(0,0,0), Vector3(0,1,0))
l_gripper_z  = Feature('l_gripper_z', 'l_gripper', Feature.VERSOR,
                   Vector3(0,0,0), Vector3(0,1,0))
l_gripper_uz  = Feature('l_gripper_uz', 'l_gripper', Feature.VERSOR,
                   Vector3(0,0,0), Vector3(0,0,-1))


#r_gripper   = Feature('r_gripper', 'r_gripper', Feature.POINT,
#                  Vector3(0,0,0), Vector3(0,0,0))
#r_gripper_y = Feature('r_gripper_y', 'r_gripper', Feature.VERSOR,
#                  Vector3(0,0,0), Vector3(0,1,0))
#r_gripper_z = Feature('r_gripper_z', 'r_gripper', Feature.VERSOR,
#                  Vector3(0,0,0), Vector3(0,0,1))
#r_gripper_uz  = Feature('r_gripper_uz', 'r_gripper', Feature.VERSOR,
#                   Vector3(0,0,0), Vector3(0,0,-1))

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
constraints['chest'] = Constraint('chest', OTHER, None, None, None)
constraints['weight'] = Constraint('weight', OTHER, None, None, None)

parameters['taskright-wrist']  = ConstraintCommand('taskright-wrist', [], [], '', [1])
constraints['taskright-wrist'] = Constraint('taskright-wrist', OTHER, None, None, parameters['taskright-wrist'])

parameters['taskleft-wrist']  = ConstraintCommand('taskleft-wrist', [], [], '', [1])
constraints['taskleft-wrist'] = Constraint('taskleft-wrist', OTHER, None, None, parameters['taskleft-wrist'])

### Define the constraints with initial parameters.
# angle_gripperZ_bottleZ: the gripper is oriented with the Z axis of the bottle
parameters['angle_gripperZ_bottleZ']  = ConstraintCommand(\
  'angle_gripperZ_bottleZ', [radians(180)], [radians(180)], '', [0.5])
constraints['angle_gripperZ_bottleZ'] = Constraint ('angle_gripperZ_bottleZ', ANGLE, l_gripper_uz, bottle_z, parameters['angle_gripperZ_bottleZ'] )

# position_gripper_bottle: the gripper is at the same height as the can.
parameters['position_gripper_bottle']  = ConstraintCommand(\
  'position_gripper_bottle', [0.1, 0, 0], [0.1, 0, 0], '111', [0.5])
constraints['position_gripper_bottle'] = Constraint ('position_gripper_bottle', POSITION, l_gripper, bottle, parameters['position_gripper_bottle'])


# Constrain the rotation of the bottle for the pouring task : 
# 90* => the Z axis of the world and the Z axis of the bottle are colinear
#  0* => the bottle is horizontal
parameters['angle_pouring'] = ConstraintCommand(\
  'angle_pouring', [radians(90)], [radians(90)], '', [])
constraints['angle_pouring'] = Constraint('angle_pouring', ANGLE, bung_x, ground_z, parameters['angle_pouring'])

parameters['angle_pouring2'] = ConstraintCommand(\
  'angle_pouring2', [radians(60)], [radians(60)], '', [])
constraints['angle_pouring2'] = Constraint('angle_pouring2', ANGLE, bung_z, ground_z, parameters['angle_pouring2'])


# Constrain the rotation of the gripper to keep the hand horizontal 
parameters['angle_gripperY_in_ground_plane'] = angle_gripperY_in_ground_plane_Param = ConstraintCommand(\
  'angle_gripperY_in_ground_plane', [radians(0)], [radians(0)], '', [])
constraints['angle_gripperY_in_ground_plane'] = Constraint('angle_gripperY_in_ground_plane',  ANGLE,  ground_plane, l_gripper_y, parameters['angle_gripperY_in_ground_plane'])

# Distance bottle / r_hand
parameters['distance_bottle_gripper'] = ConstraintCommand(\
  'distance_bottle_gripper', [radians(0)], [radians(0)], '', [])
constraints['distance_bottle_gripper'] = Constraint('distance_bottle_gripper', DISTANCE, l_gripper, bottle, parameters['distance_bottle_gripper'])


# ---- TASKS corresponding the manipulation of the bottle ---
################################ #######################
## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
   'position_bung_Z', [0.0], [0.0], '100', [])
  #'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_Z_bung'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])

#######################################################
parameters['position_rg_XY'] = ConstraintCommand(\
  'position_rg_XY', [0.02], [100], '', [])
constraints['position_rg_XY'] = Constraint('position_rg_XY', DISTANCE, cup, l_gripper, parameters['position_rg_XY'])


#######################################################
## position of the bottle above the target.
## inequality task: we want the bottle to be above the recipient
parameters['position_bung_XY'] = ConstraintCommand(\
  'position_bung_XY', [-0.025,-0.025], [ 0.025, 0.025], '011', [])
constraints['position_bung_XY'] = Constraint('position_bung_XY', POSITION, cup, bung, parameters['position_bung_XY'])

#parameters['position_bung_XY'] = ConstraintCommand(\
#  'position_bung_XY', [0], [ 0.05], '011', [])
#constraints['position_bung_XY'] = Constraint('distance_bung_XY', POSITION, cup, bung, parameters['position_bung_XY'])

# ---- TASKS corresponding the manipulation of the bottle ---
################################ #######################
## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
   'position_bung_Z', [0.0], [0.0], '100', [])
constraints['position_bung_Z'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])



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
  stepIndex = 0       # current step
  pubStack = None      # publisher for the constraint
  pubParam = None      # publisher for the constrainCommand
  gripperCall = None   # Gripper actionlib calls

  oldCriticalTask = ''
  criticalTask = ''         # critical task: the task indicating the level of accomplishment of the step 
  criticTaskListener = None # Critical task listener.

  robot = "" # Name of the robot
  ### Define the stack that will be sent to the robot.
  stack = []

  """ Constructor """
  def __init__(self, pubStack, pubParam):
    self.oldCriticalTask = ''
    self.pubStack = pubStack
    self.pubParam = pubParam

    self.robot = (rospy.get_param("robot")).lower()
    self.kinematics = rospy.get_param("kinematics")

    self.stepList = [] 
    self.stepList.append(lambda:self.reset())
    self.stepList.append(lambda:self._step0())
#    self.stepList.append(lambda:self._step2b())
    self.stepList.append(lambda:self._step3())
    self.stepList.append(lambda:self._step4())
    self.stepList.append(lambda:self._step5())
    self.stepList.append(lambda:self._step5b())
    self.stepList.append(lambda:self._step6())

    # are we in kinematic simulation or not?
    if self.kinematics == False and self.robot == "pr2" :
      self.createGripper()


  """ create the action gripper """
  def createGripper(self):
    self.gripperCall = actionlib.SimpleActionClient('r_gripper_controller/gripper_action',Pr2GripperCommandAction)
    rospy.logwarn ("Waiting for the gripper server")
    self.gripperCall.wait_for_server()
    print "Gripper server acquired"


  """ open the gripper through action """
  def openGripper(self):
    if self.gripperCall != None :
      self.gripperCall.send_goal(Pr2GripperCommandGoal(\
        Pr2GripperCommand(position = 0.3, max_effort = -1)))
      self.gripperCall.wait_for_result()
      result = self.gripperCall.get_result()
      rospy.loginfo ("I have finished the gripper opening")
#      self.step()


  """ close the gripper through action """
  def closeGripper(self):
    if self.gripperCall != None :
      self.gripperCall.send_goal(Pr2GripperCommandGoal(\
          Pr2GripperCommand(position = 0.05, max_effort = 50)))
      self.gripperCall.wait_for_result()
      result = self.gripperCall.get_result()
      print result
      rospy.loginfo ("I have finished the gripper closing")
#      self.step()


  """ reinitialize the cram """
  def reset(self):
    rospy.loginfo ("Reset")

    # Add the basic tasks for humanoid/mobile robot
    isHumanoid = (self.robot == "hrp4" or self.robot == "romeo")

    if isHumanoid == True:
      self.stack = []
      self.stack.append(constraints['robot_task_com'])
      self.stack.append(constraints['robot_task_left-ankle'])
      self.stack.append(constraints['robot_task_right-ankle'])
      self.stack.append(constraints['robot_task_position'])
    else:
      self.stack = []
      self.stack.append(constraints['taskcontact'])
      self.stack.append(constraints['taskbase'])
      self.stack.append(constraints['taskJL'])
      self.stack.append(constraints['robot_task_position'])
      self.stack.append(constraints['weight'])

    self.stepIndex = 0
    self.pubStack.publish(ConstraintConfig('test', self.stack))

    self.openGripper()
    
    return EmptyResponse()


  # graps
  def _step0(self):
    rospy.loginfo ("release position task")
    safeRemove(self.stack, constraints['robot_task_position'])

  def _step0a(self):
    rospy.loginfo ("going in front of the bottle")
    self.stack.append(constraints['position_gripper_bottle'])
    self.criticalTask = 'position_gripper_bottle'

  # Close the gripper
  def _step1(self):
    rospy.loginfo ("Step: Add gripper task")   

  # Add a task to go to the bottle
  def _step2(self):
    rospy.loginfo ("Step: Going to the bottle")
    safeRemove(self.stack, constraints['position_gripper_bottle'])
    self.stack.append(constraints['taskleft-wrist'])
    self.criticalTask = 'taskleft-wrist'

  # bent the bottle a little
  def _step2a(self):
    rospy.loginfo ("Step: Grasping")

    # replace the task controlling the orientation of the bottle by the pouring one.
    safeRemove(self.stack, constraints['angle_gripperZ_bottleZ'])
    self.closeGripper()



  def _step2b(self):
    rospy.loginfo ("Step: Going to the bottle")
    safeRemove(self.stack, constraints['position_gripper_bottle'])
    self.stack.append(constraints['taskleft-wrist'])
    self.criticalTask = 'taskleft-wrist'

    # replace the task controlling the orientation of the bottle by the pouring one.
    safeRemove(self.stack, constraints['angle_gripperZ_bottleZ'])
    self.closeGripper()





    #todo: estimate the position of the bottle neck
 
  # go above the glass.
  def _step3(self):
    rospy.loginfo ("Step: Start pouring")
    safeRemove(self.stack, constraints['position_gripper_bottle'])
    safeRemove(self.stack, constraints['angle_gripperZ_bottleZ'])
    safeRemove(self.stack, constraints['taskleft-wrist'])
    self.stack.append(constraints['position_bung_Z'])
    self.stack.append(constraints['position_bung_XY'])
    self.stack.append(constraints['angle_pouring'])
    self.stack.append(constraints['angle_gripperY_in_ground_plane'])

    # TODO: if the task already exists in the sot database, its parameters are not used: correct that, otherwise the reload does not work.
    parameters['angle_pouring'].pos_lo = [radians(90)]
    parameters['angle_pouring'].pos_hi = [radians(90)]
    self.pubParam.publish(parameters['angle_pouring'])

#    self.criticalTask = 'position_bung_Z'


  # pour a little
  def _step4(self):
    rospy.loginfo ("Step: Pouring more")
    parameters['angle_pouring'].pos_lo = [radians(135)]
    parameters['angle_pouring'].pos_hi = [radians(135)]
    self.pubParam.publish(parameters['angle_pouring'])
    self.criticalTask = 'angle_pouring'


  # Pour more
  def _step5(self):
    rospy.loginfo ("Step: And more")
    safeRemove(self.stack, constraints['angle_pouring'])
    self.stack.append(constraints['angle_pouring2'])
    self.pubParam.publish(parameters['angle_pouring2'])

  # Pour more
  def _step5b(self):
    rospy.loginfo ("Step: And more")
    parameters['angle_pouring2'].pos_lo = [radians(0)]
    parameters['angle_pouring2'].pos_hi = [radians(0)]
    self.pubParam.publish(parameters['angle_pouring2'])
    self.criticalTask = 'angle_pouring2'


  # Step: going to initial position
  def _step6(self):
    rospy.loginfo ("Step: going to initial position")
    safeRemove(self.stack, constraints['position_bung_Z'])
    safeRemove(self.stack, constraints['position_bung_XY'])
    safeRemove(self.stack, constraints['angle_gripperY_in_ground_plane'])
    safeRemove(self.stack, constraints['angle_pouring2'])
    self.stack.append(constraints['robot_task_position'])
#    self.stack.append(constraints['taskleft-wrist'])
#    self.criticalTask = 'taskleft-wrist'

  """ get the critical task """
  def getCriticalTask(self):
    return self.criticalTask


  """ manipulation of the stacks before the update of the stack state"""
  def pre_update(self):
    return

  """ manipulation of the stacks before the after of the stack state"""
  def post_update(self):
    if self.robot == "pr2":
      # move the weighting task at the end of the stack
      moveLast(self.stack, constraints['weight'])
    return

  """ update the stack: run the index given in index """
  def updateStack(self, index):
    self.pre_update()
    self.stepList[self.stepIndex]()
    self.post_update()




  """ run a step """
  def step(self):
    if self.stepIndex < len(self.stepList):
      # backup critical task
      self.oldCriticalTask = self.criticalTask

      # next step
      self.updateStack(self.stepIndex)

      # post step command.
      moveLast(self.stack, constraints['weight'])

      # destroy / pause the current task listener
      if(self.criticalTask != '' and self.criticTaskListener != None):
        if self.criticalTask != self.oldCriticalTask:
          self.criticTaskListener.destroy()
          del self.criticTaskListener
          self.criticTaskListener = None

      # send the new stack to the SoT
      print "step ", self.stepIndex
      self.pubStack.publish(ConstraintConfig('test', self.stack))

      # change the critical task listened to.
      if self.criticalTask != '':
        if self.criticTaskListener == None:
          self.criticTaskListener = CritiqueTaskListener(self, self.criticalTask)
        else:
          self.criticTaskListener.reactivateListener()

      # increase step number
      self.stepIndex = self.stepIndex + 1
    else:
      print "No more things to do."
      print "You can reset the demonstration using the cram_reset service "
#      self.reset()
#      self.step()

    return EmptyResponse()


"""
Create a listener for the given task
The listener tasks in parameter the name of the task and its completion range (optional)
"""
class CritiqueTaskListener:
  sequencer = None
  subscriber = None
  finished  = False 
  threshold = 0
  taskname  = ''
  numEvaluation = 1
  
  # 
  def __init__(self, sequencer, taskname, threshold = 1e-2):
    self.finished  = False 
    self.taskname  = taskname
    self.sequencer = sequencer
    self.threshold = threshold
    self.numEvaluation = 0

    # create the subscriber for the given task
    if taskname != '':
      sig_name = '/sot/'+taskname+'_error_norm'
      sig_name = sig_name.replace('-', '_')
      print "Start listening to ", sig_name
      self.subscriber = rospy.Subscriber(sig_name, Float64, self.callback)

  # Callback: if the task is finished, call a step
  def callback(self, val):
    if self.finished == True:
      return

    if(val.data <= self.threshold):
      print self.numEvaluation, "   val.data  ", val.data
      if self.numEvaluation <= 0 :
        self.finished = True
        if (self.sequencer.getCriticalTask() == self.taskname):
          self.sequencer.step()
      else:
        self.numEvaluation = self.numEvaluation - 1

  def reactivateListener(self, numEvaluation = 5):
    self.finished = False
    self.numEvaluation = numEvaluation

  def destroy(self):
    print "unregister ", self.taskname
    self.subscriber.unregister

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

