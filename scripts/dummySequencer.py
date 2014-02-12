#!/usr/bin/python

# Constraint configuration for pancake flipping.
# A feature-based configuration, imitating the cylinder/orientation setup.

import roslib
roslib.load_manifest('robohow_common_msgs')
import rospy

from robohow_common_msgs.msg import ConstraintConfig, Constraint, Feature, ConstraintCommand
from geometry_msgs.msg import Vector3
from numpy import radians
from starter import Starter



################################# Create the tasks for the pouring task


# ---- TASKS corresponding the manipulation of the bottle ---

# Constrain the rotation of the bottle for the pouring task : 
# 90* => the Z axis of the world and the Z axis of the bottle are colinear
#  0* => the bottle is horizontal
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



### gripper
#createTask(robot, 'angle_pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (radians(90)), upperBound = (radians(90)))
#createTask(robot, 'angle_gripperY_in_ground_plane', 'ground_plane', 'r_gripper_y', 'angle', lowerBound = (0), upperBound = (0))

# Distance bottle / r_hand
#createTask(robot, 'distance_bottle_gripper', 'r_gripper', 'bottle', 'distance', lowerBound = (0), upperBound = (0))
#createTask(robot, 'position_rg_XY', 'cup', 'r_gripper', 'distance', lowerBound = (0.02,), upperBound = (100,))

################################ #######################
## height of the bottle above the target
#createExpression(robot, PointElement('cup', robot, 'cup'))
#createExpression(robot, PointElement('bung', robot, 'bung'))
#createTask(robot, 'position_bung_Z', 'bung', 'cup', 'position', lowerBound = (0,), upperBound = (0,))

#robot.features['position_bung_Z'].selestack.value ='100' #TODO
#robot.features['position_bung_Z'].reference.value = (-0.05,)





# define the constraints.....
constraints = {} # dictionnary of constraint
parameters  = {}      # dictionnary of constraint parameters

constraints['robot_task_com']         = Constraint('robot_task_com', 'other', None, None)
constraints['robot_task_left-ankle']  = Constraint('robot_task_left-ankle', 'other', None, None)
constraints['robot_task_right-ankle'] = Constraint('robot_task_right-ankle', 'other', None, None)
constraints['robot_task_position']    = Constraint('robot_task_position', 'other', None, None)

stack = []
stack.append(constraints['robot_task_com'])
stack.append(constraints['robot_task_left-ankle'])
stack.append(constraints['robot_task_right-ankle'])
stack.append(constraints['robot_task_position'])


#stack.append(Constraint('position_bung_XY', 'position', cup, bung))

#stack.append(Constraint('position_bung_XY', 'distance', r_gripper ,   l_gripper))
#stack.append(Constraint('dist', 'distance', r_gripper ,   l_gripper))
#stack.append(Constraint('angle', 'angle',   r_gripper_z , l_gripper_z))

# TODO angle_gripperZ_bottleZ =  'angle', lowerBound = radians(180), upperBound = radians(180))
parameters['angle_gripperZ_bottleZ']  = ConstraintCommand(\
  'angle_gripperZ_bottleZ', 0, [radians(180)], [radians(180)], '', [1])
constraints['angle_gripperZ_bottleZ'] = Constraint ('angle_gripperZ_bottleZ', 'angle', r_gripper_uz, bottle_z, parameters['angle_gripperZ_bottleZ'] )


parameters['position_gripper_bottle']  = ConstraintCommand(\
  'position_gripper_bottle', 0, [0.1, 0, 0], [0.1, 0, 0], '111', [1])
constraints['position_gripper_bottle'] = Constraint ('position_gripper_bottle', 'position', r_gripper, bottle, parameters['position_gripper_bottle'])
# lowerBound = (0,), upperBound = (0,),)
#    robot.features['position-gripper_bottle'].selec.value ='111' 
#    robot.features['position-gripper_bottle'].reference.value = (0.1, 0, 0)


    # Constrain the rotation of the bottle for the pouring task : 
    # 90* => the Z axis of the world and the Z axis of the bottle are colinear
    #  0* => the bottle is horizontal
parameters['angle_pouring'] = ConstraintCommand(\
  'angle_pouring', 0, [radians(90)], [radians(90)], '', [])
constraints['angle_pouring'] = Constraint('angle_pouring', 'angle', bung_x, ground_z, parameters['angle_pouring'])
#    createTask(robot, 'angle-pouring', 'bung_x', 'ground_z', 'angle', lowerBound = (radians(90)), upperBound = (radians(90)))



    # Constrain the rotation of the gripper to keep the hand horizontal 
parameters['angle_gripperY_in_ground_plane'] = angle_gripperY_in_ground_plane_Param = ConstraintCommand(\
  'angle_gripperY_in_ground_plane', 0, [radians(0)], [radians(0)], '', [])
constraints['angle_gripperY_in_ground_plane'] = Constraint('angle_gripperY_in_ground_plane',  'angle',  ground_plane, r_gripper_y, parameters['angle_gripperY_in_ground_plane'])
#angle_gripperY_in_ground_plane = createTask(robot, 'angle_gripperY_in_ground_plane',  'angle',  ground_plane, r_gripper_y,lowerBound = (0), upperBound = (0))

    # Distance bottle / r_hand
parameters['distance_bottle_gripper'] = ConstraintCommand(\
  'distance_bottle_gripper', 0, [radians(0)], [radians(0)], '', [])
constraints['distance_bottle_gripper'] = Constraint('distance_bottle_gripper', 'distance', r_gripper, bottle, parameters['distance_bottle_gripper'])
#distance_bottle_gripper = createTask('distance_bottle_gripper', 'distance', r_gripper, bottle, lowerBound = (0), upperBound = (0))



    # ---- TASKS corresponding the manipulation of the bottle ---
    ################################ #######################
    ## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
  'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_Z_bung'] = Constraint('position_bung_Z', 'position', bung, cup, parameters['position_bung_Z'])
# position_bung_Z = createTask('position_bung_Z', 'position', 'bung', 'cup', lowerBound = (0,), upperBound = (0,))
#    robot.features['position_bung_Z'].selec.value ='100' #TODO
#    robot.features['position_bung_Z'].reference.value = (-0.05,)
#    setTaskGoal(robot, 'position_bung_Z', (-0.05,), (-0.05,))


    #######################################################
parameters['position_rg_XY'] = ConstraintCommand(\
  'position_rg_XY', 0, [0.02], [100], '', [])
constraints['position_rg_XY'] = Constraint('position_rg_XY', 'distance', cup, r_gripper, parameters['position_rg_XY'])
#    createTask(robot, 'position-rg_XY', 'cup', 'r_gripper', 'distance', lowerBound = (0.02,), upperBound = (100,))


    #######################################################
    ## position of the bottle above the target.
    ## inequality task: we want the bottle to be above the recipient
parameters['position_bung_XY'] = ConstraintCommand(\
  'position_bung_XY', 0, [-0.025,-0.025], [ 0.025, 0.025], '011', [])
constraints['position_bung_XY'] = Constraint('position_bung_XY', 'position', cup, bung, parameters['position_bung_XY'])

#    createTask(robot, 'position_bung_XY', 'cup', 'bung', 'position', lowerBound = (0,0,), upperBound = (0,1,))
#    robot.features['position_bung_XY'].selec.value = '011'
#    robot.features['position_bung_XY'].reference.value = (0, 0)
#    setTaskGoal(robot, 'position_bung_XY', (-0.025,-0.025), ( 0.025, 0.025))


    # ---- TASKS corresponding the manipulation of the bottle ---
    ################################ #######################
    ## height of the bottle above the target
parameters['position_bung_Z'] = ConstraintCommand(\
  'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
constraints['position_bung_Z'] = Constraint('position_bung_Z', 'position', bung, cup, parameters['position_bung_Z'])

#position_bung_Z = createTask('position_bung_Z', 'bung', 'cup', 'position', lowerBound = (0,), upperBound = (0,))
#    robot.features['position_bung_Z'].selec.value ='100' #TODO
#    robot.features['position_bung_Z'].reference.value = (-0.05,)
#    setTaskGoal(robot, 'position_bung_Z', (-0.05,), (-0.05,))

parameters['tips']  = ConstraintCommand('tips', 0, [2.5], [2.5], '', [])
constraints['tips'] = Constraint('tips', 'angle', ground_x, r_gripper_y, parameters['tips'] )


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
#c.append(Constraint('position_bung_Z', 'position',   bung, cup, '100'))




#c.append(Constraint('dist', 'distance', tool_center, up))
#c.append(Constraint('height', 'height', tool_center, up))

#c.append(Constraint('align_front', 'perpendicular', tool_front_rev, up))
#c.append(Constraint('align_side',  'perpendicular', tool_side,      up))
#c.append(Constraint('pointing_at', 'pointing_at',   tool_forward,   up))



class DummySequencer:
  criticalTask = None
  stepIndex = -1
  pubStack = None
  pubParam = None

  def __init__(self, pubStack, pubParam):
    rospy.loginfo('init')
    self.pubStack = pubStack
    #self.pubParam = pubParam

  # graps
  def _step0(self):
    print "going in front of the bottle"
#    self.steps.Step(append(add=['position_gripper_bottle', 'angle_gripperZ_bottleZ'], rm=[]))
    stack.remove(constraints['robot_task_position'])
    stack.append(constraints['position_gripper_bottle'])
    stack.append(constraints['angle_gripperZ_bottleZ'])
    self.criticalTask = 'position_gripper_bottle'
    pubStack.publish(ConstraintConfig('test', stack))

  def _step1(self):
    print "task"
    #fk self.solver.push(self.r_gripper_angle.task)
    #fk self.r_gripper_angle.featureDes.errorIN.value = (1,0)
    
    # update the criticalTask
    #fk self.criticalTask = self.r_gripper_angle.task
#self.tasks['angle_pouring']

  # close the gripper
  def _step2(self):
    print "going to the bottle"
    # Add a task to go to the bottle
    stack.append(constraints['distance_bottle_gripper'])
    #self.solver.push(self.robot.tasks[''])
    stack.remove(constraints['position_gripper_bottle'])
    #self.solver.remove(self.robot.tasks['position_gripper_bottle'])
    self.criticalTask = 'distance_bottle_gripper'

  # bent the bottle a little
  def _step2a(self):
    print "grasping"
    #fk self.r_gripper_angle.featureDes.errorIN.value = (1,0.4)
#    self.r_gripper_angle.close()
    # update the criticalTask
    #fk self.criticalTask = self.r_gripper_angle.task

    # replace the task controlling the orientation of the bottle by the pouring one.
    stack.remove(constraints['angle_gripperZ_bottleZ'])
#    self.solver.remove(self.tasks['distance-gripperX_bottleX'])
    stack.append(constraints['angle_pouring'])


    #todo: estimate the position of the bottle neck
 
  # go above the glass.
  def _step3(self):
    print "Start pouring"
    stack.remove(constraints['distance_bottle_gripper'])
    stack.append(constraints['position_bung_Z'])
    stack.append(constraints['position_rg_XY'])
    stack.append(constraints['position_bung_XY'])
    stack.append(constraints['angle_gripperY_in_ground_plane'])
    stack.append(constraints['tips'])
    #TODO stack.insert(robot_task_angle_pouring)

    self.criticalTask = 'position_bung_Z'

  # pour a little
  def _step4(self):
    print "Pouring more"
    #TODO self.robot.features['angle_pouring'].reference.value = radians(100)
    parameters['angle_pouring'] = ConstraintCommand(\
      'angle_pouring', 0, [radians(100)], [radians(100)], [], '')
    self.criticalTask = 'angle_pouring'

  # pour ...
  def _step5(self):
    print "And more"
    #TODO self.robot.features['angle_pouring'].reference.value = radians(115)
    parameters['angle_pouring'] = ConstraintCommand(\
      'angle_pouring', 0, [radians(115)], [radians(115)], [], '')
    self.criticalTask = 'angle_pouring'

  def _step6(self):
    #TODO stack.remove(constraints['FoV'])
    stack.remove(constraints['tips'])
    stack.remove(constraints['position_bung_Z'])
    stack.remove(constraints['position_bung_XY'])
    parameters['angle_pouring'].pos_lo = [radians(85)]
    parameters['angle_pouring'].pos_hi = [radians(85)]

    #TODO self.robot.features['angle_pouring'].reference.value = radians(85)
    #TODO stack.remove(constraints['taskRH'])
    #TODO self.solver.sot.up(self.robot.tasks['taskRH'].name)

    #mhpo = MatrixHomoToPose('mhpo')
    #plug(self.robot.rightWrist.position, mhpo.sin)
    #mhpo.sout.recompute(self.robot.rightWrist.position.time)
    #target=mhpo.sout.value
    #gotoNd(self.taskRH,target,'111',(4.9,0.9,0.01,0.9))

    #self.criticalTask = 'taskRH'
    


  def _step7(self):
    stack.remove(constraints['position_rg_XY'])
    stack.remove(constraints['angle_gripperY_in_ground_plane'])
    stack.remove(constraints['angle_pouring'])
#    stack.remove(constraints['taskRH'])

    stack.append(constraints['distance_bottle_gripper'])
    stack.append(constraints['angle_gripperZ_bottleZ'])
    criticalTask = 'distance_bottle_gripper'

  def _step8(self):
    #r_gripper_angle.featureDes.errorIN.value = (1,0)
    criticalTask = 'r_gripper_angle.task'

  def _step9(self):
    #solver.remove(self.r_gripper_angle.task)
    criticalTask = None

  def step(self):
    rospy.loginfo('yatta ')
    if(self.stepIndex == -1):
      self._step0()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 0):
      self._step1()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 1):
      self._step2()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 2):
      self._step2a()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 3):
      self._step3()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 4):
      self._step4()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 5):
      self._step5()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 6):
      self._step6()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 7):
      self._step7()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 8):
      self._step8()
      self.pubStack.publish(ConstraintConfig('test', stack))
    elif(self.stepIndex == 9):
      self._step9()
      self.pubStack.publish(ConstraintConfig('test', stack))
    print "step ", self.stepIndex

    # increase step number
    self.stepIndex = self.stepIndex + 1

    # publishes the param for each task in the SOT
    # todo: wait for the confirmation that the pubStack has been realized.
#    rospy.sleep(0.05)
#    for task in stack:
#      rospy.loginfo('in the sot: ' + task.name)
#      if task.function != 'other':
#        self.pubParam.publish(parameters[task.name])
#        rospy.loginfo('param: ' + task.name)

if __name__ == '__main__':
  ## OK, let's go!
  rospy.init_node('constraint_config')
  pubStack = rospy.Publisher('/constraint_config', ConstraintConfig, latch=True)
  pubParam = rospy.Publisher('/constraint_command', ConstraintCommand, latch=True)

  d=DummySequencer(pubStack, pubParam)

  starter = Starter()
  # simple try to sync with rviz...
  #rospy.wait_for_service('/rviz/reload_shaders')

#  rate = rospy.Rate(int(1./timeStep))
  while not rospy.is_shutdown():
    if starter.getValue() > 0:
      print "d.step()"
      d.step()
      starter.reset()

#for i in range(0,12):
#  rospy.loginfo(i)
#d.step()


#rospy.spin()

#requires a dummy button step.



