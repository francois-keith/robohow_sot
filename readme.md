Pouring Task with SoT
=============================
Launching the example
------------
In a terminal:
```
$ roslaunch robohow_sot dummy_cram.launch
```
In a second terminal, trig the node by calling the services
```
$ rosservice call /start_dynamic_graph 
$ rosservice call /cram_run_step
```
The first command  starts SoT, the second one execute one step for the _exampleInterface_ Node.

__Only three sequential steps are implemented__. 
1. load the tasks (writring in _/constraint_config_), and execute a task with a setpoint of 90 degrees (writing in _/constraint_command_).
1. change the setpoint for pouring
(writing in _/constraint_command_).
1. change the setpoint again to 90 degrees.
(writing in _/constraint_command_).


In the demo for the year 2, this position is supposed to be archieved in advance, hence the pouring starts supposing that the
 bottle is grasped.
 
## constraints 
### constraints: robot dependent
The following tasks are not needed/mandatory to be defined,
since the SoT Bridge takes care of adding them automatically for the PR2
However, they are here for sake of completeness

    constraints['taskcontact'] = Constraint('taskcontact', OTHER, None, None, None)
    
mantains the robot on the floor (heigth, two rotations)

---
    constraints['taskbase'] = Constraint('taskbase', OTHER, None, None, None)
    
Fixes the bases on the plane (so that the base cannot move)

---
    constraints['taskJL'] = Constraint('taskJL', OTHER, None, None, None)

Joint  limit constraints

---    
    constraints['weight'] = Constraint('weight', OTHER, None, None, None)
Force to use the arms before moving the torso joint.


## constraints 
### constraints: task dependent

 In this example, we have only this task to be accomplished.
 These lines defines the constraints, numbers are hardcoded for testing
 Constrain the rotation of the bottle for the pouring task : 
- 90 deg => the Z axis of the world and the Z axis of the bottle are colinear
-  0 deg => the bottle is horizontal

######parameters

1. is the name
1. a dummy int that is never used (__TODO__: remove?).
1.  the task lower  constraints value.
1. are the task  upper constraint values. Leave empty if you do not want to change those bounds.
1.  (' ') is the selector. Only used for the position task: allows to define the axes that will be considered (Z,Y,X) (e.g. '100' selects the z component only), __they are inverted because inverse polish notation is used!__
1. [] is the gain (aka the velocity at which the task will be realized). Leave empty for the default value.


    parameters['angle_pouring'] = ConstraintCommand(\
    'angle_pouring', 0, [radians(90)], [radians(90)], '', [])
    constraints['angle_pouring'] =    Constraint('angle_pouring', ANGLE, bung_x, ground_z, parameters['angle_pouring'])

    # Constrain the rotation of the gripper to keep the hand horizontal 
    parameters['angle_gripperY_in_ground_plane'] = ConstraintCommand(\
    'angle_gripperY_in_ground_plane', 0, [radians(0)], [radians(0)], '', [])
    constraints['angle_gripperY_in_ground_plane'] = Constraint('angle_gripperY_in_ground_plane',  ANGLE,  ground_plane, r_gripper_y, parameters['angle_gripperY_in_ground_plane'])
---
    # ---- TASKS corresponding the manipulation of the bottle ---

    ## height of the bottle above the     target
    parameters['position_bung_Z'] = ConstraintCommand(\
      'position_bung_Z', 0, [-0.05], [-0.05], '100', [])
    constraints['position_bung_Z'] = Constraint('position_bung_Z', POSITION, bung, cup, parameters['position_bung_Z'])

---
__NOTE: Inverse polish notation: the  '011' is z off, y on, and x on.__
    ## position of the bottle above     the target.
    ## inequality task: we want the bottle to be above the recipient
    parameters['position_bung_XY'] = ConstraintCommand(\
      'position_bung_XY', 0, [-0.025,-0.025], [ 0.025, 0.025], '011', [])
    constraints['position_bung_XY'] = Constraint('position_bung_XY', POSITION, cup, bung, parameters['position_bung_XY'])
    
###To be added:

###Error feedback
For each task scheduled in SoT, a relative feedback ROS topic is published.
As convention, the name of the topic is the same name of the relative task, while the datatype exchanged is of type dynamic_graph/Vector (formally, float64[])
The vector contains the error relatives to the constrained variables, plus a normalized value.
That means, if the constraint is a Scalar, then the vector contains two element; if it a position, it contains 4 elements (3 variable errors, plus their norm value)
It is up to the SoT client keep track of the names and the expected dimension of the return vector.

