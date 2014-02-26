Example Pouring Task with SoT

Launch with:
$ roslaunch robohow_sot dummy_cram.launch

In a second terminal, trig the node by calling the services
$ rosservice call /start_dynamic_graph 
$ rosservice call /cram_run_step

The first one trigger the running of SoT, the second one execute one step for the exampleInterface Node.
Only two, sequential, steps are implemented. By calling the service "/cram_run_step" twice, the second step is executed

NOTE: The real "pouring" task is executed in the last step, the first just load the task, without executing it
The others are needed only for set a starting position.
In the demo for the year 2, this position is supposed to be archieved in advance, hence the pouring starts
already with the grasped bottle.

-- HOW IT WORKS --
By triggering the provided services, the node streams
1)on /constraint_config topic, the ConstraintConfig.msg with the list of tasks to be solved
2)on /constraint_command, additional parameters for the tasks (see ConstraintCommand), that is low/high boundaries, gains, ...

-- HOW TO INTEGRATE for the DEMO --
Option I: USE THE NODE AS IT IS
The python script is self descripting! However, remember to REMOVE the call of the function "reset" in the init of the main class
(as commented into the code!)

Option II: REPLACE the example interface node with cram.
CRAM sofware should streams the tasks needed to be executed, through the topics defined as follow (namely)

- /constraint_config, type: ConstraintConfig.msg
- /constraint_command, type: ConstraintCommand.msg

Please, check the messages in robohow_common_msgs package

Option III: rosbag record + rosbag play of the streamed data

For the Pouring DEMO, some comments from the messages analysis on the data filled in
controller_id: pouring -> REFERENCE NAME. For the demo, only this exists
constraints: -> List of constraints. All the following are anyway filled in internally by SoT
  - 
    name: taskcontact
    function: 3 ->OTHER type
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
  - 
    name: taskbase
    function: 3
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
  - 
    name: taskJL
    function: 3
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
      
STEP 0
controller_id: pouring
constraints: 
  - 
    name: taskcontact
    function: 3
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
  - 
    name: taskbase
    function: 3
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
  - 
    name: taskJL
    function: 3
    tool_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ''
      frame_id: ''
      type: 0
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: ''
      movement_id: 0
      pos_lo: []
      pos_hi: []
      selec: ''
      gain: []
  -                         NOTE: this ones are the new task added!!!!!!!!!!!
    name: position_bung_Z  
    function: 2
    tool_feature: 
      name: bung
      frame_id: bung
      type: 2
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: cup
      frame_id: cup
      type: 2
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: position_bung_Z
      movement_id: 0
      pos_lo: [-0.05]
      pos_hi: [-0.05]
      selec: 100
      gain: []
  - 
    name: position_bung_XY
    function: 2
    tool_feature: 
      name: cup
      frame_id: cup
      type: 2
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: bung
      frame_id: bung
      type: 2
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 0.0
    command: 
      controller_id: position_bung_XY
      movement_id: 0
      pos_lo: [-0.025, -0.025]
      pos_hi: [0.025, 0.025]
      selec: 011
      gain: []
  - 
    name: angle_pouring
    function: 0
    tool_feature: 
      name: bung_x
      frame_id: bung
      type: 3
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 1.0
        y: 0.0
        z: 0.0
    world_feature: 
      name: ground_z
      frame_id: ground
      type: 3
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 1.0
    command: 
      controller_id: angle_pouring
      movement_id: 0
      pos_lo: [2.007128639793479]
      pos_hi: [2.007128639793479]
      selec: ''
      gain: []
  - 
    name: angle_gripperY_in_ground_plane
    function: 0
    tool_feature: 
      name: ground_plane
      frame_id: ground
      type: 1
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 0.0
        z: 1.0
    world_feature: 
      name: r_gripper_y
      frame_id: r_gripper
      type: 3
      position: 
        x: 0.0
        y: 0.0
        z: 0.0
      direction: 
        x: 0.0
        y: 1.0
        z: 0.0
    command: 
      controller_id: angle_gripperY_in_ground_plane
      movement_id: 0
      pos_lo: [0.0]
      pos_hi: [0.0]
      selec: ''
      gain: []

      
COMMAND, this is sent on the Step 1, hence the real execution pooring. It sets up the limits.
Some values are not needed in this demo. (pos_lo = pos_hi, equality constraint)
controller_id: angle_pouring
movement_id: 0
pos_lo: [2.007128639793479]
pos_hi: [2.007128639793479]
selec: ''
gain: []
     
