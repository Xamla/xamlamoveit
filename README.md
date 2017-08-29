# XamlaMoveit

Provides specialized planning functions and sensore interfaces.

## Todo

### Robot emulation

Node: [tvpSimulationNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/tvpSimulationNode.lua)

Dieser Node ist aktuell nur auf den sda10d eingestellt. Dies kann aber leicht noch erweitert werden auf beliebige roboter typen.
Der Node startet für jede controller gruppe, die in config spezfiziert ist, einen tvp-controler.
Die config tabele muss mit der controller.yaml übereinstimmen damit moveit trajektorien abspielen kann.
Die Joint states aktuallisieren sich mit einem feedback delay (parameters können noch nicht von aussen gestzt werden):

- delay: 0.150 sec
- cycleTime: 0.008 sec

### Jogging in JointSpace

 Node: [rosJoggingMotions](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/rosJoggingMotions.lua)

#### Topics

- ```/joggingJoystickServer/jogging_command```
  - Message: trajectory_msgs/JointTrajectory

#### Services

- ```/joggingJoystickServer/get_controller_name```
  - Message: xamlamoveit_msgs/GetSelected
  - Response: in `selected` wird der actuelle Controller Namespace mitgeteilt

- ```/joggingJoystickServer/get_movegroup_name```
  - Message: xamlamoveit_msgs/GetSelected
  - Response:
    - `selected` actuelle MoveGroup
    - `collection` alle möglichen MoveGroups

- ```/joggingJoystickServer/set_movegroup_name```
  - Message: xamlamoveit_msgs/SetString
  - Request:
    - `data` new MoveGroup name
  - Response:
    - `success`  indicate successful run of triggered service
    - `message` informational, e.g. for error messages

- ```/joggingJoystickServer/set_controller_name```
  - Message: xamlamoveit_msgs/SetString
  - Response:
    - `selected` actuelle Controller Namespace

- ```/joggingJoystickServer/start_stop_tracking```
  - Message: std_srvs/SetBool
  - Response:

- ```/joggingJoystickServer/status```
  - Message: xamlamoveit_msgs/StatusController
  - Response:
    - ```bool is_running```
    - ```string in_topic```
    - ```string out_topic```
    - ```string move_group_name```
    - ```string[] joint_names```
    - ```string status_message_tracking```

- ```/joggingJoystickServer/set_velocity_limits```
  - NOT IMPLEMENTED

- ```/joggingJoystickServer/get_velocity_limits```
  - NOT IMPLEMENTED

Simple test script: [testJoggingController](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/tests/testJoggingController.lua)

- press m to start. Dann 1 - 7 oder (-1) - (-7)
- press s to stop.
- press q to quit-

## Based on torch-moveit

This project uses fragments of [torch-moveit](https://github.com/xamla/torch-moveit).
