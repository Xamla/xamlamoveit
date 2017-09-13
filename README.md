# XamlaMoveit

Provides specialized planning functions and sensore interfaces.


## MoveJ commands
MoveJ Kommandos sollen immer von einem definierten Startpunkt zu einem definierten Endpunkt führen.
Hierbei sollen bestimmte Garantien erfüllt werden. Genauigkeit bei der Trajectory-Verfolgung, sowohl in Räumlichen als auch Zeitlichen Rahmen, müssen in gewissen Tolleranzen bleiben.

### global parameters:
- path_execution_tolerance and max_start_point_distance_tolerance (gegebenenfals gleich?): Dies ist die Fehlertoleranz wärend der Ausführung der Trajectorie, in radians.
- duration_tolerance: Diese Property soll die Tolereanz in sek. angeben, die der Roboter hat, um den letzten Punkt der Trajektorie zu erreichen der Kommandiert wurde.
- goal_tolerance: Toleranz für jedes Joint um den Zielzustand zu erreichen. Wenn die Zielposition mit +/- goal_tolerance erreicht wird, war die Ausführung erfolgreich.

### meta parameter für movej mit optim path planner
- max Vel,Acc <- pro joint
- bool collisionCheck
- double maxDeviation: Gibt an wie genau die Wegpunkte angefahren werden sollen unter Berücksichtigung von max Vel,Acc.

## Todo

### Execution Monitoring
- was wurde als command geschickt?
- was wurde gefahren
- detection von der initialen Roboterbewegung zur t0 Festlegung

### NodeServer:
Plänne und Jogging Kommandos sollen hier durchgehen:
- jogging wird nur activiert wenn alle actions aus sind
- moveJ nur mit start und goal 0 velocity am ende und start
- Collision Checks mit distanz informationen


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
