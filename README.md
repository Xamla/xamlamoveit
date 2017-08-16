# XamlaMoveit

Provides specialized planning functions and sensore interfaces.

## Todo:

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
    - Message: xamlamoveit_msgs/GetSelected
    - Response:
        - `selected` actuel active MoveGroup
        - `collection` alle möglichen MoveGroups

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
    -  NOT IMPLEMENTED

- ```/joggingJoystickServer/get_velocity_limits```
    -  NOT IMPLEMENTED


## Based on torch-moveit

This project uses fragments of [torch-moveit](https://github.com/xamla/torch-moveit).
