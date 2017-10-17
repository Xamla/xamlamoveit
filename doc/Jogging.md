### Jogging Nodes

#### Task Space
Node: [rosJoystickServiceNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/rosJoystickServiceNode.lua)

##### Topics

- ```jogging_command```
  - Message: sensor_msgs/Joy

Use XBOX360 Joystick.

#### Konfiguraiton/Joint Space Jogging

 Node: [rosJointJoggingServiceNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/rosJointJoggingServiceNode.lua)

##### Topics

- ```jogging_command```
  - Message: trajectory_msgs/JointTrajectory

Simple test script: [testJoggingController](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/tests/testJoggingController.lua)

- press m to start. Dann 1 - 7 oder (-1) - (-7)
- press s to stop.
- press q to quit-

### Services

- ```get_controller_name```
  - Message: xamlamoveit_msgs/GetSelected
  - Response:
    - `selected` actueller Controller Namespace
    - `collection` alle möglichen Controller (siehe ```controller list``` im ros param server)
- ```get_movegroup_name```
  - Message: xamlamoveit_msgs/GetSelected
  - Response:
    - `selected` actuelle MoveGroup
    - `collection` alle möglichen MoveGroups

- ```set_movegroup_name```
  - Message: xamlamoveit_msgs/SetString
  - Request:
    - `data` new MoveGroup name
  - Response:
    - `success`  indicate successful run of triggered service
    - `message` informational, e.g. for error messages

- ```set_controller_name```
  - Message: xamlamoveit_msgs/SetString
  - Response:
    - `selected` actuelle Controller Namespace

- ```start_stop_tracking```
  - Message: std_srvs/SetBool
  - Response:

- ```status```
  - Message: xamlamoveit_msgs/StatusController
  - Response:
    - ```bool is_running```
    - ```string in_topic```
    - ```string out_topic```
    - ```string move_group_name```
    - ```string[] joint_names```
    - ```string status_message_tracking```

- ```set_velocity_scaling```
  - Message: xamlamoveit_msgs/SetFloat

- ```get_velocity_scaling```
  - Message: xamlamoveit_msgs/GetFloat
