# XamlaMoveit

This package containts extentions to the ros/moveit framework, which is used in ROSVITA for the motion generation.

Major features are:

1. Resource locking system
2. Time optimal trajectory generation
3. Teleoperation functionality with diverse modalities
4. Supervised trajectory execution for debugging with user interaction

## Resource locking system

Code example
```lua
--- Import libs
local ros = require 'ros'
local core = require 'xamlamoveit.core'
local LeasedBaseLockClient = core.LeasedBaseLockClient

local my_resources = {'JointA', 'CamA', 'Sensor'}
--- Create node
ros.init('lockTest')
local nh = ros.NodeHandle('~')
local sp = ros.AsyncSpinner() -- background job
sp:start()

--- Create lock client
local lock_client = LeasedBaseLockClient(node_handle)
local lock = nil

-- get lock
lock = lock_client:lock(my_resources)

-- update lock
lock = lock_client:lock(lock.resources, lock.id)

-- release lock
lock_client:release (lock.resources, lock.id)

-- clean up
sp:stop()
ros.shutdown()

```

## Time optimal trajectory generation

This framework provides moveL (executes trajectory planed in cartesian space), moveJ (executes trajectory planed in joint space) motion calles. Both can be parameterized with velocity and acceleration limits in an tvp planning component.

Code example moveJ
```lua
```

Code example moveL
```lua
```

## Teleoperation functionality with diverse modalities

Code example
```lua
```

## Supervised trajectory execution for debugging with user interaction

Code example can be found in `xamlamoveit/demos/moveInSteps.lua`

## start xamlamoveit

### prerequirements

Ros-Moveit setup needs to be started.

### Launch-File

start all xamla services/actions with:

```
<?xml version="1.0" encoding="utf-8" standalone="yes"?>
<launch>
  <group>
    <node name="xamlaMoveActions" pkg="xamlamoveit_server" type="moveJActionServer.lua" output="screen" />
    <node name="xamlaMoveGroupServices" pkg="xamlamoveit_server" type="moveGroupServices.lua" output="screen" />
    <node name="xamlaPlanningServices" pkg="xamlamoveit_server" type="planningServices.lua" output="screen" />
    <node name="xamlaResourceLockService" pkg="xamlamoveit_server" type="resourceLockService.lua" output="screen" />
  </group>
</launch>
```

## Heartbeats

If all components are healthy can be monitored by `xamla_sysmon`.
Configure the xamla_sysmon node to listen to these topics:

```
/xamlaJointMonitor/heartbeat
/xamlaMoveActions/heartbeat
/xamlaMoveGroupServices/heartbeat
/xamlaPlanningServices/heartbeat
/xamlaResourceLockService/heartbeat
```

### Robot simulation

Node: [tvpSimulationControllerNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/tvpSimulationControllerNode.lua)

This node expects on the ros param server a `controller_list` in its namespace.
 Here all joints for each controller are specified. Each joint get then published to `joint_states`. The Joint values are zero initialized. A tvp controller is used to simulate the kinematics of the joints using the velocitiy limits specified in the `urdf`.

The Joint states can be published with a feedback delay and a specific refresh rate:

- delay: 0.150 sec
- cycleTime: 0.008 sec

Node: [tvpSimulationActionNode](https://github.com/Xamla/Rosvita.Control/blob/master/lua/xamlamoveit/actionNodes/tvpSimulationActionNode.lua)

This nodes organizes `follow_trajectory_actions` used by moveit and will connect to the `tvpSimulationControllerNode`

These two nodes are unmanaged controller nodes which enables to test your setup with moveit with out having the real robot in the loop.

## Based on torch-moveit

This project uses fragments of [torch-moveit](https://github.com/xamla/torch-moveit).
