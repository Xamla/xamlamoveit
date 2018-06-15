
## Xamlamoveit

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This package containts extentions to the ros/moveit framework, which is used in the robot programming IDE ROSVITA for motion generation.

License information can be found in the LICENSE file.

This distribution may include materials developed by third parties.
For license and attribution notices for these materials, please refer to the LICENSE file.

For more information on Rosvita, visit
  http://xamla.com/en/

Xamlamoveit is brought to you by the robotics team at Xamla.

#### Major features are:

1. [Resource locking system](#resource-locking-system)
2. [Time optimal trajectory generation](#time-optimal-trajectory-generation)
3. [Supervised trajectory execution with user interaction](#supervised-trajectory-execution-with-user-interaction)

### Resource locking system

Multiple agents may try to use resources in one robotic setup. To avoid crosstalk a leased base locking system is available in xamlamoveit. The following code example shows how to use it in your program.

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
lock_client:release(lock.resources, lock.id)

-- clean up
sp:stop()
ros.shutdown()

```

### Time optimal trajectory generation

This framework provides moveL (executes trajectory planed in cartesian space) and moveJ (executes trajectory planed in joint space) motion calles. Both can be parameterized with velocity and acceleration limits in an tvp planning component.

#### Code example moveJ (full example can be found in `xamlamoveit/demos/moveJ.lua`)
```lua
--Create you dedicated motion service
local motion_service = motionLibrary.MotionService(nh)
local move_group_name = 'controller'
--Query necessary information about setup
local move_group_names, move_groups_details = motion_service:queryAvailableMoveGroups()
assert(table.indexof(move_group_names, move_group_name) > 0, 'Setup has no move group with name: ' .. move_group_name)

--Define Xamla move group
local xamla_mg = motionLibrary.MoveGroup(motion_service, move_group_name) -- motion client

local MoveJTest01
--{...} -- create joint value target
local MoveJTest02
--{...} -- create joint value target

local velocity_scaling = 1
local check_for_collisions = true
--Start motion
for i = 1, 10 do
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest01, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest02, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
end
```

#### Code example moveL (full example can be found in `xamlamoveit/demos/moveL.lua`)
```lua
--Create you dedicated motion service
local motion_service = motionLibrary.MotionService(nh)

--Query necessary information about setup
local end_effector_names, end_effector_details = motion_service:queryAvailableEndEffectors()

--Select one MoveIt end effector
local end_effector_name = end_effector_names[1]

--Define Xamla move group
local move_group_name = end_effector_details[end_effector_name].move_group_name
local xamla_mg = motionLibrary.MoveGroup(motion_service, move_group_name) -- motion client
local xamla_ee = xamla_mg:getEndEffector(end_effector_name)

--Specify targets relative to 'end_effector_link_name'.
local end_effector_link_name = xamla_ee.link_name
local A, B, C, D, E, F
--{...} -- create targets

local velocity_scaling = 1
local acceleration_scaling = 1
local check_for_collisions = true
--Start motion
for i, target in ipairs({A, B, C, D, E, F}) do
    xamla_ee:movePoseLinear(target, velocity_scaling, check_for_collisions, acceleration_scaling)
end
```

### Supervised trajectory execution with user interaction

The aforementioned motion calles can also be executed in a step-by-step fashon. Especialy, for recovery or debug motions these functions can be used.

#### Code example can be found in `xamlamoveit/demos/moveInSteps.lua`

```lua
--Start motion
local handle = xamla_ee:movePoseLinearSupervised(target, velocity_scaling, check_for_collisions,acceleration_scaling, done_cb)

-- step forward
handle:next()

-- step backward
handle:previous()

--abort motion
handle:abort()

```

### start xamlamoveit without rosvita

#### prerequirements

This project uses fragments of [torch-moveit](https://github.com/xamla/torch-moveit).

#### Launch
Ros-Moveit setup needs to be started.
Then start all xamla services/actions with:

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