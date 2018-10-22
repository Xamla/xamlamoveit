--[[
moveJ.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local xamlamoveit = require 'xamlamoveit'
local motionLibrary = xamlamoveit.motionLibrary
local JointSet = xamlamoveit.datatypes.JointSet
local JointValues = xamlamoveit.datatypes.JointValues

--[[
    This example works with meca500
    for other setup settings adjustments are necessary
]]
--Init ros node
ros.init('MoveL')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local motion_service = motionLibrary.MotionService(nh)
local move_group_name = 'controller'
--Query necessary information about setup
local move_group_names, move_groups_details = motion_service:queryAvailableMoveGroups()
assert(table.indexof(move_group_names, move_group_name) > 0, 'Setup has no move group with name: ' .. move_group_name)

--Define Xamla move group
local xamla_mg = motionLibrary.MoveGroup(motion_service, move_group_name) -- motion client

local MoveJTest01 =
    JointValues(
    JointSet(
        {
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        }
    ),
    torch.Tensor {
        0.8248696438037854,
        0.525459033218468,
        0.303317589690019,
        0.9728705248327094,
        -1.0943555283416977,
        -0.5926176060964492
    }
)

local MoveJTest02 =
    JointValues(
    JointSet(
        {
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6'
        }
    ),
    torch.Tensor {
        -0.9103169285136579,
        0.6015421475506321,
        0.03215847631624999,
        -1.139648725643356,
        -1.0536745466524786,
        0.8213946485139784
    }
)

local velocity_scaling = 1
local check_for_collisions = true
--Start motion
tic('startExperiment')
for i = 1, 10 do
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest01, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
    ros.INFO('start moveJoints')
    xamla_mg:moveJoints(MoveJTest02, velocity_scaling, check_for_collisions)
    ros.INFO('finished moveJoints')
end
toc('startExperiment')
-- shutdown ROS
sp:stop()

ros.shutdown()
