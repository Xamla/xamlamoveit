--[[
moveL.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local motionLibrary = require 'xamlamoveit.motionLibrary'
local poseClass = require 'xamlamoveit.datatypes.Pose'

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
A = poseClass.new()
A:setFrame(end_effector_link_name)
A:setTranslation(torch.Tensor {0.02, 0.0, 0.01})
B = poseClass.new()
B:setFrame(end_effector_link_name)
B:setTranslation(torch.Tensor {-0.02, 0.02, -0.01})
C = poseClass.new()
C:setFrame(end_effector_link_name)
C:setTranslation(torch.Tensor {0.02, 0.0, 0.01})
D = poseClass.new()
D:setFrame(end_effector_link_name)
D:setTranslation(torch.Tensor {-0.01, 0.01, -0.01})
E = poseClass.new()
E:setFrame(end_effector_link_name)
E:setTranslation(torch.Tensor {-0.01, -0.01, 0.01})
F = poseClass.new()
F:setFrame(end_effector_link_name)
F:setTranslation(torch.Tensor { 0.0, -0.02, -0.01})

local velocity_scaling = 1
local acceleration_scaling = 1
local check_for_collisions = true
--Start motion
for i, target in ipairs({A, B, C, D, E, F}) do
    xamla_ee:movePoseLinear(target, velocity_scaling, check_for_collisions, acceleration_scaling)
end

-- shutdown ROS
sp:stop()

ros.shutdown()
