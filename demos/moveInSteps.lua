local ros = require 'ros'
local tf = ros.tf
local xutils = require 'xamlamoveit.xutils'

--[[
    This example works with meca500
    for other setup settings adjustments are necessary
    look for 'Adapt according to your setup'
]]

--Init ros node
ros.init('MoveInSteps')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local mc = require 'xamlamoveit.motionLibrary'.MotionService(nh)

--Query necessary information about setup
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()

--Select one moveit move group
local move_group = move_group_names[1]
local end_effector_name = 'EE_manipulator_meca' --Adapt according to your setup
local end_effector_link_name = 'meca_wrist_3_link' --Adapt according to your setup

--Define Xamla Movegroup
local xamla_mg = require 'xamlamoveit.motionLibrary'.MoveGroup(mc, move_group) -- motion client

--Specify target relative to 'end_effector_link_name'
local target = tf.StampedTransform()
target:set_frame_id(end_effector_link_name)
target:setOrigin(torch.Tensor {0.02, 0.0, 0.01}) -- meter

local velocity_scaling = 1
local check_for_collisions = true
--Start motion
xamla_mg:steppedMoveL(end_effector_name, end_effector_link_name, target, velocity_scaling, check_for_collisions)

-- shutdown ROS
sp:stop()

ros.shutdown()
