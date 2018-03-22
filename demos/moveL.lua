local ros = require 'ros'
local tf = ros.tf
local xutils = require 'xamlamoveit.xutils'

--[[
    This example works with meca500
    for other setup settings adjustments are necessary
    look for 'Adapt according to your setup'
]]

--Init ros node
ros.init('MoveL')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local mc = require 'xamlamoveit.motionLibrary'.MotionService(nh)

--Query necessary information about setup
local move_group_names, move_group_details = mc:queryAvailableMoveGroups()

--Select one moveit move group
local move_group = move_group_names[1]

--Define Xamla Movegroup
local xamla_mg = require 'xamlamoveit.motionLibrary'.MoveGroup(mc, move_group) -- motion client
local end_effector = xamla_mg:getEndEffector()
local end_effector_name = end_effector.name
local end_effector_link_name = end_effector.link_name
--Specify targets relative to 'end_effector_link_name'
local A, B, C, D, E, F
A = tf.StampedTransform()
A:set_frame_id(end_effector_link_name)
A:setOrigin(torch.Tensor {0.02, 0.0, 0.01})
B = tf.StampedTransform()
B:set_frame_id(end_effector_link_name)
B:setOrigin(torch.Tensor {-0.02, 0.02, -0.01})
C = tf.StampedTransform()
C:set_frame_id(end_effector_link_name)
C:setOrigin(torch.Tensor {0.02, 0.0, 0.01})
D = tf.StampedTransform()
D:set_frame_id(end_effector_link_name)
D:setOrigin(torch.Tensor {-0.01, 0.01, -0.01})
E = tf.StampedTransform()
E:set_frame_id(end_effector_link_name)
E:setOrigin(torch.Tensor {-0.01, -0.01, 0.01})
F = tf.StampedTransform()
F:set_frame_id(end_effector_link_name)
F:setOrigin(torch.Tensor { 0.0, -0.02, -0.01})

local velocity_scaling = 1
local check_for_collisions = true
--Start motion
for i, target in ipairs({A, B, C, D, E, F}) do
    xamla_mg:moveL(end_effector_name, end_effector_link_name, target, velocity_scaling, check_for_collisions)
end

-- shutdown ROS
sp:stop()

ros.shutdown()
