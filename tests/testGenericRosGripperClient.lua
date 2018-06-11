local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local grippers = require 'xamlamoveit.grippers.env'

ros.init("generic_gripper_test")
local node_handle = ros.NodeHandle()

local gripper = grippers.GenericRosGripperClient.new(node_handle, '/xamla/wsg_driver/wsg50/gripper_command')

local t = gripper:move(0.005)
print('move', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
print(t:getResult().gripper_status)

t = gripper:move(0.03, 0.2)
print('move', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
print(t:getResult().gripper_status)

t = gripper:grasp()
print('grasp', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
print(t:getResult().gripper_status)

t = gripper:release(0.04)
print('release', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
print(t:getResult().gripper_status)

gripper:shutdown()
node_handle:shutdown()
ros.shutdown()
