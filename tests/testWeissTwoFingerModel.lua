local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local grippers = require 'xamlamoveit.grippers.env'

ros.init("weiss_gripper_test")
local node_handle = ros.NodeHandle()

local gripper = grippers.WeissTwoFingerModel.new(node_handle, '/xamla/wsg_driver/wsg50', 'gripper_control')

local t = gripper:home()
print('home', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
t = gripper:move(0.04)
print('move', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
t = gripper:grasp(0.01, 0.02)
print('grasp', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
t = gripper:release(0.02, 0.02)
print('release', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

gripper:shutdown()
node_handle:shutdown()
ros.shutdown()
