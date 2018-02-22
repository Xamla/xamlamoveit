local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local grippers = require 'xamlamoveit.grippers.env'

ros.init("generic_gripper_test")
local node_handle = ros.NodeHandle()

local gripper = grippers.GenericRosGripperClient.new(node_handle, '/xamla/robotiq_driver/robotiq2finger85/gripper_command')
print(gripper:tryMoveGripperSync(0.085, 50, 5))
gripper:shutdown()
node_handle:shutdown()
ros.shutdown()
