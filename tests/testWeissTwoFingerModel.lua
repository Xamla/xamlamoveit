local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local grippers = require 'xamlamoveit.grippers.env'

ros.init("weiss_gripper_test")
local node_handle = ros.NodeHandle()

local gripper = grippers.WeissTwoFingerModel.new(node_handle, '/xamla/wsg_driver/wsg50', 'gripper_control')
-- local gripper = grippers.GenericRosGripperClient.new(node_handle, '/xamla/wsg_driver/wsg50/gripper_command')

local t = gripper:home()
print('home', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)
t = gripper:move{width=0.04, speed=0.2, force=0}
print('move open', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

sys.sleep(5)

-- grasp with unknown part width
t = gripper:move{width=0.0, speed=0.02, force=20, stop_on_block=false}
print('move grasp', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

sys.sleep(5)

t = gripper:move{width=0.04}
print('move open', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

sys.sleep(5)

t = gripper:grasp{width=0.03, speed=0.02}
print('grasp', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

sys.sleep(5)

t = gripper:release{width=0.04, speed=0.02}
print('release', t:hasCompleted(), t:hasCompletedSuccessfully(), t:getResult().error_message)

gripper:shutdown()
node_handle:shutdown()
ros.shutdown()
