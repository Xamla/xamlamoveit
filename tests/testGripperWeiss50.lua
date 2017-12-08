ros = require 'ros'
moveit = require 'moveit'
grippers = require 'xamlamoveit.grippers'
tf = ros.tf
ros.init('MoveTest')
-- What does this command?

local sp = ros.AsyncSpinner()  -- background job
sp:start()
local nodehandle = ros.NodeHandle()
ros.INFO("INIT")
local weissGripper = grippers.Weiss50ModelXamla(nodehandle)
ros.spinOnce()
ros.INFO("connect")
weissGripper.speed_value = 0.05
local namespace = "/xamla/wsg_25_driver/gripper" -- "xamla/wsg50_driver/wsg50"
local actionname = "gripper_controller"
while not weissGripper:connect(namespace, actionname) and ros.ok() do
    ros.ERROR("cannot connect")
    ros.spinOnce()
end

--weissGripper:homingViaAction()


    weissGripper:reset()

ros.spinOnce()

--ros.INFO("reset")
--weissGripper:reset()
--weissGripper:SetGripForce(80)
ros.spinOnce()
if not weissGripper:isOpen (0.05) then
    ros.INFO("gripper should be open")
    ros.spinOnce()
    weissGripper:openViaAction(0.04)
end
--[[
ros.INFO("close")
weissGripper:closeViaAction()
]]
--[[
ros.spinOnce()
--weissGripper:SetGripForce(80)
ros.spinOnce()
if not weissGripper:isOpen (0.1) then
    ros.INFO("open " .. weissGripper.seq)
    weissGripper:openViaAction()
end
ros.spinOnce()


weissGripper:moveViaAction(0.01)
ros.spinOnce()

sys.sleep(1.0)
]]
ros.shutdown()

