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
weissGripper.speed_value = 50
while not weissGripper:connect() and ros.ok() do
    ros.ERROR("cannot connect")
    ros.spinOnce()
end

if weissGripper:hasError () then
    ros.ERROR("error detected trigger reset")
    weissGripper:reset ()
end
ros.spinOnce()
--ros.INFO("reset")
--weissGripper:reset()
weissGripper:SetGripForce(80)
ros.spinOnce()
if not weissGripper:isOpen () then
    ros.INFO("gripper should be open")
    weissGripper:open()
end
ros.INFO("close")
weissGripper:close()

ros.spinOnce()
weissGripper:SetGripForce(80)
ros.spinOnce()
if not weissGripper:isOpen () then
    ros.INFO("open " .. weissGripper.seq)
    weissGripper:open()
end
ros.spinOnce()


weissGripper:move(0.0)
ros.spinOnce()

sys.sleep(1.0)

ros.shutdown()

