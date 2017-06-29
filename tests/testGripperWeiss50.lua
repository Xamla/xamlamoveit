ros = require 'ros'
moveit = require 'moveit'
grippers = require 'xamlamoveit.grippers'
tf = ros.tf
ros.init('MoveTest')
-- What does this command?
local nodehandle = ros.NodeHandle()
local sp = ros.AsyncSpinner()  -- background job
sp:start()
print("INIT")
weissGripper = grippers.Weiss50ModelXamla(nodehandle)
ros:spinOnce()
print("connect")
weissGripper:connect()
ros:spinOnce()
--print("reset")
--weissGripper:reset()
weissGripper:SetGripForce(80)
ros:spinOnce()
print("close")
weissGripper:close()
ros:spinOnce()
weissGripper:SetGripForce(80)
ros:spinOnce()
print("open")
weissGripper:open()
ros:spinOnce()


--weissGripper:move(0.0)
ros:spinOnce()

sys.sleep(1.0)

ros.shutdown()

