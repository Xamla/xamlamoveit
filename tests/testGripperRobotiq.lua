ros = require 'ros'
moveit = require 'moveit'
grippers = require 'xamlamoveit.grippers'
tf = ros.tf
ros.init('MoveTest')
-- What does this command?
local nodehandle = ros.NodeHandle()
local sp = ros.AsyncSpinner()  -- background job
sp:start()
robotiqGripper = grippers.RobotiqCModelXamla(nodehandle)
robotiqGripper:connect()
robotiqGripper:reset()
robotiqGripper:close()
ros.spinOnce()

robotiqGripper:open()
ros.spinOnce()


robotiqGripper:move(0.0)
ros.spinOnce()

sys.sleep(1.0)

ros.shutdown()

