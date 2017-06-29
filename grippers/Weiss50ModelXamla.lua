-- module gripper_interface

-- Import section
local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib


local grippers = require 'xamlamoveit.grippers.env'
local Weiss50ModelXamla = torch.class('xamla.moveit.grippers.Weiss50ModelXamla', grippers)
function Weiss50ModelXamla:__init (nodehandle, nodeIDstring)
  self.serviceGripperCtl = nil
  self.actionGripperActivation = nil
  self.actionGripperSetPosition = nil
  self.nodeID=nodeIDstring or "XamlaEgomo"
  self.nh = nodehandle
  self.gripperServices = {}
  self.speed_value = 10
end



function Weiss50ModelXamla:connect ()
  local nodehandle = self.nh -- generic node handle
  local init_b = true
  while init_b do
    ros.INFO("Try to connect Ack")
    self.gripperServices.wsgAck = nodehandle:serviceClient('/wsg_25_driver/ack', 'std_srvs/Empty')
    ros.INFO("Try to connect Grasp")
    self.gripperServices.wsgGrasp = nodehandle:serviceClient('/wsg_25_driver/grasp', 'wsg_25_common/Move')
    ros.INFO("Try to connect Homing")
    self.gripperServices.wsgHoming = nodehandle:serviceClient('/wsg_25_driver/homing', 'std_srvs/Empty')
    ros.INFO("Try to connect Move")
    self.gripperServices.wsgMove = nodehandle:serviceClient('/wsg_25_driver/move', 'wsg_25_common/Move')
    ros.INFO("Try to connect Move_incrementally")
    self.gripperServices.wsgMove_incrementally = nodehandle:serviceClient('/wsg_25_driver/move_incrementally', 'wsg_25_common/Incr')
    ros.INFO("Try to connect Release")
    self.gripperServices.wsgRelease = nodehandle:serviceClient('/wsg_25_driver/release', 'wsg_25_common/Move')
    ros.INFO("Try to connect Acceleration")
    self.gripperServices.wsgSetAcceleration = nodehandle:serviceClient('/wsg_25_driver/set_acceleration', 'wsg_25_common/Conf')
    ros.INFO("Try to connect SetForce")
    self.gripperServices.wsgSetForce = nodehandle:serviceClient('/wsg_25_driver/set_force', 'wsg_25_common/Conf')
    ros.INFO("Check connection.")
    if self.gripperServices.wsgAck:exists() then
      break
    else
      ros.WARN("cannot connect to services retrying ...")
      sys.sleep(0.5)
    end
  end
  -- call the service
  local response = self.gripperServices.wsgAck:call()
end


--reset (reset is executed only on the transition 1 -> 0 (falling edge))
function Weiss50ModelXamla:reset ()
    -- call the service
  self.gripperServices.wsgAck:call()
  self.gripperServices.wsgHoming:call()
  return true
end


function Weiss50ModelXamla:resetViaAction(execute_timeout, preempt_timeout)
  ros.WARN("[Weiss50ModelXamla:resetViaAction] Not implemented")
  return false
end


function Weiss50ModelXamla:open ()
  local req_msg =self.gripperServices.wsgRelease:createRequest()
  req_msg:fillFromTable({width=100, speed=self.speed_value})
  local response = self.gripperServices.wsgRelease:call(req_msg)
  return true
end


function Weiss50ModelXamla:openViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
    ros.WARN("[Weiss50ModelXamla:openViaAction] Not implemented")
  return false
end


function Weiss50ModelXamla:close ()
  local req_msg =self.gripperServices.wsgGrasp:createRequest()
  req_msg:fillFromTable({width=11, speed=self.speed_value})
  local response = self.gripperServices.wsgGrasp:call(req_msg)
  return true
end


function Weiss50ModelXamla:closeViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  ros.WARN("[Weiss50ModelXamla:closeViaAction] Not implemented")
  return false
end


function Weiss50ModelXamla:move (value)
  local req_msg =self.gripperServices.wsgMove:createRequest()
  req_msg:fillFromTable({width=value*1000, speed=self.speed_value})
  local response = self.gripperServices.wsgMove:call(req_msg)
  
  return true
end


function Weiss50ModelXamla:moveViaAction(pos_goal, execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  ros.WARN("[Weiss50ModelXamla:moveViaAction] Not implemented")
  return false
end


function Weiss50ModelXamla:SetMoveSpeed(value)
  value = value or 5.1
  self.speed_value = value
  return true
end


function Weiss50ModelXamla:SetGripForce(value)
  value = value or 190
  local req_msg = self.gripperServices.wsgSetForce:createRequest()
  req_msg:fillFromTable({val=value})  
  local response = self.gripperServices.wsgSetForce:call(req_msg)
  return response.error == 0
end


function Weiss50ModelXamla:sentGripperMessage (name, value)
  local m=ros.Message(self.serviceGripperCtl.spec.request_spec) -- get a message template of that service
  m.command_name=name
  m.value=value

  -- send the message
  local msgGripper = self.serviceGripperCtl:call(m)
  if msgGripper == nil then
    --print("Command failed")
    return nil
  end

  return msgGripper
end

return Weiss50ModelXamla
