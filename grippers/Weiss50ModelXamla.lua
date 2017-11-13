-- module gripper_interface

-- Import section
local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local grippers = require 'xamlamoveit.grippers.env'
local Weiss50ModelXamla = torch.class('xamlamoveit.grippers.Weiss50ModelXamla', grippers)
function Weiss50ModelXamla:__init (nodehandle, nodeIDstring)
  self.serviceGripperCtl = nil
  self.actionGripperActivation = nil
  self.actionGripperSetPosition = nil
  self.nodeID= nodeIDstring or "XamlaEgomo"
  self.nh = nodehandle
  self.gripperServices = {}
  self.gripperStatus = nil
  self.speed_value = 10
  self.width = -1
  self.gripState = nil
  self.seq = 0
end

local function stateCb(self, msg, header)
  ros.DEBUG("received message")
  self.width = msg.width
  self.gripState = msg.status_id
  self.seq = self.seq + 1
end

function Weiss50ModelXamla:connect (namespace)
  local nodehandle = self.nh -- generic node handle
  local init_b = true
  local namespace = namespace or "wsg_50_driver"

    self.gripperStatus = self.nh:subscribe(string.format('/%s/status',namespace), 'wsg_50_common/Status', 10)

    self.gripperStatus:registerCallback(function (msg, header) stateCb(self, msg, header) end)
   while init_b and ros.ok() do

    ros.INFO("Try to connect Ack")
    self.gripperServices.wsgAck = nodehandle:serviceClient(string.format('/%s/ack',namespace), 'std_srvs/Empty')
    ros.INFO("Try to connect Grasp")
    self.gripperServices.wsgGrasp = nodehandle:serviceClient(string.format('/%s/grasp',namespace), 'wsg_50_common/Move')
    ros.INFO("Try to connect Homing")
    self.gripperServices.wsgHoming = nodehandle:serviceClient(string.format('/%s/homing',namespace), 'std_srvs/Empty')
    ros.INFO("Try to connect Move")
    self.gripperServices.wsgMove = nodehandle:serviceClient(string.format('/%s/move',namespace), 'wsg_50_common/Move')
    ros.INFO("Try to connect Move_incrementally")
    self.gripperServices.wsgMove_incrementally = nodehandle:serviceClient(string.format('/%s/move_incrementally',namespace), 'wsg_50_common/Incr')
    ros.INFO("Try to connect Release")
    self.gripperServices.wsgRelease = nodehandle:serviceClient(string.format('/%s/release',namespace), 'wsg_50_common/Move')
    ros.INFO("Try to connect Acceleration")
    self.gripperServices.wsgSetAcceleration = nodehandle:serviceClient(string.format('/%s/set_acceleration',namespace), 'wsg_50_common/Conf')
    ros.INFO("Try to connect SetForce")
    self.gripperServices.wsgSetForce = nodehandle:serviceClient(string.format('/%s/set_force',namespace), 'wsg_50_common/Conf')
    ros.INFO("Check connection.")
    self.actionGripper = actionlib.SimpleActionClient('control_msgs/GripperCommand', "/" .. namespace .. "/gripper_cmd", nodehandle)
    ros.spinOnce()

    while not (self.gripperStatus:getNumPublishers() > 0) and ros.ok() do
      ros.ERROR(string.format('no connection to /%s/status',namespace))
      ros.spinOnce()
    end
    while self.seq <= 0 and ros.ok() do
      ros.spinOnce()
    end

    if self.gripperServices.wsgAck:exists() then
      break
    else
      ros.ERROR("cannot connect to services retrying ...")
      sys.sleep(0.5)
      return false
    end

  end
  -- call the service
  local response = self.gripperServices.wsgAck:call()
  return true
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


function Weiss50ModelXamla:open (value)
  local req_msg =self.gripperServices.wsgRelease:createRequest()
  req_msg:fillFromTable({width=value or 100, speed=self.speed_value})
  local response = self.gripperServices.wsgRelease:call(req_msg)
  return true
end


function Weiss50ModelXamla:openViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)

    if not self.actionGripper:isServerConnected() then
        ros.ERROR("no connection to server")
        return false
    end
    local g = self.actionGripper:createGoal()
    g.command.position = 0.05 -- in m
    g.command.max_effort = force or 10.0 -- N
    print(g)
    local done = false

    local function action_done(state, result)
        ros.INFO('actionDone')
        ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
        ros.INFO('Result:\n%s', result)
        done = true
    end

    local function action_active()
        ros.INFO('executeAsync Action_active')
    end

    local function action_feedback(feedback)
        ros.INFO('Action_feedback \n\t%s ', tostring(feedback))
    end
    self.actionGripper:sendGoal(g, action_done, action_active, action_feedback)
    while not done and ros.ok() do
        sys.sleep(0.1)
        ros.spinOnce()
    end
    return true
end


function Weiss50ModelXamla:close (value)
  local req_msg =self.gripperServices.wsgGrasp:createRequest()
  req_msg:fillFromTable({width= value or 11, speed=self.speed_value})
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


function Weiss50ModelXamla:isOpen (width)
  print("seq:".. self.seq)
  print("width:".. self.width)
  return self.width > (width or 100)
end

function Weiss50ModelXamla:hasError ()
  print(self.gripState)
  if self.gripState == ros.Message('wsg_50_common/Status').ERROR then
    return true
  end
  return false
end

return Weiss50ModelXamla
