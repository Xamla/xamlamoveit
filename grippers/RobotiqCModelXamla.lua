-- module gripper_interface

-- Import section
local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib


local grippers = require 'xamlamoveit.grippers.env'
local RobotiqCModelXamla = torch.class('xamla.moveit.grippers.RobotiqCModelXamla', grippers)
function RobotiqCModelXamla:__init (nodehandle, nodeIDstring)
  self.serviceGripperCtl = nil
  self.actionGripperActivation = nil
  self.actionGripperSetPosition = nil
  self.nodeID=nodeIDstring or "XamlaEgomo"
  self.nh = nodehandle
end


function RobotiqCModelXamla:connect ()
  local nodehandle = self.nh -- generic node handle
  print("/"..self.nodeID.."/SendCommand")
  self.serviceGripperCtl = nodehandle:serviceClient("/"..self.nodeID.."/SendCommand", "egomo_msgs/SendCommand")
  self.actionGripperActivation = actionlib.SimpleActionClient('egomo_msgs/EgomoGripperActivate', "/" .. self.nodeID .. "/gripper_activation_action", nodehandle)
  self.actionGripperSetPosition = actionlib.SimpleActionClient('egomo_msgs/EgomoGripperPos', "/" .. self.nodeID .. "/gripper_pos_action", nodehandle)
  print(self.serviceGripperCtl.spec)
  return true
end


--reset (reset is executed only on the transition 1 -> 0 (falling edge))
function RobotiqCModelXamla:reset ()
  self:SetMoveSpeed(255)
  self:sentGripperMessage("reset", 1)
  sys.sleep(0.5) -- TODO is this still necessary?
  self:sentGripperMessage("reset", 0)
  self:SetMoveSpeed(255)  -- set defaults for movement speed
  self:SetGripForce(100)  -- and gripping force
end


function RobotiqCModelXamla:resetViaAction(execute_timeout, preempt_timeout)
  local goal = self.actionGripperActivation:createGoal()
  goal.activate = true
  return self.actionGripperActivation:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end


function RobotiqCModelXamla:open ()
  return self:move(0.087)
end


function RobotiqCModelXamla:openViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = 0.087
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end


function RobotiqCModelXamla:close ()
  return self:move(0.0)
end


function RobotiqCModelXamla:closeViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = 0.0
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end


function RobotiqCModelXamla:move (value)
  return self:sentGripperMessage ("pos_cmd", value)
end


function RobotiqCModelXamla:moveViaAction(pos_goal, execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = pos_goal
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end


function RobotiqCModelXamla:SetMoveSpeed(value)
  value = value or 255
  return self:sentGripperMessage ("max_speed", value)
end


function RobotiqCModelXamla:SetGripForce(value)
  value = value or 190
  --max_force (0-255)
  return self:sentGripperMessage ("max_force", value)
end


function RobotiqCModelXamla:sentGripperMessage (name, value)
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

return RobotiqCModelXamla
