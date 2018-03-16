local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local GenericRosGripperClient = torch.class('xamlamoveit.grippers.GenericRosGripperClient', grippers)
GenericRosGripperClient.ERROR_TYPE = {
  ACTION_CLIENT_TIMEOUT = "ACTION_CLIENT_TIMEOUT",
  ACTION_PREEMPT_TIMEOUT = "ACTION_PREEMPT_TIMEOUT",
  ACTION_NOT_DONE = "ACTION_NOT_DONE",
  GRIPPER_STALLED = "GRIPPER_STALLED",
  DID_NOT_REACH_GOAL = "DID_NOT_REACH_GOAL",
  NO_RESULT_RECEIVED = "NO_RESULT_RECEIVED",
  INITIALIZATION_FAILED = "INITIALIZATION_FAILED",
  UNKNOWN_ERROR = "UNKOWN_ERROR"
}


local function initializeActionClient(self, gripper_action_name)
  self.action_client = actionlib.SimpleActionClient('control_msgs/GripperCommand', gripper_action_name, self.node_handle)
  print('initializeActionClient')
end


function GenericRosGripperClient:__init(node_handle, gripper_action_name)
  self.node_handle = node_handle
  print('GenericRosGripperClient:__init')
  local ok, err = pcall(function() initializeActionClient(self, gripper_action_name) end)
  if not ok then
    ros.ERROR('Initialization of gripper action client has failed: ' .. err)
    error(GenericRosGripperClient.ERROR_TYPE.INITIALIZATION_FAILED)
  end
end


function GenericRosGripperClient:moveGripperAsync(width_in_m, force_in_n)
  width_in_m = width_in_m or 0
  force_in_n = force_in_n or 0
  if self.action_client:waitForServer(ros.Duration(5.0)) then
    local g = self.action_client:createGoal()

    g.command.position = width_in_m
    g.command.max_effort = force_in_n
    self.action_client:sendGoal(g)
  else
    ros.ERROR("Could not contact gripper action server")
    error(GenericRosGripperClient.error_type.ACTION_CLIENT_TIMEOUT)
  end
end


function GenericRosGripperClient:waitForMove(execute_timeout_in_s)
  local state = self:waitForResult(execute_timeout_in_s)
  local result = self.action_client:getResult()

  -- Check if gripper has been homed successful
  if state == 7 and result ~= nil and result.stalled == false and result.reached_goal == true then
    ros.INFO("Executed move command successfully")
  else
    if result == nil then
      ros.ERROR("Result message from action server was empty.")
      error(GenericRosGripperClient.error_type.NO_RESULT_RECEIVED)
    elseif state ~= 7 then
      ros.ERROR("Action has not been suceeded and is in state: %s", SimpleClientGoalState[state])
      error(GenericRosGripperClient.error_type.ACTION_NOT_DONE)
    elseif result.stalled == true then
      ros.ERROR("Gripper reports that it is stalled.")
      error(GenericRosGripperClient.error_type.GRIPPER_STALLED)
    elseif result.reached_goal == false then
      ros.ERROR("Gripper reports that it has not reached the goal.")
      error(GenericRosGripperClient.error_type.DID_NOT_REACH_GOAL)
    else
      ros.ERROR("An unkown error has ocurred.")
      error(GenericRosGripperClient.error_type.UNKNOWN_ERROR)
    end
  end
end


function GenericRosGripperClient:waitForResult(execute_timeout_in_s)
  if self.action_client:waitForResult(execute_timeout_in_s) then
    return self.action_client:getState()
  end
  ros.DEBUG_NAMED("actionlib", "Goal didn't finish within specified execute_timeout [%.2f]", execute_timeout_in_s:toSec())
  -- it didn't finish in time, so we need to preempt it
  self.action_client:cancelGoal()
  -- now wait again and see if it finishes
  if self.action_client:waitForResult(execute_timeout) then
    ros.DEBUG_NAMED("actionlib", "Preempt finished within execute_timeout [%.2f]", execute_timeout_in_s:toSec())
  else
    ros.DEBUG_NAMED("actionlib", "Preempt didn't finish within execute_timeout [%.2f]", execute_timeout_in_s:toSec())
    error(GenericRosGripperClient.error_type.ACTION_PREEMPT_TIMEOUT)
  end

  return self.action_client:getState()
end


function GenericRosGripperClient:moveGripperSync(width_in_m, force_in_n, execute_timeout_in_s)
  self:moveGripperAsync(width_in_m, force_in_n)
  self:waitForMove(ros.Duration(execute_timeout_in_s))
end


function GenericRosGripperClient:tryMoveGripperSync(width_in_m, force_in_n, execute_timeout_in_s)
  return pcall(
    function()
      self:moveGripperSync(width_in_m, force_in_n, execute_timeout_in_s)
    end
  )
end


function GenericRosGripperClient:shutdown()
  if self.action_client ~= nil then
    self.action_client:shutdown()
  end
end


--[[
    tentative gripper interface:
        connect()
        openSync(width, force, speed, acceleration, execute_timeout)
        closeSync(width, force, speed, acceleration, execute_timeout)
        disconnect()    
]]


function GenericRosGripperClient:connect()
    print('calling interface method GenericRosGripperClient:connect()')
end


function GenericRosGripperClient:disconnect()
    self:shutdown()
end


function GenericRosGripperClient:open()
    local width_open = 0.065    
    local force = 50   
    local execute_timeout = 5
    self:tryMoveGripperSync(width_open, force, execute_timeout)
end


function GenericRosGripperClient:close()
    local width_close = 0.0025    
    local force = 50   
    local execute_timeout = 5
    self:tryMoveGripperSync(width_close, force, execute_timeout)
end


return GenericRosGripperClient
