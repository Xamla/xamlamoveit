local ros = require "ros"
require "ros.actionlib.SimpleActionClient"
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local WeissTwoFingerModel = torch.class("xamlamoveit.grippers.WeissTwoFingerModel", grippers)
WeissTwoFingerModel.ERROR_TYPE = {
  ACTION_CLIENT_TIMEOUT = "ACTION_CLIENT_TIMEOUT",
  ACTION_PREEMPT_TIMEOUT = "ACTION_PREEMPT_TIMEOUT",
  ACTION_NOT_DONE = "ACTION_NOT_DONE",
  NO_RESULT_RECEIVED = "NO_RESULT_RECEIVED",
  INITIALIZATION_FAILED = "INITIALIZATION_FAILED",
  UNKNOWN_ERROR = "UNKOWN_ERROR",
  ACKNOWLEDGE_SERVICE_TIMEOUT = "ACKNOWLEDGE_SERVICE_TIMEOUT",
  STATUS_SERVICE_TIMEOUT = "STATUS_SERVICE_TIMEOUT",
  WRONG_GRIPPER_STATE = "WRONG_GRIPPER_STATE"
}

local GraspingState = {
  Idle = 0,
  NoPartFound = 2,
  PartLost = 3,
  Hold = 4,
}

local GripperCommand = {
  Move = 101,
  Grasp = 102,
  Release = 103,
  Homeing = 104,
}


local function initializeActionClient(self, gripper_action_name)
  self.gripper_action_client = actionlib.SimpleActionClient('wsg_50_common/Command', gripper_action_name, self.nh)
end


function WeissTwoFingerModel:__init(nodehandle, gripper_namespace, gripper_action_name)
    self.nh = nodehandle
    self.gripperStatus = nil
    self.gripper_status_client = self.nh:serviceClient(gripper_namespace .. '/get_gripper_status', 'wsg_50_common/GetGripperStatus')
    self.ack_error_client = self.nh:serviceClient(gripper_namespace .. '/acknowledge_error', 'std_srvs/Empty')

    local ok, err = pcall(function() initializeActionClient(self, gripper_action_name) end)
    if not ok then
      ros.ERROR('Initialization of gripper action client has failed: ' .. err)
      error(WeissTwoFingerModel.ERROR_TYPE.INITIALIZATION_FAILED)
    end
end


function WeissTwoFingerModel:ackGripper()
  if self.ack_error_client:exists() then
    local req = self.ack_error_client:createRequest()
    local r = self.ack_error_client:call(req)
    if r == nil then
        ros.ERROR("Could not contact error acknowledge service")
        error(WeissTwoFingerModel.ERROR_TYPE.ACKNOWLEDGE_SERVICE_TIMEOUT)
    else
        ros.INFO("Acknowledged error of gripper")
    end
  else
    ros.ERROR("Could not contact error acknowledge service")
    error(WeissTwoFingerModel.ERROR_TYPE.ACKNOWLEDGE_SERVICE_TIMEOUT)
  end
end


function WeissTwoFingerModel:getGripperStatus()
  if self.gripper_status_client:exists() then
    local gripper_status = self.gripper_status_client:call()
    if gripper_status == nil then
      ros.ERROR("Could not contact gripper status service")
      error(WeissTwoFingerModel.ERROR_TYPE.STATUS_SERVICE_TIMEOUT)
    else
      self.gripperStatus = gripper_status
      return self.gripperStatus
    end
  else
    ros.ERROR("Could not contact gripper status service")
    error(WeissTwoFingerModel.ERROR_TYPE.STATUS_SERVICE_TIMEOUT)
  end
end


function WeissTwoFingerModel:waitForResult(execute_timeout)
  if self.gripper_action_client:waitForResult(execute_timeout) then
    ros.DEBUG_NAMED("actionlib", "Goal finished within specified execute_timeout [%.2f]", execute_timeout:toSec())
    return self.gripper_action_client:getState()
  end
  ros.DEBUG_NAMED("actionlib", "Goal didn't finish within specified execute_timeout [%.2f]", execute_timeout:toSec())
  -- it didn't finish in time, so we need to preempt it
  self.gripper_action_client:cancelGoal()
  -- now wait again and see if it finishes
  if self.gripper_action_client:waitForResult(execute_timeout) then
    ros.DEBUG_NAMED("actionlib", "Preempt finished within execute_timeout [%.2f]", execute_timeout:toSec())
  else
    ros.DEBUG_NAMED("actionlib", "Preempt didn't finish within execute_timeout [%.2f]", execute_timeout:toSec())
    error(WeissTwoFingerModel.ERROR_TYPE.ACTION_PREEMPT_TIMEOUT)
  end
  return self.gripper_action_client:getState()
end


function WeissTwoFingerModel:homeGripperAsync()
  print('Homeing gripper...')
  if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
    local g = self.gripper_action_client:createGoal()
    g.command.command_id = GripperCommand.Homeing -- 104
    self.gripper_action_client:sendGoal(g)
  else
    ros.ERROR("Could not contact gripper action server.")
    error(WeissTwoFingerModel.ERROR_TYPE.ACTION_CLIENT_TIMEOUT)
  end
end


function WeissTwoFingerModel:waitForHome(execute_timeout)
  local state = self:waitForResult(execute_timeout)
  local result = self.gripper_action_client:getResult()
  -- Check if gripper has been homed successful
  if state == 7 and result ~= nil and result.status ~= nil and result.status.return_code == 0 then
    ros.INFO("Homed gripper successfully")
  else
    ros.ERROR("Could not home gripper")
    if result == nil then
      ros.ERROR("Result message from action server was empty.")
      error(WeissTwoFingerModel.ERROR_TYPE.NO_RESULT_RECEIVED)
    elseif state ~= 7 then
      ros.ERROR("Action has not been suceeded and is in state: %s", SimpleClientGoalState[state])
      error(WeissTwoFingerModel.ERROR_TYPE.ACTION_NOT_DONE)
    else
      ros.ERROR("An unkown error has ocurred.")
      error(WeissTwoFingerModel.ERROR_TYPE.UNKNOWN_ERROR)
    end
  end
end


function WeissTwoFingerModel:homeGripper(execute_timeout)
  self:homeGripperAsync()
  self:waitForHome(execute_timeout)
end


function WeissTwoFingerModel:openGripperAsync(width, force, speed, acceleration)
  width = width or 0.05
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  print('Opening gripper...')
  if self.gripper_status_client:exists() then
    if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
      local gripper_status = self.gripper_status_client:call()
      local g = self.gripper_action_client:createGoal()
      if (gripper_status ~= nil and gripper_status.status ~= nil) then
        if gripper_status.status.grasping_state_id == GraspingState.Idle then
          g.command.command_id = GripperCommand.Move -- 101
        elseif gripper_status.status.grasping_state_id == GraspingState.Hold
          or gripper_status.status.grasping_state_id == GraspingState.NoPartFound
          or gripper_status.status.grasping_state_id == GraspingState.PartLost then
          g.command.command_id = GripperCommand.Release -- 103
        else
          ros.ERROR("Gripper is in wrong state. Please acknowledge any error and use homing. Grasping State: %d", gripper_status.status.grasping_state_id)
          error(WeissTwoFingerModel.ERROR_TYPE.WRONG_GRIPPER_STATE)
        end
      else
        ros.ERROR("Gripper is in wrong state. Please acknowledge any error and use homing. Grasping State: nil")
        error(WeissTwoFingerModel.ERROR_TYPE.WRONG_GRIPPER_STATE)
      end
      g.command.width = width
      g.command.speed = speed
      g.command.acceleration = acceleration
      g.command.force = force
      self.gripper_action_client:sendGoal(g)
    else
      ros.ERROR("Could not contact gripper action server")
      error(WeissTwoFingerModel.ERROR_TYPE.ACTION_CLIENT_TIMEOUT)
    end
  else
    ros.ERROR("Could not query gripper status")
    error(WeissTwoFingerModel.ERROR_TYPE.STATUS_SERVICE_TIMEOUT)
  end
end


function WeissTwoFingerModel:waitForOpen(execute_timeout)
  execute_timeout = execute_timeout or ros.Duration(5.0)
  local state = self:waitForResult(execute_timeout)
  local result = self.gripper_action_client:getResult()
  -- Check if gripper has been opened successful
  if state == 7 and result ~= nil and result.status ~= nil and result.status.return_code == 0 then
    ros.INFO("Opened gripper successfully.")
  else
    ros.ERROR("Could not open gripper.")
    if result == nil then
      ros.ERROR("Result message from action server was empty.")
      error(WeissTwoFingerModel.ERROR_TYPE.NO_RESULT_RECEIVED)
    elseif state ~= 7 then
      ros.ERROR("Action has not been suceeded and is in state: %s", SimpleClientGoalState[state])
      error(WeissTwoFingerModel.ERROR_TYPE.ACTION_NOT_DONE)
    else
      ros.ERROR("An unkown error has ocurred.")
      error(WeissTwoFingerModel.ERROR_TYPE.UNKNOWN_ERROR)
    end
  end
end


function WeissTwoFingerModel:openGripper(width, force, speed, acceleration, execute_timeout)
  width = width or 0.05
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  execute_timeout = execute_timeout or ros.Duration(5.0)
  self:openGripperAsync(width, force, speed, acceleration)
  self:waitForOpen(execute_timeout)
end


function WeissTwoFingerModel:closeGripperAsync(width, force, speed, acceleration)
  width = width or 0.001
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  execute_timeout = execute_timeout or ros.Duration(5.0)
  print('Closing gripper...')
  if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
    local g = self.gripper_action_client:createGoal()
    g.command.command_id = GripperCommand.Grasp -- 102
    g.command.width = width
    g.command.speed = speed
    g.command.acceleration = acceleration
    g.command.force = force
    self.gripper_action_client:sendGoal(g)
  else
    ros.ERROR("Could not contact gripper action server")
    error(WeissTwoFingerModel.ERROR_TYPE.ACTION_CLIENT_TIMEOUT)
  end
end


function WeissTwoFingerModel:waitForClose(execute_timeout)
  execute_timeout = execute_timeout or ros.Duration(5.0)
  local state = self:waitForResult(execute_timeout)
  local result = self.gripper_action_client:getResult()
  -- Check if gripper has been closed successful
  if state == 7 then
    if result ~= nil and result.status ~= nil then
      if result.status.return_code == 0 and result.status.grasping_state_id == GraspingState.Hold then
        ros.INFO("Picked part successfully")
      else
        ros.ERROR("Could not pick part. gripper return code: %d, grasping state %d", result.status.return_code, result.status.grasping_state_id)
        error(WeissTwoFingerModel.ERROR_TYPE.ACTION_NOT_DONE)
      end
    else
      if result == nil then
        ros.ERROR("Result message from action server was empty.")
        error(WeissTwoFingerModel.ERROR_TYPE.NO_RESULT_RECEIVED)
      else
        ros.ERROR("An unkown error has ocurred.")
        error(WeissTwoFingerModel.ERROR_TYPE.UNKNOWN_ERROR)
      end
    end
  else
    if state == 6 then
      if result ~= nil and result.status ~= nil then
        ros.INFO("Closed gripper successfully")
      else
        if result == nil then
          ros.ERROR("Result message from action server was empty.")
          error(WeissTwoFingerModel.ERROR_TYPE.NO_RESULT_RECEIVED)
        else
          ros.ERROR("An unkown error has ocurred.")
          error(WeissTwoFingerModel.ERROR_TYPE.UNKNOWN_ERROR)
        end
      end
    else
      ros.ERROR("Could not close gripper: action server returned state: %d: %s", state, SimpleClientGoalState[state])
      error(WeissTwoFingerModel.ERROR_TYPE.ACTION_NOT_DONE)
    end
  end
end


function WeissTwoFingerModel:closeGripper(width, force, speed, acceleration, execute_timeout)
  width = width or 0.001
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  execute_timeout = execute_timeout or ros.Duration(5.0)
  self:closeGripperAsync(width, force, speed)
  self:waitForClose(execute_timeout)
end


function WeissTwoFingerModel:connect()
  -- could home the gripper at this point
  -- self:homeGripper(execute_timeout)
end


function WeissTwoFingerModel:shutdown()
  if self.gripper_action_client ~= nil then
    self.gripper_action_client:shutdown()
  end
end


function WeissTwoFingerModel:open(width, force, speed, acceleration, execute_timeout_in_s)
  width = width or 0.05
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  execute_timeout_in_s = execute_timeout_in_s or 5
  self:openGripper(width, force, speed, acceleration, ros.Duration(execute_timeout_in_s))
end


function WeissTwoFingerModel:close(width, force, speed, acceleration, execute_timeout_in_s)
  width = width or 0.001
  force = force or 10
  speed = speed or 0.2
  acceleration = acceleration or 0.8
  execute_timeout_in_s = execute_timeout_in_s or 5
  self:closeGripper(width, force, speed, acceleration, ros.Duration(execute_timeout_in_s))
end


return WeissTwoFingerModel
