local ros = require "ros"
require "ros.actionlib.SimpleActionClient"
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local Task = ros.std.Task
local TaskState = ros.std.TaskState

local WeissTwoFingerModel = torch.class("xamlamoveit.grippers.WeissTwoFingerModel", grippers)
WeissTwoFingerModel.ERROR_TYPE = {
  INITIALIZATION_FAILED = "INITIALIZATION_FAILED",
  ACKNOWLEDGE_SERVICE_TIMEOUT = "ACKNOWLEDGE_SERVICE_TIMEOUT",
  STATUS_SERVICE_TIMEOUT = "STATUS_SERVICE_TIMEOUT",
}

local WeissWsgReturnCode = {
  "E_SUCCESS",
  "E_NOT_AVAILABLE",
  "E_NO_SENSOR",
  "E_NOT_INITIALIZED",
  "E_ALREADY_RUNNING",
  "E_FEATURE_NOT_SUPPORTED",
  "E_INCONSISTENT_DATA",
  "E_TIMEOUT",
  "E_READ_ERROR",
  "E_WRITE_ERROR",
  "E_INSUFFICIENT_RESOURCES",
  "E_CHECKSUM_ERROR",
  "E_NO_PARAM_EXPECTED",
  "E_NOT_ENOUGH_PARAMS",
  "E_CMD_UNKNOWN",
  "E_CMD_FORMAT_ERROR",
  "E_ACCESS_DENIED",
  "E_ALREADY_OPEN",
  "E_CMD_FAILED",
  "E_CMD_ABORTED",
  "E_INVALID_HANDLE",
  "E_NOT_FOUND",
  "E_NOT_OPEN",
  "E_IO_ERROR",
  "E_INVALID_PARAMETER",
  "E_INDEX_OUT_OF_BOUNDS",
  "E_CMD_PENDING",
  "E_OVERRUN",
  "E_RANGE_ERROR",
  "E_AXIS_BLOCKED",
  "E_FILE_EXISTS",
}

local ConnectionState = {
  NotConnected = 0,
  Connected = 1,
  'NOT_CONNECTED', 'CONNECTED',
}

local GraspingState = {
  Idle = 0,
  Grasping = 1,
  NoPartFound = 2,
  PartLost = 3,
  Holding = 4,
  Releasing = 5,
  Positioning = 6,
  Error = 7,
  Unkown = -1,
  'IDLE', 'GRASPING', 'NO_PART_FOUND', 'PART_LOST', 'HOLDING', 'RELEASING', 'POSITIONING', 'ERROR'
}

local GripperCommand = {
  Move = 101,
  Grasp = 102,
  Release = 103,
  Homing = 104,
}


local function initializeActionClient(self, gripper_action_name)
  ros.INFO('Connect to gripper action server \'%s\'', gripper_action_name)
  self.gripper_action_client = actionlib.SimpleActionClient('wsg_50_common/Command', gripper_action_name, self.nh)
end


function WeissTwoFingerModel:__init(nodehandle, gripper_namespace, gripper_action_name)
    self.nh = nodehandle
    self.gripperStatus = nil
    self.gripper_status_client = self.nh:serviceClient(gripper_namespace .. '/get_gripper_status', 'wsg_50_common/GetGripperStatus')
    self.ack_error_client = self.nh:serviceClient(gripper_namespace .. '/acknowledge_error', 'std_srvs/Empty')
    self.gripper_action = gripper_namespace .. '/' .. gripper_action_name

    local ok, err = pcall(function() initializeActionClient(self, self.gripper_action) end)
    if not ok then
      ros.ERROR('Initialization of gripper action client has failed: ' .. err)
      error(WeissTwoFingerModel.ERROR_TYPE.INITIALIZATION_FAILED)
    end
end


function WeissTwoFingerModel:acknowledgeError()
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


local function readGripperStatus(goal_result)
  local gripper_return_code = '<No return code>'
  local gripper_grasping_state = 'Unkown state'
  local gripper_connection_state = 'Unkown state'

  if goal_result and goal_result.status then
    if goal_result.status.return_code then
      gripper_return_code = WeissWsgReturnCode[goal_result.status.return_code + 1]
    end

    if goal_result.status.grasping_state_id and goal_result.status.grasping_state_id ~= -1 then
      gripper_grasping_state = GraspingState[goal_result.status.grasping_state_id + 1]
    end

    if goal_result.status.connection_state then
      gripper_connection_state = ConnectionState[goal_result.status.connection_state + 1]
    end
  end

  return gripper_return_code, gripper_grasping_state, gripper_connection_state
end


local function doneCallbackHandler(task, action_type, goal_state, goal_result)
  ros.INFO('%s action completed with state: %d (%s)', action_type, goal_state, SimpleClientGoalState[goal_state])
  local status
  if goal_result and goal_result.status then
    status = goal_result.status
  end

  if goal_state == SimpleClientGoalState.SUCCEEDED then
    task:setResult(TaskState.Succeeded, {gripper_status = status, error_message = ''})
  elseif goal_state == SimpleClientGoalState.RECALLED or goal_state == SimpleClientGoalState.PREEMPTED then
    task:setResult(TaskState.Cancelled, {gripper_status = status, error_message = string.format('%s was cancelled. Action client reported: \'%s\'', action_type, SimpleClientGoalState[goal_state])})
  elseif goal_state == SimpleClientGoalState.REJECTED or goal_state == SimpleClientGoalState.ABORTED or goal_state == SimpleClientGoalState.LOST then
    local return_code, grasping_state, gripper_connection_state = readGripperStatus(goal_result)
    task:setResult(TaskState.Failed, {gripper_status = status, error_message = string.format('%s failed. Action client reported: \'%s\'. Connection state: \'%s\', Grasping state: \'%s\', Command return code: \'%s\'', action_type, SimpleClientGoalState[goal_state], gripper_connection_state, grasping_state, return_code)})
  else
    local return_code, grasping_state, gripper_connection_state = readGripperStatus(goal_result)
    task:setResult(TaskState.Failed, {gripper_status = status, error_message = string.format('%s failed with unkown error. Action client reported: \'%s\'. Connection state: \'%s\', Grasping state: \'%s\', Command return code: \'%s\'', action_type, SimpleClientGoalState[goal_state], gripper_connection_state, grasping_state, return_code)})
  end
end


local function waitForCommand(task, action_type, timeout_in_ms)
  timeout_in_ms = timeout_in_ms or 5000
  local completed_in_time = task:waitForCompletion(timeout_in_ms)
  if not completed_in_time then
    ros.ERROR('%s timed out.', action_type)
    task:cancel()
    error(string.format('%s command timed out.', action_type))
  end

  if task:hasCompletedSuccessfully() then
    ros.INFO('%s was successful.', action_type)
  else
    local result = task:getResult()
    local err = result.error_message or 'Unkown error'
    ros.ERROR('%s failed. The following error was reported: \'%s\'', action_type, err)
    error(result.error_message)
  end
end


function WeissTwoFingerModel:homeGripperAsync()
  local start_handler = function(task)
    local done_callback = function(goal_state, goal_result)
      doneCallbackHandler(task, 'Home', goal_state, goal_result)
    end

    if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
      local g = self.gripper_action_client:createGoal()
      g.command.command_id = GripperCommand.Homing -- 104
      self.gripper_action_client:sendGoal(g, done_callback)
    else
      ros.ERROR("Could not contact gripper action server")
      task:setResult(TaskState.Failed, {error_message = string.format('Could not contact action server: \'%s\'', self.gripper_action)})
    end
  end

  local cancel_handler = function(task)
    self.gripper_action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


function WeissTwoFingerModel:waitForHome(task, timeout_in_ms)
  waitForCommand(task, 'Home', timeout_in_ms)
end


function WeissTwoFingerModel:home(timeout_in_ms)
  local task = self:homeGripperAsync()
  local success = pcall(function() self:waitForHome(task, timeout_in_ms) end)
  return task
end


function WeissTwoFingerModel:releaseAsync(width, speed)
  width = width or 0.05
  speed = speed or 0.2

  local start_handler = function(task)
    local done_callback = function(goal_state, goal_result)
      doneCallbackHandler(task, 'Release', goal_state, goal_result)
    end

    if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
      local g = self.gripper_action_client:createGoal()
      g.command.command_id = GripperCommand.Release -- 103
      g.command.width = width
      g.command.speed = speed
      self.gripper_action_client:sendGoal(g, done_callback)
    else
      ros.ERROR("Could not contact gripper action server")
      task:setResult(TaskState.Failed, {error_message = string.format('Could not contact action server: \'%s\'', self.gripper_action)})
    end
  end

  local cancel_handler = function(task)
    self.gripper_action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


function WeissTwoFingerModel:waitForRelease(task, timeout_in_ms)
  waitForCommand(task, 'Release', timeout_in_ms)
end


function WeissTwoFingerModel:release(width, speed, timeout_in_ms)
  width = width or 0.05
  speed = speed or 0.2
  timeout_in_ms = timeout_in_ms or 5000
  local task = self:releaseAsync(width, speed)
  local success = pcall(function() self:waitForRelease(task, timeout_in_ms) end)
  return task
end


function WeissTwoFingerModel:graspAsync(width, speed, force)
  width = width or 0.001
  force = force or 10
  speed = speed or 0.2

  local start_handler = function(task)
    local done_callback = function(goal_state, goal_result)
      doneCallbackHandler(task, 'Grasp', goal_state, goal_result)
    end

    if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
      local g = self.gripper_action_client:createGoal()
      g.command.command_id = GripperCommand.Grasp -- 102
      g.command.width = width
      g.command.speed = speed
      g.command.force = force
      self.gripper_action_client:sendGoal(g, done_callback)
    else
      ros.ERROR("Could not contact gripper action server")
      task:setResult(TaskState.Failed, {error_message = string.format('Could not contact action server: \'%s\'', self.gripper_action)})
    end
  end

  local cancel_handler = function(task)
    self.gripper_action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


function WeissTwoFingerModel:waitForGrasp(task, timeout_in_ms)
  waitForCommand(task, 'Grasp', timeout_in_ms)
end


function WeissTwoFingerModel:grasp(width, speed, force, timeout_in_ms)
  width = width or 0.001
  force = force or 10
  speed = speed or 0.2
  timeout_in_ms = timeout_in_ms or 5000
  local task = self:graspAsync(width, speed, force)
  local success = pcall(function() self:waitForGrasp(task, timeout_in_ms) end)
  return task
end


function WeissTwoFingerModel:moveAsync(width, speed)
  width = width or 0.001
  force = 0
  speed = speed or 0.2

  local start_handler = function(task)
    local done_callback = function(goal_state, goal_result)
      doneCallbackHandler(task, 'Move', goal_state, goal_result)
    end

    if self.gripper_action_client:waitForServer(ros.Duration(5.0)) then
      local g = self.gripper_action_client:createGoal()
      g.command.command_id = GripperCommand.Move -- 101
      g.command.width = width
      g.command.speed = speed
      g.command.force = force
      self.gripper_action_client:sendGoal(g, done_callback)
    else
      ros.ERROR("Could not contact gripper action server")
      task:setResult(TaskState.Failed, {error_message = string.format('Could not contact action server: \'%s\'', self.gripper_action)})
    end
  end

  local cancel_handler = function(task)
    self.gripper_action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


function WeissTwoFingerModel:waitForMove(task, timeout_in_ms)
  waitForCommand(task, 'Move', timeout_in_ms)
end


function WeissTwoFingerModel:move(width, speed, timeout_in_ms)
  width = width or 0.001
  speed = speed or 0.2
  timeout_in_ms = timeout_in_ms or 5000
  local task = self:moveAsync(width, speed)
  local success = pcall(function() self:waitForMove(task, timeout_in_ms) end)
  return task
end


function WeissTwoFingerModel:shutdown()
  if self.gripper_action_client ~= nil then
    self.gripper_action_client:shutdown()
  end
end


return WeissTwoFingerModel
