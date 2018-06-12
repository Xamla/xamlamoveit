local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local GenericRosGripperClient = torch.class('xamlamoveit.grippers.GenericRosGripperClient', grippers)
local Task = ros.std.Task
local TaskState = ros.std.TaskState

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


local function doneCallbackHandler(task, action_type, goal_state, goal_result)
  ros.INFO('%s action completed with state: %d (%s)', action_type, goal_state, SimpleClientGoalState[goal_state])
  print(goal_result)
end


local function waitForCommand(task, action_type, timeout)
  timeout = timeout or 5000
  local completed_in_time = task:waitForCompletion(timeout)
  if not completed_in_time then
    ros.ERROR('%s timed out.', action_type)
    task:cancel(string.format('%s command timed out.', action_type))
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


--- Moves the gripper asynchronous
-- @param args table which allows the following arguments: width, force, stop_on_block
-- @return ros.std.Task
function GenericRosGripperClient:moveAsync(args)
  if args == nil then
    args = {}
  end

  args.width = args.width or 0
  args.force = args.force or 0
  if args.stop_on_block == nil then
    args.stop_on_block = true
  end

  local start_handler = function(task)
    local done_callback = function(goal_state, goal_result)
      if goal_result ~= nil and goal_result.stalled ~= nil and goal_result.reached_goal ~= nil then
        if goal_result.stalled == false and goal_result.reached_goal == true then
          task:setResult(TaskState.Succeeded, {gripper_status = goal_result, error_message = ''})
        else
          if args.stop_on_block == true then
            local stalled = 'false'
            if goal_result.stalled == true then
              stalled = 'true'
            end
            local reached_goal = 'false'
            if goal_result.reached_goal == true then
              reached_goal = 'true'
            end
            task:setResult(TaskState.Failed, {gripper_status = goal_result, error_message = string.format('Gripper did not reach goal. Action server reported: \'%s\', Stalled: %s, Reached goal: %s', SimpleClientGoalState[goal_state], stalled, reached_goal)})
          else
            task:setResult(TaskState.Succeeded, {gripper_status = goal_result, error_message = ''})
          end
        end
      else
        task:setResult(TaskState.Failed, {gripper_status = goal_result, error_message = 'Action server returned invalid result.'})
      end
    end

    if self.action_client:waitForServer(ros.Duration(5.0)) then
      local g = self.action_client:createGoal()
      g.command.position = args.width
      g.command.max_effort = args.force
      self.action_client:sendGoal(g, done_callback)
    else
      ros.ERROR("Could not contact gripper action server")
      task:setResult(TaskState.Failed, {error_message = string.format('Could not contact action server: \'%s\'', self.gripper_action)})
    end
  end

  local cancel_handler = function(task)
    self.action_client:cancelGoal()
  end

  return Task.create(start_handler, cancel_handler, nil, true)
end


function GenericRosGripperClient:waitForMove(task, timeout)
  waitForCommand(task, 'Move', timeout)
end


function GenericRosGripperClient:waitForRelease(task, timeout)
  waitForCommand(task, 'Release', timeout)
end


function GenericRosGripperClient:waitForGrasp(task, timeout)
  waitForCommand(task, 'Grasp', timeout)
end


--- Moves the gripper and waits for the task to end
-- @param args table which allows the following arguments: width, force, stop_on_block, timeout
-- @return ros.std.Task
function GenericRosGripperClient:move(args)
  if args == nil then
    args = {}
  end

  args.timeout = args.timeout or 5000
  local task = self:moveAsync(args)
  local sucess = pcall(function () self:waitForMove(task, args.timeout) end)
  return task
end


function GenericRosGripperClient:shutdown()
  if self.action_client ~= nil then
    self.action_client:shutdown()
  end
end


--- Performs a release and waits for the task to complete
-- @param args table which allows the following arguments: width, timeout
-- @return ros.std.Task
function GenericRosGripperClient:release(args)
  if args == nil then
    args = {}
  end

  args.timeout = args.timeout or 5000
  local task = self:releaseAsync(args)
  local sucess = pcall(function () self:waitForRelease(task, args.timeout) end)
  return task
end


--- Performs a grasp and waits for the task
-- @param args table which allows the following arguments: width, force, timeout
-- @return ros.std.Task
function GenericRosGripperClient:grasp(args)
  if args == nil then
    args = {}
  end

  args.timeout = args.timeout or 5000
  local task = self:graspAsync(args)
  local sucess = pcall(function () self:waitForGrasp(task, args.timeout) end)
  return task
end


--- Performs a release asynchronous
-- @param args table which allows the following arguments: width
-- @return ros.std.Task
function GenericRosGripperClient:releaseAsync(args)
  if args == nil then
    args = {}
  end
  args.width = args.width or 0.04
  args.force = args.force or 1
  return self:moveAsync(args)
end


--- Performs a grasp asynchronous
-- @param args table which allows the following arguments: width,  force
-- @return ros.std.Task
function GenericRosGripperClient:graspAsync(args)
  if args == nil then
    args = {}
  end
  args.width = args.width or  0.0025
  args.force = args.force or 10
  args.stop_on_block = false

  return self:moveAsync(args)
end


function GenericRosGripperClient:home()
  return Task.create(function(task) ros.WARN("home not implemented") task:setResult(TaskState.Succeeded, {error_message = ''}) end, nil, nil, true)
end


function GenericRosGripperClient:homeAsync()
  return Task.create(function() ros.WARN("homeAsync not implemented") task:setResult(TaskState.Succeeded, {error_message = ''}) end, nil, nil, true)
end


function GenericRosGripperClient:waitForHome(task, timeout)
  waitForCommand(task, 'Home', timeout)
end


return GenericRosGripperClient
