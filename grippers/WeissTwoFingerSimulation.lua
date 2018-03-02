local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local core = require 'xamlamoveit.core'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local WeissTwoFingerSimulation = torch.class('xamlamoveit.grippers.WeissTwoFingerSimulation', grippers)

WeissTwoFingerSimulation.ERROR_TYPE = {
  INITIALIZATION_FAILED = 'INITIALIZATION_FAILED',
  JOINT_MONITOR_NOT_READY = 'JOINT_MONITOR_NOT_READY'
}

WeissTwoFingerSimulation.GRASPING_STATE_ID = {
  IDLE = 0,
  GRASPING = 1,
  NO_PART_FOUND = 2,
  PART_LOST = 3,
  HOLDING = 4,
  RELEASING = 5,
  RELEASING = 6,
  ERROR = 7
}

WeissTwoFingerSimulation.GRASPING_STATE = {
  "IDLE", "GRASPING", "NO_PART_FOUND", "PART_LOST", "HOLDING", "RELEASING", "RELEASING", "ERROR"
}

WeissTwoFingerSimulation.COMMAND_ID = {
  STOP = 100,
  MOVE = 101,
  GRASP = 102,
  RELEASE = 103,
  HOMING = 104,
  ACKNOWLEDGE_ERROR = 105,
}

WeissTwoFingerSimulation.WORKER_RESPONSE = {
  COMPLETED = 0,
  ABORTED = 1,
  CANCELED = 2
}

local function initializeActionServerAndServices(self)
  local set_value_spec = ros.SrvSpec('wsg_50_common/SetValue')
  local get_status_spec = ros.SrvSpec('wsg_50_common/GetGripperStatus')
  local empty_service_spec = ros.SrvSpec('std_srvs/Empty')

  -- Initialize publisher
  self.state_spec = ros.MsgSpec('wsg_50_common/Status')
  self.state_publisher = self.node_handle:advertise('status', self.state_spec, 10)

  -- Initialize services
  self.services = {}
  self.services.set_acceleration = self.node_handle:advertiseService(
    'set_accelartion',
    set_value_spec,
    function(request, response, header) return self:handleSetService(request, response, header) end
  )
  self.services.set_force = self.node_handle:advertiseService(
    'set_force',
    set_value_spec,
    function(request, response, header) return self:handleSetService(request, response, header) end
  )
  self.services.acknowledge_error = self.node_handle:advertiseService(
    'acknowledge_error',
    empty_service_spec,
    function(request, response, header) return self:handleEmptyService(request, response, header) end
  )
  self.services.soft_stop = self.node_handle:advertiseService(
    'soft_stop',
    empty_service_spec,
    function(request, response, header) return self:handleEmptyService(request, response, header) end
  )
  self.services.fast_stop = self.node_handle:advertiseService(
    'fast_stop',
    empty_service_spec,
    function(request, response, header) return self:handleEmptyService(request, response, header) end
  )
  self.services.get_gripper_status = self.node_handle:advertiseService(
    'get_gripper_status',
    get_status_spec,
    function(request, response, header) return self:handleGetStatus(request, response, header) end
  )

  -- Intialize action server
  self.action_server = actionlib.ActionServer(self.node_handle, 'gripper_control', 'wsg_50_common/Command')
  self.action_server:registerGoalCallback(
    function(goal_handle) self:handleGoalCallback(goal_handle) end
  )
  self.action_server:registerCancelCallback(
    function(goal_handle) self:handleCancleCallback(goal_handle) end
  )
  self.action_server:start()

  self.joint_monitor = core.JointMonitor({self.actuated_joint_name})

  -- Intialize joint monitor
  local ready = false
  local last_print = ros.Time(0)
  local counter = 0
  while not ready and ros.ok() do
      ready = self.joint_monitor:waitReady(20.0)
      if ros.Time.now():toSec() - last_print:toSec() > 1 then
          ros.INFO('JointMonitor waits for first update of joint: %s.', self.actuated_joint_name)
          last_print = ros.Time.now()
          counter = counter + 1
      end
      if counter > 30 then
          ros.ERROR('Joint monitor did not get ready in the desired time. Failed to receive update for joint: %s', gripper_joint_name)
          error(WeissTwoFingerSimulation.ERROR_TYPE.JOINT_MONITOR_NOT_READY)
      end
  end

  self.joint_command_node_handle = ros.NodeHandle(self.joint_command_namespace)
  self.joint_command_worker = core.JointCommandWorker.new(self.joint_command_node_handle, self.joint_monitor)
end


function WeissTwoFingerSimulation:__init(node_handle, joint_command_namespace, actuated_joint_name)
  self.joint_command_namespace = joint_command_namespace
  self.actuated_joint_name = actuated_joint_name
  self.node_handle = node_handle
  self.gripper_sim = gripper_sim
  self.current_state = {
    grasping_state_id = WeissTwoFingerSimulation.GRASPING_STATE_ID.IDLE,
    moving_gripper = false
  }
  self.default_values = {
    grasping_force = 10
  }

  local ok, err = pcall(function() initializeActionServerAndServices(self) end)
  if not ok then
    ros.ERROR(string.format('Initialization of WeissTwoFingerSimulation has failed: %s', err))
    error(WeissTwoFingerSimulation.ERROR_TYPE.INITIALIZATION_FAILED)
  end
end


function WeissTwoFingerSimulation:createResult(goal_handle)
  local result = goal_handle:createResult()
  local current_positions = self.joint_monitor:getPositionsOrderedTensor(joint_names)
  result.status.width = current_positions[1] * 2.0
  result.status.return_code = 0
  result.status.connection_state = 1

  return result;
end


function WeissTwoFingerSimulation:dispatchJointCommand(joint_value)
  local callbacks = {
    accept = function() return true end,
    proceed = function() return self.current_state.proceed end,
    cancel = function() self:handleJointCommandFinished(WeissTwoFingerSimulation.WORKER_RESPONSE.CANCELED) end,
    abort = function() self:handleJointCommandFinished(WeissTwoFingerSimulation.WORKER_RESPONSE.ABORTED) end,
    completed = function(traj)
      self:handleJointCommandFinished(WeissTwoFingerSimulation.WORKER_RESPONSE.COMPLETED, traj)
    end
  }
  self.joint_command_worker:setCallbacks(callbacks)
  self.joint_command_worker:move({self.actuated_joint_name}, {joint_value})
end


function WeissTwoFingerSimulation:handleCancleCallback(goal_handle)
  if self.current_state.moving_gripper == true then
    self.current_state.proceed = false
  end
end


function WeissTwoFingerSimulation:handleEmptyService(request, response, header)
  return true
end


function WeissTwoFingerSimulation:handleGetStatus(request, response, header)
  response.status = self.gripper_sim:getStatusResponse()
  return true
end


function WeissTwoFingerSimulation:handleGoalCallback(goal_handle)
  if goal_handle ~= nil and goal_handle.goal ~= nil then
    print(goal_handle.goal)
    if self.current_state.moving_gripper == true then
      goal_command:setRejected('Gripper is already executing a command.')
      return
    end

    if self.current_state.grasping_state_id == WeissTwoFingerSimulation.GRASPING_STATE_ID.ERROR then
      goal_command:setRejected('Gripper is in error state. You need to call the acknowlede_error service.')
      return
    end

    print(goal_handle.goal.goal.command)
    local goal_command = goal_handle.goal.goal.command
    if goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.MOVE then
      self:handleMoveCommand(goal_handle)
    elseif goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.GRASP then
      self:handleGraspCommand(goal_handle)
    elseif goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.RELEASE then
      self:handleMoveCommand(goal_handle)
    elseif goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.HOMING then
      goal_handle.goal.goal.command.width = 0
      self:handleMoveCommand(goal_handle)
    elseif goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.STOP or
      goal_command.command_id == WeissTwoFingerSimulation.COMMAND_ID.ACKNOWLEDGE_ERROR
    then
      goal_handle:setAccepted()
      local result = self:createResult(goal_handle)
      goal_handle:setSucceeded(result)
    else
      local out = goal_command or -1
      ros.WARN('Received invalid command id: %d', out)
      goal_handle:setRejected()
    end
  else
    ros.WARN('Received invalid goal.')
  end
end


function WeissTwoFingerSimulation:handleGraspCommand(goal_handle)
  local goal_command = goal_handle.goal.goal.command
  self.current_state.moving_gripper = true
  self.current_state.target_grasping_state_id = WeissTwoFingerSimulation.GRASPING_STATE_ID.HOLDING
  self.current_state.target_force = self.default_values.grasping_force
  self.current_state.time_of_joint_command = ros.Time.now()
  self.current_state.goal_handle = goal_handle
  self.current_state.proceed = true

  self:dispatchJointCommand(goal_command.width / 2.0)
  goal_handle:setAccepted()
end


function WeissTwoFingerSimulation:handleJointCommandFinished(worker_response, trajectory)
  self.current_state.moving_gripper = false

  if self.current_state.goal_handle == nil then
    ros.ERROR('Joint worker finished, but goal handle is nil. Cannot complete goal. Try again.')
    return
  end

  local result = self:createResult(self.current_state.goal_handle)

  if worker_response == WeissTwoFingerSimulation.WORKER_RESPONSE.COMPLETED then
    self.current_state.grasping_state_id = self.current_state.target_grasping_state_id
    result.status.current_force = self.current_state.target_force
    result.status.grasping_state_id = self.current_state.target_grasping_state_id
    result.status.grasping_state = WeissTwoFingerSimulation.GRASPING_STATE[self.current_state.target_grasping_state_id + 1]

    self.current_state.goal_handle:setSucceeded(result)
  else
    self.current_state.grasping_state_id = WeissTwoFingerSimulation.GRASPING_STATE_ID.ERROR
    result.status.current_force = 0
    result.status.grasping_state_id = WeissTwoFingerSimulation.GRASPING_STATE_ID.ERROR
    result.status.grasping_state = WeissTwoFingerSimulation.GRASPING_STATE[self.current_state.target_grasping_state_id + 1]

    if worker_response == WeissTwoFingerSimulation.WORKER_RESPONSE.CANCELED then
      self.current_state.goal_handle:setCanceled(result)
    else
      self.current_state.goal_handle:setAborted(result)
    end
  end

  self.current_state.goal_handle = nil
  self.current_state.target_grasping_state_id = nil
  self.current_state.target_force = nil
end


function WeissTwoFingerSimulation:handleMoveCommand(goal_handle)
  local goal_command = goal_handle.goal.goal.command
  self.current_state.moving_gripper = true
  self.current_state.target_grasping_state_id = WeissTwoFingerSimulation.GRASPING_STATE_ID.IDLE
  self.current_state.target_force = 0
  self.current_state.time_of_joint_command = ros.Time.now()
  self.current_state.goal_handle = goal_handle
  self.current_state.proceed = true

  self:dispatchJointCommand(goal_command.width / 2.0)
  goal_handle:setAccepted()
end


function WeissTwoFingerSimulation:handleSetService(request, response, header)
  response.error = 0
  return true
end


function WeissTwoFingerSimulation:shutdown()
  if self.action_server ~= nil then
    self.action_server:shutdown()
  end

  if self.state_publisher ~= nil then
    self.state_publisher:shutdown()
  end

  if self.joint_command_node_handle ~= nil then
    self.joint_command_node_handle:shutdown()
  end

  for k, v in pairs(self.services) do
    if v ~= nil then
      v:shutdown()
    end
  end
end


function WeissTwoFingerSimulation:spin()
  self.joint_command_worker:spin()
end


return WeissTwoFingerSimulation
