local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local timer = torch.Timer()
local planning = require 'xamlamoveit.planning'
local xutils = require 'xamlamoveit.xutils'
local clamp = xutils.clamp

local SPEED_LIMIT = { fast = 1, slow = 0.2 }
local SENSITIVITY_THRESHOLD = 0.2
local currentSpeedLimit = 'slow'


local axesValues = {}
local stepSizeZ = 1.
local stepSizeR = 0.1

local publisherPointPositionCtrl
local lastJoystickMessage


local test_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
-- JointPosition --
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')

function jointCommandCb(self, msg, header)
  local joint_state_names = self.joint_monitor:getJointNames()
  if #msg.points > 0 then
    for i,name in ipairs(msg.joint_names) do
      local index = table.indexof(joint_state_names, name)
      if index > -1 then
        self.lastCommandJointVelocity[self.positionNameMap[name]] = msg.points[1].velocities[i]
      end
    end
    self.seq = self.seq + 1
    self.new_message = true
  end
end

---
--@param desired joint angle position
local function sendPositionCommand(q_des, q_dot, group)
  ros.INFO('sendPositionCommand')
  local m = ros.Message(joint_pos_spec)
  local mPoint = ros.Message(test_spec)
  local names = std.StringVector()

  group:getActiveJoints(names)

  m.joint_names = {}
  for ii = 1, q_des:size(1) do
    m.joint_names[ii] = names[ii]
  end
  mPoint.positions:set(q_des)
  mPoint.velocities:set(q_dot)
  mPoint.time_from_start = ros.Duration(0.1)
  m.points = {mPoint}

  publisherPointPositionCtrl:publish(m)
end

local function createVariableNameMap(self)
  local variable_names = self.state:getVariableNames()
  local joint_state_names = self.joint_monitor:getJointNames()
  local map = {}
  local counter = 1
  for i,v in ipairs(variable_names) do
    local index = table.indexof(joint_state_names, v)
    if index > -1 then
      map[v] = counter
      counter = counter + 1
    end
  end
  return map
end


local max_acc = nil
local max_vel = nil
local controller = require 'xamlamoveit.controller.env'
local JointJoggingController = torch.class('xamlamoveit.controller.JointJoggingController',controller)

function JointJoggingController:__init(node_handle, move_group, ctr_name, dt, debug)

  self.debug = debug or false
  if self.debug then
    ros.console.set_logger_level(nil,ros.console.Level.Debug)
  end

  self.CONVERED = false
  self.FIRSTPOINT = true
  self.nh = node_handle
  self.q_des = nil
  self.move_group = move_group or error('move_group should not be nil')
  self.state = move_group:getCurrentState()
  self.lastCommandJointPositons = self.state:copyJointGroupPositions(move_group:getName()):clone()
  self.lastCommandJointVelocity = torch.zeros(self.lastCommandJointPositons:size())
  self.joint_monitor = xutils.JointMonitor(move_group:getActiveJoints():totable())
  self.positionNameMap = createVariableNameMap(self)
  self.time_last = ros.Time.now()

  if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
    self.dt = ros.Duration(dt)
  elseif torch.type(dt) == 'nil' then
    self.dt = ros.Duration(0.01)
  elseif torch.type(dt) == 'ros.Duration' then
    ros.INFO("rosDuration as dt received")
    self.dt = dt
  else
    error('dt has unsupported type')
  end


  self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
  self.start_time = ros.Time.now()
  BEGIN_EXECUTION = ros.Time.now()

  self.controller_name = ctr_name or 'pos_based_pos_traj_controller'
  self.seq = 1
  self.new_message = false

  self.robot_model_loader = moveit.RobotModelLoader("robot_description")
  self.kinematic_model = self.robot_model_loader:getModel()
  self.planning_scene = moveit.PlanningScene(self.kinematic_model)
  self.planning_scene:syncPlanningScene()
  print(self.kinematic_model:printModelInfo())
end

local function satisfiesBounds(self, positions)
  local state = self.state:clone()
  state:setVariablePositions(positions, self.joint_monitor:getJointNames())
  self.planning_scene:syncPlanningScene()
  local collisions = self.planning_scene:checkSelfCollision(state) or self.planning_scene:isStateColliding(self.move_group,state)
  if state:satisfiesBounds(0.0) then
    if collisions then
      ros.ERROR("Self Collision detected")
      return false, 'Self Collision detected!!'
    end
  else
    state:enforceBounds()
    positions:copy(state:copyJointGroupPositions(self.move_group:getName()):clone())
    collisions = self.planning_scene:checkSelfCollision(state)
    if not collisions then
      ros.WARN('Target position is out of bounds!!')
      return true, 'Target position is out of bounds!!'
    else
      return false, 'Self Collision detected!!'
    end
  end
  return true, 'Success'
end

--@param desired joint angle position
function JointJoggingController:isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
  local diff = 2
  if q_des:nDimension() > 0 then
    ros.DEBUG('q_des checked')
    if satisfiesBounds(self, q_des) then
      ros.DEBUG('satisfiesBounds')
      if q_curr then
        diff= torch.norm(q_curr-q_des)
      end
    end
  end
  ros.DEBUG(string.format('Difference between q_des and q_curr diff = %f', diff))
  return diff<1
end


function JointJoggingController:connect(topic, timeout)
  local timeout = timeout
  if torch.type(timeout) == 'number' or torch.type(timeout) == 'nil' then
    timeout = ros.Duration(timeout)
  elseif torch.type(timeout) == 'nil' then
    timeout = ros.Duration(0.01)
  elseif torch.type(timeout) == 'ros.Duration' then
  else
    error('dt has unsupported type')
  end

  local topic = topic or 'joy'
  self.subscriber_jog = self.nh:subscribe(topic, joint_pos_spec , 1)
  self.subscriber_jog:registerCallback(function (msg, header) jointCommandCb(self, msg, header) end)
  ros.INFO(string.format('Subscribed to \'%s\' node. Please start using your jogging device.',topic))
  local ready = self.joint_monitor:waitReady()
  if not ready then
    return false, 'Could not collect joint states'
  end
  self:getNewRobotState()
  return true, 'Success'
end


function JointJoggingController:getInTopic()
  return self.subscriber_jog:getTopic()
end


function JointJoggingController:getOutTopic()
  return string.format('/%s/joint_command',self.controller_name)
end


function JointJoggingController:shutdown()
  self.subscriber_jog:shutdown()
end


function JointJoggingController:updateDeltaT()
    local dt = ros.Time.now() - self.time_last
    while dt:toSec() < self.dt:toSec() do -- dt set to 125Hz
      dt = ros.Time.now() - self.time_last
    end
    self.time_last = ros.Time.now();
    ros.DEBUG(string.format('Delta Time: %dHz', 1/self.dt:toSec()))
end


function JointJoggingController:tracking(q_dot, duration)
  if type(duration) == 'number' then
    duration = ros.Duration(duration)
  end

  duration = duration or ros.Time.now() - BEGIN_EXECUTION
  local group = self.move_group
  local state = self.state:clone()

  local q_des = self.lastCommandJointPositons + q_dot
  if not self.CONVERED or self.FIRSTPOINT then
    if self:isValid(q_des, self.lastCommandJointPositons) then
      if not publisherPointPositionCtrl then
        local myTopic = string.format('%s/joint_command',self.controller_name)
        ros.WARN(myTopic)
        publisherPointPositionCtrl = self.nh:advertise(myTopic, joint_pos_spec)
      end
      --if self.FIRSTPOINT then
      --  sendPositionCommand(self.lastCommandJointPositons, q_dot:zero(), group, duration)
      --  self.FIRSTPOINT = false
      --else
        sendPositionCommand(q_des, q_dot, group, duration)
      --end
      self.lastCommandJointPositons:copy(q_des)
    else
      if publisherPointPositionCtrl then
        publisherPointPositionCtrl:shutdown()
      end
      publisherPointPositionCtrl = nil
      ros.ERROR('command is not valid!!!')
      return false, 'command is not valid!!!'
    end
  end

  if self.debug then
    ros.DEBUG('q_curr')
    ros.DEBUG(tostring(self.lastCommandJointPositons))
    ros.DEBUG('q_des')
    ros.DEBUG(tostring(q_des))
    ros.DEBUG('q_dot')
    ros.DEBUG((tostring(q_dot)))
  end

  if q_dot:norm() < 1e-12 then
    self.CONVERED = true
    --BEGIN_EXECUTION = ros.Time.now()
    self.FIRSTPOINT = true
  else
    self.CONVERED = false
    self.FIRSTPOINT = false
    ros.WARN('tracking is NOT CONVERED')
  end
  return true, 'success'
end


function JointJoggingController:getNewRobotState()
  local p,l = self.joint_monitor:getNextPositionsTensor(self.dt*2)
  self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
  self.lastCommandJointPositons:copy(p)
end


function JointJoggingController:update()
  self:getNewRobotState()
  if self.CONVERED then
    self.start_time = ros.Time.now()
  end

  self:updateDeltaT()

  local curr_time = ros.Time.now()
  local succ, msg = true, "IDLE"
  if self.new_message and ros.ok() then
    succ, msg = self:tracking(self.lastCommandJointVelocity:clone(), self.dt)
    print(self.lastCommandJointVelocity)
    self.lastCommandJointVelocity:zero()
    self.new_message = false
  end

  return succ, msg
end


function JointJoggingController:setMoveGroupInterface(name)
  self.move_group = moveit.MoveGroupInterface(name)
  local ready = self:reset(ros.Duration(0.01))
  local move_group_name = self.move_group:getName()
  if ready and (move_group_name == name) then
    return true, 'Successfully changed movegroup.'
  else
    if move_group_name ~= name then
      return false, string.Format('Could not set moveGroup with name: %s. ',name)
    end
    if not ready then
      return false, 'joint state is not available in time.'
    end
  end
end


function JointJoggingController:reset(timeout)
  self.state = self.move_group:getCurrentState()
  self.lastCommandJointPositons = self.state:copyJointGroupPositions(self.move_group:getName()):clone()
  self.lastCommandJointVelocity = self.lastCommandJointPositons:clone():zeros()
  self.joint_monitor:shutdown()
  self.joint_monitor = xutils.JointMonitor(self.move_group:getActiveJoints():totable())
  self.positionNameMap = createVariableNameMap(self)
  self.time_last = ros.Time.now()
  return self.joint_monitor:waitReady(timeout or ros.Duration(0.1))
end


function JointJoggingController:setSpeed(speed)
  ros.INFO(string.format('Set speed to %s', speed))
end


return JointJoggingController
