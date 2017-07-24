local ros = require('ros')
local moveit = require('moveit')
local tf = ros.tf
local timer = torch.Timer()
local planning = require 'xamlamoveit.planning'
local xutils = require 'xamlamoveit.xutils'
local clamp = xutils.clamp

local SPEED_LIMIT = { fast = 1, slow = 0.2 }
local SENSITIVITY_THRESHOLD = 0.2
local currentSpeedLimit = "slow"


local axesValues = {}
local stepSizeZ = 1.
local stepSizeR = 0.1

local publisherPointPositionCtrl
local lastJoystickMessage


local test_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
-- JointPosition --
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
---
--@param desired joint angle position
function sendPositionCommand(q_des, q_dot, group)
  ros.INFO("sendPositionCommand")
  local m = ros.Message(joint_pos_spec)
  local mPoint = ros.Message(test_spec)
  local names = std.StringVector()

  group:getActiveJoints(names)

  m.joint_names = {}
  for ii = 1, q_des:size(1) do
    m.joint_names[ii] = names[ii]
  end
  mPoint.positions:set(q_des)
  mPoint.velocities:set(q_dot) --TODO this is probably not optimal.
  mPoint.time_from_start = ros.Time.now() - BEGIN_EXECUTION
  m.points = {mPoint}

  publisherPointPositionCtrl:publish(m)
end
local max_acc = nil
local max_vel = nil
local controller = require 'xamlamoveit.controller.env'
local JointJoggingController = torch.class("xamlamoveit.controller.JointJoggingController",controller)

function JointJoggingController:__init(node_handle, move_group, ctr_name, dt, debug)

  self.debug = debug or false
  if self.debug then
    ros.console.set_logger_level(nil,ros.console.Level.Debug)
  end

  self.CONVERED = false
  self.FIRSTPOINT = true
  self.nh = node_handle
  self.q_des = nil
  self.move_group = move_group or error("move_group should not be nil")
  self.state = move_group:getCurrentState()
  self.lastCommandJointPositons = self.state:copyJointGroupPositions(move_group:getName()):clone()

  self.joint_monitor = xutils.JointMonitor(move_group:getActiveJoints():totable())
  self.time_last = ros.Time.now()

  if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
    self.dt = ros.Duration(dt)
  elseif torch.type(dt) == 'nil' then
    self.dt = ros.Duration(0.01)
  elseif torch.type(dt) == 'ros.Duration' then
    self.dt = dt
  else
    error("dt has unsupported type")
  end


  self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
  self.start_time = ros.Time.now()
  BEGIN_EXECUTION = ros.Time.now()

  self.controller_name = ctr_name or 'pos_based_pos_traj_controller'
end

local function satisfiesBounds(self, positions)
  local state = self.state:clone()
  state:setVariablePositions(positions, self.joint_monitor:getJointNames())
  if not state:satisfiesBounds() then
    ros.ERROR("Target position is out of bounds!!")
    return false
  end
  return true
end

--@param desired joint angle position
function JointJoggingController:isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
  local diff = 2
  if q_des:nDimension() > 0 then
    ros.DEBUG("q_des checked")
    if satisfiesBounds(self, q_des) then
      ros.DEBUG("satisfiesBounds")
      if q_curr then
        diff= torch.norm(q_curr-q_des)
      end
    end
  end
  ros.DEBUG(string.format("Difference between q_des and q_curr diff = %f", diff))
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
    error("dt has unsupported type")
  end

  local topic = topic or 'joy'
  local joy_spec = 'sensor_msgs/Joy' --TODO
  self.subscriber_jog = self.nh:subscribe(topic, joy_spec , 1)
  ros.INFO('Subscribed to \'joy\' node. Please start using your jogging device.')
  local ready = self.joint_monitor:waitReady(timeout)
  if not ready then
    return false, "Could not collect joint states"
  end
  self:getNewRobotState()
  return true, "Success"
end

function JointJoggingController:getInTopic()
  return self.subscriber_jog:getTopic()
end

function JointJoggingController:getOutTopic()
  return string.format("%s/joint_command",self.controller_name)
end

function JointJoggingController:shutdown()
  self.subscriber_jog:shutdown()
end


--- dummy function to simulate forces. (joystick is required)
function JointJoggingController:getTeleoperationForces()
  local new_message
  local msg
  local joint_names = self.joint_monitor:getJointNames()
  local deltaQ = torch.zeros(#joint_names)
  while self.subscriber_jog:hasMessage() do
    new_message = true
    msg = self.subscriber_jog:read()
  end
  if new_message then
    -- check if joint names are correct
  end

  return deltaQ, new_message
end


function JointJoggingController:updateDeltaT()
    self.dt = ros.Time.now() - self.time_last
    while self.dt:toSec() < 0.008 do -- dt set to 125Hz
      self.dt = ros.Time.now() - self.time_last
    end
    self.dt_monitor:add(torch.zeros(1)+self.dt:toSec())
    local tmpDT = torch.mean(self.dt_monitor.buffer[{{1,self.dt_monitor:count()},{}}])
    --self.dt:fromSec(tmpDT)
    self.time_last = ros.Time.now();
    ros.DEBUG(string.format("Delta Time: %dHz", 1/self.dt:toSec()))
end


function JointJoggingController:tracking(q_dot, duration)
  if type(duration) == 'number' then
    duration = ros.Duration(duration)
  end

  duration = duration or ros.Time.now() - BEGIN_EXECUTION
  local group = self.move_group
  local state = self.state:clone()

  local q_des = self.lastCommandJointPositons + q_dot

  if not self.CONVERED then
    if self:isValid(q_des, self.lastCommandJointPositons) then
      if not publisherPointPositionCtrl then
        local myTopic = string.format("%s/joint_command",self.controller_name)
        ros.WARN(myTopic)
        publisherPointPositionCtrl = self.nh:advertise(string.format(myTopic, self.controller_name), joint_pos_spec)
      end
      if self.FIRSTPOINT then
        sendPositionCommand(self.lastCommandJointPositons, q_dot:zero(), group, duration)
        self.FIRSTPOINT = false
      else
        sendPositionCommand(q_des, q_dot, group, duration)
      end
      self.lastCommandJointPositons:copy(q_des)
    else
      if publisherPointPositionCtrl then
        publisherPointPositionCtrl:shutdown()
      end
      publisherPointPositionCtrl = nil
      ros.ERROR("command is not valid!!!")
      --os.exit()
    end
  else
    self.lastCommandJointPositons = state:copyJointGroupPositions(group:getName()):clone()
  end
  if self.debug then
    ros.DEBUG("q_curr")
    ros.DEBUG(tostring(self.lastCommandJointPositons))
    ros.DEBUG("q_des")
    ros.DEBUG(tostring(q_des))
    ros.DEBUG("q_dot")
    ros.DEBUG((tostring(q_dot)))
  end

  if q_dot:norm() < 1e-12 then
    self.CONVERED = true
    --BEGIN_EXECUTION = ros.Time.now()
    self.FIRSTPOINT = true
  else
    self.CONVERED = false
    self.FIRSTPOINT = false
    ros.WARN("tracking is NOT CONVERED")
  end
end


function JointJoggingController:getNewRobotState()
  ros.spinOnce()
  local p,l = self.joint_monitor:getNextPositionsTensor()
  self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
  self.lastCommandJointPositons:copy(p)
  --ros.ERROR("getNew RobotSTATE")
end


function JointJoggingController:update()
  --xBox 360 Joystick
  --self.current_pose = self.move_group:getCurrentPose()
  local deltaQ, new_message = self:getTeleoperationForces()

  if (torch.norm(deltaForces)+torch.norm(detlaTorques)) < 0.1 then -- only reset the timer if almost zero changes are applied to avoid rapid accellarations
    self.start_time = ros.Time.now()
  end

  if self.CONVERED then
    self:getNewRobotState()
  else
    self.state:setVariablePositions(self.lastCommandJointPositons, self.joint_monitor:getJointNames())
    self.current_pose = self:getCurrentPose()
  end

  self:updateDeltaT()

  local curr_time = ros.Time.now()

  self:tracking(deltaQ:clone(), self.dt)
end

function JointJoggingController:setMoveGroupInterface(name)
  self.move_group = moveit.MoveGroupInterface(name)
  local ready = self:reset(ros.Duration(0.01))
  local move_group_name = self.move_group:getName()
  if ready and (move_group_name == name) then
    return true, "Successfully changed movegroup."
  else
    if move_group_name ~= name then
      return false, string.Format("Could not set moveGroup with name: %s. ",name)
    end
    if not ready then
      return false, "joint state is not available in time."
    end


  end
end

function JointJoggingController:reset(timeout)
  self.state = self.move_group:getCurrentState()
  self.lastCommandJointPositons = self.state:copyJointGroupPositions(self.move_group:getName()):clone()
  self.joint_monitor:shutdown()
  self.joint_monitor = xutils.JointMonitor(self.move_group:getActiveJoints():totable())
  self.time_last = ros.Time.now()
  ros.INFO("waitReady")
  return self.joint_monitor:waitReady(timeout or ros.Duration(0.1))
end

function JointJoggingController:setSpeed(speed)
  ros.INFO(string.format("Set speed to %s", speed))
end


return JointJoggingController
