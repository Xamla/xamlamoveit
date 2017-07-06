local ros = require('ros')
local tf = ros.tf
local timer = torch.Timer()
local planning = require 'xamlamoveit.planning'
local xutils = require 'xamlamoveit.xutils'

local DEFAULT_HEIGHT = 0.1 -- m
local CARRIER_WIDTH = 0.3 -- m
local CARRIER_HEIGHT = 0.2 -- m
local SPEED_LIMIT = { fast = 1, slow = 0.2 }
local SENSITIVITY_THRESHOLD = 0.2
local currentSpeedLimit = "slow"
local continuousMode = false

local blockedAxis = {
  [1] = false, --
  [2] = false,
  [3] = false
}

local buttonMapping = {
  A = 1,
  B = 2,
  X = 3,
  Y = 4,
  LB = 5,
  RB = 6,
  BACK = 7,
  START = 8,
  XBOX = 9,
  LS = 10,
  RS = 11,
  LT = 3,
  RT = 6
}

local axesMapping = {
  LEFT_jOY = 1,
  RIGHT_jOY = 2,
  DPAD1 = 7,
  DPAD2 = 8,
}

local axesValues = {}
local stepSizeZ = 1.
local stepSizeR = 0.1

local buttonEvents = {}
for k, v in pairs(buttonMapping) do
  if k ~= "RT" and k ~= "LT" then
    buttonEvents[k] = { timespan = ros.Time.now():toSec(), value = 0 }
  else
    buttonEvents[k] = { timespan = ros.Time.now():toSec(), value = 1 }
  end
end

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

local controller = require 'xamlamoveit.controller.env'
local JoystickController = torch.class("xamlamoveit.controller.JoystickController",controller)

function JoystickController:__init(node_handle, move_group, ft_topic, ctr_name, dt, debug)
  
  self.debug = debug or false
  self.dt_monitor = xutils.MonitorBuffer(100,1)
  self.nh = node_handle
  self.x_des = nil
  self.q_des = nil
  self.move_group = move_group or error("move_group should not be nil")
  self.state = move_group:getCurrentState()
  
  self.joint_monitor = xutils.JointMonitor(move_group:getActiveJoints():totable())
  self.time_last = ros.Time.now()
  
  local ready = false
  while not ready and ros.ok() do
    ready = self.joint_monitor:waitReady(0.01)
    ros.ERROR('joint states not ready')
  end

  print(self.joint_monitor:getNextPositionsTensor())

  if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
    self.dt = ros.Duration(dt)
  elseif torch.type(dt) == 'nil' then
    self.dt = ros.Duration(0.01)
  elseif torch.type(dt) == 'ros.Duration' then
    self.dt = dt
  else
    error("dt has unsupported type")
  end


  self.current_pose = self.move_group:getCurrentPose()
  self.left_joy = torch.zeros(3)
  self.right_joy = torch.zeros(3)
  self.dpad = torch.zeros(2)

  self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
  self.start_time = ros.Time.now()
  BEGIN_EXECUTION = ros.Time.now()

  self.controller_name = ctr_name or 'pos_based_pos_traj_controller'
end


--- calculates the weighted pseudoInverse of M
-- @param M: Matrix which needs to be inversed
-- @param W: weight Matrix. (Optional)
local function pseudoInverse(M,W)
  local Weights = W or torch.eye(M:size(1))
  assert(M:size(1) == Weights:size()[1],'Data matrix M and weight matrix W need to have the same number of cols')
  local inv = M:t()*Weights*M
  -- make it definite
  inv:add(1e-12, torch.eye(inv:size(1)))
  return torch.inverse(inv) * M:t()*Weights
end


--@param desired joint angle position
function JoystickController:isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
  local diff = 2
  if q_des:nDimension() > 0 then
    if q_curr then
      diff= torch.norm(q_curr-q_des)
    end
  end
  print("diff")
  print(diff)
  return diff<1
end


function JoystickController:connect(topic)
  local topic = topic or 'joy'
  local joy_spec = 'sensor_msgs/Joy'
  self.subscriber_joy = self.nh:subscribe(topic, joy_spec , 1)
  print('Subscribed to \'joy\' node. Please start using your joystick.')
end


function JoystickController:shutdown()
  self.subscriber_joy:shutdown()
end


--- dummy function to simulate forces. (joystick is required)
function JoystickController:getTeleoperationForces()
  local newMessage
  local msg

  buttonEvents.empty = true
  for k, v in pairs(buttonMapping) do
    buttonEvents[k].changed = nil
  end

  while self.subscriber_joy:hasMessage() do
    newMessage = true
    msg = self.subscriber_joy:read()
  end
  if newMessage then
    local axis = msg.axes

    self.left_joy = axis[{{1,3}}]:type('torch.DoubleTensor')
    local tmp = torch.abs(self.left_joy)
    self.left_joy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
    self.left_joy[tmp:gt(SENSITIVITY_THRESHOLD)] = self.left_joy[tmp:gt(SENSITIVITY_THRESHOLD)] -SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

    self.right_joy = axis[{{4,6}}]:type('torch.DoubleTensor')
    tmp = torch.abs(self.right_joy)
    self.right_joy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
    self.right_joy[tmp:gt(SENSITIVITY_THRESHOLD)] = self.right_joy[tmp:gt(SENSITIVITY_THRESHOLD)] -SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

    self.dpad = axis[{{7,8}}]:type('torch.DoubleTensor')

    for k, v in pairs(buttonMapping) do
      local currentButtonValue
      if k == "LT" or k == "RT" then
        if msg.axes[v] < 0 then
          currentButtonValue = -1
        else
          currentButtonValue = 1
        end
      else
        currentButtonValue = msg.buttons[v]
      end
      if buttonEvents[k].value ~= currentButtonValue then
        buttonEvents[k].changed = true
        buttonEvents.empty = nil
      end
      buttonEvents[k].value = currentButtonValue
      buttonEvents[k].timespan = msg.header.stamp:toSec() - buttonEvents[k].timespan
    end

    if self.debug then
      ros.DEBUG(string.format('axis: joystic left: %f04,%f04,%f04',self.left_joy[1],self.left_joy[2],self.left_joy[3]))
      ros.DEBUG(string.format('axis: joystic Right: %f04,%f04,%f04',self.right_joy[1],self.right_joy[2],self.right_joy[3]))
      ros.DEBUG(string.format('axis: joystic dpad: %f04,%f04',self.dpad[1],self.dpad[2]))
    end
  end

  local deltaforces = self.left_joy:clone()
  deltaforces[3] = -self.dpad[2] * stepSizeZ


  local deltatorques = self.right_joy:clone()
  deltatorques[1] = self.right_joy[1] -- Pitch
  deltatorques[2] = self.right_joy[2] -- Yaw
  deltatorques[3] = -self.dpad[1] * stepSizeR -- Roll

  return deltaforces, deltatorques, buttonEvents, newMessage --TODO make this more suffisticated
end


function JoystickController:updateDeltaT()
  
    self.dt = ros.Time.now() - self.time_last
    while self.dt:toSec() < 0.008 do
      self.dt = ros.Time.now() - self.time_last
    end
    self.dt_monitor:add(torch.zeros(1)+self.dt:toSec())
    print(self.dt_monitor.buffer)
    local tmpDT = torch.mean(self.dt_monitor.buffer[{{1,self.dt_monitor:count()},{}}])
    self.dt:fromSec(tmpDT)
    self.time_last = ros.Time.now();
    ros.INFO(string.format("Delta Time: %dHz", 1/self.dt:toSec()))
end


function JoystickController:tracking(q_dot, duration)
  if type(duration) == 'number' then
    duration = ros.Duration(duration)
  end
  duration = duration or ros.Time.now() - BEGIN_EXECUTION
  local group = self.move_group
  local state = self.state:clone()
  local q_curr = state:copyJointGroupPositions(group:getName()):clone()
  local q_des = q_curr + q_dot
  if self:isValid(q_des,q_curr) then
    if not publisherPointPositionCtrl then
      local myTopic = string.format("%s/joint_command",self.controller_name)
      ros.WARN(myTopic)
      publisherPointPositionCtrl = self.nh:advertise(string.format(myTopic, self.controller_name), joint_pos_spec)
    end
    sendPositionCommand(q_des, q_dot, group, duration)
  else
    if publisherPointPositionCtrl then
      publisherPointPositionCtrl:shutdown()
    end
    publisherPointPositionCtrl = nil
    ros.ERROR("command is not valid!!!")
    print(q_des)
  end
end


function JoystickController:getStep( D_force, D_torques, timespan )
  local D_torques = D_torques or torch.zeros(3)
  local opt = {}
  opt.stiffness = 1.0
  opt.damping   =  0.2
  
  local K,D = opt.stiffness,opt.damping--stiffness and damping
  local s = timespan or 1.0

  local suc
  local offset = -D_force/(K+D*s:toSec())

  for index, blocked in ipairs(blockedAxis) do
    if blocked == true then
      offset[index] = 0
    end
  end
  
  local x_rot_des = (- D_torques/(K+D*s:toSec())) -- want this in EE KO
  
  local vel6D = torch.DoubleTensor(6)
  vel6D[{{1,3}}]:copy(offset)
  vel6D[{{4,6}}]:copy(x_rot_des)
  
  print("-------------> time")
  print(s)
  print("-------------> D_force")
  print(D_force)
  local q_dot_des = self:getQdot(vel6D)
  print("-------------> q_dot_des")
  print(q_dot_des)
  return q_dot_des
end


function JoystickController:handleButtonsEvents(buttonEvents)
  --print(buttonEvents)
  if buttonEvents.RT.value == -1 then
    if buttonEvents.X.changed then
      self:blockAxis(1, true)
    elseif buttonEvents.Y.changed then
      self:blockAxis(2, true)
    elseif buttonEvents.B.changed then
      self:blockAxis(3, true)
    elseif buttonEvents.A.changed then
      self:resetAxes()
    end
  elseif buttonEvents.LT.changed then
    if (buttonEvents.LT.value < 0) then
      self:setContinuousMode(true)
    else
      self:setContinuousMode(false)
    end
  elseif buttonEvents.XBOX.changed and buttonEvents.XBOX.timespan > 0.1 then
    --self:moveToCenter()
  elseif buttonEvents.BACK.changed then
    self:setSpeed("slow")
  elseif buttonEvents.START.changed then
    self:setSpeed("fast")
  elseif buttonEvents.B.changed and buttonEvents.B.timespan > 0.1 then
    --self:moveToQuadrant(1)
  elseif buttonEvents.Y.changed and buttonEvents.Y.timespan > 0.1 then
    --self:moveToQuadrant(2)
  elseif buttonEvents.X.changed and buttonEvents.X.timespan > 0.1 then
    --self:moveToQuadrant(3)
  elseif buttonEvents.A.changed and buttonEvents.A.timespan > 0.1 then
    --self:moveToQuadrant(4)
  elseif buttonEvents.LB.changed and buttonEvents.LB.timespan > 0.1 then
    self:moveToOrthogonalPose()
  end
end


function JoystickController:getQdot(vel6D)
  local jac = self.state:getJacobian(self.move_group:getName())
  print(jac)
  local inv_jac = pseudoInverse(jac)
  local jacobian_condition = torch.norm(jac) * torch.norm(inv_jac)
  if jacobian_condition > 50 then
    ros.ERROR(strint.format("detected ill-conditioned Jacobian: %f", jacobian_condition))
  end
  return inv_jac*(vel6D*self.dt:toSec())
end


function JoystickController:getNewRobotState()
  ros.spinOnce()
  local p,l = self.joint_monitor:getNextPositionsTensor()
  self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
  --self.state = self.move_group:getCurrentState()
end


function JoystickController:update()
  --xBox 360 Joystick
  --self.current_pose = self.move_group:getCurrentPose()
  local deltaForces, detlaTorques, buttonEvents, newMessage = self:getTeleoperationForces()
  if newMessage then
    self:handleButtonsEvents(buttonEvents)
  end

  if (torch.norm(deltaForces)+torch.norm(detlaTorques)) < 0.1 then -- only reset the timer if almost zero changes are applied to avoid rapid accellarations
    self.start_time = ros.Time.now()
  end

  self:getNewRobotState()
  self:updateDeltaT()
  local curr_time = ros.Time.now()
  local q_dot = self:getStep(deltaForces, detlaTorques, curr_time-self.start_time)
  if self.dt:toSec()>0.15 then
    q_dot:zero()
  end
  if buttonEvents.empty then
    self:tracking(q_dot, self.dt)
    --[[
    local succ, plan = self.ctrl:movep(self.x_des,nil,nil, SPEED_LIMIT[currentSpeedLimit])
    if succ then
      self.move_group:execute(plan)
    end
    ]]
  end
end


function JoystickController:setSpeed(speed)
  ros.INFO(string.format("Set speed to %s", speed))
  currentSpeedLimit = speed
  if speed == "fast" then
    stepSizeZ = 1
    stepSizeR = 5.
  else
    stepSizeZ = 0.1
    stepSizeR = 0.5
  end
end


function JoystickController:blockAxis(axisIndex, value)
  ros.INFO(string.format("Set block axis %d to %s", axisIndex, value))
  blockedAxis[axisIndex] = value
end


function JoystickController:setContinuousMode(active)
  ros.INFO(string.format("Set continuousMode to %s", active))
  continuousMode = active
end


function JoystickController:resetAxes()
  ros.INFO("Free all axes")
  blockedAxis[1] = false
  blockedAxis[2] = false
  blockedAxis[3] = false
end


function JoystickController:moveToOrthogonalPose()
  local function normalize(x)
    return x / x:norm()
  end

  ros.INFO("Drive to orthogonal pose")
  local zAxis = normalize(torch.Tensor({0,0,1}))
  local basePose = self.move_group:getCurrentPose():toTensor()
  local xAxis = basePose[{1,{1,3}}]
  local yAxis = -normalize(torch.cross(xAxis, zAxis))
  xAxis = normalize(torch.cross(yAxis, zAxis))

  local p = torch.eye(4)
  p[{1,{1,3}}] = xAxis
  p[{2,{1,3}}] = yAxis
  p[{3,{1,3}}] = zAxis
  p[{{1,3},4}] = basePose[{{1,3}, 4}]   -- keep position

  self.ctrl:movep(p, nil, nil, SPEED_LIMIT[currentSpeedLimit])
  local succ, plan = self.ctrl:movep(p, nil, nil, SPEED_LIMIT[currentSpeedLimit])
  if succ then
    self.move_group:execute(plan)
  end
end

return JoystickController
