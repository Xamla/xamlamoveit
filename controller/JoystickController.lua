local ros = require('ros')
local tf = ros.tf
local timer = torch.Timer()
local pcl = require 'pcl'
local planning = require 'xamlamoveit.planning'

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
  LEFTJOY = 1,
  RIGHTJOY = 2,
  DPAD1 = 7,
  DPAD2 = 8,
}

local axesValues = {}
local stepSizeZ = 0.1
local stepSizeR = 1.

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
local function sendPositionCommand(qDes, group, duration)
  local duration = duration or ros.Duration(0.5)
  local m = ros.Message(joint_pos_spec)
  local mPoint = ros.Message(test_spec)
  local names = std.StringVector()
  group:getActiveJoints(names)
  m.joint_names = {}
  for ii = 1, qDes:size(1) do
    m.joint_names[ii] = names[ii]
  end
  local state = group:getCurrentState()
  local qCurr = state:copyJointGroupPositions(group:getName()):clone()
  mPoint.positions:set(qDes)
  mPoint.velocities:set((qDes-qCurr)/(duration:toSec()*10)) --TODO this is probably not optimal.
  mPoint.time_from_start = duration
  m.points = {mPoint}
  publisherPointPositionCtrl:publish(m)
end

local controller = require 'xamlamoveit.controller.env'
local JoystickController = torch.class("xamlamoveit.controller.JoystickController",controller)

function JoystickController:__init(node_handle, move_group, ft_topic, ctr_name, dt, debug)
  self.debug = debug or false
  self.workspaceOrigin = workspaceOrigin
  self.nh = node_handle
  self.xDes= nil
  self.qDes= nil
  self.moveGroup = move_group or error("moveGroup should not be nil")

  if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
    self.dt = ros.Duration(dt)
  elseif torch.type(dt) == 'nil' then
    self.dt = ros.Duration(0.01)
  elseif torch.type(dt) == 'ros.Duration' then
    self.dt = dt
  else
    error("dt has unsupported type")
  end


  self.currentPose = self.moveGroup:getCurrentPose()
  self.leftJoy = torch.zeros(3)
  self.rightJoy = torch.zeros(3)
  self.dpad = torch.zeros(2)

  self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
  self.start_time = timer:time().real

  self.controllerName = ctrName or 'pos_based_pos_traj_controller'
end


--@param desired joint angle position
function JoystickController:isValid(qDes) -- avoid large jumps in posture values and check if qDes tensor is valid
  local group = self.moveGroup
  local diff = 2
  local state = group:getCurrentState()
  local qCurr = state:copyJointGroupPositions(group:getName()):clone()
  if qDes:nDimension() > 0 then
    if qCurr then
      diff= torch.norm(qCurr-qDes)
    end
  end
  return diff<1
end


function JoystickController:publishCommand(qDes,group, duration)
  if self:isValid(qDes) then
    if not publisherPointPositionCtrl then
      publisherPointPositionCtrl = self.nh:advertise(string.format("%s/command",self.controllerName), joint_pos_spec)
    end
    sendPositionCommand(qDes, group, duration)
  else
    if publisherPointPositionCtrl then
      publisherPointPositionCtrl:shutdown()
    end
    publisherPointPositionCtrl = nil
    log("command is not valid!!!")
  end
end


local function generatePose(partSpec, height)
  height = height or 0
  local translation = pcl.affine.translate(partSpec.x, partSpec.y, partSpec.z + height):double()
  local rotation = pcl.affine.rotateAxis({1,0,0}, math.pi):double() * pcl.affine.rotateAxis({0,0,-1}, partSpec.angle):double()
  return translation * rotation
end


function JoystickController:connect(topic)
  local topic = topic or 'joy'
  local joy_spec = 'sensor_msgs/Joy'
  self.subscriber_joy = self.nh:subscribe(topic, joy_spec , 10)
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

    self.leftJoy = axis[{{1,3}}]:type('torch.DoubleTensor')
    local tmp = torch.abs(self.leftJoy)
    self.leftJoy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
    self.leftJoy[tmp:gt(SENSITIVITY_THRESHOLD)] = self.leftJoy[tmp:gt(SENSITIVITY_THRESHOLD)] -SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

    self.rightJoy = axis[{{4,6}}]:type('torch.DoubleTensor')
    tmp = torch.abs(self.rightJoy)
    self.rightJoy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
    self.rightJoy[tmp:gt(SENSITIVITY_THRESHOLD)] = self.rightJoy[tmp:gt(SENSITIVITY_THRESHOLD)] -SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

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
      print(string.format('axis: joystic left: %f04,%f04,%f04',self.leftJoy[1],self.leftJoy[2],self.leftJoy[3]))
      print(string.format('axis: joystic Right: %f04,%f04,%f04',self.rightJoy[1],self.rightJoy[2],self.rightJoy[3]))
      print(string.format('axis: joystic dpad: %f04,%f04',self.dpad[1],self.dpad[2]))
    end
  end

  local deltaforces = self.leftJoy:clone()
  deltaforces[3] = -self.dpad[2] * stepSizeZ


  local deltatorques = self.rightJoy:clone()
  deltatorques[1] = self.rightJoy[1] -- Pitch
  deltatorques[2] = self.rightJoy[2] -- Yaw
  deltatorques[3] = -self.dpad[1] * stepSizeR -- Roll

  return deltaforces/10, deltatorques/10, buttonEvents, newMessage --TODO make this more suffisticated
end


function JoystickController:tracking(xDes, duration)
  if type(duration) == 'number' then
    duration = ros.Duration(duration)
  end
  duration = duration or ros.Duration(0.25)
  local group = self.moveGroup
  local state = group:getCurrentState():clone() --desired joint values

  local suc
  suc, self.qDes = state:setFromIK(group:getName(),xDes)
  local qDes = self.qDes:clone()
  if self:isValid(qDes) then
    if not publisherPointPositionCtrl then
      publisherPointPositionCtrl = self.nh:advertise(string.format("%s/command",self.controllerName), joint_pos_spec)
    end
    sendPositionCommand(qDes, group, duration)
  else
    if publisherPointPositionCtrl then
      publisherPointPositionCtrl:shutdown()
    end
    publisherPointPositionCtrl = nil
    --log("command is not valid!!!")
  end
end


function JoystickController:getStep( D_force, D_torques, timespan )
  local D_torques = D_torques or torch.zeros(3)
  local opt = {}
  opt.stiffness = 10.0
  opt.damping   =  2.0
  local x_curr = self.currentPose:clone()
  self.xDes = x_curr:clone() --pose desired
  local q_des = self.moveGroup:getCurrentState() --desired joint values
  local K,D = opt.stiffness,opt.damping--stiffness and damping
  local s = timespan or 1.0

  local suc
  local offset = -D_force/(K+D*s)

  for index, blocked in ipairs(blockedAxis) do
    if blocked == true then
      offset[index] = 0
    end
  end

  local x_pos_des = (x_curr:getOrigin() + offset)

  local curr_rot = self.xDes:getRotation() --
  local x_rot_des = (- D_torques/(K+D*s)) -- want this in EE KO
  local curr_rot = self.xDes:getRotation()
  local rot = tf.Quaternion()
  rot:setRPY(x_rot_des[1], x_rot_des[2], x_rot_des[3])
  local relTrafo = tf.Transform()
  relTrafo:setOrigin(x_pos_des):setRotation(curr_rot*rot)

  self.xDes = self.currentPose:clone()
  self.xDes = self.xDes:inverse():mul(relTrafo:mul(self.xDes))
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


function JoystickController:update()
  --xBox 360 Joystick
  self.currentPose = self.moveGroup:getCurrentPose()

  local deltaForces, detlaTorques, buttonEvents, newMessage = self:getTeleoperationForces()
  if newMessage then
    self:handleButtonsEvents(buttonEvents)
  end

  if (torch.norm(deltaForces)+torch.norm(detlaTorques)) < 0.1 then -- only reset the timer if almost zero changes are applied to avoid rapid accellarations
    self.start_time = timer:time().real
  end

  local curr_time = timer:time().real
  self:getStep(deltaForces, detlaTorques, curr_time-self.start_time)

  if buttonEvents.empty then
    self:tracking(self.xDes)
    --self.ctrl:movep(self.xDes, SPEED_LIMIT[currentSpeedLimit])
  end
  self.xDes = nil
end


function JoystickController:moveToCenter()
  local center = generatePose( { x = 1 * CARRIER_HEIGHT / 2,
                                 y = 1 * CARRIER_WIDTH / 2,
                                 z = 0.0 , angle = 0.0}, DEFAULT_HEIGHT)

  ros.INFO("Move robot to quadrant center")
  local currentPoseInRobot = self.moveGroup:getCurrentPose():toTensor()
  local targetPoseInRobot = self.workspaceOrigin * center
  targetPoseInRobot[{{1, 3},{1, 3}}] = currentPoseInRobot[{{1, 3},{1, 3}}]
  print(targetPoseInRobot)
  self.ctrl:movep(targetPoseInRobot, SPEED_LIMIT[currentSpeedLimit])
end


--- @brief Moves robot to the center of the given quadrant up-right: 1, up-left: 2, down-left: 3,
--   down-right: 4
function JoystickController:moveToQuadrant(quadrant)
  local x = 0
  local y = 0

  if quadrant == 1 then
    x = 1
  elseif quadrant == 2 then
    x = 1
    y = 1
  elseif quadrant == 3 then
    y = 1
  end

  local quadrantCenter = generatePose( { x = x * CARRIER_HEIGHT / 2 + CARRIER_HEIGHT / 4,
                                         y = y * CARRIER_WIDTH / 2 + CARRIER_WIDTH / 4,
                                         z = 0.0 , angle = 0.0}, DEFAULT_HEIGHT)
  local currentPoseInRobot = self.moveGroup:getCurrentPose():toTensor()
  local targetPoseInRobot = self.workspaceOrigin * quadrantCenter
  targetPoseInRobot[{{1, 3},{1, 3}}] = currentPoseInRobot[{{1, 3},{1, 3}}]

  ros.INFO(string.format("Move robot to quadrant %d", quadrant))
  print(targetPoseInRobot)
  self.ctrl:movep(targetPoseInRobot, SPEED_LIMIT[currentSpeedLimit])
end


function JoystickController:setSpeed(speed)
  ros.INFO(string.format("Set speed to %s", speed))
  currentSpeedLimit = speed
  if speed == "fast" then
    stepSizeZ = 1
    stepSizeR = 5.
  else
    stepSizeZ = 0.1
    stepSizeR = 1.
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
  local zAxis = normalize(torch.Tensor({0,0,-1}))
  local basePose = self.moveGroup:getCurrentPose():toTensor()
  local xAxis = basePose[{1,{1,3}}]
  local yAxis = -normalize(torch.cross(xAxis, zAxis))
  xAxis = normalize(torch.cross(yAxis, zAxis))

  local p = torch.eye(4)
  p[{1,{1,3}}] = xAxis
  p[{2,{1,3}}] = yAxis
  p[{3,{1,3}}] = zAxis
  p[{{1,3},4}] = basePose[{{1,3}, 4}]   -- keep position

  self.ctrl:movep(p, SPEED_LIMIT[currentSpeedLimit])
end

return JoystickController
