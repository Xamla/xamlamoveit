local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local timer = torch.Timer()
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'

local DEFAULT_HEIGHT = 0.1 -- m
local CARRIER_WIDTH = 0.3 -- m
local CARRIER_HEIGHT = 0.2 -- m
local SPEED_LIMIT = {fast = 1, slow = 0.5}
local SENSITIVITY_THRESHOLD = 0.2
local currentSpeedLimit = 'slow'
local continuousMode = false

function math.sign(x)
    assert(type(x) == 'number')
    return x > 0 and 1 or x < 0 and -1 or 0
end

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
    DPAD2 = 8
}

local axesValues = {}
local max_stepSizeZ = 1. / 1000 --1 cm
local max_stepSizeR = 0.003 -- 0.03deg
local stepSizeZ = max_stepSizeZ * SPEED_LIMIT[currentSpeedLimit]
local stepSizeR = max_stepSizeR * SPEED_LIMIT[currentSpeedLimit]

local buttonEvents = {}
for k, v in pairs(buttonMapping) do
    if k ~= 'RT' and k ~= 'LT' then
        buttonEvents[k] = {timespan = ros.Time.now():toSec(), value = 0}
    else
        buttonEvents[k] = {timespan = ros.Time.now():toSec(), value = 1}
    end
end

local publisherPointPositionCtrl
local lastJoystickMessage

local test_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
-- JointPosition --
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
---
--@param desired joint angle position
local function sendPositionCommand(q_des, q_dot, group)
    ros.INFO('sendPositionCommand to: ' .. publisherPointPositionCtrl:getTopic())
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
local JoystickControllerOpenLoop = torch.class('xamlamoveit.controller.JoystickControllerOpenLoop', controller)

function JoystickControllerOpenLoop:__init(node_handle, move_group, ctr_name, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    else
    end
    self.velocity_scaling = 1
    self.FIRSTPOINT = true
    self.dt_monitor = core.MonitorBuffer(100, 1)
    self.nh = node_handle
    self.x_des = nil
    self.q_des = nil
    self.move_group = move_group or error('move_group should not be nil')
    self.state = move_group:getCurrentState()
    self.lastCommandJointPositons = self.state:copyJointGroupPositions(move_group:getName()):clone()
    self.current_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.converged = false
    self.joint_monitor = core.JointMonitor(move_group:getActiveJoints():totable())
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
        error('dt has unsupported type')
    end

    self.left_joy = torch.zeros(3)
    self.right_joy = torch.zeros(3)
    self.dpad = torch.zeros(2)

    self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
    self.start_time = ros.Time.now()
    BEGIN_EXECUTION = ros.Time.now()

    self.controller_name = ctr_name or 'pos_based_pos_traj_controller'

    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.kinematic_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.kinematic_model)
    self.lock_client = core.LeasedBaseLockClient(node_handle)
    self.resource_lock = nil
end

function JoystickControllerOpenLoop:getInTopic()
    return self.subscriber_joy:getTopic()
end

function JoystickControllerOpenLoop:getOutTopic()
    return string.format('/%s/joint_command', self.controller_name)
end

local function clamp(t, min, max)
    return torch.cmin(t, max):cmax(min)
end

--- calculates the weighted pseudoInverse of M
-- @param M: Matrix which needs to be inversed
-- @param W: weight Matrix. (Optional)
local function pseudoInverse(M, W)
    local Weights = W or torch.eye(M:size(1))
    assert(M:size(1) == Weights:size()[1], 'Data matrix M and weight matrix W need to have the same number of cols')
    local inv = M:t() * Weights * M
    -- make it definite
    inv:add(1e-15, torch.eye(inv:size(1)))
    return torch.inverse(inv) * M:t() * Weights
end

--- calculates the weighted pseudoInverse of M
-- @param M: Matrix which needs to be inversed
local function inverse(M)
    return M:t() * torch.inverse(M * M:t())
end

function JoystickControllerOpenLoop:getCurrentPose()
    return self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
end

local function satisfiesBounds(self, positions)
    local state = self.state:clone()
    state:setVariablePositions(positions, self.joint_monitor:getJointNames())
    state:update()
    local collisions = self.planning_scene:checkSelfCollision(state)
    if state:satisfiesBounds(0.0) then
        if collisions then
            ros.ERROR('Self Collision detected')
            return false, 'Self Collision detected!!'
        end
    else
        state:enforceBounds()
        state:update()
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
function JoystickControllerOpenLoop:isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
    local diff = 2
    if q_des:nDimension() > 0 then
        ros.DEBUG('q_des checked')
        if satisfiesBounds(self, q_des) then
            ros.DEBUG('satisfiesBounds')
            if q_curr then
                diff = torch.norm(q_curr - q_des)
            end
        end
    end
    ros.DEBUG(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 1
end

function JoystickControllerOpenLoop:connect(topic)
    local topic = topic or 'joy'
    local joy_spec = 'sensor_msgs/Joy'
    self.subscriber_joy = self.nh:subscribe(topic, joy_spec, 1)
    ros.INFO("Subscribed to 'joy' node. Please start using your joystick.")
    self:getNewRobotState()
    return true
end

function JoystickControllerOpenLoop:shutdown()
    self.subscriber_joy:shutdown()
end

--- dummy function to simulate forces. (joystick is required)
function JoystickControllerOpenLoop:getTeleoperationForces()
    local newMessage
    local msg

    buttonEvents.empty = true
    for k, v in pairs(buttonMapping) do
        buttonEvents[k].changed = nil
    end

    while self.subscriber_joy:hasMessage() do
        ros.WARN('NEW MESSAGE RECEIVED')
        newMessage = true
        msg = self.subscriber_joy:read()
    end
    if newMessage then
        local axis = msg.axes

        self.left_joy = axis[{{1, 3}}]:type('torch.DoubleTensor')
        local tmp = torch.abs(self.left_joy)
        self.left_joy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
        self.left_joy[tmp:gt(SENSITIVITY_THRESHOLD)] =
            self.left_joy[tmp:gt(SENSITIVITY_THRESHOLD)] - SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

        self.right_joy = axis[{{4, 6}}]:type('torch.DoubleTensor')
        tmp = torch.abs(self.right_joy)
        self.right_joy[tmp:lt(SENSITIVITY_THRESHOLD)] = 0.0 --reduce noise to avoid drifting
        self.right_joy[tmp:gt(SENSITIVITY_THRESHOLD)] =
            self.right_joy[tmp:gt(SENSITIVITY_THRESHOLD)] - SENSITIVITY_THRESHOLD --reduce noise to avoid drifting

        self.dpad = axis[{{7, 8}}]:type('torch.DoubleTensor')

        for k, v in pairs(buttonMapping) do
            local currentButtonValue
            if k == 'LT' or k == 'RT' then
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
            ros.DEBUG(
                string.format(
                    'axis: joystic left: %f04,%f04,%f04',
                    self.left_joy[1],
                    self.left_joy[2],
                    self.left_joy[3]
                )
            )
            ros.DEBUG(
                string.format(
                    'axis: joystic Right: %f04,%f04,%f04',
                    self.right_joy[1],
                    self.right_joy[2],
                    self.right_joy[3]
                )
            )
            ros.DEBUG(string.format('axis: joystic dpad: %f04,%f04', self.dpad[1], self.dpad[2]))
        end
    end

    local deltaforces = self.left_joy:clone()
    deltaforces[3] = -self.dpad[2] * stepSizeZ

    local deltatorques = self.right_joy:clone()
    deltatorques[1] = math.abs(self.right_joy[1]) > 0.01 and math.sign(self.right_joy[1]) * stepSizeR or 0 -- Pitch
    deltatorques[2] = math.abs(self.right_joy[2]) > 0.01 and math.sign(self.right_joy[2]) * stepSizeR or 0 -- Yaw
    deltatorques[3] = -self.dpad[1] * stepSizeR -- Roll

    return deltaforces, deltatorques, buttonEvents, newMessage --TODO make this more suffisticated
end

function JoystickControllerOpenLoop:updateDeltaT()
    self.dt = ros.Time.now() - self.time_last
    while self.dt:toSec() < 0.008 do -- dt set to 125Hz
        self.dt = ros.Time.now() - self.time_last
    end
    self.dt_monitor:add(torch.zeros(1) + self.dt:toSec())
    local tmpDT = torch.mean(self.dt_monitor.buffer[{{1, self.dt_monitor:count()}, {}}])
    --self.dt:fromSec(tmpDT)
    self.time_last = ros.Time.now()
    ros.DEBUG(string.format('Delta Time: %dHz', 1 / self.dt:toSec()))
end

function JoystickControllerOpenLoop:tracking(q_dot, duration)
    ros.INFO('tracking')
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    duration = duration or ros.Time.now() - BEGIN_EXECUTION
    local group = self.move_group
    local state = self.state:clone()

    local q_des = self.lastCommandJointPositons + q_dot

    if not self.converged then
        if self:isValid(q_des, self.lastCommandJointPositons) then
            if not publisherPointPositionCtrl then
                local myTopic = string.format('/%s/joint_command', self.controller_name)
                ros.WARN(myTopic)
                publisherPointPositionCtrl = self.nh:advertise(myTopic, joint_pos_spec)
            end
            if self.FIRSTPOINT then
                sendPositionCommand(self.lastCommandJointPositons, q_dot:zero(), group, duration)
                self.FIRSTPOINT = false
                ros.DEBUG('FIRSTPOINT')
            else
                ros.DEBUG('GoGoGo')
                sendPositionCommand(q_des, q_dot, group, duration)
            end
            self.lastCommandJointPositons:copy(q_des)
        else
            --os.exit()
            if publisherPointPositionCtrl then
                publisherPointPositionCtrl:shutdown()
            end
            publisherPointPositionCtrl = nil
            ros.ERROR('command is not valid!!!')
        end
    else
        self.lastCommandJointPositons = state:copyJointGroupPositions(group:getName()):clone()
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
        self.converged = true
        --BEGIN_EXECUTION = ros.Time.now()
        self.FIRSTPOINT = true
    else
        self.converged = false
        self.FIRSTPOINT = false
        ros.WARN('tracking is NOT CONVERGED')
    end
end

function targetTransformation(target_frame, offset, rotation_rpy)
    local tmp_offset_tf = tf.Transform():setOrigin(offset)
    local rotation = tmp_offset_tf:getRotation()
    --tmp_offset_tf:setRotation(rotation:setRPY(rotation_rpy[1],rotation_rpy[2],rotation_rpy[3]))

    local curr_pose_inv = target_frame:clone():inverse()
    local curr_pose = target_frame:clone()
    tmp_offset_tf:fromTensor(curr_pose:toTensor() * tmp_offset_tf:toTensor() * curr_pose_inv:toTensor())

    return tmp_offset_tf:getOrigin(), rotation_rpy
    --tmp_offset_tf:getRotation():getRPY()
end

function JoystickControllerOpenLoop:getStep(D_force, D_torques, timespan)
    local D_torques = D_torques or torch.zeros(3)
    local opt = {}
    opt.stiffness = 1.0
    opt.damping = 0.2

    local K, D = opt.stiffness, opt.damping
    --stiffness and damping
    local s = timespan or 1.0
    local offset = -D_force / (K + D * s:toSec())
    local x_rot_des = (-D_torques / (K + D * s:toSec())) -- want this in EE KO
    local suc

    for index, blocked in ipairs(blockedAxis) do
        if blocked == true then
            offset[index] = 0
        end
    end

    offset, x_rot_des = targetTransformation(self.current_pose, offset, x_rot_des)
    offset = clamp(offset, -stepSizeZ * SPEED_LIMIT[currentSpeedLimit], stepSizeZ * SPEED_LIMIT[currentSpeedLimit])
    x_rot_des =
        clamp(x_rot_des, -stepSizeR * SPEED_LIMIT[currentSpeedLimit], stepSizeR * SPEED_LIMIT[currentSpeedLimit])

    local vel6D = torch.DoubleTensor(6)
    vel6D[{{1, 3}}]:copy(offset * self.velocity_scaling)
    vel6D[{{4, 6}}]:copy(x_rot_des * self.velocity_scaling)
    local q_dot_des, jacobian = self:getQdot(vel6D / self.dt:toSec())

    ros.DEBUG(string.format('-------------> time: %d', s:toSec()))
    ros.DEBUG(string.format('-------------> D_force %s', tostring(D_force)))
    ros.DEBUG(string.format('-------------> q_dot_des %s', tostring(q_dot_des)))

    return q_dot_des, jacobian
end

function JoystickControllerOpenLoop:handleButtonsEvents(buttonEvents)
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
        self:setSpeed('slow')
    elseif buttonEvents.START.changed then
        self:setSpeed('fast')
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

function JoystickControllerOpenLoop:getQdot(vel6D)
    local jac = self.state:getJacobian(self.move_group:getName())

    local inv_jac = inverse(jac)
    local jacobian_condition = torch.norm(jac) * torch.norm(inv_jac)
    if jacobian_condition > 50 then
        ros.ERROR(string.format('detected ill-conditioned Jacobian: %f', jacobian_condition))
    --vel6D:zero()
    end

    return inv_jac * (vel6D * self.dt:toSec()), jac
end

function JoystickControllerOpenLoop:getNewRobotState()
    local p, l = self.joint_monitor:getNextPositionsTensor()
    self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
    self.state:update()
    self.lastCommandJointPositons:copy(p)
    self.current_pose = self:getCurrentPose()
    --ros.ERROR("getNew RobotSTATE")
end

function JoystickControllerOpenLoop:update()
    --xBox 360 Joystick
    --self.current_pose = self.move_group:getCurrentPose()
    local deltaForces, detlaTorques, buttonEvents, newMessage = self:getTeleoperationForces()
    ros.DEBUG('deltaForces')
    ros.DEBUG(tostring(deltaForces))
    ros.DEBUG('detlaTorques')
    ros.DEBUG(tostring(detlaTorques))
    if newMessage then
        self:handleButtonsEvents(buttonEvents)
    end

    if (torch.norm(deltaForces) + torch.norm(detlaTorques)) < 0.1 then -- only reset the timer if almost zero changes are applied to avoid rapid accellarations
        self.start_time = ros.Time.now()
    end

    if self.converged then
        self:getNewRobotState()
    else
        self.state:setVariablePositions(self.lastCommandJointPositons, self.joint_monitor:getJointNames())
        self.state:update()
        self.current_pose = self:getCurrentPose()
    end

    self:updateDeltaT()

    local curr_time = ros.Time.now()
    local q_dot = self:getStep(deltaForces, detlaTorques, curr_time - self.start_time)
    if self.dt:toSec() > 0.15 then
        ros.WARN('dt is to large !!' .. self.dt:toSec())
        newMessege = false
        q_dot:zero()
    end

    if buttonEvents.empty then
        if newMessage == true then
            ros.INFO('new Messege')
            if self.resource_lock == nil then
                ros.INFO('lock resources')
                self.resource_lock = self.lock_client:lock(self.state:getVariableNames():totable())
            else
                local dur =
                    ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
                if dur > (self.resource_lock.expiration - ros.Time.now()) then
                    self.resource_lock =
                        self.lock_client:lock(self.state:getVariableNames():totable(), self.resource_lock.id)
                end
            end

            if self.resource_lock.success then
                q_dot = self:getStep(deltaForces, detlaTorques, curr_time - self.start_time)
                self:tracking(q_dot:clone(), self.dt)
            else
                ros.WARN('could not lock resources')
                self.resource_lock = nil
            end
        else
            if self.resource_lock ~= nil then
                local dur =
                    ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
                if dur > (self.resource_lock.expiration - ros.Time.now()) then
                    print(self.resource_lock)
                    self.lock_client:release(self.resource_lock)
                    self.resource_lock = nil
                end
            end
        end
    end
end

function JoystickControllerOpenLoop:setSpeed(speed)
    ros.INFO(string.format('Set speed to %s', speed))
    currentSpeedLimit = speed
    stepSizeZ = max_stepSizeZ * SPEED_LIMIT[currentSpeedLimit]
    stepSizeR = max_stepSizeR * SPEED_LIMIT[currentSpeedLimit]
end

function JoystickControllerOpenLoop:blockAxis(axisIndex, value)
    ros.INFO(string.format('Set block axis %d to %s', axisIndex, value))
    blockedAxis[axisIndex] = value
end

function JoystickControllerOpenLoop:setContinuousMode(active)
    ros.INFO(string.format('Set continuousMode to %s', active))
    continuousMode = active
end

function JoystickControllerOpenLoop:resetAxes()
    ros.INFO('Free all axes')
    blockedAxis[1] = false
    blockedAxis[2] = false
    blockedAxis[3] = false
end

function JoystickControllerOpenLoop:moveToOrthogonalPose()
    local function normalize(x)
        return x / x:norm()
    end

    ros.INFO('Drive to orthogonal pose')
    local zAxis = normalize(torch.Tensor({0, 0, 1}))
    local basePose = self.move_group:getCurrentPose():toTensor()
    local xAxis = basePose[{1, {1, 3}}]
    local yAxis = -normalize(torch.cross(xAxis, zAxis))
    xAxis = normalize(torch.cross(yAxis, zAxis))

    local p = torch.eye(4)
    p[{1, {1, 3}}] = xAxis
    p[{2, {1, 3}}] = yAxis
    p[{3, {1, 3}}] = zAxis
    p[{{1, 3}, 4}] = basePose[{{1, 3}, 4}] -- keep position

    --self.ctrl:movep(p, nil, nil, SPEED_LIMIT[currentSpeedLimit])
    local succ, plan = self.ctrl:movep(p, self.move_group:getName(), nil, SPEED_LIMIT[currentSpeedLimit])
    if succ then
        self.move_group:execute(plan)
    end
end

function JoystickControllerOpenLoop:reset(timeout)
    self.state = self.move_group:getCurrentState()
    self.joint_monitor:shutdown()
    self.joint_monitor = core.JointMonitor(self.move_group:getActiveJoints():totable())
    self.time_last = ros.Time.now()
    return self.joint_monitor:waitReady(timeout or ros.Duration(0.1))
end

function JoystickControllerOpenLoop:setMoveGroupInterface(name)
    self.move_group = moveit.MoveGroupInterface(name)
    local ready = self:reset(ros.Duration(0.01))
    local move_group_name = self.move_group:getName()
    if ready and (move_group_name == name) then
        return true, 'Successfully changed movegroup.'
    else
        if move_group_name ~= name then
            return false, string.Format('Could not set moveGroup with name: %s. ', name)
        end
        if not ready then
            return false, 'joint state is not available in time.'
        end
    end
end

return JoystickControllerOpenLoop
