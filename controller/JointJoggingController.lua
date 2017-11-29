local ros = require 'ros'
local moveit = require 'moveit'
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'

local publisherPointPositionCtrl

local test_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')

function jointCommandCb(self, msg, header)
    local joint_state_names = self.joint_monitor:getJointNames()
    if #msg.points > 0 then
        for i, name in ipairs(msg.joint_names) do
            local index = table.indexof(joint_state_names, name)
            if index > -1 then
                self.lastCommandJointVelocity[self.positionNameMap[name]] = msg.points[1].velocities[i]
            end
        end
    end
end

function setpointCommandCb(self, msg, header)
    if #msg.points > 0 then
        if self.lastCommandSetpoint == nil then
            self.lastCommandSetpoint = self.joint_monitor:getPositionsOrderedTensor(self.joint_names)
        end
        print('---', msg)
        print('---', self.lastCommandSetpoint)
        for i, name in ipairs(msg.joint_names) do
            local index = table.indexof(self.joint_names, name)
            if index > -1 then
                print(index)
                local t = msg.points[1].positions[i]
                self.lastCommandSetpoint[index] = t
            end
        end
    end
end

---
--@param desired joint angle position
local function sendPositionCommand(q_des, q_dot, group)
    --ros.INFO('sendPositionCommand')
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
    --ros.WARN(publisherPointPositionCtrl:getTopic())
    publisherPointPositionCtrl:publish(m)
end

local function createVariableNameMap(self)
    local variable_names = self.state:getVariableNames()
    local joint_state_names = self.joint_monitor:getJointNames()
    local map = {}
    local counter = 1
    for i, v in ipairs(variable_names) do
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
local JointJoggingController = torch.class('xamlamoveit.controller.JointJoggingController', controller)

function JointJoggingController:__init(node_handle, move_group, ctr_name, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    end

    self.CONVERGED = false
    self.FIRSTPOINT = true
    self.nh = node_handle
    self.q_des = nil
    self.move_group = move_group or error('move_group should not be nil')
    self.state = move_group:getCurrentState()
    self.joint_names = self.state:getVariableNames()
    self.lastCommandJointPositons = self.state:copyJointGroupPositions(move_group:getName()):clone()
    self.lastCommandJointVelocity = torch.zeros(self.lastCommandJointPositons:size())
    self.lastCommandSetpoint = nil
    self.joint_monitor = core.JointMonitor(move_group:getActiveJoints():totable())
    self.positionNameMap = createVariableNameMap(self)
    self.time_last = ros.Time.now()

    if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
        self.dt = ros.Duration(dt)
    elseif torch.type(dt) == 'nil' then
        self.dt = ros.Duration(0.01)
    elseif torch.type(dt) == 'ros.Duration' then
        ros.INFO('rosDuration as dt received')
        self.dt = dt
    else
        error('dt has unsupported type')
    end

    self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
    self.start_time = ros.Time.now()
    BEGIN_EXECUTION = ros.Time.now()

    self.controller_name = ctr_name or 'pos_based_pos_traj_controller'

    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.kinematic_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.kinematic_model)
    self.planning_scene:syncPlanningScene()

    self.robotControllerTopic = string.format('/%s/joint_command', self.controller_name)
    self.lock_client = core.LeasedBaseLockClient(node_handle)
    self.resource_lock = nil

    self.velocity_scaling = 1.0

    if not publisherPointPositionCtrl then
        publisherPointPositionCtrl = self.nh:advertise(self.robotControllerTopic, joint_pos_spec)
    end
end

local function satisfiesBounds(self, positions)
    local state = self.state:clone()
    state:setVariablePositions(positions, self.joint_monitor:getJointNames())
    self.planning_scene:syncPlanningScene()
    local collisions = self.planning_scene:checkSelfCollision(state) --or self.planning_scene:isStateColliding(self.move_group,state)
    if state:satisfiesBounds(0.0) then
        if collisions then
            ros.ERROR('Self Collision detected')
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
                diff = torch.norm(q_curr - q_des)
            end
        end
    end
    ros.DEBUG(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 1
end

function JointJoggingController:connect(incremental_topic, setpoint_topic, timeout)
    local timeout = timeout
    if torch.type(timeout) == 'number' or torch.type(timeout) == 'nil' then
        timeout = ros.Duration(timeout)
    elseif torch.type(timeout) == 'nil' then
        timeout = ros.Duration(0.01)
    elseif torch.type(timeout) == 'ros.Duration' then
    else
        error('dt has unsupported type')
    end

    self.subscriber_jog = self.nh:subscribe(incremental_topic, joint_pos_spec, 1)
    self.subscriber_jog:registerCallback(
        function(msg, header)
            jointCommandCb(self, msg, header)
        end
    )

    self.subscriber_setpoint = self.nh:subscribe(setpoint_topic, joint_pos_spec, 1)
    self.subscriber_setpoint:registerCallback(
        function(msg, header)
            setpointCommandCb(self, msg, header)
        end
    )

    ros.INFO(string.format("Subscribed to '%s' node. Please start using your jogging device.", incremental_topic))
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
    return self.robotControllerTopic
end

function JointJoggingController:shutdown()
    if publisherPointPositionCtrl then
        publisherPointPositionCtrl:shutdown()
        publisherPointPositionCtrl = nil
    end
    self.subscriber_jog:shutdown()
end

function JointJoggingController:updateDeltaT()
    local dt = ros.Time.now() - self.time_last
    while dt:toSec() < self.dt:toSec() do -- dt set to 125Hz
        dt = ros.Time.now() - self.time_last
    end
    self.time_last = ros.Time.now()
    ros.DEBUG(string.format('Delta Time: %dHz', 1 / self.dt:toSec()))
end

function JointJoggingController:tracking(q_dot, duration)
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    duration = duration or ros.Time.now() - BEGIN_EXECUTION
    local group = self.move_group
    local q_des = self.lastCommandJointPositons + q_dot

    if not self.CONVERGED or self.FIRSTPOINT then
        if self:isValid(q_des, self.lastCommandJointPositons) then
            if self.FIRSTPOINT then
              sendPositionCommand(self.lastCommandJointPositons, torch.zeros(q_dot:size()), group, duration)
              self.FIRSTPOINT = false
              self.CONVERGED = false
            else
              sendPositionCommand(q_des, q_dot, group, duration)
            end
            --sendPositionCommand(q_des, q_dot, group, duration)
            self.lastCommandJointPositons:copy(q_des)
        else
            ros.ERROR('command is not valid!!!')
            return false, 'command is not valid!!!'
        end
    elseif self.CONVERGED and publisherPointPositionCtrl ~= nil then
        -- Repeat last position command to prevent robot driver from going into idle state
        sendPositionCommand(self.lastCommandJointPositons, torch.zeros(q_dot:size()), group, duration)
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
        self.CONVERGED = true
        --BEGIN_EXECUTION = ros.Time.now()
        self.FIRSTPOINT = true
    else
        self.CONVERGED = false
        self.FIRSTPOINT = false
        --ros.WARN('tracking is NOT CONVERGED')
    end
    return true, 'success'
end

function JointJoggingController:setpointTracking(setpoint, duration)
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end
    duration = duration or ros.Time.now() - BEGIN_EXECUTION

    local current_joint_positions = self.joint_monitor:getPositionsOrderedTensor(self.joint_names)
    if self:isValid(setpoint, current_joint_positions) then
        sendPositionCommand(setpoint, torch.zeros(setpoint:size()), self.move_group, duration)
        self.lastCommandJointPositons:copy(setpoint)
        self.lastCommandJointVelocity:zero()
        self.FIRSTPOINT = true
    else
        return false, 'Command is not valid.'
    end

    return true, 'success'
end

function JointJoggingController:getNewRobotState()
    local p, l = self.joint_monitor:getNextPositionsTensor(self.dt * 2)
    self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
    self.lastCommandJointPositons:copy(p)
    self.state:update()
end

function JointJoggingController:update()
    if self.CONVERGED then
        self.start_time = ros.Time.now()
    else
        local p = self.joint_monitor:getPositionsTensor(self.joint_monitor:getJointNames())
        self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
    end

    self:updateDeltaT()

    local succ, msg = true, 'IDLE'
    if ros.ok() then
        --ros.INFO('New message lock resource')
        if self.resource_lock == nil then
            self.resource_lock = self.lock_client:lock(self.state:getVariableNames())
        else
            local dur = ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
            if dur > (self.resource_lock.expiration - ros.Time.now()) then
                self.resource_lock = self.lock_client:lock(self.state:getVariableNames(), self.resource_lock.id)
            end
        end

        if self.resource_lock.success then
            --ros.INFO('Lock resource successful')
            if self.lastCommandSetpoint ~= nil then
                succ, msg = self:setpointTracking(self.lastCommandSetpoint, self.dt)
                self.lastCommandSetpoint = nil
            else
                succ, msg = self:tracking(self.lastCommandJointVelocity:clone(), self.dt)
            end
        else
            ros.WARN('Lock resource unsuccessful')
            self.resource_lock = nil
        end
        self.lastCommandJointVelocity:zero()
    else
        if self.resource_lock ~= nil then
            local dur = ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
            if dur > (self.resource_lock.expiration - ros.Time.now()) then
                self.lock_client:release(self.resource_lock)
                self.resource_lock = nil
            end
        end
    end

    return succ, msg
end

function JointJoggingController:setMoveGroupInterface(name)
    self.move_group = moveit.MoveGroupInterface(name)
    local ready = self:reset(ros.Duration(0.01))
    local move_group_name = self.move_group:getName()
    if ready and (move_group_name == name) then
        return true, 'Successfully changed MoveGroup.'
    else
        if move_group_name ~= name then
            return false, string.Format('Could not set MoveGroup with name: %s.', name)
        end
        if not ready then
            return false, 'Joint state is not available in time.'
        end
    end
end

function JointJoggingController:reset(timeout)
    self.state = self.move_group:getCurrentState()
    self.joint_names = self.state:getVariableNames()
    self.lastCommandJointPositons = self.state:copyJointGroupPositions(self.move_group:getName()):clone()
    self.lastCommandJointVelocity = self.lastCommandJointPositons:clone():zero()
    self.joint_monitor:shutdown()
    self.joint_monitor = core.JointMonitor(self.move_group:getActiveJoints():totable())
    self.positionNameMap = createVariableNameMap(self)
    self.time_last = ros.Time.now()
    if not self.joint_monitor:waitReady(timeout or ros.Duration(0.1)) then
        return false
    end
    self:getNewRobotState()

    return true
end

function JointJoggingController:setSpeed(speed)
    ros.INFO(string.format('Set speed to %s', speed))
end

return JointJoggingController
