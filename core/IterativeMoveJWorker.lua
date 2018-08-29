--[[
IterativeMoveJWorker.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local core = require 'xamlamoveit.core'
local xutils = require 'xamlamoveit.xutils'

local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local GoalStatus = actionlib.GoalStatus

local error_codes = core.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))

local IterativeMoveJWorker = torch.class('IterativeMoveJWorker')

local function checkMoveGroupName(self, name)
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.plan_scene = moveit.PlanningScene(self.robot_model_loader:getModel())
    self.plan_scene:syncPlanningScene()
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    ros.INFO('available move_groups:\n%s', tostring(all_group_joint_names))
    for k, v in pairs(all_group_joint_names) do
        if name == v then
            return true
        end
    end
    ros.ERROR('could not find move_group: ' .. name)
    return false
end

local function checkParameterForAvailability(self, topic, wait_duration, max_counter)
    wait_duration = wait_duration or ros.Duration(1.0)
    max_counter = max_counter or 10
    local counter = 0
    local value
    while value == nil and counter < max_counter and ros.ok() do
        ros.WARN('/move_group/trajectory_execution not available trying again in 1 sec')
        wait_duration:sleep()
        value = self.nodehandle:getParamVariable(topic)
        counter = counter + 1
        ros.spinOnce()
    end
    if counter >= max_counter then
        error('could not initialize!! ' .. topic)
    end
    return value
end

local function checkStartState(self, trajectory)
    local start_state = moveit.RobotState.createFromModel(self.robot_model) --manipulator:getCurrentState()
    local p, latency = self.joint_monitor:getPositionsTensor()

    start_state:setVariablePositions(p, self.joint_monitor:getJointNames())
    local ori_start_state = start_state:clone()
    start_state:setVariablePositions(trajectory.points[1].positions, trajectory.joint_names)
    start_state:setVariableVelocities(trajectory.points[1].velocities, trajectory.joint_names)
    if trajectory.points[1].accelerations:size() == trajectory.points[1].positions:size() then
        ros.INFO('Set Accelerations')
        start_state:setVariableAccelerations(trajectory.points[1].accelerations, trajectory.joint_names)
    end
    start_state:update()

    local distance = ori_start_state:distance(start_state)
    ros.INFO('start state distance to current state: %f', distance)
    if distance > self.allowed_start_tolerance then
        local msg = string.format('start state is too far away from current state. tolerance: %f', self.allowed_start_tolerance)
        ros.ERROR(msg)
        return self.error_codes.START_STATE_VIOLATES_PATH_CONSTRAINTS, msg
    end
    return self.error_codes.SUCCESS, ''
end

function IterativeMoveJWorker:__init(nh, joint_monitor)
    self.trajectoryQueue = {} -- list of pending trajectories
    self.syncCallbacks = {}
    self.nodehandle = nh
    self.error_codes = error_codes
    self.allowed_start_tolerance =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_start_tolerance')

    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.plan_scene = moveit.PlanningScene(self.robot_model_loader:getModel())
    self.plan_scene:syncPlanningScene()
    if torch.isTypeOf(joint_monitor, core.JointMonitor) then
        self.joint_monitor = joint_monitor
    else
        ros.INFO('[IterativeMoveJWorker] created own joint monitor')
        self.joint_monitor = core.JointMonitor(self.robot_model:getActiveJointNames():totable(), nil, nil, nh)
    end
    local once = true
    local ready = false
    while not ready and ros.ok() do
        if once then
            ros.ERROR('[IterativeMoveJWorker] joint states not ready')
            once = false
        end
        ready = self.joint_monitor:waitReady(20.0)
    end
end

function IterativeMoveJWorker:doTrajectoryAsync(traj)
    table.insert(self.trajectoryQueue, traj)
end

function IterativeMoveJWorker:addSyncCallback(fn)
    table.insert(self.syncCallbacks, fn)
end

function IterativeMoveJWorker:removeSyncCallback(fn)
    for i, x in ipairs(self.syncCallbacks) do
        if x == fn then
            table.remove(self.syncCallbacks, i)
            return
        end
    end
end

function IterativeMoveJWorker:sync()
    for i, fn in ipairs(self.syncCallbacks) do
        fn(self)
    end
    return true
end

function IterativeMoveJWorker:cancelCurrentPlan(abortMsg)
    if self.current_plan ~= nil then
        local traj = self.current_plan.traj
        if traj.cancel ~= nil then
            traj:cancel(abortMsg or 'Canceled') -- abort callback
        end
    end
end

local function checkJointNames(self, move_group_name, joint_names)
    local ori_joint_names = self.robot_model:getGroupJointNames(move_group_name)
    return table.isSimilar(ori_joint_names, joint_names)
end

local function findGroupNameFromJointNames(self, joint_names)
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    for i, v in pairs(all_group_joint_names) do
        if checkJointNames(self, v, joint_names) then
            return v
        end
    end
    return nil
end

local function handleMoveJTrajectory(self, traj)
    local group_name
    local status = 0
    local msg

    ros.INFO('xamlamoveit_msgs/moveJActionGoal')
    group_name = findGroupNameFromJointNames(self, traj.goal.goal.trajectory.joint_names)
    ros.INFO('Specified groupName: %s', tostring(group_name))
    if not group_name then
        status = self.error_codes.INVALID_GOAL
        msg = string.format('could not find group name: %s', group_name)
    else
        traj.joint_monitor = self.joint_monitor
        traj.move_group_name = group_name
    end

    if status == 0 then
        status, msg = checkStartState(self, traj.goal.goal.trajectory)
    end
    return traj, status, msg
end

local function dispatchTrajectory(self)
    local status = 0
    if self.current_plan == nil then
        if #self.trajectoryQueue > 0 then -- check if new trajectory is available
            while #self.trajectoryQueue > 0 do
                local traj = table.remove(self.trajectoryQueue, 1)
                traj.joint_monitor = self.joint_monitor
                local msg
                traj, status, msg = handleMoveJTrajectory(self, traj)

                if traj.accept == nil or traj:accept() then -- call optional accept callback
                    traj.status = math.min(traj.status, status)
                    self.current_plan = {
                        startTime = sys.clock(), -- debug information
                        traj = traj,
                        error_msg = msg
                    }
                    break
                end
            end
        end
    end

    -- ensure first points are send to robot immediately after accepting trajectory execution
    if self.current_plan ~= nil then -- if we have an exsting trajectory
        local traj = self.current_plan.traj
        status = traj.status
        local error_msg = self.current_plan.error_msg
        if status >= 0 then
            -- check if trajectory execution is still desired (e.g. not canceled)
            if traj:proceed() == false then
                -- robot not ready or proceed callback returned false
                status = traj.status
                local msg = string.format('Stop plan execution. %s', self.error_codes[status])
                ros.ERROR('Proceed method returned false. ' .. msg)
                self:cancelCurrentPlan(msg, status)
            end
        end
        -- execute main update call
        if status < 0 then -- error
            if traj.abort ~= nil then
                local msg = string.format('status: %s, %s, %d', error_msg, self.error_codes[status], status)
                ros.ERROR(msg)
                traj:abort(msg, status) -- abort callback
            end
            self.current_plan = nil
        elseif status == self.error_codes.SUCCESS then
            ros.INFO('Successful')
            if traj.completed ~= nil then
                ros.INFO('Successful completed')
                traj:completed() -- completed callback
            end
            self.current_plan = nil
        end
    end
end

function IterativeMoveJWorker:reset()
    if self.current_plan then
        local traj = self.current_plan.traj
        if traj.abort ~= nil then
            ros.ERROR('[IterativeMoveJWorker:reset] current_plan failed')
            traj:abort()
        end
        self.current_plan = nil
    end
    if #self.trajectoryQueue > 0 then -- check if new trajectory is available
        while #self.trajectoryQueue > 0 do
            --print('#self.trajectoryQueue ' .. #self.trajectoryQueue)
            local traj = table.remove(self.trajectoryQueue, 1)
            if traj.abort ~= nil then
                traj:abort()
            end
        end
    end
    self.allowed_start_tolerance =
        checkParameterForAvailability(self, '/move_group/trajectory_execution/allowed_start_tolerance')
end

local function IterativeMoveJWorkerCore(self)
    dispatchTrajectory(self)
end

function IterativeMoveJWorker:spin()
    --local ok, err = pcall(function() IterativeMoveJWorkerCore(self) end)
    local ok = true
    IterativeMoveJWorkerCore(self)
    -- abort current trajectory
    if (not ok) and self.current_plan then
        local traj = self.current_plan.traj
        if traj.abort ~= nil then
            ros.ERROR('[IterativeMoveJWorker:spin] current_plan failed')
            traj:abort()
        end
        self.current_plan = nil
    end
end

function IterativeMoveJWorker:shutdown()
end

return IterativeMoveJWorker
