--[[
TrajectoryExecutionRequest.lua

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
local components = require 'xamlamoveit.components.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local xutils = require 'xamlamoveit.xutils'
local epsilon = 1e-2

local TrajectoryExecutionRequest = torch.class('xamlamoveit.components.TrajectoryExecutionRequest', components)

local errorCodes = require 'xamlamoveit.core.ErrorCodes'.error_codes
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))

function TrajectoryExecutionRequest:__init(goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle
    self.manipulator = nil
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal_handle.goal.goal.check_collision
end

function TrajectoryExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.goal_handle:setAccepted('Starting trajectory execution')
        self.starttime_debug = ros.Time.now()
        return self.joint_monitor:waitReady(10.0)
    else
        ros.WARN('Status of queued trajectory is not pending but %d.', self.goal_handle:getGoalStatus().status)
        return false
    end
end

-- Returns joints positions at time 't'
local function interpolateCubic(t, t0, t1, p0, p1, v0, v1)
    local dt = t1 - t0
    if dt < 1e-6 then
        return p1, p0:clone():zero()
    end
    local pos = p0:clone()
    local vel = p0:clone()
    for i = 1, p0:size(1) do
        local a = p0[i]
        local b = v0[i]
        local c = (-3 * p0[i] + 3 * p1[i] - 2 * dt * v0[i] - dt * v1[i]) / dt ^ 2
        local d = (2 * p0[i] - 2 * p1[i] + dt * v0[i] + dt * v1[i]) / dt ^ 3
        pos[i] = a + b * t + c * t ^ 2 + d * t ^ 3
        vel[i] = b + 2 * c * t + 3 * d * t ^ 2
    end
    return pos, vel
end

function TrajectoryExecutionRequest:proceed()
    ros.DEBUG('proceed')
    local status = self.goal_handle:getGoalStatus().status
    if  status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then
        if self.manipulator == nil then
            ros.ERROR('[TrajectoryExecutionRequest] move group interface is nil')
            self.status = errorCodes.PREEMPTED
            return false
        elseif self.joint_monitor then
            local now = ros.Time.now()
            local traj = self.goal.goal.trajectory
            local index = 1
            local joint_names = traj.joint_names
            local ok, p, l = self.joint_monitor:getNextPositionsOrderedTensor(ros.Duration(0.01), joint_names)
            assert(ok, 'exceeded timeout for next robot joint state.')
            local dist_to_start = (traj.points[index].positions - p):norm()
            if dist_to_start <= epsilon and traj.points[index].velocities:norm() > 1e-12 then
                self.starttime = now
            else
                self.starttime_debug = now
            end
            local t = now - self.starttime
            if t < ros.Duration(0) then
                t = ros.Duration(0)
            end

            -- linear search for last point < t, increment index when t is greater than next (index+1) point's time_from_start
            while index < #traj.points and t:toSec() >= traj.points[index + 1].time_from_start:toSec() do
                index = index + 1
            end

            local k = math.min(index + 1, #traj.points)
            local t0, t1 = traj.points[index].time_from_start:toSec(), traj.points[k].time_from_start:toSec()
            local p0, v0 = traj.points[index].positions, traj.points[index].velocities
            local p1, v1 = traj.points[k].positions, traj.points[k].velocities
            local q, qd = interpolateCubic(t:toSec() - t0, t0, t1, p0, p1, v0, v1)
            local delta = torch.abs(q - p)
            ros.DEBUG('time planned after start: ' .. t0)
            ros.DEBUG('time after start: ' .. t:toSec())
            ros.DEBUG('time after index: %d, %f sec ', index, t:toSec() - t0)
            ros.DEBUG('position error: ' .. tostring(delta))
            ros.DEBUG('current position: ' .. tostring(p))
            ros.DEBUG('goal position: ' .. tostring(q))
            ros.DEBUG('Latency: ' .. tostring(l))
            if now:toSec() - self.starttime_debug:toSec() > ros.Duration(5):toSec() then
                ros.ERROR('[TrajectoryExecutionRequest] Trajectory start is not working.')
                self.status = errorCodes.CONTROL_FAILED
                return false
            end
        end
        ros.DEBUG('moving')
        return true
    else
        ros.DEBUG(
            '[TrajectoryExecutionRequest] Goal status of current trajectory no longer ACTIVE (actual: %d).',
            self.goal_handle:getGoalStatus().status
        )
        self.status = errorCodes.SIGNAL_LOST
        return false
    end
end

function TrajectoryExecutionRequest:abort(msg, code)
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        local code = code or errorCodes.FAILURE
        local r = self.goal_handle:createResult()
        r.result = code
        ros.WARN(tostring(r))
        ros.WARN(tostring(msg))
        ros.WARN(tostring(code))
        self.goal_handle:setAborted(r, msg or 'Error')
    elseif status == GoalStatus.PREEMPTING then
        ros.INFO("Notifying client about PREEMTING state")
        local code = code or errorCodes.PREEMPTED
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setAborted(r, msg or 'Abort')
    elseif status == GoalStatus.RECALLING then
        ros.INFO("Notifying client about RECALLING state")
        self.goal_handle:setCanceled(nil, msg or 'Canceled')
    else
        ros.INFO("nothing to be done")
    end
    collectgarbage()
end

function TrajectoryExecutionRequest:completed()
    local r = self.goal_handle:createResult()
    r.result = errorCodes.SUCCESS
    self.goal_handle:setSucceeded(r, 'Completed')
    collectgarbage()
end

function TrajectoryExecutionRequest:cancel()
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        self.goal_handle:setCancelRequested()
    end
end

return TrajectoryExecutionRequest
