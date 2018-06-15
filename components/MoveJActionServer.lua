--[[
MoveJActionServer.lua

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
require 'xamlamoveit.core.MoveJWorker'
require 'xamlamoveit.xutils'
local TrajectoryExecutionRequest = require 'xamlamoveit.components.TrajectoryExecutionRequest'

require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib

local components = require 'xamlamoveit.components.env'
local MoveJActionServer,
    parent = torch.class('xamlamoveit.components.MoveJActionServer', 'xamlamoveit.components.RosComponent', components)

function MoveJActionServer:__init(nh, joint_monitor)
    self.node_handle = nh
    self.joint_monitor = joint_monitor
    self.worker = nil
    self.action_server = nil
    parent.__init(self, nh)
end

local function GoalCallBack(self, goal_handle)
    ros.INFO('Received new moveJ Goal')
    local traj = TrajectoryExecutionRequest.new(goal_handle)
    self:doTrajectoryAsync(traj) -- queue for processing
end

local function CancelCallBack(self, goal_handle)
    ros.INFO('Cancel moveJ Goal')
    if self.worker.currentPlan ~= nil and self.worker.currentPlan.traj.goal_handle == goal_handle then
        ros.INFO('Cancel active trajectory')
        self.worker:cancelCurrentPlan('Trajectory canceled')
    else
        ros.INFO('\tCancel queued trajectory')
        -- check if trajectory is in trajectoryQueue
        local i =
            table.findIndex(
            self.worker.trajectoryQueue,
            function(x)
                return x.goal_handle == goal_handle
            end
        )
        if i > 0 then
            -- entry found, simply remove from queue
            table.remove(self.worker.trajectoryQueue, i)
        else
            ros.WARN("Trajectory to cancel with goal handle '%s' not found.", goal_handle:getGoalID().id)
        end
        goal_handle:setCanceled(nil, 'Canceled')
    end
end

function MoveJActionServer:onInitialize()
    self.worker = MoveJWorker(self.node_handle, self.joint_monitor)
    self.action_server = actionlib.ActionServer(self.node_handle, 'moveJ_action', 'xamlamoveit_msgs/moveJ')
    self.action_server:registerGoalCallback(
        function(gh)
            GoalCallBack(self, gh)
        end
    )
    self.action_server:registerCancelCallback(
        function(gh)
            CancelCallBack(self, gh)
        end
    )
    --TODO feedback
end

function MoveJActionServer:onStart()
    self.action_server:start()
end

function MoveJActionServer:onProcess()
    self.worker:spin()
end

function MoveJActionServer:onReset()
    self.action_server:start()
end

function MoveJActionServer:onStop()
    self.worker:reset()
end

function MoveJActionServer:onShutdown()
    self.worker:shutdown()
    self.action_server:shutdown()
end

function MoveJActionServer:hasTrajectoryActive()
    return self.worker.currentPlan ~= nil
end

function MoveJActionServer:doTrajectoryAsync(traj)
    self.worker:doTrajectoryAsync(traj)
end

return MoveJActionServer
