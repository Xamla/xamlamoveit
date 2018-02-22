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

function MoveJActionServer:__init(nh)
    self.node_handle = nh
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
    self.worker = MoveJWorker(self.node_handle)
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
