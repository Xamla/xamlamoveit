local ros = require 'ros'
local moveit = require 'moveit'
require 'xamlamoveit.core.IterativeMoveJWorker'
local TrajectorySteppingExecutionRequest = require 'xamlamoveit.components.TrajectorySteppingExecutionRequest'

require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib

local components = require 'xamlamoveit.components.env'
local MoveJSafeSteppingActionServer,
    parent = torch.class('xamlamoveit.components.MoveJSafeSteppingActionServer', 'xamlamoveit.components.RosComponent', components)

function MoveJSafeSteppingActionServer:__init(nh, joint_monitor)
    self.node_handle = nh
    self.joint_monitor = joint_monitor
    self.worker = nil
    self.action_server = nil
    parent.__init(self, nh)
end

local function GoalCallBack(self, goal_handle)
    ros.INFO('Received new moveJ stepping Goal with id: ' .. goal_handle:getGoalID().id)
    local traj = TrajectorySteppingExecutionRequest.new(self.node_handle, goal_handle)
    self:doTrajectoryAsync(traj) -- queue for processing
end

local function CancelCallBack(self, goal_handle)
    ros.INFO('MoveJSafeSteppingActionServerCancel')
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

function MoveJSafeSteppingActionServer:onInitialize()
    self.worker = IterativeMoveJWorker(self.node_handle, self.joint_monitor)
    self.action_server = actionlib.ActionServer(self.node_handle, '/moveJ_step_action', 'xamlamoveit_msgs/moveJ')
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

function MoveJSafeSteppingActionServer:onStart()
    self.action_server:start()
end

function MoveJSafeSteppingActionServer:onProcess()
    self.worker:spin()
end

function MoveJSafeSteppingActionServer:onReset()
    self.action_server:start()
end

function MoveJSafeSteppingActionServer:onStop()
    self.worker:reset()
end

function MoveJSafeSteppingActionServer:onShutdown()
    self.worker:shutdown()
    self.action_server:shutdown()
end

function MoveJSafeSteppingActionServer:hasTrajectoryActive()
    return self.worker.currentPlan ~= nil
end

function MoveJSafeSteppingActionServer:doTrajectoryAsync(traj)
    self.worker:doTrajectoryAsync(traj)
end

return MoveJSafeSteppingActionServer