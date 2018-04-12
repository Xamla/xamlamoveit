local ros = require 'ros'
local moveit = require 'moveit'
require 'xamlamoveit.core.IterativeMoveJWorker'
local TrajectorySteppingExecutionRequest = require 'xamlamoveit.components.TrajectorySteppingExecutionRequest'

require 'ros.actionlib.ActionServer'
local actionlib = ros.actionlib

local components = require 'xamlamoveit.components.env'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/SetString')

local function queryGoalIdServiceHandler(self, request, response, header)

    local data = request.data
    ros.INFO("my requested goal id " .. data)
    local i =
    table.findIndex(
        self.worker.trajectoryQueue,
        function(x)
            return x.goal_handle:getGoalID().id == data
        end
    )
    local isCurrentGoalId = false
    if self.worker.current_plan then
        isCurrentGoalId = self.worker.current_plan.traj.goal_handle:getGoalID().id == data
        ros.INFO("my current goal id " .. self.worker.current_plan.traj.goal_handle:getGoalID().id )
    end
    if i > 0 or isCurrentGoalId == true then
        response.success = true
        response.message = "Valid goal Id"
    else
        response.success = false
        response.message = "Invalid goal Id"
    end

    return true
end

local MoveJSafeSteppingActionServer, parent =
    torch.class(
    'xamlamoveit.components.MoveJSafeSteppingActionServer',
    'xamlamoveit.components.RosComponent',
    components
)

function MoveJSafeSteppingActionServer:__init(nh, joint_monitor)
    self.node_handle = nh
    self.joint_monitor = joint_monitor
    self.worker = nil
    self.action_server = nil
    self.info_service = nil
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
end

function MoveJSafeSteppingActionServer:onStart()
    self.action_server:start()
    self.info_service =
        self.node_handle:advertiseService(
        'query_active_goalIds',
        srv_spec,
        function(request, response, header)
            return queryGoalIdServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
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
    if self.worker then
        self.worker:shutdown()
        self.worker = nil
    end

    if self.action_server then
        self.action_server:shutdown()
        self.action_server = nil
    end

    if self.info_service then
        self.info_service:shutdown()
        self.info_service = nil
    end
end

function MoveJSafeSteppingActionServer:hasTrajectoryActive()
    return self.worker.currentPlan ~= nil
end

function MoveJSafeSteppingActionServer:doTrajectoryAsync(traj)
    self.worker:doTrajectoryAsync(traj)
end

return MoveJSafeSteppingActionServer
