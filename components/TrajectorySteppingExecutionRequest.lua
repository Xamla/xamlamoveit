local ros = require 'ros'
local components = require 'xamlamoveit.components.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local xutils = require 'xamlamoveit.xutils'
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
local progress_spec = ros.MsgSpec('xamlamoveit_msgs/TrajectoryProgress')
local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')

local TrajectorySteppingExecutionRequest =
    torch.class('xamlamoveit.components.TrajectorySteppingExecutionRequest', components)

local errorCodes = require 'xamlamoveit.core.ErrorCodes'.error_codes
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))

local function checkStatusJoggingNode(self)
    local req = self.jogging_status_service:createRequest()
    local res = self.jogging_status_service:call(req)

    if res then
        return res.is_running, res.status_message_tracking
    else
        return false
    end
end

local function setJoggingServiceStatus(self, status)
    local req = self.jogging_activation_service:createRequest()
    req.data = status
    local res = self.jogging_activation_service:call(req)
    if res then
        self.abort_action = not res.success
    else
        self.abort_action = true
    end
end

local function activateJoggingService(self)
    setJoggingServiceStatus(self, true)
end

local function deactivateJoggingService(self)
    setJoggingServiceStatus(self, false)
end

function TrajectorySteppingExecutionRequest:__init(node_handle, goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle

    self.node_handle = node_handle
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal.goal.check_collision
    self.position_deviation_threshold = math.rad(1)
    self.publisher = nil
    self.subscriber_next = nil
    self.subscriber_prev = nil
    self.jogging_activation_service = nil
    self.jogging_status_service = nil
    self.abort_action = false
    self.status = 0
    self.index = 1
    self.progress_topic = 'feedback'
    self.allow_index_switch = false
end

local function next(self, msg, header)
    if self.goal_handle:getGoalID().id == msg.id and self.allow_index_switch then
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'NEXT: %d/%d', self.index + 1, #self.goal.goal.trajectory.points)
        self.index = math.min(self.index + 1, #self.goal.goal.trajectory.points)
        self.allow_index_switch = false
    end
end

local function prev(self, msg, header)
    if self.goal_handle:getGoalID().id == msg.id and self.allow_index_switch then
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'PREV: %d/%d', self.index - 1, #self.goal.goal.trajectory.points)
        self.index = math.max(self.index - 1, 1)
        self.allow_index_switch = false
    end
end

function TrajectorySteppingExecutionRequest:connect(jogging_topic, start_stop_service, status_service)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create service client for jogging node.')
    self.jogging_activation_service = ros.ServiceClient(start_stop_service, set_bool_spec)
    self.jogging_status_service = ros.ServiceClient(status_service, get_status_spec)
    local is_running, status_msg = checkStatusJoggingNode(self)
    if is_running then
        return false
    end
    activateJoggingService(self)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create command publisher')
    self.publisher = self.node_handle:advertise(jogging_topic, joint_pos_spec, 1, false)
    self.feedback = self.node_handle:advertise(self.progress_topic, progress_spec )

    self.subscriber_next = self.node_handle:subscribe('next', 'actionlib_msgs/GoalID', 2)
    self.subscriber_next:registerCallback(
        function(msg, header)
            next(self, msg, header)
        end
    )
    self.subscriber_prev = self.node_handle:subscribe('prev', 'actionlib_msgs/GoalID', 2)
    self.subscriber_prev:registerCallback(
        function(msg, header)
            prev(self, msg, header)
        end
    )
    return true
end

function TrajectorySteppingExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.starttime_debug = ros.Time.now()
        self.status = 0
        local ok = self:connect(
            '/xamlaJointJogging/jogging_command',
            '/xamlaJointJogging/start_stop_tracking',
            '/xamlaJointJogging/status'
        )

        if not ok then
            local code = errorCodes.PREEMPTED
            self.status = code
            local r = self.goal_handle:createResult()
            r.result = code
            self.goal_handle:setRejected(r, 'Jogging node is not ready.')
            self:shutdown()
            return false
        end

        ok = self.joint_monitor:waitReady(10.0)
        if not ok then
            local code = errorCodes.PREEMPTED
            self.status = code
            local r = self.goal_handle:createResult()
            r.result = code
            self.goal_handle:setRejected(r, 'Joint state is not ready.')
            self:shutdown()
            return false
        end
        self.goal_handle:setAccepted('Starting trajectory execution')
        return true
    else
        ros.WARN_NAMED('TrajectorySteppingExecutionRequest','Status of queued trajectory is not pending but %d.', self.goal_handle:getGoalStatus().status)
        return false
    end
end

function TrajectorySteppingExecutionRequest:proceed()
    local status = self.goal_handle:getGoalStatus().status
    if  status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then
        self.status = 0
        local is_running, msg = checkStatusJoggingNode(self)
        if self.abort_action == true or is_running == false then
            if self.abort_action then
                ros.WARN('[TrajectorySteppingExecutionRequest] could not set status on Jogging node')
            end
            if is_running == false then
                ros.WARN('[TrajectorySteppingExecutionRequest] Jogging node should be running but is not.')
            end
            self.status = errorCodes.PREEMPTED
            return false
        elseif self.joint_monitor then
            local traj = self.goal.goal.trajectory
            local p = self.joint_monitor:getPositionsOrderedTensor(traj.joint_names)

            local q_dot = traj.points[self.index].positions - p
            local dist = torch.abs(q_dot)
            if dist:gt(self.position_deviation_threshold):sum() < 1 then
                self.allow_index_switch = true
            end

            local mPoint = ros.Message(joint_point_spec)
            local m = ros.Message(joint_pos_spec)
            m.joint_names = traj.joint_names
            mPoint.positions:set(traj.points[self.index].positions) --TODO this is probably not optimal.
            mPoint.time_from_start = ros.Duration(0.0)
            m.points = {mPoint}
            self.publisher:publish(m)
            if self.index >= #traj.points and self.allow_index_switch then
                self.status = errorCodes.SUCCESS
            end
            local fb = ros.Message(progress_spec)
            fb.index = self.index
            fb.num_of_points = #traj.points
            fb.progress = self.index/#traj.points
            fb.control_frequency = -1
            fb.error_msg = 'ACTIVE'
            fb.error_code = self.status
            self.feedback:publish(fb)
        end
        ros.DEBUG('moving')
        return true
    else
        ros.WARN(
            '[TrajectorySteppingExecutionRequest] Goal status of current trajectory no longer ACTIVE (actual: %d).',
            self.goal_handle:getGoalStatus().status
        )
        self.status = errorCodes.SIGNAL_LOST
        return false
    end
end

function TrajectorySteppingExecutionRequest:abort(msg, code)
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        local code = code or errorCodes.FAILURE
        local r = self.goal_handle:createResult()
        r.result = code
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', tostring(r))
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', tostring(msg))
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', tostring(code))
        self.goal_handle:setAborted(r, msg or 'Error')
    elseif status == GoalStatus.PREEMPTING then
        ros.INFO("[TrajectorySteppingExecutionRequest] Notifying client about PREEMTING state")
        local code = code or errorCodes.PREEMPTED
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setAborted(r, msg or 'Abort')
    elseif status == GoalStatus.RECALLING then
        ros.INFO("[TrajectorySteppingExecutionRequest] Notifying client about RECALLING state")
        self.goal_handle:setCanceled(nil, msg or 'Canceled')
    else
        ros.INFO("[TrajectorySteppingExecutionRequest] nothing to be done")
    end
    self:shutdown()
    collectgarbage()
end

function TrajectorySteppingExecutionRequest:completed()
    local r = self.goal_handle:createResult()
    r.result = errorCodes.SUCCESS
    self.goal_handle:setSucceeded(r, 'Completed')
    self:shutdown()
end

function TrajectorySteppingExecutionRequest:cancel()
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.PENDING or status == GoalStatus.ACTIVE then
        self.goal_handle:setCancelRequested()
    end
end

function TrajectorySteppingExecutionRequest:shutdown()
    if self.feedback then
        self.feedback:shutdown()
    end
    self.feedback = nil

    if self.publisher then
        self.publisher:shutdown()
    end
    self.publisher = nil

    if self.subscriber_next then
        self.subscriber_next:shutdown()
    end
    self.subscriber_next = nil

    if self.subscriber_prev then
        self.subscriber_prev:shutdown()
    end
    self.subscriber_prev = nil

    if self.jogging_activation_service then
        deactivateJoggingService(self)
        self.jogging_activation_service:shutdown()
    end
    self.jogging_activation_service = nil

    if self.jogging_status_service then
        self.jogging_status_service:shutdown()
    end
    self.jogging_status_service = nil
end

return TrajectorySteppingExecutionRequest