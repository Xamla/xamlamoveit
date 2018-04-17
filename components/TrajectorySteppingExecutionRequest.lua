local ros = require 'ros'
local components = require 'xamlamoveit.components.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local xutils = require 'xamlamoveit.xutils'
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
local progress_spec = ros.MsgSpec('xamlamoveit_msgs/TrajectoryProgress')
local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
local set_double_sec = ros.SrvSpec('xamlamoveit_msgs/SetFloat')
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

local function setJoggingCheckCollisionState(self, state)
    local req = self.jogging_state_collision_check_service:createRequest()
    req.data = state
    local res = self.jogging_state_collision_check_service:call(req)
    if res then
        self.abort_action = not res.success
    else
        self.abort_action = true
    end
    return not self.abort_action
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

local function setJoggingServiceVelocityScaling(self, scaling)
    local req = self.jogging_velocity_scaling_service:createRequest()
    req.data = scaling
    local res = self.jogging_velocity_scaling_service:call(req)
    if res then
        self.abort_action = not res.success
        ros.WARN('set scaling %d, success = %s', scaling, self.abort and 'TRUE' or 'FALSE')
    else
        self.abort_action = true
    end
    return not self.abort_action
end

local function activateJoggingService(self)
    setJoggingServiceStatus(self, true)
end

local function deactivateJoggingService(self)
    setJoggingServiceStatus(self, false)
end

local MOTIONDIRECTIONS = {stopped = 0, backward = -1, forward = 1}

function TrajectorySteppingExecutionRequest:__init(node_handle, goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle

    self.node_handle = node_handle
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal.goal.check_collision
    self.velocity_scaling = self.goal.goal.veloctiy_scaling
    self.position_deviation_threshold = math.rad(5)
    self.publisher = nil
    self.subscriber_next = nil
    self.subscriber_prev = nil
    self.subscriber_cancel = nil
    self.jogging_activation_service = nil
    self.jogging_velocity_scaling_service = nil
    self.jogging_status_service = nil
    self.jogging_state_collision_check_service = nil
    self.abort_action = false
    self.status = 0
    self.index = 1
    self.progress_topic = 'feedback'
    self.allow_index_switch = false
    self.is_canceled = false

    self.current_direction = MOTIONDIRECTIONS.stopped
end

local function next(self, msg, header)
    if
        self.goal_handle:getGoalID().id == msg.id and
            (self.allow_index_switch or self.current_direction ~= MOTIONDIRECTIONS.forward)
     then
        ros.DEBUG_NAMED(
            'TrajectorySteppingExecutionRequest',
            'NEXT: %d/%d',
            self.index + 1,
            #self.goal.goal.trajectory.points
        )
        self.index = math.min(self.index + 1, #self.goal.goal.trajectory.points)
        self.allow_index_switch = false
        self.current_direction = MOTIONDIRECTIONS.forward
    end
end

local function prev(self, msg, header)
    if
        self.goal_handle:getGoalID().id == msg.id and
            (self.allow_index_switch or self.current_direction ~= MOTIONDIRECTIONS.backward)
     then
        ros.DEBUG_NAMED(
            'TrajectorySteppingExecutionRequest',
            'PREV: %d/%d',
            self.index - 1,
            #self.goal.goal.trajectory.points
        )
        self.index = math.max(self.index - 1, 1)
        self.allow_index_switch = false
        self.current_direction = MOTIONDIRECTIONS.backward
    end
end

local function cancel(self, msg, header)
    if self.goal_handle:getGoalID().id == msg.id then
        ros.DEBUG_NAMED(
            'TrajectorySteppingExecutionRequest',
            'CANCEL: %d/%d',
            self.index,
            #self.goal.goal.trajectory.points
        )
        self.is_canceled = true
    end
end

function TrajectorySteppingExecutionRequest:connect(
    jogging_topic,
    start_stop_service,
    status_service,
    set_velocity_service,
    set_collision_check_state)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create service client for jogging node.')
    self.jogging_activation_service = ros.ServiceClient(start_stop_service, set_bool_spec)
    self.jogging_velocity_scaling_service = ros.ServiceClient(set_velocity_service, set_double_sec)
    self.jogging_status_service = ros.ServiceClient(status_service, get_status_spec)
    self.jogging_state_collision_check_service = ros.ServiceClient(set_collision_check_state, set_bool_spec)
    local is_running, status_msg = checkStatusJoggingNode(self)
    if is_running then
        return false
    end
    activateJoggingService(self)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create command publisher')
    self.publisher = self.node_handle:advertise(jogging_topic, joint_pos_spec, 1, false)
    self.feedback = self.node_handle:advertise(self.progress_topic, progress_spec)

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
    self.subscriber_cancel = self.node_handle:subscribe('cancel', 'actionlib_msgs/GoalID', 2)
    self.subscriber_cancel:registerCallback(
        function(msg, header)
            cancel(self, msg, header)
        end
    )
    return true
end

function TrajectorySteppingExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.starttime_debug = ros.Time.now()
        self.status = 0
        local ok =
            self:connect(
            '/xamlaJointJogging/jogging_command',
            '/xamlaJointJogging/start_stop_tracking',
            '/xamlaJointJogging/status',
            '/xamlaJointJogging/set_velocity_scaling',
            '/xamlaJointJogging/activate_collision_check'
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

        ok = setJoggingServiceVelocityScaling(self, self.velocity_scaling)
        if not ok then
            local code = errorCodes.PREEMPTED
            self.status = code
            local r = self.goal_handle:createResult()
            r.result = code
            self.goal_handle:setRejected(r, 'Could not set velocity scaling on jogging node.')
            self:shutdown()
            return false
        end

        ok = setJoggingCheckCollisionState(self, self.check_collision)
        if not ok then
            local code = errorCodes.PREEMPTED
            self.status = code
            local r = self.goal_handle:createResult()
            r.result = code
            self.goal_handle:setRejected(r, 'Could not set collision check flag on jogging node.')
            self:shutdown()
            return false
        end

        self.goal_handle:setAccepted('Starting trajectory execution')
        return true
    else
        ros.WARN_NAMED(
            'TrajectorySteppingExecutionRequest',
            'Status of queued trajectory is not pending but %d.',
            self.goal_handle:getGoalStatus().status
        )
        return false
    end
end

function TrajectorySteppingExecutionRequest:proceed()
    local status = self.goal_handle:getGoalStatus().status
    if status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then
        self.status = 0
        local is_running, msg = checkStatusJoggingNode(self)
        if self.abort_action == true or is_running == false or self.is_canceled then
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
            mPoint.positions:set(traj.points[self.index].positions)
            mPoint.time_from_start = ros.Duration(0.008) --TODO this is probably not optimal.
            m.points = {mPoint}
            self.publisher:publish(m)
            if self.index >= #traj.points and self.allow_index_switch then
                self.status = errorCodes.SUCCESS
            end
            local fb = ros.Message(progress_spec)
            fb.index = self.index
            fb.num_of_points = #traj.points
            fb.progress = self.index / #traj.points
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
        ros.INFO('[TrajectorySteppingExecutionRequest] Notifying client about PREEMTING state')
        local code = code or errorCodes.PREEMPTED
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setAborted(r, msg or 'Abort')
    elseif status == GoalStatus.RECALLING then
        ros.INFO('[TrajectorySteppingExecutionRequest] Notifying client about RECALLING state')
        self.goal_handle:setCanceled(nil, msg or 'Canceled')
    else
        ros.INFO('[TrajectorySteppingExecutionRequest] nothing to be done')
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
        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'Send Cancel Request: %d', status)
        self.goal_handle:setCancelRequested()
    end
end

function disposeRos(self, unit)
    if self[unit] then
        self[unit]:shutdown()
        self[unit] = nil
    end
end

function TrajectorySteppingExecutionRequest:shutdown()
    disposeRos(self, 'feedback')
    disposeRos(self, 'publisher')
    disposeRos(self, 'subscriber_next')
    disposeRos(self, 'subscriber_prev')
    disposeRos(self, 'subscriber_cancel')

    if self.jogging_activation_service then
        deactivateJoggingService(self)
        self.jogging_activation_service:shutdown()
    end
    self.jogging_activation_service = nil

    disposeRos(self, 'jogging_status_service')
    disposeRos(self, 'jogging_velocity_scaling_service')
    disposeRos(self, 'jogging_state_collision_check_service')
    assert(self.jogging_state_collision_check_service == nil)
    assert(self.jogging_velocity_scaling_service == nil)
    assert(self.jogging_status_service == nil)
end

return TrajectorySteppingExecutionRequest
