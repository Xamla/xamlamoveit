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
    print("res", res)
    if res then

        return res.is_running, res.status_message_tracking
    else
        return false
    end
end

local function connect_cb(self, name, topic)
    print('subscriber connected: ' .. name .. " (topic: '" .. topic .. "')")
    local req = self.jogging_activation_service:createRequest()
    req.data = true
    local res = self.jogging_activation_service:call(req)
    if res then
        self.abort_action = not res.success
    else
        self.abort_action = true
    end
end

local function disconnect_cb(self, name, topic)
    print('subscriber diconnected: ' .. name .. " (topic: '" .. topic .. "')")
    local req = self.jogging_activation_service:createRequest()
    req.data = false
    local res = self.jogging_activation_service:call(req)
    if res then
        self.abort_action = not res.success
    else
        self.abort_action = true
    end
end

function TrajectorySteppingExecutionRequest:__init(node_handle, goal_handle)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle

    self.node_handle = node_handle
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal_handle.goal.goal.check_collision
    self.position_deviation_threshold = 50.3
    self.publisher = nil
    self.subscriber_next = nil
    self.subscriber_prev = nil
    self.jogging_activation_service = nil
    self.jogging_status_service = nil
    self.abort_action = false
    self.status = 0
    self.index = 1
    self.progress_topic = 'feedback'
end

local function next(self, msg, header)
    ros.WARN('NEXT: %d/%d', self.index + 1, #self.goal.goal.trajectory.points)
    self.index = math.min(self.index + 1, #self.goal.goal.trajectory.points)
end

local function prev(self, msg, header)
    ros.WARN('PREV')
    self.index = math.max(self.index - 1, 1)
end

function TrajectorySteppingExecutionRequest:connect(jogging_topic, start_stop_service, status_service)
    ros.INFO('Create service client for jogging node.')
    self.jogging_activation_service = ros.ServiceClient(start_stop_service, set_bool_spec)
    self.jogging_status_service = ros.ServiceClient(status_service, get_status_spec)

    local function ccb(name, topic)
        connect_cb(self, name, topic)
    end
    local function dcb(name, topic)
        disconnect_cb(self, name, topic)
    end
    ros.INFO('Create command publisher')
    self.publisher = self.node_handle:advertise(jogging_topic, joint_pos_spec, 1, false, ccb, dcb)
    self.feedback = self.node_handle:advertise(self.progress_topic, progress_spec )

    self.subscriber_next = self.node_handle:subscribe('next', 'std_msgs/Empty', 16)
    self.subscriber_next:registerCallback(
        function(msg, header)
            next(self, msg, header)
        end
    )
    self.subscriber_prev = self.node_handle:subscribe('prev', 'std_msgs/Empty', 16)
    self.subscriber_prev:registerCallback(
        function(msg, header)
            prev(self, msg, header)
        end
    )
end

function TrajectorySteppingExecutionRequest:accept()
    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.goal_handle:setAccepted('Starting trajectory execution')
        self.starttime_debug = ros.Time.now()
        self:connect(
            '/xamlaJointJogging/jogging_command',
            '/xamlaJointJogging/start_stop_tracking',
            '/xamlaJointJogging/status'
        )

        return self.joint_monitor:waitReady(10.0)
    else
        ros.WARN('Status of queued trajectory is not pending but %d.', self.goal_handle:getGoalStatus().status)
        return false
    end
end

function TrajectorySteppingExecutionRequest:proceed()
    local status = self.goal_handle:getGoalStatus().status
    if  status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then
        --[[
        local is_running, status_msg = checkStatusJoggingNode(self)
        if not is_running or status_msg ~= 'IDLE'then
            print(is_running, status_msg)
            --self.abort_action = true
        end
        ]]

        if self.abort_action == true then
            self.status = errorCodes.ABORT
            return false
        elseif self.joint_monitor then
            local traj = self.goal.goal.trajectory
            local ok, p = self.joint_monitor:getNextPositionsTensor(0.1, traj.joint_names)
            if not ok then
                ros.ERROR('[TrajectorySteppingExecutionRequest] joint tracking could not get actual joint states.')
                self.status = errorCodes.CONTROL_FAILED
                return false
            end
            local q_dot = traj.points[self.index].positions - p
            local dist = torch.abs(q_dot)
            if dist:gt(self.position_deviation_threshold):sum() > 0 then
                ros.ERROR('[TrajectorySteppingExecutionRequest] joint tracking error is too big!!')
                self.status = errorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS
                return false
            end

            local mPoint = ros.Message(joint_point_spec)
            local m = ros.Message(joint_pos_spec)
            m.joint_names = traj.joint_names
            mPoint.positions:set(traj.points[self.index].positions) --TODO this is probably not optimal.
            mPoint.time_from_start = ros.Duration(0.0)
            m.points = {mPoint}
            self.publisher:publish(m)
            if self.index == #traj.points then
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
        local req = self.jogging_activation_service:createRequest()
        req.data = false
        local res = self.jogging_activation_service:call(req)
        self.jogging_activation_service:shutdown()
    end
    self.jogging_activation_service = nil

    if self.jogging_status_service then
        self.jogging_status_service:shutdown()
    end
    self.jogging_status_service = nil
end

return TrajectorySteppingExecutionRequest