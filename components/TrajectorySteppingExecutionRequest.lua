--[[
TrajectorySteppingExecutionRequest.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local components = require 'xamlamoveit.components.env'
local GoalStatus = require 'ros.actionlib.GoalStatus'
local xutils = require 'xamlamoveit.xutils'
local JointValues = require 'xamlamoveit.datatypes.JointValues'
local JointSet = require 'xamlamoveit.datatypes.JointSet'
local TrajectoryRecorder = require 'xamlamoveit.core.TrajectoryRecorder'
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
local progress_spec = ros.MsgSpec('xamlamoveit_msgs/TrajectoryProgress')
local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
local set_double_sec = ros.SrvSpec('xamlamoveit_msgs/SetFloat')
local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')
local set_string_spec = ros.SrvSpec('xamlamoveit_msgs/SetString')
local set_flag_spec = ros.SrvSpec('xamlamoveit_msgs/SetFlag')

local jogging_feedback_spec = ros.MsgSpec('xamlamoveit_msgs/ControllerState')

local TrajectorySteppingExecutionRequest =
    torch.class('xamlamoveit.components.TrajectorySteppingExecutionRequest', components)

local errorCodes = require 'xamlamoveit.core.ErrorCodes'.error_codes
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))
local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end

local FLAGS = {
    [1] = 'self_collision_check_enabled',
    [2] = 'scene_collision_check_enabled',
    [3] = 'joint_limits_check_enabled'
}

local function checkStatusJoggingNode(self)
    local req = self.jogging_status_service:createRequest()
    local res = self.jogging_status_service:call(req)

    if res then
        return res.is_running, res.status_message_tracking
    else
        return nil, 'Jogging status service is not reachable.'
    end
end

local function setMoveGroupName(self)
    local req = self.jogging_move_group_name_service:createRequest()
    req.data = self.move_group_name or ''
    local res = self.jogging_move_group_name_service:call(req)
    if res then
        self.abort_action = not res.success
    else
        self.abort_action = true
    end
    return not self.abort_action, res and res.message
end

local function setJoggingCheckCollisionState(self, state)
    for i = 1, 2 do
        local req = self.jogging_state_collision_check_service:createRequest()
        req.value = state
        req.name = FLAGS[i]
        local res = self.jogging_state_collision_check_service:call(req)
        if res then
            self.abort_action = not res.success
        else
            self.abort_action = true
            return not self.abort_action
        end
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

local MOTIONDIRECTIONS = {stopped = 0, backward = -1, forward = 1}

function TrajectorySteppingExecutionRequest:__init(node_handle, goal_handle, do_plot)
    self.starttime = ros.Time.now()
    self.starttime_debug = ros.Time.now()
    self.goal_handle = goal_handle
    if do_plot == nil then
        self.do_plot = false
    else
        self.do_plot = do_plot
    end
    self.node_handle = node_handle
    self.joint_monitor = nil
    self.goal = goal_handle:getGoal()
    self.error_codes = errorCodes
    self.check_collision = self.goal.goal.check_collision
    self.jogging_velocity_scaling = 1.0     -- value send to jogging service
    self.global_velocity_scaling = 0.85      -- global trajectory velocity scaling (avoid trajectory run-away due to jogging limits)
    self.position_deviation_threshold = math.rad(5)
    self.publisher = nil
    self.subscriber_next = nil
    self.subscriber_prev = nil
    self.subscriber_step = nil
    self.subscriber_cancel = nil
    self.jogging_activation_service = nil
    self.jogging_velocity_scaling_service = nil
    self.jogging_status_service = nil
    self.jogging_state_collision_check_service = nil
    self.jogging_move_group_name_service = nil
    self.last_jogging_feedback_msg = nil
    self.subscriber_jogging_feedback = nil
    self.move_group_name = nil
    self.abort_action = false
    self.status = 0
    self.index = 1
    self.scaling_target = 0.5          -- dynamic scaling factor (e.g. modified by step messages)
    self.scaling_current = 0
    self.ramp_up_time = 0.1            -- ramp-up time for acceleartion to 1.0 velocity scaling
    self.progress_topic = 'feedback'
    self.allow_index_switch = false
    self.is_canceled = false
    self.simulated_time = ros.Duration(0)
    self.dt = ros.Duration(1 / 30)
    self.time_of_last_command_request = ros.Time.now()
    self.current_direction = MOTIONDIRECTIONS.stopped
    self.current_tracking_error = 0
    self.last_send_joint_target = nil
    self.max_since_last_call = ros.Duration(0.02)
    self.last_proceed = nil
    self.joint_set = JointSet(self.goal.goal.trajectory.joint_names)
    self.trajectory_recorder = TrajectoryRecorder(self.joint_set:getNames())
end

local function step(self, goal_id, velocity_scaling)
    if self.goal_handle:getGoalID().id == goal_id then
        velocity_scaling = math.max(-1, math.min(1, velocity_scaling))      -- clamp value to [-1, 1] range
        ros.DEBUG_NAMED(
            'TrajectorySteppingExecutionRequest',
            'STEP: %d/%d, t: %f, scaling: %f',
            self.index,
            #self.goal.goal.trajectory.points,
            self.simulated_time:toSec(),
            velocity_scaling
        )
        local direction
        if velocity_scaling >= 0 then
            direction = MOTIONDIRECTIONS.forward
        else
            direction = MOTIONDIRECTIONS.backward
        end
        if self.current_direction ~= direction then
            self.current_direction = direction
            self.scaling_current = 0
        end
        self.scaling_target = math.abs(velocity_scaling)
        self.scaling_current = math.min(self.scaling_current, self.scaling_target)
        self.time_of_last_command_request = ros.Time.now()
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

local function jogging_feedback(self, msg, header)
    self.last_jogging_feedback_msg = msg
end

local function rescaleDeltaT(self, max_dt, distance)
    local alpha = 0.00045
    local sigmoid = 1/(1+ math.exp((distance-0.04)/alpha))
    return ros.Duration(max_dt:toSec()*sigmoid)
end

function TrajectorySteppingExecutionRequest:connect(
    jogging_topic,
    start_stop_service,
    status_service,
    set_velocity_service,
    set_flag,
    set_move_group_name,
    feedback_topic)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create service client for jogging node.')
    self.jogging_activation_service = ros.ServiceClient(start_stop_service, set_bool_spec)
    self.jogging_velocity_scaling_service = ros.ServiceClient(set_velocity_service, set_double_sec)
    self.jogging_status_service = ros.ServiceClient(status_service, get_status_spec)
    self.jogging_state_collision_check_service = ros.ServiceClient(set_flag, set_flag_spec)
    self.jogging_move_group_name_service = ros.ServiceClient(set_move_group_name, set_string_spec)

    local is_running, status_msg = checkStatusJoggingNode(self)
    if is_running then
        return false
    elseif is_running == nil then
        ros.ERROR(status_msg)
        return false
    end
    activateJoggingService(self)
    ros.INFO('[TrajectorySteppingExecutionRequest] Create command publisher')
    self.publisher = self.node_handle:advertise(jogging_topic, joint_pos_spec, 1, false)
    self.feedback = self.node_handle:advertise(self.progress_topic, progress_spec)
    self.subscriber_jogging_feedback = self.node_handle:subscribe(feedback_topic, jogging_feedback_spec, 2)
    self.subscriber_jogging_feedback:registerCallback(
        function(msg, header)
            jogging_feedback(self, msg, header)
        end
    )

    self.subscriber_step = self.node_handle:subscribe('step', 'xamlamoveit_msgs/Step', 2)
    self.subscriber_step:registerCallback(
        function(msg, header)
            step(self, msg.id, msg.velocity_scaling)
        end
    )
    self.subscriber_next = self.node_handle:subscribe('next', 'actionlib_msgs/GoalID', 2)
    self.subscriber_next:registerCallback(
        function(msg, header)
            step(self, msg.id, 0.5)
        end
    )
    self.subscriber_prev = self.node_handle:subscribe('prev', 'actionlib_msgs/GoalID', 2)
    self.subscriber_prev:registerCallback(
        function(msg, header)
            step(self, msg.id, -0.5)
        end
    )
    self.subscriber_cancel = self.node_handle:subscribe('cancel', 'actionlib_msgs/GoalID', 2)
    self.subscriber_cancel:registerCallback(
        function(msg, header)
            cancel(self, msg, header)
        end
    )
    ros.INFO('[TrajectorySteppingExecutionRequest] Connected')
    self.trajectory_recorder:start()
    return true
end

function TrajectorySteppingExecutionRequest:accept()
    local function rejectCall(msg)
        local code = errorCodes.PREEMPTED
        self.status = code
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setRejected(r, msg or 'unknown')
        ros.WARN_NAMED('TrajectorySteppingExecutionRequest',msg)
        self:shutdown()
    end

    if self.goal_handle:getGoalStatus().status == GoalStatus.PENDING then
        self.starttime_debug = ros.Time.now()
        self.status = 0
        local ns = "/xamlaJointJogging"
        local ok, err =
            xpcall (
                function () return self:connect(
                    paths.concat(ns, 'jogging_command'),
                    paths.concat(ns, 'start_stop_tracking'),
                    paths.concat(ns, 'status'),
                    paths.concat(ns, 'set_velocity_scaling'),
                    paths.concat(ns, 'set_flag'),
                    paths.concat(ns, 'set_movegroup_name'),
                    paths.concat(ns, 'feedback')
                )
            end, error_msg_func
        )

        if not ok then
            rejectCall('Jogging node is not ready.')
            return false
        end

        ok = self.joint_monitor:waitReady(10.0)
        if not ok then
            rejectCall('Joint state is not ready.')
            return false
        end

        ok = setJoggingServiceVelocityScaling(self, self.jogging_velocity_scaling)
        if not ok then
            rejectCall('Could not set velocity scaling on jogging node.')
            return false
        end

        ok = setJoggingCheckCollisionState(self, self.check_collision)
        if not ok then
            rejectCall('Could not set collision check flag on jogging node.')
            return false
        end

        ok, msg = setMoveGroupName(self)
        if not ok then
            local reject_msg = string.format('Could not set move grou name on jogging node. [%s]', self.move_group_name)
            rejectCall(reject_msg)
            return false
        end

        self.goal_handle:setAccepted('Starting trajectory execution')
        self.simulated_time = ros.Duration(0)
        self.last_proceed = nil
        self.scaling_target = 0.5
        self.scaling_current = 0
        self.index = 1
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

local function determineNextTarget(self, traj, time)
    local index = 1
    time = time or self.simulated_time:toSec()
    local point_count = #traj.points
    while index < point_count and time >= traj.points[index + 1].time_from_start:toSec() do
        index = index + 1
    end
    local k = math.min(index + 1, point_count)
    local t0, t1 = traj.points[index].time_from_start:toSec(), traj.points[k].time_from_start:toSec()
    local p0, v0 = traj.points[index].positions, traj.points[index].velocities
    local p1, v1 = traj.points[k].positions, traj.points[k].velocities
    local q, qd = interpolateCubic(time - t0, t0, t1, p0, p1, v0, v1)
    return q, qd, index
end

local function syncTrajectory(self, traj, dt, simulated_time, p)
    local q = determineNextTarget(self, traj, simulated_time)
    local err = (p - q):norm()
    local point_count = #traj.points
    if point_count < 1 then
        return simulated_time
    end
    local end_time = traj.points[point_count].time_from_start:toSec()
    local t = simulated_time + dt
    while true do
        local t = math.min(math.max(0, simulated_time + dt), end_time)
        local q = determineNextTarget(self, traj, t)
        local new_err = (p - q):norm()
        if new_err >= err then      -- if the new position is not nearer, stop
            break
        end
        err = new_err
        simulated_time = t
    end

    if math.abs(dt) > 1e-3 then
        return syncTrajectory(self, traj, dt * 0.5, simulated_time, p)
    end

    return simulated_time
end

function TrajectorySteppingExecutionRequest:proceed()
    local status = self.goal_handle:getGoalStatus().status
    local current_tracking_error = self.current_tracking_error
    if status == GoalStatus.ACTIVE or status == GoalStatus.PENDING or status == GoalStatus.PREEMPTING then
        self.status = 0
        local is_running, msg = checkStatusJoggingNode(self)
        if self.last_jogging_feedback_msg and (self.last_jogging_feedback_msg.self_collision_check_enabled and self.last_jogging_feedback_msg.error_code == -2 or
           self.last_jogging_feedback_msg.scene_collision_check_enabled and self.last_jogging_feedback_msg.error_code == -3) then
            self.abort_action = true
        end
        if self.abort_action == true or is_running == false or self.is_canceled == true then
            if self.abort_action == true then
                ros.WARN('[TrajectorySteppingExecutionRequest] could not set status on Jogging node\n %s', tostring(self.last_jogging_feedback_msg))
            end
            if is_running == false then
                ros.WARN('[TrajectorySteppingExecutionRequest] Jogging node should be running but is not.\n %s', tostring(msg))
            end
            self.status = errorCodes.PREEMPTED
            return false
        elseif self.joint_monitor then
            if self.last_jogging_feedback_msg then
                local err_norm = self.last_jogging_feedback_msg.joint_distance:norm()
                ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest',"current_tracking_error norm: %f, max %f", err_norm, current_tracking_error)
            end
            local time_stamp_now = ros.Time.now()
            local traj = self.goal.goal.trajectory
            local p = self.joint_monitor:getPositionsOrderedTensor(traj.joint_names)

            local fb = ros.Message(progress_spec)
            if status == GoalStatus.ACTIVE then
                fb.error_msg = 'ACTIVE'
            elseif status == GoalStatus.PENDING then
                fb.error_msg = 'PENDING'
            elseif status == GoalStatus.PREEMPTING then
                fb.error_msg = 'PREEMPTING'
            end
            local mPoint = ros.Message(joint_point_spec)
            local m = ros.Message(joint_pos_spec)
            m.joint_names = traj.joint_names

            local dt_since_last_call
            if self.last_proceed then
                dt_since_last_call = time_stamp_now - self.last_proceed
                if dt_since_last_call > self.max_since_last_call then
                    dt_since_last_call = self.max_since_last_call
                end
            else
                dt_since_last_call = ros.Duration(0)
            end

            -- send stop signal on timeout or when step message with velocity_scaling == 0 was received
            local q, qd, index = determineNextTarget(self, traj)
            if (time_stamp_now - self.time_of_last_command_request) > ros.Duration(0.1) or self.scaling_target == 0 then
                local q_dot = q - p
                local dist = torch.abs(q_dot)
                self.current_tracking_error = dist:max(1)[1]
                if self.current_direction ~= MOTIONDIRECTIONS.stopped then
                    ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest','trajectory step error norm: %f, max: %f', q_dot:norm(), dist:max(1)[1])
                    --mPoint.velocities:set(qd:zero())
                    self.scaling_current = 0
                    if self.current_direction == MOTIONDIRECTIONS.forward then
                        self.simulated_time = ros.Duration(syncTrajectory(self, traj, 0.05, self.simulated_time:toSec(), p))
                    elseif self.current_direction == MOTIONDIRECTIONS.backward then
                        self.simulated_time = ros.Duration(syncTrajectory(self, traj, -0.05, self.simulated_time:toSec(), p))
                    end
                    self.current_direction = MOTIONDIRECTIONS.stopped
                    ros.DEBUG_NAMED(
                        'TrajectorySteppingExecutionRequest',
                        'Initiate STOP %f',
                        (time_stamp_now - self.time_of_last_command_request):toSec()
                    )
                    self.last_proceed = nil
                end
                mPoint.velocities:set(qd:zero())
            else
                if self.current_direction == MOTIONDIRECTIONS.forward then
                    if self.scaling_current == 0 then
                        local before = self.simulated_time
                        self.simulated_time = ros.Duration(syncTrajectory(self, traj, 0.05, self.simulated_time:toSec(), p))
                        self.last_proceed = nil
                        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'forward sync from %f -> %f, delta: %f', before:toSec(), self.simulated_time:toSec(), (self.simulated_time - before):toSec())
                    else
                        local max_dt = dt_since_last_call * math.min(self.scaling_current, self.global_velocity_scaling)
                        local new_dt = rescaleDeltaT(self, max_dt, current_tracking_error)
                        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'dt: %f', new_dt:toSec())
                        self.simulated_time = self.simulated_time + new_dt
                        local last_index = #traj.points
                        if self.simulated_time > traj.points[last_index].time_from_start then
                            self.simulated_time = traj.points[last_index].time_from_start
                        end
                    end
                elseif self.current_direction == MOTIONDIRECTIONS.backward then
                    if self.scaling_current == 0 then
                        local before = self.simulated_time
                        self.simulated_time = ros.Duration(syncTrajectory(self, traj, -0.05, self.simulated_time:toSec(), p))
                        self.last_proceed = nil
                        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest', 'backward sync from %f -> %f, delta: %f', before:toSec(), self.simulated_time:toSec(), (self.simulated_time - before):toSec())
                    else
                        local max_dt = dt_since_last_call * math.min(self.scaling_current, self.global_velocity_scaling)
                        local new_dt = rescaleDeltaT(self, max_dt, current_tracking_error)
                        ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest','dt: %f', new_dt:toSec())
                        self.simulated_time = self.simulated_time - new_dt
                        if self.simulated_time:toSec() < 0 then
                            self.simulated_time = ros.Duration(0)
                        end
                    end
                end
                q, qd, index = determineNextTarget(self, traj, self.simulated_time:toSec())
                self.index = index
                local q_dot = q - p
                local dist = torch.abs(q_dot)
                ros.DEBUG_THROTTLE(
                    'direction',
                    0.1,
                    string.format('comparison: %04f', q_dot / q_dot:norm() * qd / qd:norm())
                )
                ros.DEBUG_NAMED('TrajectorySteppingExecutionRequest','trajectory step error norm: %f, max: %f', q_dot:norm(), dist:max(1)[1])
                self.last_send_joint_target = q
                self.current_tracking_error = dist:max(1)[1]
                mPoint.positions:set(q)
                if self.index >= #traj.points and dist:gt(1e-3):sum() == 0 then
                    self.status = errorCodes.SUCCESS
                end

                if self.scaling_current < self.scaling_target then
                    -- ramp up to target scaling
                    self.scaling_current = math.min(self.scaling_target, self.scaling_current + dt_since_last_call:toSec() / self.ramp_up_time)
                end
            end

            mPoint.time_from_start = ros.Duration(0.0)
            m.points = {mPoint}
            self.publisher:publish(m)
            local sample
            if mPoint.positions and mPoint.positions:nElement() > 0 then
                sample = JointValues(self.joint_set, mPoint.positions)
            else
                if self.last_send_joint_target then
                    sample = JointValues(self.joint_set, self.last_send_joint_target)
                end
            end
            if sample then
                self.trajectory_recorder:add(sample)
            end
            fb.index = self.index
            fb.num_of_points = #traj.points
            fb.progress = self.simulated_time:toSec() / traj.points[#traj.points].time_from_start:toSec()
            fb.control_frequency = 1 / self.dt:toSec()

            fb.error_code = self.status
            self.feedback:publish(fb)
            self.last_proceed = ros.Time.now()
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
        ros.INFO('[TrajectorySteppingExecutionRequest] Send Cancel Request: %d', status)
        self.goal_handle:setCancelRequested()
    elseif status == GoalStatus.PREEMPTING then
        ros.INFO('[TrajectorySteppingExecutionRequest] Notifying client about PREEMTING state')
        local code = code or errorCodes.PREEMPTED
        local r = self.goal_handle:createResult()
        r.result = code
        self.goal_handle:setAborted(r, msg or 'Abort')
    end
    self:shutdown()
end

local function disposeRos(self, unit)
    if self[unit] then
        self[unit]:shutdown()
        self[unit] = nil
    end
end

function TrajectorySteppingExecutionRequest:shutdown()
    disposeRos(self, 'feedback')
    disposeRos(self, 'publisher')
    disposeRos(self, 'subscriber_step')
    disposeRos(self, 'subscriber_next')
    disposeRos(self, 'subscriber_prev')
    disposeRos(self, 'subscriber_cancel')
    disposeRos(self, 'subscriber_cancel')
    disposeRos(self, 'subscriber_jogging_feedback')

    if self.jogging_activation_service then
        setJoggingCheckCollisionState(self, true)
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
    if self.do_plot then
        self.trajectory_recorder:save()
    end
    self.trajectory_recorder:reset()
end

return TrajectorySteppingExecutionRequest
