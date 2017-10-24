#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local actionlib = ros.actionlib
local ActionServer = actionlib.ActionServer
local GoalStatus = actionlib.GoalStatus

local GenerativeSimulationWorker = require 'xamlamoveit.xutils.GenerativeSimulationWorker'

local xamla_sysmon = require 'xamla_sysmon'
local global_state_summary
-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
    SUCCESSFUL = 0,
    INVALID_GOAL = -1,
    INVALID_JOINTS = -2,
    OLD_HEADER_TIMESTAMP = -3,
    PATH_TOLERANCE_VIOLATED = -4,
    GOAL_TOLERANCE_VIOLATED = -5
}

local config = {}
local node_handle, sp, worker
local joint_monitor_collection = {}
local function initSetup(ns)
    ros.init(ns)
    node_handle = ros.NodeHandle('~')
    --service_queue = ros.CallbackQueue()

    sp = ros.AsyncSpinner() -- background job
    sp:start()
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
end

local function isSubset(A, B)
    for ia, a in ipairs(A) do
        if table.indexof(B, a) == -1 then
            return false
        end
    end
    return true
end

local function isSimilar(A, B)
    if #A == #B then
        return isSubset(A, B)
    else
        return false
    end
    return true
end

local new_message = false
local seq = 1
local joint_name_collection = {}
local last_command_joint_position = {}
local controller = {}
local action_server = {}
local feedback_buffer_pos = {}
local feedback_buffer_vel = {}

local function decodeJointTrajectoryMsg(trajectory)
    local point_count = #trajectory.points
    local dim = #trajectory.joint_names
    local time = torch.zeros(point_count) -- convert trajectory to internal tensor format
    local pos = torch.zeros(point_count, dim)
    local vel = torch.zeros(point_count, dim)
    local acc = torch.zeros(point_count, dim)
    local has_velocity = true
    local has_acceleration = true

    for i = 1, point_count do
        local pt = trajectory.points[i]
        time[i] = pt.time_from_start:toSec()
        pos[i]:copy(pt.positions)

        if pt.velocities ~= nil and pt.velocities:nElement() > 0 then
            vel[i]:copy(pt.velocities)
        else
            has_velocity = false
        end

        if pt.accelerations ~= nil and pt.accelerations:nElement() > 0 then
            acc[i]:copy(pt.accelerations)
        else
            has_acceleration = false
        end
    end

    if not has_acceleration then
        acc = nil
    end

    if not has_velocity then
        vel = nil
    end

    return time, pos, vel, acc
end

local function moveJAction_serverGoal(global_state_summary, goal_handle, joint_monitor, target_joint_names)
    ros.INFO('moveJAction_serverGoal')
    local g = goal_handle:getGoal()
    if not isSubset(g.goal.trajectory.joint_names, target_joint_names) then
        ros.ERROR('not correct set of joints for this group')
        goal_handle:setRejected(nil, 'not correct set of joints for this group')
        return false
    end
    if global_state_summary then
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state then
            ros.ERROR('Global state is NO GO ')
            print(global_state_summary)
            goal_handle:setRejected(nil, 'Global state is NO GO')
            return false
        end
    end
    -- decode trajectory
    local time, pos, vel, acc = decodeJointTrajectoryMsg(g.goal.trajectory)
    local traj = {
        time = time,
        pos = pos,
        vel = vel,
        acc = acc,
        goalHandle = goal_handle,
        goal = g,
        joint_monitor = joint_monitor,
        joint_names = g.goal.trajectory.joint_names,
        state_joint_names = joint_name_collection,
        accept = function()
            if goal_handle:getGoalStatus().status == GoalStatus.PENDING then
                goal_handle:setAccepted('Starting trajectory execution')
                return true
            else
                ros.WARN('Status of queued trajectory is not pending but %d.', goal_handle:getGoalStatus().status)
                return false
            end
        end,
        proceed = function()
            if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
                return true
            else
                ros.WARN(
                    'Goal status of current trajectory no longer ACTIVE (actual: %d).',
                    goal_handle:getGoalStatus().status
                )
                return false
            end
        end,
        abort = function(self, msg)
            goal_handle:setAborted(nil, msg or 'Error')
        end,
        completed = function()
            local r = goal_handle:createResult()
            r.error_code = worker.errorCodes.SUCCESSFUL
            goal_handle:setSucceeded(r, 'Completed')
        end
    }
    if traj.pos:nElement() == 0 then -- empty trajectory
        local r = goal_handle:createResult()
        r.error_code = TrajectoryResultStatus.SUCCESSFUL
        goal_handle:setSucceeded(r, 'Completed (nothing to do)')
        ros.WARN('Received empty FollowJointTrajectory request (goal: %s).', goal_handle:getGoalID().id)
    else
        worker:doTrajectoryAsync(traj) -- queue for processing
        ros.INFO('Trajectory queued for execution (goal: %s).', goal_handle:getGoalID().id)
    end
end

local function FollowJointTrajectory_Cancel(goalHandle)
    ros.INFO('FollowJointTrajectory_Cancel')

    -- check if trajectory is in trajectoryQueue
    local i =
        table.findIndex(
        worker.trajectoryQueue,
        function(x)
            return x.goalHandle == goalHandle
        end
    )
    if i > 0 then
        -- entry found, simply remove from queue
        table.remove(worker.trajectoryQueue, i)
        goalHandle:setCanceled(nil, 'Canceled')
    elseif worker.currentTrajectory ~= nil and worker.currentTrajectory.goalHandle == goalHandle then
        worker:cancelCurrentTrajectory('Canceled')
    end
end

local error_state = false

local function queryControllerList(node_handle)
    local config = node_handle:getParamVariable(string.format('%s/controller_list', node_handle:getNamespace()))
    local start_time = ros.Time.now()
    local current_time = ros.Time.now()
    local attemts = 0
    while config == nil do
        attemts = attemts + 1
        ros.WARN('no controller specified in "%s/controller_list". Retry in 5sec', node_handle:getNamespace())
        while current_time:toSec() - start_time:toSec() < 5 do
            current_time = ros.Time.now()
            sys.sleep(0.01)
        end
        start_time = ros.Time.now()
        current_time = ros.Time.now()
        config = node_handle:getParamVariable(string.format('%s/controller_list', node_handle:getNamespace()))

        if not ros.ok() then
            return -1, config, 'Ros is not ok'
        end

        if attemts > 5 then
            return -2, config, 'Reached max attempts'
        end
    end
    return 1, config, 'Success'
end

local function initActions()
    ros.INFO('queryControllerList')
    local suc, config, msg = queryControllerList(node_handle)
    if suc < 0 then
        ros.Error('[queryControllerList] ' .. msg)
    end
    local ns
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(joint_name_collection, vv) == -1 then
                joint_name_collection[#joint_name_collection + 1] = vv
            end
        end

        if #v.name > 0 then
            action_server[v.name] =
                ActionServer(
                node_handle,
                string.format('%s/joint_trajectory_action', v.name),
                'control_msgs/FollowJointTrajectory'
            )
        else
            action_server[v.name] =
                ActionServer(node_handle, 'joint_trajectory_action', 'control_msgs/FollowJointTrajectory')
        end
        ns = string.split(action_server[v.name].node:getNamespace(), '/')
    end
    ros.INFO('Init JointMonitor')
    local joint_monitor = xutils.JointMonitor(joint_name_collection)
    ros.INFO('Wait for JointMonitor')
    local timeout = ros.Duration(5.0)
    if not joint_monitor:waitReady(timeout) then
        error('FAILED init')
    else
        ready = true
    end

    --[[local start = ros.Time.now()
    local wait_duration = ros.Rate(10)
    local timeout = ros.Duration(2.01)
    local ready = true
    while not joint_monitor:isReady() do
        local elapsed = ros.Time.now() - start
        if (not ros.ok()) or ((timeout ~= nil) and (elapsed > timeout)) then
            ros.WARN(string.format('Joint states not available during: %s sec', tostring(elapsed)))
            return false
        end
        ros.spinOnce()
        wait_duration:sleep()
    end
    --]]
    if not ready then
        ros.ERROR('joint_monitor has difficulties finding joints')
    else
        ros.INFO('JointMonitor is ready')
    end
    for i, v in ipairs(config) do
        action_server[v.name]:registerGoalCallback(
            function(gh)
                moveJAction_serverGoal(global_state_summary, gh, joint_monitor, v.joints)
            end
        )
        joint_monitor_collection[#joint_monitor_collection + 1] = joint_monitor
        action_server[v.name]:registerCancelCallback(FollowJointTrajectory_Cancel)
        action_server[v.name]:start()
        print(v.name)
    end

    if worker ~= nil then
        ros.INFO('Shutdown GenerativeSimulationWorker')
        worker:shutdown()
        worker.nodehandle:shutdown()
    end
    ros.INFO('Init GenerativeSimulationWorker')
    worker = GenerativeSimulationWorker.new(ros.NodeHandle(string.format('/%s', ns[1])))
    return 0, 'Success'
end

local function shutdownAction_server()
    for i, v in pairs(action_server) do
        v:shutdown()
        v = nil
    end
    for i, v in ipairs(joint_monitor_collection) do
        v:shutdown()
    end
    worker:shutdown()
end

local cmd = torch.CmdLine()

local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
initSetup(parameter['__name']) -- TODO

local function init()
    ros.INFO('initActions')
    local err, msg = initActions()

    if err < 0 then
        ros.ERROR('Could not initialize controller. ' .. msg)
        return
    end

    return ros.Rate(1 / worker.servoTime)
end

local function reset()
    ros.WARN('caling reset')
    shutdownAction_server()
    return init()
end

local function simulation()
    local heartbeat = xamla_sysmon.Heartbeat.new()
    heartbeat:start(node_handle, 10) --[Hz]
    heartbeat:updateStatus(heartbeat.GO, '')
    heartbeat:publish()
    local sysmon_watch = xamla_sysmon.Watch.new(node_handle, 3.0)
    local dt = init()
    local initialized = true

    global_state_summary = sysmon_watch:getGlobalStateSummary()
    while ros.ok() do
        global_state_summary = sysmon_watch:getGlobalStateSummary()
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state == false then
            if initialized == false then
                xutils.tic('Initialize')
                ros.INFO('Reinizialise')
                dt = init()
                heartbeat:updateStatus(heartbeat.GO, '')
                initialized = true
                xutils.toc('Initialize')
            end
        else
            if initialized then
            --ros.ERROR('error state')
            --heartbeat:updateStatus(heartbeat.SECONDARY_ERROR, 'Global State is NOGO, shutting down until it is GO again.')
            --shutdownAction_server()
            --initialized = false
            end
        end

        ros.spinOnce()
        heartbeat:publish()
        worker:spin()
        dt:sleep()
    end
    shutdownAction_server()
end

simulation()
shutdownSetup()
