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

local function moveJAction_serverGoal(goal_handle, joint_monitor, target_joint_names)
    ros.INFO('moveJAction_serverGoal')
    local g = goal_handle:getGoal()
    if not isSubset(g.goal.trajectory.joint_names, target_joint_names) then
        ros.ERROR('not correct set of joints for this group')
        return
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
local function updateSystemState(msg, header)
    if msg.system_status ~= 0 and error_state == false then
        error_state = true
    elseif msg.system_status == 0 and error_state == true then
        error_state = false
    end
end


local function initActions()

    config = node_handle:getParamVariable('/move_group/controller_list')
    local start_time = ros.Time.now()
    local current_time = ros.Time.now()
    local attemts = 0
    while config == nil do
        attemts = attemts + 1
        ros.WARN('no controller specified in "/move_group/controller_list". Retry in 5sec')
        while current_time:toSec() - start_time:toSec() < 5 do
            current_time = ros.Time.now()
            sys.sleep(0.01)
        end
        start_time = ros.Time.now()
        current_time = ros.Time.now()
        config = node_handle:getParamVariable('/move_group/controller_list')

        if not ros.ok() then
            return -1, 'Ros is not ok'
        end

        if attemts > 5 then
            return -2, 'Reached max attempts'
        end
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
    print(joint_name_collection)
    local joint_monitor = xutils.JointMonitor(joint_name_collection)
    local timeout = ros.Duration(2.01)
    if not joint_monitor:waitReady(timeout) then
        error("FAILED init")
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
        ros.ERROR("joint_monitor has difficulties finding joints")
    end
    for i, v in ipairs(config) do
        action_server[v.name]:registerGoalCallback(
            function(gh)
                moveJAction_serverGoal(gh, joint_monitor, v.joints)
            end
        )
        action_server[v.name]:registerCancelCallback(FollowJointTrajectory_Cancel)
        action_server[v.name]:start()
        print(v.name)
    end


    worker = GenerativeSimulationWorker.new(ros.NodeHandle(string.format('/%s', ns[1])))
    return 0, 'Success'
end

local function shutdownAction_server()
    for i, v in pairs(action_server) do
        v:shutdown()
    end
end

local cmd = torch.CmdLine()

local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
initSetup(parameter['__name']) -- TODO


local function init()
    local err, msg = initActions()

    if err < 0 then
        ros.ERROR('Could not initialize controller. ' .. msg)
        return
    end

    return ros.Rate(1/worker.servoTime)
end

local function reset()
    shutdownAction_server()
    return init()
end

local function simulation()
    local heartbeat = xamla_sysmon.Heartbeat.new()
    heartbeat:start(node_handle, 0.5) --[Hz]
    heartbeat:updateStatus(heartbeat.BUSY, 'WORKING ...')
    heartbeat:publish()
    local system_state_subscriber =
        node_handle:subscribe(
        '/xamla_sysmon/system_status',
        'xamla_sysmon_msgs/SystemStatus',
        1,
        {'udp', 'tcp'},
        {tcp_nodelay = true}
    )
    local dt = init()
    local initialized = true
    system_state_subscriber:registerCallback(updateSystemState)

    while ros.ok() do
        if error_state == false then
            if initialized == false then
                xutils.tic('Initialize')
                ros.INFO('Reinizialise')
                dt = init()
                heartbeat:updateStatus(heartbeat.BUSY, 'WORKING ...')
                initialized = true
                xutils.toc('Initialize')
            end
        else
            if initialized then
                ros.ERROR('error state')
                shutdownAction_server()
                initialized = false
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
