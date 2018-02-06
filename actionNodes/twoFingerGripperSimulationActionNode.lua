#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local core = xamlamoveit.core
local actionlib = ros.actionlib
local ActionServer = actionlib.ActionServer
local GoalStatus = actionlib.GoalStatus

local GenerativeSimulationWorker = require 'xamlamoveit.core.GenerativeSimulationWorker'

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


local new_message = false
local seq = 1
local joint_name_collection = {}
local last_command_joint_position = {}
local controller = {}
local action_server = {}
local services = {}
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

local function queryJointLimits(joint_names)
    local max_vel = torch.zeros(#joint_names)
    local max_acc = torch.zeros(#joint_names)
    local max_min_pos = torch.zeros(#joint_names, 2)
    local nh = node_handle
    local root_path = 'robot_description_planning/joint_limits'
    for i, name in ipairs(joint_names) do
        local has_pos_param = string.format('/%s/%s/has_position_limits', root_path, name)
        local get_max_pos_param = string.format('/%s/%s/max_position', root_path, name)
        local get_min_pos_param = string.format('/%s/%s/min_position', root_path, name)
        local has_vel_param = string.format('/%s/%s/has_velocity_limits', root_path, name)
        local get_vel_param = string.format('/%s/%s/max_velocity', root_path, name)
        local has_acc_param = string.format('/%s/%s/has_acceleration_limits', root_path, name)
        local get_acc_param = string.format('/%s/%s/max_acceleration', root_path, name)
        if nh:getParamVariable(has_pos_param) then
            max_min_pos[i][1] = nh:getParamVariable(get_max_pos_param)
            max_min_pos[i][2] = nh:getParamVariable(get_min_pos_param)
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end
        if nh:getParamVariable(has_vel_param) then
            max_vel[i] = nh:getParamVariable(get_vel_param)
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end
        if nh:getParamVariable(has_acc_param) then
            max_acc[i] = nh:getParamVariable(get_acc_param)
        else
            max_acc[i] = max_vel[i] * 0.5
            ros.WARN('Joint: %s has no acceleration limit. Will be set to %f', name, max_acc[i])
        end
    end

    return max_min_pos, max_vel, max_acc
end

local function generateSimpleTvpTrajectory(start, goal, max_velocities, max_accelerations, dt)
    local dim = goal:size(1)
    print('dim', dim)
    print('goal', goal)
    local controller = require 'xamlamoveit.controller'.TvpController(dim)
    controller.max_vel:copy(max_velocities)
    controller.max_acc:copy(max_accelerations)
    local result = controller:generateOfflineTrajectory(start, goal, dt)
    local positions = torch.zeros(#result, dim)
    local velocities = torch.zeros(#result, dim)
    local accelerations = torch.zeros(#result, dim)
    local time = {}
    for i = 1, #result do
        time[i] = dt * i
        positions[{i, {}}]:copy(result[i].pos)
        velocities[{i, {}}]:copy(result[i].vel)
        accelerations[{i, {}}]:copy(result[i].acc)
    end
    time = torch.Tensor(time)
    return time, positions, velocities, accelerations
end

local function createTrajectory(goal_handle, target_joint_names)
    local current_position = joint_monitor:getPositionsTensor(target_joint_names)
    local command = current_position:clone()
    command:fill(g.goal.cmd.position)
    local max_min_pos, max_vel, max_acc = queryJointLimits(target_joint_names)

    local time, pos, vel, acc = generateSimpleTvpTrajectory(current_position, command, max_vel, max_acc, 0.016)
    local traj = {
        time = time,
        pos = pos,
        vel = vel,
        acc = acc,
        goalHandle = goal_handle,
        goal = g,
        joint_monitor = joint_monitor,
        joint_names = target_joint_names,
        state_joint_names = joint_name_collection,
        accept = function()
            if goal_handle:getGoalStatus().status == GoalStatus.PENDING then
                goal_handle:setAccepted('Starting gripper trajectory execution')
                return true
            else
                ros.WARN(
                    'Status of queued gripper trajectory is not pending but %d.',
                    goal_handle:getGoalStatus().status
                )
                return false
            end
        end,
        proceed = function()
            if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
                local fb = goal_handle:createFeeback()
                fb.reached_goal = false
                fb.stalled = false
                local pos = joint_monitor:getPositionsTensor(target_joint_names)
                fb.position = pos[1]
                goal_handle:publishFeedback(fb)
                return true
            else
                ros.WARN(
                    'Goal status of current gripper trajectory no longer ACTIVE (actual: %d).',
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
            r.reached_goal = true
            r.stalled = false
            local pos = joint_monitor:getPositionsTensor(target_joint_names)
            r.position = pos[1]
            goal_handle:setSucceeded(r, 'Completed')
        end
    }
    return traj
end

local function moveGripperAction_serverGoal(global_state_summary, goal_handle, joint_monitor, target_joint_names)
    local g = goal_handle:getGoal()
    if global_state_summary then
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state then
            ros.ERROR('Global state is NO GO ')
            print(global_state_summary)
            goal_handle:setRejected(nil, 'Global state is NO GO')
            return false
        end
    end
    local traj = createTrajectory(goal_handle, target_joint_names)
    if traj.pos:nElement() == 0 then -- empty trajectory
        local r = goal_handle:createResult()
        r.error_code = TrajectoryResultStatus.SUCCESSFUL
        goal_handle:setSucceeded(r, 'Completed (nothing to do)')
        ros.WARN('Received empty GripperCommand request (goal: %s).', goal_handle:getGoalID().id)
    else
        worker:doTrajectoryAsync(traj) -- queue for processing
        ros.INFO('Gripper trajectory queued for execution (goal: %s).', goal_handle:getGoalID().id)
    end
end

local function GripperCommand_Cancel(goalHandle)
    ros.INFO('GripperCommand_Cancel')
    if global_state_summary then
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state then
            ros.ERROR('Global state is NO GO ')
            print(global_state_summary)
            goal_handle:setRejected(nil, 'Global state is NO GO')
            return false
        end
    end
end


local function moveWeissGripperAction_serverGoal(global_state_summary, goal_handle, joint_monitor, target_joint_names)
    local g = goal_handle:getGoal()
    if global_state_summary then
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state then
            ros.ERROR('Global state is NO GO ')
            print(global_state_summary)
            goal_handle:setRejected(nil, 'Global state is NO GO')
            return false
        end
    end
    local traj = createTrajectory()
    if traj.pos:nElement() == 0 then -- empty trajectory
        local r = goal_handle:createResult()
        r.error_code = TrajectoryResultStatus.SUCCESSFUL
        goal_handle:setSucceeded(r, 'Completed (nothing to do)')
        ros.WARN('Received empty GripperCommand request (goal: %s).', goal_handle:getGoalID().id)
    else
        worker:doTrajectoryAsync(traj) -- queue for processing
        ros.INFO('Gripper trajectory queued for execution (goal: %s).', goal_handle:getGoalID().id)
    end
end

local function WeissGripperCommand_Cancel(goalHandle)
    ros.INFO('WeissGripperCommand_Cancel')
    if global_state_summary then
        error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
        if error_state then
            ros.ERROR('Global state is NO GO ')
            print(global_state_summary)
            goal_handle:setRejected(nil, 'Global state is NO GO')
            return false
        end
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

function moveSrv(request, response, header)
    return true
end

function graspSrv(request, response, header)
    return true
end

function releaseSrv(request, response, header)
    return true
end

function homingSrv(request, response, header)
    return true
end

function stopSrv(request, response, header)
    return true
end

function ackSrv(request, response, header)
    return true
end

function setForceSrv(request, response, header)
    return true
end

local function createServices(controller_name)
    local service_set = {}
    service_set.moveSS = nh:advertiseService(controller_name + '/move', moveSrv)
    service_set.graspSS = nh:advertiseService(controller_name + '/grasp', graspSrv)
    service_set.releaseSS = nh:advertiseService(controller_name + '/release', releaseSrv)
    service_set.homingSS = nh:advertiseService(controller_name + '/homing', homingSrv)
    service_set.stopSS = nh:advertiseService(controller_name + '/stop', stopSrv)
    service_set.ackSS = nh:advertiseService(controller_name + '/ack', ackSrv)
    service_set.setForceSS = nh:advertiseService(controller_name + '/set_force', setForceSrv)
    return service_set
end

local function initActions()
    ros.INFO('queryControllerList')
    local suc, config, msg = queryControllerList(node_handle)
    if suc < 0 then
        ros.Error('[queryControllerList] ' .. msg)
    end
    local ns = ''
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(joint_name_collection, vv) == -1 then
                joint_name_collection[#joint_name_collection + 1] = vv
            end
        end
        if v.type == 'GripperCommand' then
            action_server[v.name] =
                ActionServer(node_handle, string.format('%s/%s', v.name, v.action_ns), 'control_msgs/GripperCommand')
        elseif v.type == 'WeissGripperCmd' then
            action_server[v.name] =
                ActionServer(node_handle, string.format('%s/%s', v.name, v.action_ns), 'wsg_50_common/Command')
        end
        ns = string.split(action_server[v.name].node:getNamespace(), '/')
    end
    ros.INFO('Init JointMonitor')
    local joint_monitor = core.JointMonitor(joint_name_collection)
    ros.INFO('Wait for JointMonitor')
    local timeout = ros.Duration(5.0)
    if not joint_monitor:waitReady(timeout) then
        error('FAILED init')
    else
        ready = true
    end

    if not ready then
        ros.ERROR('joint_monitor has difficulties finding joints')
    else
        ros.INFO('JointMonitor is ready')
    end

    for i, v in ipairs(config) do
        if v.type == 'GripperCommand' then
            action_server[v.name]:registerGoalCallback(
                function(gh)
                    moveGripperAction_serverGoal(
                        global_state_summary,
                        gh,
                        joint_monitor,
                        v.joints,
                        action_server[v.name]
                    )
                end
            )
            action_server[v.name]:registerCancelCallback(GripperCommand_Cancel)
        elseif v.type == 'WeissGripperCmd' then
            action_server[v.name]:registerGoalCallback(
                function(gh)
                    moveWeissGripperAction_serverGoal(
                        global_state_summary,
                        gh,
                        joint_monitor,
                        v.joints,
                        action_server[v.name]
                    )
                end
            )
            action_server[v.name]:registerCancelCallback(WeissGripperCmd_Cancel)
        end
        joint_monitor_collection[#joint_monitor_collection + 1] = joint_monitor
        services[v.name] = createServices(v.name)
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
    ros.WARN('calling reset')
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
                ros.INFO('Reinitializing...')
                dt = init()
                heartbeat:updateStatus(heartbeat.GO, '')
                initialized = true
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
