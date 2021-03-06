#!/usr/bin/env th
--[[
rosJointJoggingServiceNode.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local controller = xamlamoveit.controller
local core = xamlamoveit.core
local xamla_sysmon = require 'xamla_sysmon'
local planning = xamlamoveit.planning

local xutils = xamlamoveit.xutils
local printf = xutils.printf
local xtable = xutils.Xtable

local node_handle, sp
local sysmon_watch
local cntr
local run = false

local all_EE_parent_group_names, all_EE_parent_link_names = {}, {}
local all_group_joint_names = {}

local last_status_message_tracking = 'IDLE'
local do_plot = false

local function printSplash()
    print(
        [[
     _  __                __
    | |/ /___ _____ ___  / /___ _     "I like to move it move it"
    |   / __ `/ __ `__ \/ / __ `/
   /   / /_/ / / / / / / / /_/ /
  /_/|_\__,_/_/ /_/ /_/_/\__,_/
  Xamla Jogging Node v1.0
  ]]
    )
end

local function initSetup(name)
    ros.init(name)
    node_handle = ros.NodeHandle('~')
    service_queue = ros.CallbackQueue()
    sysmon_watch = xamla_sysmon.Watch.new(node_handle, 1)

    sp = ros.AsyncSpinner() -- background job
    sp:start()
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
    print("shutdown complete")
end

local FLAGS = {
    [1] = 'self_collision_check_enabled',
    [2] = 'scene_collision_check_enabled',
    [3] = 'joint_limits_check_enabled'
}

local set_flag_spec = ros.SrvSpec('xamlamoveit_msgs/SetFlag')
local get_flag_spec = ros.SrvSpec('xamlamoveit_msgs/GetFlag')
local set_float_spec = ros.SrvSpec('xamlamoveit_msgs/SetFloat')
local get_float_spec = ros.SrvSpec('xamlamoveit_msgs/GetFloat')
local get_strings_spec = ros.SrvSpec('xamlamoveit_msgs/GetStrings')
local get_string_spec = ros.SrvSpec('xamlamoveit_msgs/GetSelected')
local set_string_spec = ros.SrvSpec('xamlamoveit_msgs/SetString')
local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')
local trigger_spec = ros.SrvSpec('std_srvs/Trigger')

local function findString(my_string, collection)
    local index = -1
    if torch.type(collection) == 'table' then
        index = table.indexof(collection, my_string)
    elseif torch.type(collection) == 'std.StringVector' then
        index = table.indexof(collection:totable(), my_string)
    else
        error('unknown type: ' .. torch.type(collection))
    end

    return index > -1, index
end

local function startJogging()
    local sys_state = sysmon_watch:getGlobalStateSummary()
    if sys_state.go == true then
        run = true
        cntr:reset()
        return true, 'Success'
    else
        local message = 'Jogging not started, because system state is NOGO: ' .. sys_state.error_message
        ros.WARN(message)
        return false, message
    end
end

local function stopJogging()
    run = false
    if do_plot then
        cntr:save()
    end
end

--setEndEffector(name)

local function setEndEffectorHandler(request, response, header)
    local new_end_effector_name = request.data
    --stopJogging()
    if cntr:getCurrentEndEffector() == new_end_effector_name then
        response.success = true
        response.message = 'Set end effector successfuly'
    else
        local succ, msg = cntr:setEndEffector(new_end_effector_name)
        response.success = succ
        response.message = msg
        if not succ then
            ros.ERROR(msg)
        end
        ros.INFO(msg)
    end
    return true
end

local function getEndEffectorHandler(request, response, header)
    response.success = true
    response.selected = cntr:getCurrentEndEffector() or ''
    response.collection = {}
    return true
end

local function setMoveGroupHandler(request, response, header)
    local new_move_group_name = request.data
    --stopJogging()
    if cntr:getCurrentMoveGroup():getName() == new_move_group_name then
        response.success = true
        response.message = 'Set move_group successfuly'
    else
        if findString(new_move_group_name, all_group_joint_names) then
            local succ, msg = cntr:setMoveGroupInterface(new_move_group_name)
            response.success = succ
            response.message = msg
            if not succ then
                ros.ERROR(msg)
            end
            ros.INFO(msg)
        else
            local response_message = string.format('Unknown group name! Choose from: %s', new_move_group_name)
            for i, v in ipairs(all_group_joint_names) do
                response_message = string.format('%s %s;', response_message, v)
            end

            response.success = false
            response.message = response_message
        end
    end
    return true
end

local function getMoveGroupHandler(request, response, header)
    response.success = true
    response.selected = cntr:getCurrentMoveGroup():getName()
    response.collection = all_group_joint_names
    return true
end

local function getFlagNamesHandler(request, response, header)
    response.result = FLAGS
    response.success = true
    return true
end

local function setFlagHandler(request, response, header)
    local flag_name = request.name
    local value = request.value
    local suc, index = findString(flag_name, FLAGS)
    if suc then
        response.success = false
    end

    if index == 1 then
        cntr:setSelfCollisionChecksState(value)
    elseif index == 2 then
        cntr:setSceneCollisionChecksState(value)
    elseif index == 3 then
        cntr:setJointLimitsChecks(value)
    end

    response.success = true
    return true
end

local function getFlagHandler(request, response, header)
    local flag_name = request.name
    local value = nil
    local suc, index = findString(flag_name, FLAGS)
    if suc then
        response.success = false
    end

    if index == 1 then
        value = cntr:getSelfCollisionChecksState()
    elseif index == 2 then
        value = cntr:getSceneCollisionChecksState()
    elseif index == 3 then
        value = cntr:getJointLimitsChecks()
    end

    response.value = value
    response.success = value ~= nil
    return true
end

local function startStopHandler(request, response, header)
    local startStop = request.data
    if startStop == nil then
        response.success = false
        response.message = 'bool is nil'
        return true
    end
    local success, message = true, 'Success'
    if startStop == true then
        success, message = startJogging()
    else
        stopJogging()
    end
    response.success = success
    response.message = message
    return true
end

local function getVelocityLimitsHandler(request, response, header)
    response.data = cntr.speed_scaling
    response.success = true
    return true
end

local function setVelocityLimitsHandler(request, response, header)
    local scaling = request.data
    scaling = math.min(1.0, math.max(0, scaling))
    cntr:setSpeedScaling(scaling)
    response.success = true
    response.message = 'Scaling set to ' .. scaling
    return true
end

local function getStatusHandler(request, response, header)
    ros.DEBUG('getStatusHandler')
    response.is_running = run
    response.move_group_name = cntr:getCurrentMoveGroup():getName()
    response.joint_names = cntr.lastCommandJointPositions:getNames()
    response.out_topic = ''
    response.in_topic = ''
    response.status_message_tracking = tostring(last_status_message_tracking)
    return true
end

local function resetErrorHandler(request, response, header)
    ros.DEBUG('resetErrorHandler')
    cntr:resetError()
    response.success = true
    response.message = "ok"
    return true
end

local function joggingServer(name)
    printSplash()
    initSetup(name or 'joggingServer')
    local nh = node_handle
    local ns = nh:getNamespace()
    local dt = ros.Rate(250)
    local idle_dt = ros.Rate(10)
    ros.INFO('Get robot description for robot model.')
    local robot_model_loader = moveit.RobotModelLoader('robot_description')
    local robot_model = robot_model_loader:getModel()

    ros.INFO('Extract move groups and end_effectors')
    all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    all_group_joint_names = robot_model:getJointModelGroupNames() or {}
    local planningGroup, succ = nh:getParamString('move_group')
    if not succ or (table.indexof(all_group_joint_names, planningGroup or '') < 0) then
        if #all_group_joint_names>0 then
            planningGroup = all_group_joint_names[1]
        else
            ros.INFO("no planning groups available. [Idle] mode.")
            while ros.ok() do
                idle_dt:sleep()
            end
        end
    end


    ros.INFO('Connect controller.')
    local sub = nh:subscribe('/execute_trajectory/status', 'actionlib_msgs/GoalStatusArray')
    while ros.ok() and sub:getNumPublishers() == 0 do
        ros.spinOnce(0.1)
        idle_dt:sleep()
    end
    sub:shutdown()
    sub = nil
    local config = nh:getParamVariable(string.format('%s/controller_list', nh:getNamespace()))
    ros.INFO('get move group interface')
    assert(ros.ok(), "ros is not running!")
    local move_group = moveit.MoveGroupInterface(planningGroup)
    local joint_monitor = core.JointMonitor(robot_model:getActiveJointNames():totable())
    local ready = false
    local once = true
    while not ready and ros.ok() do
        ready = joint_monitor:waitReady(30.0)
        if once and not ready then
            ros.ERROR('joint states not ready')
            once = false
            ros.spinOnce(0.1)
        end
    end
    ros.INFO('joint states ready')
    cntr = controller.JoggingControllerOpenLoop(nh, joint_monitor, move_group, config, dt)

    while not cntr:connect('jogging_command', 'jogging_setpoint', 'jogging_twist') do
        dt:sleep()
        ros.spinOnce(0.1)
    end

    value, suc = nh:getParamDouble('timeout')
    if suc then
        cntr:setTimeout(value)
    end
    ---Services
    --set_limits
    ros.INFO('Start services')
    local set_limits_server = nh:advertiseService('set_velocity_scaling', set_float_spec, setVelocityLimitsHandler)
    local get_limits_server = nh:advertiseService('get_velocity_scaling', get_float_spec, getVelocityLimitsHandler)
    --set_movegroup
    local set_movegroup_server = nh:advertiseService('set_movegroup_name', set_string_spec, setMoveGroupHandler)
    local get_movegroup_server = nh:advertiseService('get_movegroup_name', get_string_spec, getMoveGroupHandler)
    --set endeffector
    local set_endeffector_server = nh:advertiseService('set_endeffector_name', set_string_spec, setEndEffectorHandler)
    local get_endeffector_server = nh:advertiseService('get_endeffector_name', get_string_spec, getEndEffectorHandler)

    local start_stop_server = nh:advertiseService('start_stop_tracking', set_bool_spec, startStopHandler)

    local set_flag_server = nh:advertiseService('set_flag', set_flag_spec, setFlagHandler)
    local get_flag_server = nh:advertiseService('get_flag', get_flag_spec, getFlagHandler)
    local get_flag_names_server = nh:advertiseService('get_flag_names', get_strings_spec, getFlagNamesHandler)

    --status
    local status_server = nh:advertiseService('status', get_status_spec, getStatusHandler)

    --reset error
    local reset_error_server = nh:advertiseService('reset_error', trigger_spec, resetErrorHandler)

    local print_idle_once = false
    local print_running_once = false
    local last_loop_start
    while ros.ok() do
        last_loop_start = ros.Time.now()

        --collectgarbage()
        local sys_state = sysmon_watch:getGlobalStateSummary()
        if sys_state.no_go == true and run == true then
            ros.WARN('Jogging stopped because system state is NOGO. ' .. sys_state.error_message)
            stopJogging()
        end
        if run then
            if not print_running_once then
                ros.INFO('Running!')
                print_running_once = true
                print_idle_once = false
            end
            local success
            success, last_status_message_tracking = cntr:update()
            if not success then
                ros.WARN(tostring(last_status_message_tracking))
            end
            ros.spinOnce(dt:expectedCycleTime())
        else
            if not print_idle_once then
                ros.INFO('IDLE')
                print_idle_once = true
                print_running_once = false
            end
            cntr:releaseResources()
            ros.spinOnce(idle_dt:expectedCycleTime())
            idle_dt:sleep()
        end
        dt:sleep()
        local new_dt = ros.Time.now() - last_loop_start
        if new_dt < dt:expectedCycleTime() * 10 then
            cntr:setDeltaT(new_dt)
        else
            cntr:setDeltaT(dt)
        end
    end
    set_limits_server:shutdown()
    get_limits_server:shutdown()
    set_movegroup_server:shutdown()
    get_movegroup_server:shutdown()
    set_endeffector_server:shutdown()
    get_endeffector_server:shutdown()
    start_stop_server:shutdown()
    set_flag_server:shutdown()
    get_flag_server:shutdown()
    get_flag_names_server:shutdown()
    status_server:shutdown()
    reset_error_server:shutdown()
    cntr:shutdown()
end

local result = xutils.parseRosParametersFromCommandLine(arg) or {}
joggingServer(result['__name'])
shutdownSetup()
