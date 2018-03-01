#!/usr/bin/env th
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
end

local set_float_spec = ros.SrvSpec('xamlamoveit_msgs/SetFloat')
local get_float_spec = ros.SrvSpec('xamlamoveit_msgs/GetFloat')
local get_string_spec = ros.SrvSpec('xamlamoveit_msgs/GetSelected')
local set_string_spec = ros.SrvSpec('xamlamoveit_msgs/SetString')
local get_status_spec = ros.SrvSpec('xamlamoveit_msgs/StatusController')
local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')

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
        cntr:reset()
        run = true
        return true, 'Success'
    else
        local message = 'Jogging not started, because system state is NOGO: ' .. sys_state.error_message
        ros.WARN(message)
        return false, message
    end
end

local function stopJogging()
    run = false
    cntr:reset()
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
    response.joint_names = cntr.joint_monitor:getJointNames()
    response.out_topic = ''
    response.in_topic = ''
    response.status_message_tracking = tostring(last_status_message_tracking)
    return true
end

local function joggingServer(name)
    printSplash()
    initSetup(name or 'joggingServer')
    local nh = node_handle
    local ns = nh:getNamespace()
    local psi = moveit.PlanningSceneInterface()
    local dt = ros.Rate(125)
    ros.INFO('Get robot description for robot model.')
    local robot_model_loader = moveit.RobotModelLoader('robot_description')
    local robot_model = robot_model_loader:getModel()

    all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    all_group_joint_names = robot_model:getJointModelGroupNames()
    local planningGroup, succ = nh:getParamString('move_group')
    if not succ or (table.indexof(all_group_joint_names, planningGroup or '') < 0) then
        planningGroup = all_group_joint_names[1]
    end

    ros.INFO('Connect controller.')
    local idle_dt = ros.Rate(10)
    local sub = nh:subscribe('/execute_trajectory/status', 'actionlib_msgs/GoalStatusArray')
    while ros.ok and sub:getNumPublishers() == 0 do
        ros.spinOnce()
        idle_dt:sleep()
    end
    sub:shutdown()
    sub = nil
    local config = nh:getParamVariable(string.format('%s/controller_list', nh:getNamespace()))
    ros.INFO('get move group interface')

    local joint_monitor = core.JointMonitor(robot_model:getActiveJointNames():totable())
    local ready = false
    local once = true
    while not ready and ros.ok() do
        ready = joint_monitor:waitReady(20.0)
        if once then
            ros.ERROR('joint states not ready')
            once = false
        end
    end
    ros.INFO('joint states ready')
    if not ros.ok() then
        return
    end
    local move_group = moveit.MoveGroupInterface(planningGroup)
    cntr = controller.JoggingControllerOpenLoop(nh, joint_monitor, move_group, config, dt)

    while not cntr:connect('jogging_command', 'jogging_setpoint', 'jogging_twist') do
        dt:sleep()
        ros.spinOnce()
    end

    value, suc = nh:getParamDouble('timeout')
    if suc then
        cntr:setTimeout(value)
    end
    ---Services
    --set_limits
    ros.INFO('Start services')
    set_limits_server = nh:advertiseService('set_velocity_scaling', set_float_spec, setVelocityLimitsHandler)
    get_limits_server = nh:advertiseService('get_velocity_scaling', get_float_spec, getVelocityLimitsHandler)
    --set_movegroup
    set_movegroup_server = nh:advertiseService('set_movegroup_name', set_string_spec, setMoveGroupHandler)
    get_movegroup_server = nh:advertiseService('get_movegroup_name', get_string_spec, getMoveGroupHandler)

    start_stop_server = nh:advertiseService('start_stop_tracking', set_bool_spec, startStopHandler)
    --status
    status_server = nh:advertiseService('status', get_status_spec, getStatusHandler)

    local success = true
    local print_idle_once = false
    local print_running_once = false

    while ros.ok() do
        ros.spinOnce()
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
            success, last_status_message_tracking = cntr:update()
            if not success then
                ros.WARN(tostring(last_status_message_tracking))
                success = true -- one warning should be fine
            end
        else
            if not print_idle_once then
                ros.INFO('IDLE')
                print_idle_once = true
                print_running_once = false
            end
            cntr:releaseResources()
            idle_dt:sleep()
        end
        dt:sleep()
    end
end

local result = xutils.parseRosParametersFromCommandLine(arg) or {}
joggingServer(result['__name'])
shutdownSetup()
