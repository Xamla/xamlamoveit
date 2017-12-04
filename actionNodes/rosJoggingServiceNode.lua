#!/usr/bin/env th
local ros = require "ros"
local tf = ros.tf
local moveit = require "moveit"
local xamlamoveit = require "xamlamoveit"
local controller = xamlamoveit.controller
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

local last_status_message_tracking = "IDLE"

local function initSetup(name)
    ros.init(name)
    node_handle = ros.NodeHandle("~")
    service_queue = ros.CallbackQueue()
    sysmon_watch = xamla_sysmon.Watch.new(node_handle, 1)

    sp = ros.AsyncSpinner() -- background job
    sp:start()
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
end

local set_float_spec = ros.SrvSpec("xamlamoveit_msgs/SetFloat")
local get_float_spec = ros.SrvSpec("xamlamoveit_msgs/GetFloat")
local get_string_spec = ros.SrvSpec("xamlamoveit_msgs/GetSelected")
local set_string_spec = ros.SrvSpec("xamlamoveit_msgs/SetString")
local get_status_spec = ros.SrvSpec("xamlamoveit_msgs/StatusController")
local set_bool_spec = ros.SrvSpec("std_srvs/SetBool")

local function findString(my_string, collection)
    local index = -1
    if torch.type(collection) == "table" then
        index = table.indexof(collection, my_string)
    elseif torch.type(collection) == "std.StringVector" then
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
end

local function setMoveGroupHandler(request, response, header)
    local new_move_group_name = request.data
    stopJogging()
    if cntr.move_group:getName() == new_move_group_name then
        response.success = true
        response.message = "Set move_group successfuly"
    else
        if findString(new_move_group_name, all_group_joint_names) then
            local succ, msg = cntr:setMoveGroupInterface(new_move_group_name)
            response.success = succ
            response.message = msg
        else
            local response_message = string.format("Unknown group name! Choose from: %s", new_move_group_name)
            for i, v in ipairs(all_group_joint_names) do
                response_message = string.format("%s %s;", response_message, v)
            end

            response.success = false
            response.message = response_message
        end
    end
    return true
end

local function getMoveGroupHandler(request, response, header)
    response.success = true
    response.selected = cntr.move_group:getName()
    response.collection = all_group_joint_names
    print(response)
    return true
end

local function setControllerNameHandler(request, response, header)
    local new_controller_name = request.data
    if new_controller_name == nil or new_controller_name == "" then
        response.success = false
        response.message = "string is empty"
    end
    stopJogging()
    cntr.controller_name = new_controller_name
    cntr.robotControllerTopic =  string.format('/%s/joint_command', cntr.controller_name)
    response.success = true
    response.message = "Success"
    return true
end

local function getControllerNameHandler(request, response, header)
    local name = cntr.controller_name
    if name then
        response.selected = name
    else
        response.selected = ""
    end
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
    response.data = cntr.velocity_scaling
    return true
end

local function setVelocityLimitsHandler(request, response, header)
    local scaling = request.data
    scaling = math.min(1.0,math.max(0,scaling))
    cntr.velocity_scaling = scaling
    response.success = true
    response.message = "Scaling set to " .. scaling
    return true
end

local function getStatusHandler(request, response, header)
    response.is_running = run
    response.move_group_name = cntr.move_group:getName()
    response.joint_names = cntr.joint_monitor:getJointNames()
    response.out_topic = cntr:getOutTopic()
    response.in_topic = cntr:getInTopic()
    response.status_message_tracking = last_status_message_tracking
    return true
end

local function joggingServer(name)
    initSetup(name or "joggingServer")
    local nh = node_handle
    local ns = nh:getNamespace()
    local psi = moveit.PlanningSceneInterface()
    local dt = ros.Duration(1 / 125)
    local controller_name, succ = nh:getParamString("controller_name")
    if not succ then
        controller_name = "sda10d"
    end
    local robot_model_loader = moveit.RobotModelLoader("robot_description")
    local robot_model = robot_model_loader:getModel()

    all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    all_group_joint_names = robot_model:getJointModelGroupNames()
    local planningGroup, succ = nh:getParamString("move_group")
    if not succ or (table.indexof(all_group_joint_names, planningGroup or "") < 0) then
        planningGroup = all_group_joint_names[1]
    end

    cntr = controller.JoggingControllerOpenLoop(nh, moveit.MoveGroupInterface(planningGroup), controller_name, dt)
    ---Services
    --set_limits
    set_limits_server = nh:advertiseService("set_velocity_scaling", set_float_spec, setVelocityLimitsHandler)
    get_limits_server = nh:advertiseService("get_velocity_scaling", get_float_spec, getVelocityLimitsHandler)
    --set_movegroup
    set_movegroup_server = nh:advertiseService("set_movegroup_name", set_string_spec, setMoveGroupHandler)
    get_movegroup_server = nh:advertiseService("get_movegroup_name", get_string_spec, getMoveGroupHandler)

    set_controller_name_server = nh:advertiseService("set_controller_name", set_string_spec, setControllerNameHandler)
    get_controller_name_server = nh:advertiseService("get_controller_name", get_string_spec, getControllerNameHandler)

    start_stop_server = nh:advertiseService("start_stop_tracking", set_bool_spec, startStopHandler)
    --status
    status_server = nh:advertiseService("status", get_status_spec, getStatusHandler)

    while not cntr:connect("jogging_command", "jogging_setpoint") do
        dt:sleep()
        ros.spinOnce()
    end
    local idle_dt = ros.Rate(10)
    local success = true
    while ros.ok() do
        ros.spinOnce()
        local sys_state = sysmon_watch:getGlobalStateSummary()
        if sys_state.no_go == true and run == true then
            ros.WARN('Jogging stopped because system state is NOGO. ' .. sys_state.error_message)
            stopJogging()
        end
        if run then
            ros.DEBUG("RUNNING")
            success, last_status_message_tracking = cntr:update()
            if not success then
                ros.WARN(last_status_message_tracking)
                success = true -- one warning should be fine
            end
        else
            idle_dt:sleep()
        end
        dt:sleep()
        collectgarbage()
    end
    shutdownSetup()
end

local result = xutils.parseRosParametersFromCommandLine(arg) or {}

joggingServer(result["__name"])
