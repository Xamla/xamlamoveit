#!/usr/bin/env th
--[[
planningServices.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local core = xamlamoveit.core
local jtps = xamlamoveit.components.JointTrajectoryPlanningService
local jpps = xamlamoveit.components.JointPathPlanningService
local cpps = xamlamoveit.components.LinearCartesianTrajectoryPlanningService
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
local parameter = xamlamoveit.xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaPlanningServices')

local nh = ros.NodeHandle('~')
local sp = ros.AsyncSpinner() -- background job
sp:start()
local system_state_subscriber =
    nh:subscribe(
    '/xamla_sysmon/system_status',
    'xamla_sysmon_msgs/SystemStatus',
    1,
    {'udp', 'tcp'},
    {tcp_nodelay = true}
)
local heartbeat = xamla_sysmon.Heartbeat.new()
heartbeat:start(nh, 10) --[Hz]
heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()

local robot_model_loader = moveit.RobotModelLoader('robot_description')
local robot_model = robot_model_loader:getModel()
local joint_monitor = core.JointMonitor(robot_model:getActiveJointNames():totable(), ros.Duration(0.2))

local jtps_service = jtps(nh, joint_monitor, robot_model)
local jpps_service = jpps(nh, joint_monitor, robot_model)
local cpps_service = cpps(nh, joint_monitor, robot_model)
local services = {jtps_service, jpps_service, cpps_service}

for i, v in pairs(services) do
    system_state_subscriber:registerCallback(
        function(msg, header)
            return v:updateSystemState(msg, header)
        end
    )
    v:start()
end

heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()

local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
local emerg_stop_flag = false
while ros.ok() and not emerg_stop_flag do
    local ok = joint_monitor:waitForNextState(1/20)
    if ok then
        local status = true
        for i, v in pairs(services) do
            local single_status, err = xpcall( function() v:spin() end, error_msg_func )
            if v.current_state == 5 then
                emerg_stop_flag = true
            end
            if single_status == false then
                status = false
                heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. ' ' .. tostring(err))
            end
        end
        if status == true then
            heartbeat:updateStatus(heartbeat.GO, 'Working ...')
        end
    end
    heartbeat:publish()
    ros.spinOnce(0.5)
    collectgarbage()
end

for i, v in pairs(services) do
    v:shutdown()
end

sp:stop()
ros.shutdown()
