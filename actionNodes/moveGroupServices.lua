#!/usr/bin/env th
--[[
moveGroupServices.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local xamlamoveit = require 'xamlamoveit'
local core = xamlamoveit.core
local mg = xamlamoveit.components.MoveGroupInfoNodeService
local psis = xamlamoveit.components.PositionStateInfoService
local jpccs = xamlamoveit.components.JointPositionCollisionCheckService
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
local parameter = xamlamoveit.xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaservices')

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
local joint_monitor = core.JointMonitor(robot_model:getActiveJointNames():totable())

local mg_service = mg(nh, robot_model)
local psis_service = psis(nh, joint_monitor, robot_model)
local jpccs_service = jpccs(nh, joint_monitor, robot_model)
local services = {mg_service, psis_service, jpccs_service}

for i, v in pairs(services) do
    system_state_subscriber:registerCallback(
        function(msg, header)
            return v:updateSystemState(msg, header)
        end
    )
    v:start()
end

heartbeat:updateStatus(heartbeat.GO, 'Working ...')
heartbeat:publish()

local emerg_stop_flag = false
local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() and not emerg_stop_flag do
    for i, v in pairs(services) do
        local status, err = xpcall(function() v:spin() end, error_msg_func )
        if v.current_state == 5 then
            emerg_stop_flag = true
        end
        if status == false then
            heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. " " .. tostring(err))
        end
    end
    heartbeat:publish()
    collectgarbage()
    ros.spinOnce(0.1)
end

for i, v in pairs(services) do
    v:shutdown()
end
sp:stop()
ros.shutdown()
