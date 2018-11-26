#!/usr/bin/env th
--[[
moveJActionServer.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local components = require 'xamlamoveit.components'
local core = require 'xamlamoveit.core'
local MoveJActionServer = components.MoveJActionServer
local MoveJSafeSteppingActionServer = components.MoveJSafeSteppingActionServer
local xutils = require 'xamlamoveit.xutils'
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaMoveJAction')

local sp = ros.AsyncSpinner() -- background job
sp:start()

local root_nh = ros.NodeHandle()
local local_nh = ros.NodeHandle('~')
local system_state_subscriber =
    root_nh:subscribe(
    '/xamla_sysmon/system_status',
    'xamla_sysmon_msgs/SystemStatus',
    1,
    {'udp', 'tcp'},
    {tcp_nodelay = true}
)
local heartbeat = xamla_sysmon.Heartbeat.new()
heartbeat:start(ros.NodeHandle('~'), 10) --[Hz]
heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()
local sysmon_watch = xamla_sysmon.Watch.new(root_nh, 3.0)
local robot_model_loader = moveit.RobotModelLoader('robot_description')
local robot_model = robot_model_loader:getModel()
local joint_monitor = core.JointMonitor(robot_model:getActiveJointNames():totable())
local mj_action_server = {[1] = MoveJActionServer(root_nh, joint_monitor, robot_model),
                          [2] = MoveJSafeSteppingActionServer(local_nh, joint_monitor, robot_model)}
for i, v in ipairs(mj_action_server) do
    system_state_subscriber:registerCallback(
        function(msg, header)
            return v:updateSystemState(msg, header)
        end
    )
    v:start()
end

heartbeat:updateStatus(heartbeat.GO, 'Running ...')
heartbeat:publish()
local global_state_summary = sysmon_watch:getGlobalStateSummary()
error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error

local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() do
    local ok = joint_monitor:getLatency() < 2.5
    local hasTrajectoryActive = false
    if ok then
        heartbeat:updateStatus(heartbeat.GO, 'Running ...')
        for i, v in ipairs(mj_action_server) do
            local status, err = xpcall( function() v:spin() end, error_msg_func )
            if status == false then
                ros.ERROR(tostring(err))
                heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. ' ' .. tostring(err))
                v:reset()
            end
            if not hasTrajectoryActive then
                hasTrajectoryActive = v:hasTrajectoryActive()
            end
        end
    else
        heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, 'Missing Joint States.')
    end
    heartbeat:publish()

    if hasTrajectoryActive then
        ros.spinOnce(0.008)
    else
        ros.spinOnce(0.05)
    end
end

for i, v in pairs(mj_action_server) do
    v:shutdown()
end

sp:stop()
ros.shutdown()
