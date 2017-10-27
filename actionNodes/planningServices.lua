#!/usr/bin/env th
local ros = require 'ros'

local xamlamoveit = require 'xamlamoveit'
local jtps = xamlamoveit.components.JointTrajectoryPlanningService
local jpps = xamlamoveit.components.JointPathPlanningService
local cpps = xamlamoveit.components.LinearCartesianPathPlanningService
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

local jtps_service = jtps(nh)
local jpps_service = jpps(nh)
local cpps_service = cpps(nh)
local services = {jtps_service, jpps_service, cpps_service}

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
local dt = ros.Rate(20)
while ros.ok() and not emerg_stop_flag do
    for i, v in pairs(services) do
        local status,
            err =
            pcall(
            function()
                v:spin()
            end
        )
        if v.current_state == 5 then
            emerg_stop_flag = true
        end
        if status == false then
            heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. ' ' .. tostring(err))
        end
    end

    heartbeat:publish()
    ros.spinOnce()
    dt:sleep()
end

for i, v in pairs(services) do
    v:shutdown()
end
sp:stop()
ros.shutdown()
