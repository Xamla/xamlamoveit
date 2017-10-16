#!/usr/bin/env th
local ros = require 'ros'
local components = require 'xamlamoveit.components'
local MoveJActionServer = components.MoveJActionServer
local xutils = require 'xamlamoveit.xutils'
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaMoveJAction')

local sp = ros.AsyncSpinner() -- background job
sp:start()

local nh = ros.NodeHandle()
local system_state_subscriber =
    nh:subscribe(
    '/xamla_sysmon/system_status',
    'xamla_sysmon_msgs/SystemStatus',
    1,
    {'udp', 'tcp'},
    {tcp_nodelay = true}
)
local heartbeat = xamla_sysmon.Heartbeat.new()
heartbeat:start(ros.NodeHandle('~'), 0.5) --[Hz]
heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()
local mj_action_server = MoveJActionServer(nh)
system_state_subscriber:registerCallback(
    function(msg, header)
        return mj_action_server:updateSystemState(msg, header)
    end
)
print(mj_action_server)
mj_action_server:start()
local dt = ros.Rate(125)
dt:reset()

heartbeat:updateStatus(heartbeat.GO, 'Working ...')
heartbeat:publish()
while ros.ok() and mj_action_server.current_state ~= mj_action_server.all_states.FINISHED do
    local status, err = pcall(function() mj_action_server:spin() end )
    if status == false then
        heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. " " .. tostring(err))
    end
    heartbeat:publish()
    ros.spinOnce()
    dt:sleep()
end

mj_action_server:shutdown()
sp:stop()
ros.shutdown()