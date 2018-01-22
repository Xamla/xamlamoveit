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
heartbeat:start(ros.NodeHandle('~'), 10) --[Hz]
heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()
local sysmon_watch = xamla_sysmon.Watch.new(nh, 3.0)
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

local idle_dt = ros.Rate(20)
heartbeat:updateStatus(heartbeat.GO, 'Working ...')
heartbeat:publish()
local global_state_summary = sysmon_watch:getGlobalStateSummary()
error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error
local reinitialize = false
local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() and mj_action_server.current_state ~= mj_action_server.all_states.FINISHED do

    local status, err =
        xpcall(
        function()
            mj_action_server:spin()
        end, error_msg_func
    )
    if status == false then
        ros.ERROR(tostring(err))
        heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. ' ' .. tostring(err))
        mj_action_server:reset()
        reinitialize = true
    end
    heartbeat:publish()
    ros.spinOnce()
    if mj_action_server:hasTrajectoryActive() then
        collectgarbage()
        dt:sleep()
    else
        idle_dt:sleep()
    end
end

mj_action_server:shutdown()
sp:stop()
ros.shutdown()
