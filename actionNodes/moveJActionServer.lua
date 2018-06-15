#!/usr/bin/env th
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
local mj_action_server = {[1] = MoveJActionServer(root_nh, joint_monitor), [2] = MoveJSafeSteppingActionServer(local_nh, joint_monitor)}
for i, v in ipairs(mj_action_server) do
    system_state_subscriber:registerCallback(
        function(msg, header)
            return v:updateSystemState(msg, header)
        end
    )
    v:start()
end
local dt = ros.Rate(125)
dt:reset()

local idle_dt = ros.Rate(20)
heartbeat:updateStatus(heartbeat.GO, 'Working ...')
heartbeat:publish()
local global_state_summary = sysmon_watch:getGlobalStateSummary()
error_state = global_state_summary.no_go and not global_state_summary.only_secondary_error

local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() do
    local hasTrajectoryActive = false
    for i, v in ipairs(mj_action_server) do
        local status,
            err =
            xpcall(
            function()
                v:spin()
            end,
            error_msg_func
        )
        if status == false then
            ros.ERROR(tostring(err))
            heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(v) .. ' ' .. tostring(err))
            v:reset()
        end
        if not hasTrajectoryActive then
            hasTrajectoryActive = v:hasTrajectoryActive()
        end
    end
    heartbeat:publish()
    joint_monitor:waitForNextState()
    ros.spinOnce()

    if hasTrajectoryActive then
        dt:sleep()
    else
        idle_dt:sleep()
    end
end

for i, v in pairs(mj_action_server) do
    v:shutdown()
end
sp:stop()
ros.shutdown()