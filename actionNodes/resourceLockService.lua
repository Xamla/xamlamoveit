#!/usr/bin/env th
--[[
resourceLockService.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
local ros = require 'ros'

local xamlamoveit = require 'xamlamoveit'
local lbls = xamlamoveit.components.LeaseBasedLockService
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
local parameter = xamlamoveit.xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaResourceLockServices')

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

local lbls_service = lbls(nh)
local services = { lbls_service}

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
local dt = ros.Rate(100)
local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() and not emerg_stop_flag do
    for i, v in pairs(services) do
        local status,
            err =
            xpcall(
            function()
                v:spin()
            end, error_msg_func
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
