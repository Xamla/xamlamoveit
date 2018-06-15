#!/usr/bin/env th
--[[
jointStateLatencyHeartbeatNode.lua

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
local xutils = xamlamoveit.xutils
local jsa = xamlamoveit.components.JointStateAggregator
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
cmd:option('-frequency', 60, 'Node cycle time in Hz')
cmd:option('-heartbeat', 10, 'Heartbeat cycle time in Hz')
cmd:option('-monitorTimeout', 100, 'Timeout factor. 1/frequency * factor')
local parameter = xamlamoveit.xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
ros.init(parameter['__name'] or 'xamlaJointMonitor')

local nh = ros.NodeHandle('~')
local sp = ros.AsyncSpinner() -- background job
sp:start()

local frequency, succ = nh:getParamDouble('frequency')
if not succ then
  frequency = parameter.frequency
end

local heartbeat_frequence
heartbeat_frequence, succ = nh:getParamDouble('heartbeat')
if not succ then
  heartbeat_frequence = parameter.heartbeat
end

local timeout_factor
timeout_factor, succ = nh:getParamDouble('monitorTimeout')
if not succ then
  timeout_factor = parameter.monitorTimeout
end

local monitor = jsa.new(nh)
monitor.timeout = ros.Duration(1/frequency*timeout_factor)

local system_state_subscriber =
    nh:subscribe(
    '/xamla_sysmon/system_status',
    'xamla_sysmon_msgs/SystemStatus',
    1,
    {'udp', 'tcp'},
    {tcp_nodelay = true}
)
local heartbeat = xamla_sysmon.Heartbeat.new()
heartbeat:start(nh, heartbeat_frequence) --[Hz]
heartbeat:updateStatus(heartbeat.STARTING, 'Init ...')
heartbeat:publish()

system_state_subscriber:registerCallback(
  function(msg, header)
      return monitor:updateSystemState(msg, header)
  end
)

monitor:start()
local function spin()
  monitor:spin()
end

local dt = ros.Rate(frequency)
heartbeat:updateStatus(heartbeat.GO, 'Running ...')
local error_msg_func = function(x) ros.ERROR(debug.traceback()) return x end
while ros.ok() do
  local status, err = xpcall(spin, error_msg_func)
  if status == false then
      heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, torch.type(monitor) .. ' ' .. tostring(err))
  end

  heartbeat:publish()
  ros.spinOnce()
  dt:sleep()
end
monitor:shutdown()
sp:stop()
ros.shutdown()
