#!/usr/bin/env th
local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local jsa = xamlamoveit.components.JointStateAggregator
local xamla_sysmon = require 'xamla_sysmon'

local cmd = torch.CmdLine()
cmd:option('-frequency', 60, 'Node cycle time in Hz')
cmd:option('-heartbeat', 10, 'Heartbeat cycle time in Hz')
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

local monitor = jsa.new(nh)
monitor.timeout = ros.Duration(1/frequency*4)

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
while ros.ok() do
  local status, err = pcall(spin)
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
