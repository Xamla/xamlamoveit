#!/usr/bin/env th
--[[
weissTwoFingerSimulationNode.lua

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
local grippers = require 'xamlamoveit.grippers.env'
local xutils = xamlamoveit.xutils

local node_handle
local function initSetup(ns, param)
    ros.init(ns, nil, param)
    node_handle = ros.NodeHandle('~')
end

local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
initSetup(parameter['__name'], parameter)

local joint_command_namespace = node_handle:getParamString('joint_command_namespace')
local actuated_joint_name = node_handle:getParamString('actuated_joint_name')
local action_server = grippers.WeissTwoFingerSimulation.new(node_handle, joint_command_namespace, actuated_joint_name)

print('Spinning')
local rate = ros.Rate(10)
while ros.ok() do
  ros.spinOnce()
  action_server:spin()
  rate:sleep()
end

print('Shutdown')
action_server:shutdown()
node_handle:shutdown()
ros.shutdown()
