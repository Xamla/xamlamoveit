#!/usr/bin/env th
--[[
weissTwoFingerSimulationNode.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
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
