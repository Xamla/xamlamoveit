#!/usr/bin/env th

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
local default_values = {
  closed_angle = 46.4,
  max_gap = 0.085
}
local positionToJointValue = function(pos)
    return math.rad(default_values.closed_angle - pos * (default_values.closed_angle / default_values.max_gap))
  end
local jointValueToPosition = function(jv)
    return (-math.deg(jv) + default_values.closed_angle) / (default_values.closed_angle / default_values.max_gap)
  end

local action_server = grippers.GenericTwoFingerSimActionServer.new(
  node_handle, joint_command_namespace, actuated_joint_name, jointValueToPosition, positionToJointValue
)

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
