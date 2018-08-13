
--[[
GenericTwoFingerSimActionServer.lua

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
require 'ros.actionlib.SimpleActionClient'
local core = require 'xamlamoveit.core'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local grippers = require 'xamlamoveit.grippers.env'
local GenericTwoFingerSimActionServer = torch.class('xamlamoveit.grippers.GenericTwoFingerSimActionServer', grippers)

GenericTwoFingerSimActionServer.ERROR_TYPE = {
  INITIALIZATION_FAILED = 'INITIALIZATION_FAILED',
  JOINT_MONITOR_NOT_READY = 'JOINT_MONITOR_NOT_READY'
}

GenericTwoFingerSimActionServer.WORKER_RESPONSE = {
  COMPLETED = 0,
  ABORTED = 1,
  CANCELED = 2
}

local function initializeActionServer(self)
  -- Intialize action server
  self.action_server = actionlib.ActionServer(self.node_handle, 'gripper_command', 'control_msgs/GripperCommand')
  self.action_server:registerGoalCallback(
    function(goal_handle) self:handleGoalCallback(goal_handle) end
  )
  self.action_server:registerCancelCallback(
    function(goal_handle) self:handleCancleCallback(goal_handle) end
  )
  self.action_server:start()

  -- Intialize joint monitor for Jointcommand Worker
  self.joint_monitor = core.JointMonitor({self.actuated_joint_name})
  local ready = false
  local last_print = ros.Time(0)
  local counter = 0
  while not ready and ros.ok() do
      ready = self.joint_monitor:waitReady(20.0)
      if ros.Time.now():toSec() - last_print:toSec() > 1 then
          ros.INFO('JointMonitor waits for first update of joint: %s.', self.actuated_joint_name)
          last_print = ros.Time.now()
          counter = counter + 1
      end
      if counter > 30 then
          ros.ERROR('Joint monitor did not get ready in the desired time. Failed to receive update for joint: %s', gripper_joint_name)
          error(GenericTwoFingerSimActionServer.ERROR_TYPE.JOINT_MONITOR_NOT_READY)
      end
  end

  self.joint_command_node_handle = ros.NodeHandle(self.joint_command_namespace)
  self.joint_command_worker = core.JointCommandWorker.new(self.joint_command_node_handle, self.joint_monitor)
end


function GenericTwoFingerSimActionServer:__init(node_handle, joint_command_namespace, actuated_joint_name,
  jointValueToPosition, postionToJointValue
)
  self.joint_command_namespace = joint_command_namespace
  self.actuated_joint_name = actuated_joint_name
  self.node_handle = node_handle
  self.gripper_sim = gripper_sim
  self.jointValueToPosition = jointValueToPosition or function (jv) return jv end
  self.postionToJointValue = postionToJointValue or function (p) return p end
  self.current_state = {
    moving_gripper = false
  }
  self.default_values = {
    grasping_force = 50
  }

  local ok, err = pcall(function() initializeActionServer(self) end)
  if not ok then
    ros.ERROR(string.format('Initialization of GenericTwoFingerSimActionServer has failed: %s', err))
    error(GenericTwoFingerSimActionServer.ERROR_TYPE.INITIALIZATION_FAILED)
  end
end


function GenericTwoFingerSimActionServer:dispatchJointCommand(joint_value)
  local callbacks = {
    accept = function() return true end,
    proceed = function() return self.current_state.proceed end,
    cancel = function() self:handleJointCommandFinished(GenericTwoFingerSimActionServer.WORKER_RESPONSE.CANCELED) end,
    abort = function() self:handleJointCommandFinished(GenericTwoFingerSimActionServer.WORKER_RESPONSE.ABORTED) end,
    completed = function(traj)
      self:handleJointCommandFinished(GenericTwoFingerSimActionServer.WORKER_RESPONSE.COMPLETED, traj)
    end
  }
  self.joint_command_worker:setCallbacks(callbacks)
  self.joint_command_worker:move({self.actuated_joint_name}, {joint_value})
end


function GenericTwoFingerSimActionServer:handleCancleCallback(goal_handle)
  if self.current_state.moving_gripper == true then
    self.current_state.proceed = false
  end
end


function GenericTwoFingerSimActionServer:handleGoalCallback(goal_handle)
  if goal_handle ~= nil and goal_handle.goal ~= nil then
    if self.current_state.moving_gripper == true then
      ros.WARN('Gripper is already executing a command.')
      if (goal_handle.setRejected ~= nil) then
        goal_handle:setRejected('Gripper is already executing a command.')
      end
      return
    end

    print('New goal', goal_handle.goal.goal.command)
    self:handleMoveCommand(goal_handle)
  else
    ros.WARN('Received invalid goal.')
  end
end


function GenericTwoFingerSimActionServer:handleJointCommandFinished(worker_response, trajectory)
  print('joint command finished. response: ', worker_response)
  self.current_state.moving_gripper = false

  if self.current_state.goal_handle == nil then
    ros.ERROR('Joint worker finished, but goal handle is nil. Cannot complete goal. Try again.')
    return
  end

  local result = self.current_state.goal_handle:createResult()
  local current_positions = self.joint_monitor:getPositionsOrderedTensor(joint_names)
  result.position = self.jointValueToPosition(current_positions[1])
  result.effort = self.current_state.target_force

  if worker_response == GenericTwoFingerSimActionServer.WORKER_RESPONSE.COMPLETED then
    result.reached_goal = true
    result.stalled = false
    print('success', result)
    self.current_state.goal_handle:setSucceeded(result)
  else
    result.reached_goal = false
    if worker_response == GenericTwoFingerSimActionServer.WORKER_RESPONSE.CANCELED then
      result.stalled = false
      self.current_state.goal_handle:setCanceled(result)
    else
      result.stalled = true
      self.current_state.goal_handle:setAborted(result)
    end
  end

  self.current_state.goal_handle = nil
  self.current_state.target_force = nil
end


function GenericTwoFingerSimActionServer:handleMoveCommand(goal_handle)
  local goal_command = goal_handle.goal.goal.command
  self.current_state.target_force = goal_command.max_effort
  self.current_state.time_of_joint_command = ros.Time.now()
  self.current_state.goal_handle = goal_handle
  self.current_state.proceed = true
  self.current_state.moving_gripper = true

  local target_angle = self.postionToJointValue(goal_command.position)
  self:dispatchJointCommand(target_angle)
  goal_handle:setAccepted()
end


function GenericTwoFingerSimActionServer:shutdown()
  if self.action_server ~= nil then
    self.action_server:shutdown()
  end

  if self.joint_command_node_handle ~= nil then
    self.joint_command_node_handle:shutdown()
  end
end


function GenericTwoFingerSimActionServer:spin()
  self.joint_command_worker:spin()
end


return GenericTwoFingerSimActionServer
