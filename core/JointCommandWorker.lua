--[[
JointCommandWorker.lua

Copyright (C) 2018  Xamla info@xamla.com

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
local core = require 'xamlamoveit.core.env'
local JointCommandWorker = torch.class('xamlamoveit.core.JointCommandWorker', core)
local motionLibrary = require 'xamlamoveit.motionLibrary'
local GenerativeSimulationWorker = require 'xamlamoveit.core.GenerativeSimulationWorker'

JointCommandWorker.ERROR_TYPE = {
  NO_CALLBACKS_SET = "NO_CALLBACKS_SET"
}


local function generateSimpleTvpTrajectory(start, goal, max_velocities, max_accelerations, dt)
  local dim = goal:size(1)
  local controller = require 'xamlamoveit.controller'.MultiAxisTvpController(dim)
  controller.max_vel:copy(max_velocities)
  controller.max_acc:copy(max_accelerations)
  local result = controller:generateOfflineTrajectory(start, goal, dt)
  local positions = torch.zeros(#result, dim)
  local velocities = torch.zeros(#result, dim)
  local accelerations = torch.zeros(#result, dim)
  local time = {}
  for i = 1, #result do
    time[i] = dt * i
    positions[{i, {}}]:copy(result[i].pos)
    velocities[{i, {}}]:copy(result[i].vel)
    accelerations[{i, {}}]:copy(result[i].acc)
  end
  time = torch.Tensor(time)
  return time, positions, velocities, accelerations
end


function JointCommandWorker:__init(node_handle, joint_monitor)
  self.joint_monitor = joint_monitor
  self.motion_service = motionLibrary.MotionService(node_handle)
  self.trajectory_worker = GenerativeSimulationWorker.new(node_handle)
end


---
function JointCommandWorker:move(joint_names, joint_values)
  if (self.trajectory_callbacks == nil) then
    ros.ERROR('No callback handlers for the trajectory handler were set. You have to call setCallbacks first.')
    error(JointCommandWorker.ERROR_TYPE.NO_CALLBACKS_SET)
  end

  local current_positions = self.joint_monitor:getPositionsOrderedTensor(joint_names)
  local target_positions = current_positions:clone()
  for k, v in ipairs(joint_values) do
    target_positions[k] = v
  end
  local max_min_pos, max_vel, max_acc = self.motion_service:queryJointLimits(joint_names)

  local time, pos, vel, acc = generateSimpleTvpTrajectory(
    current_positions, target_positions, max_vel, max_acc, 0.016
  )

  local trajectory = {
    time = time,
    pos = pos,
    vel = vel,
    acc = acc,
    joint_names = joint_names,
    joint_monitor = self.joint_monitor,
    state_joint_names = self.joint_monitor:getJointNames(),
    accept = self.trajectory_callbacks.accept,
    proceed = self.trajectory_callbacks.proceed,
    cancel = self.trajectory_callbacks.cancel,
    abort = self.trajectory_callbacks.abort,
    completed = self.trajectory_callbacks.completed
  }

  if trajectory.pos:nElement() == 0 then -- empty trajectory
    -- nothing to do
  else
    self.trajectory_worker:doTrajectoryAsync(trajectory) -- queue for processing
    ros.INFO('Gripper trajectory queued for execution')
  end
end


--- Set the callbacks for the trajectory handler
-- @table Five callback functions are expected by the trajectory handler and need to be set in this table
--  accept, arguments: none
--  proceed, arguments: self
--  cancel, arguments: self
--  abort, arguments: self, msg
--  completed, arguments: self
function JointCommandWorker:setCallbacks(trajectory_callbacks)
  self.trajectory_callbacks = trajectory_callbacks
end


function JointCommandWorker:shutdown()
  self.trajectory_worker:shutdown()
end


function JointCommandWorker:spin()
  self.trajectory_worker:spin()
end

return JointCommandWorker
