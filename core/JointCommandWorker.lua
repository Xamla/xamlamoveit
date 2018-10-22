--[[
JointCommandWorker.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
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
  ros.INFO(string.format('[JointCommandWorker] Generate trajectory with vel: %f, acc: %f', max_velocities[1], max_accelerations[1]))
  controller.max_vel:copy(max_velocities)
  controller.max_acc:copy(max_accelerations)
  local result = controller:generateOfflineTrajectory(start, goal, dt)
  local positions = torch.zeros(#result, dim)
  local velocities = torch.zeros(#result, dim)
  local accelerations = torch.zeros(#result, dim)
  local time = {}
  for i = 1, #result do
    time[i] = dt * (i-1)
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
function JointCommandWorker:move(joint_names, joint_values, opt)
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
  if opt ~= nil then
    print('[JointCommandWorker] opt: ', opt, max_vel, max_acc)
    for k, _ in ipairs(joint_values) do
      if opt[k] ~= nil and opt[k].max_vel ~= nil and max_vel[k] > opt[k].max_vel then
        max_vel[k] = opt[k].max_vel
      end
      if opt[k] ~= nil and opt[k].max_acc ~= nil and max_acc[k] > opt[k].max_acc then
        max_acc[k] = opt[k].max_acc
      end
    end
  end


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
