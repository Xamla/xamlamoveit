--[[
TaskSpaceController.lua

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
local tf = require 'ros'.tf
local controller = require 'xamlamoveit.controller.env'

local TaskSpaceController,
    TvpController =
    torch.class(
    'xamlamoveit.controller.TaskSpaceController',
    'xamlamoveit.controller.MultiAxisTvpController',
    controller
)

local function tensor6DToPose(vector6D)
    assert(vector6D:size(1) == 6, 'Vector should be of size 6D (offset, anglevelocities)')
    local end_pose = tf.Transform()
    end_pose:setOrigin(vector6D[{{1, 3}}])
    local end_pose_rotation = end_pose:getRotation()
    end_pose:setRotation(end_pose_rotation:setRPY(vector6D[{{4, 6}}]))
    return end_pose
end

function TaskSpaceController:__init()
    local dim = 6
    TvpController.__init(self, dim)
    self.dim = dim --xyz rpy
    self.max_vel[{{1, 3}}]:fill(0.1)
    self.max_vel[{{4, 6}}]:fill(math.pi)
    self.max_acc[{{1, 3}}]:fill(0.4)
    self.max_acc[{{4, 6}}]:fill(math.pi / 2)
    self.norm_max_vel[{{1, 3}}]:fill(0.1)
    self.norm_max_vel[{{4, 6}}]:fill(math.pi)
    self.norm_max_acc[{{1, 3}}]:fill(0.4)
    self.norm_max_acc[{{4, 6}}]:fill(math.pi / 2)
end

local function transformInput(input, ref)
    local new_input
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end

    if torch.isTypeOf(input, tf.Transform) then
        new_input = torch.zeros(6)
        new_input[{{1, 3}}] = input:getOrigin()
        local A = input:getRotation():getRPY(1)
        local B = input:getRotation():getRPY(2)
        if (A-ref):norm() < (B-ref):norm() then
            new_input[{{4, 6}}]:copy(A)
        else
            new_input[{{4, 6}}]:copy(B)
        end
    end
    if torch.isTypeOf(input, torch.DoubleTensor) then
        new_input = input
    end
    assert(
        torch.isTypeOf(new_input, torch.DoubleTensor),
        string.format('Input should be of type [torch.DoubleTensor] but is of type: [%s]', torch.type(new_input))
    )
    return new_input
end

function TaskSpaceController:update(goal, dt)
    goal = transformInput(goal)
    return TvpController.update(self, goal, dt)
end

function TaskSpaceController:getCurrentPose()
    return tensor6DToPose(self.state.pos)
end

function TaskSpaceController:setCurrentPose(pose)
    self.state.pos = transformInput(pose, self.state.pos[{{4,6}}])
end

function TaskSpaceController:generateOfflineTrajectory(start, goal, dt, start_vel)
    start = transformInput(start, torch.zeros(3))
    goal = transformInput(goal, start[{{4,6}}] )
    if start_vel then
        start_vel = transformInput(start_vel, torch.zeros(3))
    end
    return TvpController.generateOfflineTrajectory(self, start, goal, dt, start_vel)
end

return TaskSpaceController
