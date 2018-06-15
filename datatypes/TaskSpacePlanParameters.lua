--[[
TaskSpacePlanParameters.lua

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
local torch = require 'torch'

local datatypes = require 'xamlamoveit.datatypes.env'
local TaskSpacePlanParameters = torch.class('TaskSpacePlanParameters', datatypes)

function TaskSpacePlanParameters:__init(
    end_effector_name,
    max_deviation,
    max_xyz_velocity,
    max_xyz_acceleration,
    max_angular_velocity,
    max_angular_acceleration,
    collision_check,
    ik_jump_threshold,
    dt)
    self.end_effector_name = end_effector_name
    self.max_xyz_velocity = max_xyz_velocity
    self.max_xyz_acceleration = max_xyz_acceleration
    self.max_angular_velocity = max_angular_velocity
    self.max_angular_acceleration = max_angular_acceleration
    self.collision_check = collision_check == nil or collision_check
    self.max_deviation = max_deviation or 0.0
    self.ik_jump_threshold = ik_jump_threshold or 0.0
    self.dt = dt or 0.008
end

function TaskSpacePlanParameters:fromTable(t)
    assert(type(t) == 'table', 'Source table argument must not be nil.')
    for k, v in pairs(t) do
        self[k] = v
    end
end

function TaskSpacePlanParameters:toTable()
    return {
        end_effector_name = self.end_effector_name,
        collision_check = self.collision_check,
        max_deviation = self.max_deviation,
        max_velocity = self.max_velocity,
        max_acceleration = self.max_acceleration,
        ik_jump_threshold = self.ik_jump_threshold,
        max_xyz_velocity = self.max_xyz_velocity,
        max_xyz_acceleration = self.max_xyz_acceleration,
        max_angular_velocity = self.max_angular_velocity,
        max_angular_acceleration = self.max_angular_acceleration,
        dt = self.dt
    }
end

function TaskSpacePlanParameters:clone()
    local result = TaskSpacePlanParameters.new()
    result:fromTable(self:toTable())
    return result
end

function TaskSpacePlanParameters:__tostring()
    local res = 'TaskSpacePlanParameters:\n'
    for k, v in pairs(self:toTable()) do
        if type(v) == 'table'then
            local str_table = ''
            for ii,vv in ipairs(v) do
                str_table = string.format('%s %s', str_table, tostring(vv))
            end
            res = string.format('%s\t %s:\t %s\n', res, k, str_table)
        elseif torch.isTypeOf(v, torch.DoubleTensor) then
            local str = ''
            for ii = 1, v:size(1) do
                str = string.format('%s %s', str, tostring(v[ii]))
            end
            res = string.format('%s\t %s:\t %s\n', res, k, str)
        else
            res = string.format('%s\t %s:\t %s\n', res, k, tostring(v))
        end
    end
    return res
end

return TaskSpacePlanParameters
