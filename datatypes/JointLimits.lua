--[[
JointLimits.lua

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
local datatypes = require 'xamlamoveit.datatypes.env'
local js = require 'xamlamoveit.datatypes.JointSet'

local JointLimits = torch.class('xamlamoveit.datatypes.JointLimits', datatypes)

local function setValues(self, var_name, values)
    if torch.isTypeOf(values, torch.DoubleTensor) then
        assert(values:size(1) == self.joint_set:count(), string.format("%s Tensor has wrong size. %d vs. %d", var_name, values:size(1), self.joint_set:count()) )
        self[var_name] = values
    elseif torch.type(values) == 'number' then
        self[var_name] = torch.Tensor(self.joint_set:count()):fill(values)
    end
end

function JointLimits:__init(joint_set, min_pos, max_pos, max_vel, max_acc)
    assert(
        torch.isTypeOf(joint_set, datatypes.JointSet),
        'Should be xamlamoveit.datatypes.JointSet but is:' .. torch.type(joint_set)
    )
    self.joint_set = joint_set
    setValues(self, "min_pos", min_pos)
    setValues(self, "max_pos", max_pos)
    setValues(self, "max_vel", max_vel)
    setValues(self, "max_acc", max_acc)
end

function JointLimits:getMaxPosition(name)
    return self.max_pos[self.joint_set:getIndexOf(name)]
end

function JointLimits:getMaxPositions()
    return self.max_pos:clone()
end

function JointLimits:getMinPosition(name)
    return self.min_pos[self.joint_set:getIndexOf(name)]
end

function JointLimits:getMinPositions()
    return self.min_pos:clone()
end

function JointLimits:getMaxVelocity(name)
    return self.max_vel[self.joint_set:getIndexOf(name)]
end

function JointLimits:getMaxVelocities()
    return self.max_vel:clone()
end

function JointLimits:getMaxAcceleration(name)
    return self.max_acc[self.joint_set:getIndexOf(name)]
end

function JointLimits:getMaxAccelerations()
    return self.max_acc:clone()
end

function JointLimits:satisfiesBounds(joint_values)
    assert(
        torch.isTypeOf(joint_values, datatypes.JointValues),
        'Should be xamlamoveit.datatypes.JointValues but is:' .. torch.type(joint_values)
    )
    local tmp_limits = self:select(joint_values:getNames())
    for i = 1, joint_values.joint_set:count() do
        if joint_values.values[i] > tmp_limits.max_pos[i] or tmp_limits.min_pos[i] > joint_values.values[i] then
            return false
        end
    end
    return true
end

function JointLimits:clamp(joint_values)
    assert(
        torch.isTypeOf(joint_values, datatypes.JointValues),
        'Should be xamlamoveit.datatypes.JointValues but is:' .. torch.type(joint_values)
    )
    local tmp_limits = self:select(joint_values:getNames())
    for i = 1, joint_values.joint_set:count() do
        joint_values.values[i] = math.min(joint_values.values[i], tmp_limits.max_pos[i])
        joint_values.values[i] = math.max(joint_values.values[i], tmp_limits.min_pos[i])
    end
    return joint_values
end

function JointLimits:select(names)
    local tmp_jointset = js.new(names)
    local max_pos = torch.zeros(#names)
    local min_pos = torch.zeros(#names)
    local max_vel = torch.zeros(#names)
    local max_acc = torch.zeros(#names)
    for i, v in ipairs(names) do
        max_pos[i] = self:getMaxPosition(v)
        min_pos[i] = self:getMinPosition(v)
        max_vel[i] = self:getMaxVelocity(v)
        max_acc[i] = self:getMaxAcceleration(v)
    end
    return JointLimits.new(tmp_jointset, max_pos, min_pos, max_vel, max_acc)
end

function JointLimits:clone()
    return JointLimits.new(self.joint_set:clone(), self.max_pos:clone(),self.min_pos:clone(),self.max_vel:clone(),self.max_acc:clone())
end

function JointLimits:__tostring()
    local res = 'JointLimits:'
    for i, v in ipairs(self.joint_set:getNames()) do
        res = string.format('%s\n %s: %04f, %04f, %04f, %04f', res, v, self.max_pos[i], self.min_pos[i], self.max_vel[i], self.max_acc[i])
    end
    return res
end

return JointLimits
