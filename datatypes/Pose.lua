--[[
Pose.lua

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
local tf = require 'ros'.tf
local Pose = torch.class('xamlamoveit.datatypes.Pose', datatypes)

function Pose:__init()
    self.stampedTransform = tf.StampedTransform()
end

function Pose:setTranslation(trans)
    assert(
        torch.isTypeOf(trans, torch.DoubleTensor),
        string.format(
            'Wrong type of argument "trans". Should be [torch.DoubleTensor] but has type [%s]',
            torch.type(trans)
        )
    )
    self.stampedTransform:setOrigin(trans)
end

function Pose:getTranslation()
    return self.stampedTransform:getOrigin()
end

function Pose:getRotation()
    return self.stampedTransform:getRotation():toTensor()
end

function Pose:setRotation(quaternion)
    if torch.isTypeOf(quaternion, torch.DoubleTensor) then
        local tmp = quaternion:clone()
        quaternion = tf.Quaternion(tmp)
    end
    assert(
        torch.isTypeOf(quaternion, tf.Quaternion),
        string.format(
            'Wrong type of argument "trans". Should be [tf.Quaternion] or [torch.DoubleTensor] but has type [%s]',
            torch.type(quaternion)
        )
    )
    self.stampedTransform:setRotation(quaternion)
end

function Pose:setFrame(name)
    self.stampedTransform:set_frame_id(name)
end

function Pose:getFrame()
    return self.stampedTransform:get_frame_id()
end

function Pose:copy(other)
    self.stampedTransform = other.stampedTransform:clone()
end

function Pose:clone()
    local result = Pose.new()
    result:copy(self)
    return result
end

function Pose:toStampedPoseMsg()
    return self.stampedTransform:toStampedPoseMsg()
end

function Pose:toStampedTransform()
    return self.stampedTransform:clone()
end

function Pose:toTransform()
    return self.stampedTransform:toTransform()
end

function Pose:toTensor()
    return self.stampedTransform:toTensor()
end

function Pose:toTable()
    local res = {}
    res.frame_id = self.stampedTransform:get_frame_id()
    res.value = self.stampedTransform:toTensor()

    return res
end

function Pose.fromTable(t)
    assert(
        torch.type(t.frame_id) == 'string',
        'invalid argument for t.frame_id: %s \n expected arguments: *string*',
        torch.type(t.frame_id)
    )
    assert(
        torch.isTypeOf(t.value, torch.DoubleTensor),
        'invalid argument for t.value: %s \n expected arguments: *torch.DoubleTensor*',
        torch.type(t.value)
    )
    local res = datatypes.Pose()
    res.stampedTransform:fromTensor(t.value)
    res:setFrame(t.frame_id)

    return res
end

function Pose:fromTransform(other)
    assert(
        torch.isTypeOf(other, tf.Transform),
        'invalid argument: %s \n expected arguments: *tf.Transform*',
        torch.type(other)
    )
    return self.stampedTransform:setData(other)
end

function Pose:fromStampedTransform(other)
    assert(
        torch.isTypeOf(other, tf.StampedTransform),
        'invalid argument: %s \n expected arguments: *tf.StampedTransform*',
        torch.type(other)
    )
    return self:copy(other)
end

function Pose:fromTensor(other)
    assert(
        torch.isTypeOf(other, torch.DoubleTensor),
        'invalid argument: %s \n expected arguments: *torch.DoubleTensor*',
        torch.type(other)
    )
    return self.stampedTransform:fromTensor(other)
end

function Pose.__mul(a, b)
    local result = a:clone()
    result.stampedTransform:setData(a.stampedTransform:toTransform():mul(b.stampedTransform:toTransform()))
    return result
end

function Pose:__tostring()
    local res = 'Pose:'
    res = string.format('%s\n %s', res, tostring(self.stampedTransform:toTensor()))
    return res
end

return Pose
