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
