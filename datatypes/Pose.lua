--[[
Pose.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local datatypes = require 'xamlamoveit.datatypes.env'
local ros = require 'ros'
local tf = ros.tf
local sf = string.format
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

function Pose:setName(name)
    self.stampedTransform:set_child_frame_id(name)
end

function Pose:getName()
    return self.stampedTransform:get_child_frame_id() or ''
end

function Pose:setFrame(name)
    self.stampedTransform:set_frame_id(name)
end

function Pose:getFrame()
    return self.stampedTransform:get_frame_id() or ''
end

function Pose:copy(other)
    self.stampedTransform = other.stampedTransform:clone()
end

function Pose:clone()
    local result = Pose.new()
    result.stampedTransform = self.stampedTransform:clone()
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
    self.stampedTransform = other
end

function Pose:fromTensor(other)
    assert(
        torch.isTypeOf(other, torch.DoubleTensor),
        'invalid argument: %s \n expected arguments: *torch.DoubleTensor*',
        torch.type(other)
    )
    return self.stampedTransform:fromTensor(other)
end

function Pose.inverse(pose)
    local result = pose:clone()
    result:fromTensor(torch.inverse(result:toTensor()))
    local name = result:getName()
    local frame = result:getFrame()
    result:setName(frame)
    result:setFrame(name)
    return result
end

function Pose.__mul(a, b)
    assert(torch.isTypeOf(a, datatypes.Pose), sf('invalid arguments: %s \n expected arguments: *Pose* [Pose]', torch.type(a)))
    assert(torch.isTypeOf(b, datatypes.Pose), sf('invalid arguments: %s \n expected arguments: *Pose* [Pose]', torch.type(b)))
    local result = a:clone()
    result:fromTensor(a:toTensor() * b:toTensor())
    if a:getName() == b:getFrame() then
        result:setName(b:getName())
        result:setFrame(a:getFrame())
    else
        result:setName(a:getName() .. b:getName())
        result:setFrame(a:getFrame())
    end
    return result
end

function Pose.computeRelative(R01, R02)
    assert(torch.isTypeOf(R01, datatypes.Pose), 'Invalid argument `R01`: datatypes.Pose object expected.')
    assert(torch.isTypeOf(R02, datatypes.Pose), 'Invalid argument `R02`: datatypes.Pose object expected.')
    local R12 = Pose.inverse(R01) * R02
    R12:setName(R02:getName())
    R12:setFrame(R01:getName())
    -- R02 = R01 * R12
    -- inverse(R01) * R02 = R12
    return R12
  end

function Pose:__tostring()
    local res = 'Pose:'
    res = string.format('%s\n %s', res, tostring(self.stampedTransform:toTensor()))
    return res
end

return datatypes.Pose
