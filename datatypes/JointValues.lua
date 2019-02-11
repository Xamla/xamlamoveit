--[[
JointValues.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local datatypes = require 'xamlamoveit.datatypes.env'
local js = require 'xamlamoveit.datatypes.JointSet'

local sf = string.format

local JointValues = torch.class('xamlamoveit.datatypes.JointValues', datatypes)

function JointValues:__init(joint_set, values)
    assert(
        torch.isTypeOf(joint_set, datatypes.JointSet),
        'Should be xamlamoveit.datatypes.JointSet but is:' .. torch.type(joint_set)
    )
    self.joint_set = joint_set
    if torch.isTypeOf(values, torch.DoubleTensor) then
        self.values = values
    elseif torch.type(values) == 'number' then
        self.values = torch.Tensor(self.joint_set:count()):fill(values)
    end
    assert(self.values:size(1) == self.joint_set:count())
end

function JointValues:getValue(name)
    return self.values[self.joint_set:getIndexOf(name)]
end

function JointValues:getValues()
    return self.values:clone()
end

function JointValues:setValue(name, value)
    local suc, j = self.joint_set:tryGetIndexOf(name)
    if suc then
        self.values[j] = value
    else
        print('could not set value with name:', name)
    end
end

function JointValues:setValues(names, values)
    assert(#names == values:size(1), sf('Count of name and size of value should match: %dx%d', #names, values:size(1)))
    for i, v in ipairs(names) do
        self:setValue(v, values[i])
    end
end

function JointValues:getNames()
    return self.joint_set.joint_names
end

function JointValues:add(other)
    if other.joint_set:count() < self.joint_set:count() then
        for i, v in ipairs(other:getNames()) do
            self:setValue(v, self:getValue(v) + other.values[i])
        end
    else
        for i, v in ipairs(self:getNames()) do
            self.values[i] = self.values[i] + other:getValue(v)
        end
    end
end

function JointValues.__add(a, b)
    local result = a:clone()
    result:add(b)
    return result
end

function JointValues:sub(other)
    assert(other.joint_set:isSimilar(self.joint_set), 'JointSets are not similar.')
    for i, v in ipairs(self:getNames()) do
        self.values[i] = self.values[i] - other:getValue(v)
    end
end

function JointValues.__sub(a, b)
    local result = a:clone()
    result:sub(b)
    return result
end

function JointValues.__mul(a, b)
    local result
    if torch.type(a) == 'xamlamoveit.datatypes.JointValues' then
        result = a:clone()
        if torch.type(b) == 'number' then
            result.values:mul(b)
        else
            error(
                sf(
                    'invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double',
                    torch.type(a),
                    torch.type(b)
                )
            )
        end
    elseif torch.type(b) == 'xamlamoveit.datatypes.JointValues' then
        result = b:clone()
        if torch.type(a) == 'number' then
            result.values:mul(a)
        else
            error(
                sf(
                    'invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double',
                    torch.type(a),
                    torch.type(b)
                )
            )
        end
    else
        error(
            sf(
                'invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double',
                torch.type(a),
                torch.type(b)
            )
        )
    end
    return result
end

function JointValues:select(names)
    local tmp_jointset = js.new(names)
    local values = torch.zeros(#names)
    for i, v in ipairs(names) do
        values[i] = self:getValue(v)
    end
    return JointValues.new(tmp_jointset, values)
end

function JointValues.createRandomJointValues(joint_set, min, max)
    if torch.type(min) == 'number' then
        min = torch.Tensor(joint_set:count()):fill(min)
    end
    if torch.type(max) == 'number' then
        max = torch.Tensor(joint_set:count()):fill(max)
    end
    assert(
        torch.isTypeOf(min, torch.DoubleTensor) and torch.isTypeOf(max, torch.DoubleTensor),
        sf(
            'invalid arguments: %s %s \n expected arguments: *min* [JointValues] double, *max* [JointValues] double',
            torch.type(min),
            torch.type(max)
        )
    )
    assert(min:size(1) == max:size(1))
    assert(min:size(1) == #joint_set)

    local r = torch.rand(#joint_set)
    local ranges = max - min
    local joint_positions = torch.cmul(r, ranges) + min
    return datatypes.JointValues(joint_set, joint_positions)
end

function JointValues:norm()
    return self.values:norm()
end

function JointValues:abs()
    self.values:abs()
    return self
end

function JointValues.abs(other)
    assert(
        torch.isTypeOf(other, datatypes.JointValues),
        'invalid argument: %s \n expected arguments: *torch.DoubleTensor*',
        torch.type(other)
    )
    local result = other:clone()
    return result:abs()
end

function JointValues:distance(other, method)
    assert(
        torch.isTypeOf(other, datatypes.JointValues),
        'invalid argument: %s \n expected arguments: *torch.DoubleTensor*',
        torch.type(other)
    )
    local method = method or 'l2'
    local diff = (self - other)
    if method == 'l2' then
        return diff:norm()
    elseif method == 'abs' then
        return diff:abs()
    end
end

function JointValues:clone()
    return JointValues.new(self.joint_set:clone(), self.values:clone())
end

function JointValues:toTable()
    local res = {}
    res.names = self.joint_set:getNames()
    res.values = self.values

    return res
end

function JointValues.fromTable(t)
    local joint_set = datatypes.JointSet.new(t.names)
    local res = datatypes.JointValues(joint_set, t.values)

    return res
end

function JointValues:__tostring()
    local res = 'JointValues:'
    for i, v in ipairs(self.joint_set:getNames()) do
        res = sf('%s\n %s: %04f', res, v, self.values[i])
    end
    return res
end

function JointValues.__len(a)
    assert(torch.isTypeOf(a, datatypes.JointValues), string.format('Wrong type! Expected: [xamlamoveit.datatypes.JointValues] but has [%s]', torch.type(a)))
    return #a.joint_set
end

return datatypes.JointValues
