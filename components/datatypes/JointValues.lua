local datatypes = require 'xamlamoveit.components.datatypes.env'
local js = require 'xamlamoveit.components.datatypes.JointSet'

local JointValues = torch.class('xamlamoveit.components.datatypes.JointValues', datatypes)

function JointValues:__init(joint_set, values)
    assert(
        torch.type(joint_set) == 'xamlamoveit.components.datatypes.JointSet',
        'Should be xamlamoveit.components.datatypes.JointSet but is:' .. torch.type(joint_set)
    )
    self.joint_set = joint_set
    if torch.type(values) == 'torch.DoubleTensor' then
        self.values = values
    elseif torch.typ(values) == 'number' then
        self.values = torch.Tensor(self.joint_set:count()):fill(number)
    end
    assert(values:size(1) == self.joint_set:count())
end

function JointValues:getValue(name)
    return self.values[self.joint_set:getIndexOf(name)]
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
    assert(
        #names == values:size(1),
        string.format('Count of name and size of value should match: %dx%d', #names, values:size(1))
    )
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
    return result:add(b)
end

function JointValues:sub(other)
    assert(other.joint_set:count() == self.joint_set:count())
    for i, v in ipairs(self:getNames()) do
        self.values[i] = self.values[i] - other:getValue(v)
    end
end

function JointValues.__sub(a, b)
    local result = a:clone()
    return result:sub(b)
end

function JointValues.__mul(a, b)
    local result
    if torch.type(a) == 'xamlamoveit.components.datatypes.JointValues' then
        result = a:clone()
        if torch.type(b) == 'number' then
            result.values:mul(b)
        else
            error(string.format('invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double', torch.type(a), torch.type(b)))
        end
    elseif torch.type(b) == 'xamlamoveit.components.datatypes.JointValues' then
        result = b:clone()
        if torch.type(a) == 'number' then
            result.values:mul(a)
        else
            error(string.format('invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double', torch.type(a), torch.type(b)))
        end
    else
        error(string.format('invalid arguments: %s %s \n expected arguments: *JointValues* [JointValues] double', torch.type(a), torch.type(b)))
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

function JointValues:clone()
    return JointValues.new(self.joint_set:clone(), self.values:clone())
end

return JointValues
