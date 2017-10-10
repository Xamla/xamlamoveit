local datatypes = require 'xamlamoveit.components.datatypes.env'
require 'xamlamoveit.components.datatypes.JointSet'

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
    return self.joint_set:getValue(name)
end

function JointValues:getNames()
    return self.joint_set.joint_names
end

return JointValues
