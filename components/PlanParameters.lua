local torch = require 'torch'

local components = require 'xamlamoveit.components.env'
local PlanParameters = torch.class('PlanParameters', components)

function PlanParameters:__init(
    move_group_name,
    joint_names,
    collisionCheck,
    maxVelocity,
    maxAcceleration,
    sampleResolution)
    self.move_group_name = move_group_name
    self.joint_names = joint_names
    self.collisionCheck = collisionCheck or true
    self.maxVelocity = maxVelocity
    self.maxAcceleration = maxAcceleration
    self.sampleResolution = sampleResolution or 1.0
end

function PlanParameters:setFromTable(t)
    assert(t ~= nil, 'Source table argument must not be nil.')
    for k, v in pairs(t) do
        local f = self[k] -- check if field or index exists
        if f ~= nil then
            self[k] = v
        end
    end
end

function PlanParameters:toTable()
    return {
        move_group_name = self.move_group_name,
        joint_names = self.joint_names,
        collisionCheck = self.collisionCheck,
        maxVelocity = self.maxVelocity,
        maxAcceleration = self.maxAcceleration,
        sampleResolution = self.sampleResolution
    }
end

return PlanParameters
