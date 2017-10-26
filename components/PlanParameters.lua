local torch = require 'torch'

local components = require 'xamlamoveit.components.env'
local PlanParameters = torch.class('PlanParameters', components)

function PlanParameters:__init(
    move_group_name,
    joint_names,
    max_positions,
    min_positions,
    max_velocity,
    max_acceleration,
    collision_check,
    max_deviation,
    dt)
    self.move_group_name = move_group_name
    self.joint_names = joint_names
    self.max_positions = max_positions
    self.min_positions = min_positions
    self.max_velocity = max_velocity
    self.max_acceleration = max_acceleration
    self.collision_check = collision_check or true
    self.max_deviation = max_deviation or 0.0
    self.dt = dt or 0.008
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
        collision_check = self.collision_check,
        max_deviation = self.max_deviation,
        max_velocity = self.max_velocity,
        max_acceleration = self.max_acceleration,
        dt = self.dt
    }
end

return PlanParameters
