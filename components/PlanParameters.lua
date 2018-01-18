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
    self.collision_check = collision_check == nil or collision_check
    self.max_deviation = max_deviation or 0.0
    self.dt = dt or 0.008
end

function PlanParameters:setFromTable(t)
    assert(type(t) == 'table', 'Source table argument must not be nil.')
    for k, v in pairs(t) do
        self[k] = v
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

function PlanParameters:clone()
    local result = PlanParameters.new()
    result:setFromTable(self:toTable())
    return result
end

function PlanParameters:__tostring()
    local res = 'PlanParameters:\n'
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

return PlanParameters
