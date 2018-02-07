local controller = require 'xamlamoveit.controller.env'
local TvpController = torch.class('xamlamoveit.controller.TvpController', controller)

local function clamp(t, min, max)
    return torch.cmin(t, max):cmax(min)
end
function TvpController:__init(dim)
    self.dim = dim
    self.state = {
        pos = torch.zeros(dim),
        vel = torch.zeros(dim),
        acc = torch.zeros(dim)
    }
    self.max_vel = torch.ones(dim)
    self.max_acc = torch.ones(dim) * math.pi
    self.last_update = nil
    self.scale = nil
    self.convergence_threshold = 1e-4
    self.converged = true
end

function math.sign(x)
    assert(type(x) == 'number')
    return x > 0 and 1 or x < 0 and -1 or 0
end

function TvpController:update(target, dt)
    local dim = self.dim
    assert(dim == target:size(1), 'Goal element count mismatch')

    self.state.pos = self.state.pos + self.state.vel * dt + 0.5 * self.state.acc * dt * dt
    self.state.vel = self.state.vel + self.state.acc * dt

    local to_go = target - self.state.pos

    local max_eta = 0

    local a0 = torch.zeros(dim)
    for i = 1, dim do
        local s = to_go[i] -- goal position
        local v0 = self.state.vel[i] -- current velocity

        local vmax = self.max_vel[i] -- max velocity
        local amax = self.max_acc[i] -- max acceleration
        vmax = math.floor(vmax / (amax * dt)) * amax * dt

        -- calc time to reach target with max acceleration in decelerating phase
        local eta = math.sqrt(2 * math.abs(s) / amax) -- solve s=1/2 * a * t^2 for t
        max_eta = math.max(eta, max_eta)
        local eta_  = math.ceil(eta / dt) * dt -- round to full dt

        -- plan for each axis individually
        local correct_amax = 2 * s / (eta_ * eta_) -- correct amax
        local v = 0
        if eta_ > dt then
            v = correct_amax * (eta_ - dt) -- max velocity to stop on goal, decelerating
        elseif eta > 0 and v0 == 0 then
            v = correct_amax * eta -- handles case when near goal
        end

        v = math.min(math.max(v, -vmax), vmax) -- limit velocity to max velocity, constant velocity
        v = math.min(math.max(v, v0 - amax * dt), v0 + amax * dt) -- limit acceleration to max acceleration, accelerating
        
        a0[i] = (v - v0) / dt
    end

    self.state.acc:copy(clamp(a0, -self.max_acc, self.max_acc))     -- clamp to max_acc and assign to state acceleration value

    return max_eta
end

function TvpController:reset()
    self.state.pos:zero()
    self.state.vel:zero()
    self.state.acc:zero()
end

local function createState(pos, vel, acc)
    local state = {
        pos = pos:clone(),
        vel = vel:clone(),
        acc = acc:clone()
    }
    return state
end

function TvpController:generateOfflineTrajectory(start, goal, dt)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    local T = 2 * dt
    while T > dt / 8 do
        T = self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end

    local final_delta = goal - self.state.pos
    assert(final_delta:norm() < self.convergence_threshold, 'Goal distance of generated TVP trajectory is too high. ' .. final_delta:norm())
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    return result, final_delta
end

return TvpController
