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
    self.time_to_target = nil
end

function TvpController:update(target, dt)
    self.state.pos = self.state.pos + self.state.vel * dt
    self.state.vel = self.state.vel + self.state.acc * dt

    if self.time_to_target then -- check for convergence dt + 1
        self.converged = self.time_to_target:max() <= dt / 2 -- time_to_target:gt(dt / 2):sum() < 1
    end
    -- calc time to reach target with max acceleration in decelerating phase
    local distance_to_go = target - self.state.pos
    self.time_to_target = 2 * torch.sqrt(torch.abs(distance_to_go):cdiv(self.max_acc)) -- solve s=1/2 * a * t^2 for t

    ---print("distance_to_go:norm()", distance_to_go:norm())
    -- scale to discrete timesteps
    local real_time_to_target = torch.ceil(self.time_to_target / dt) * dt

    -- calc new acceleration to stop on target
    local acc = torch.cdiv(distance_to_go * 2, torch.pow(real_time_to_target, 2))

    acc[self.time_to_target:lt(dt / 2)] = 0 -- target reached
    local vel = torch.cmul(acc, torch.abs(real_time_to_target - torch.cmin(self.time_to_target, dt))) -- max next velocity to stop on target

    vel = clamp(vel, -self.max_vel, self.max_vel) -- limit to max velocity

    acc = (vel - self.state.vel) / dt
    acc = clamp(acc, -self.max_acc, self.max_acc)

    self.state.acc = acc

    return self.time_to_target:max()
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
    while T > dt / 2 do
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
