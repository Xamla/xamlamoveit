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
end

function TvpController:update(target, dt)
    self.state.pos = self.state.pos + self.state.vel * dt + self.state.acc * dt * dt / 2
    self.state.vel = self.state.vel + self.state.acc * dt

    -- calc time to reach target with max acceleration in decelerating phase
    local distance_to_go = target - self.state.pos
    local time_to_target = torch.sqrt(torch.abs(distance_to_go):cdiv(self.max_acc) * 2) -- solve s=1/2 * a * t^2 for t

    -- scale to discrete timesteps
    local real_time_to_target = torch.ceil(time_to_target / dt) * dt

    -- calc new acceleration to stop on target
    local acc = torch.cdiv(distance_to_go * 2, torch.pow(real_time_to_target, 2))

    acc[real_time_to_target:lt(dt * 0.5)] = 0 -- target reached

    local vel = torch.cmul(acc, torch.cmax(real_time_to_target - dt, 0)) -- max next velocity to stop on target

    vel = clamp(vel, -self.max_vel, self.max_vel) -- limit to max velocity

    acc = (vel - self.state.vel) / dt
    acc = clamp(acc, -self.max_acc, self.max_acc)

    self.state.acc = acc
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
    local max_counter = 1000
    while (goal - self.state.pos):norm() > 1e-5 and max_counter > counter do
        self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end
    if max_counter <= counter then
        print('not converged')
    end
    return result
end

return TvpController
