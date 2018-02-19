local controller = require 'xamlamoveit.controller.env'

local MultiAxisTvpController2 = torch.class('xamlamoveit.controller.MultiAxisTvpController2', controller)

local function clamp(t, min, max)
    if torch.type(t) == 'number' then
        return math.max(math.min(t, max), min)
    end
    return torch.cmin(t, max):cmax(min)
end

function MultiAxisTvpController2:__init(dim)
    self.dim = dim
    self.state = {
        pos = torch.zeros(dim),
        vel = torch.zeros(dim),
        acc = torch.zeros(dim)
    }
    self.max_vel = torch.ones(dim) * math.pi
    self.max_acc = torch.ones(dim) * math.pi / 2
    self.norm_max_vel = torch.ones(dim) * math.pi
    self.norm_max_acc = torch.ones(dim) * math.pi / 2
    self.last_goal = nil
    self.convergence_threshold = 1e-4
    self.converged = true

    -- temporary tensors
    self.a0 = torch.zeros(dim) -- current acceleration
    self.etas = torch.zeros(dim)
end

function MultiAxisTvpController2:update(goal, dt)
    local dim = self.dim
    assert(dim == goal:size(1), 'Goal element count mismatch')

    self.state.pos = self.state.pos + self.state.vel * dt + 0.5 * self.state.acc * dt * dt
    self.state.vel = self.state.vel + self.state.acc * dt

    local to_go = goal - self.state.pos

    if not self.last_goal or (self.last_goal - goal):norm() > self.convergence_threshold then
        self.last_goal = goal:clone()
        self.norm_max_vel:copy(self.max_vel)
        self.norm_max_acc:copy(self.max_acc)

        -- find slowest axis to reach goal
        local slowest_vel_time = torch.cdiv(torch.abs(to_go), self.max_vel)
        local _, slowest_vel_axis = torch.max(slowest_vel_time, 1)

        if slowest_vel_time[slowest_vel_axis[1]] > 0 then
            for i = 1, self.dim do
                self.norm_max_vel[i] = self.max_vel[i] * slowest_vel_time[i] / slowest_vel_time[slowest_vel_axis[1]]
            end
        end

        -- find slowest axis to reach norm_max_vel
        local slowest_acc_time = torch.abs(torch.cdiv(self.norm_max_vel, self.max_acc))
        local _, slowest_acc_axis = torch.max(slowest_acc_time, 1)
        if slowest_acc_time[slowest_acc_axis[1]] > 0 then
            for i = 1, self.dim do
                self.norm_max_acc[i] = self.max_acc[i] * slowest_acc_time[i] / slowest_acc_time[slowest_acc_axis[1]]
            end
        end
    end

    -- compute eta
    local eta = 0
    for i = 1, dim do
        local p1 = to_go[i] -- goal position
        local amax = self.norm_max_acc[i] -- max acceleration
        local time_to_goal = math.sqrt(2 * math.abs(p1) / amax) -- time to goal
        if time_to_goal > eta then
            eta = time_to_goal
        end
    end

    local eta_ = math.ceil(eta / dt) * dt -- round to full dt

    -- now compute tvp step
    local a0 = self.a0
    for i = 1, dim do
        -- plan for each axis individually
        local p1 = to_go[i] -- goal position
        local v0 = self.state.vel[i] -- current velocity

        local vmax = self.norm_max_vel[i] -- max velocity
        local amax = self.norm_max_acc[i] -- max acceleration

        local correct_amax = 2 * p1 / (eta_ * eta_) -- correct amax
        local v = 0
        if eta_ > dt then
            v = correct_amax * (eta_ - dt) -- max velocity to stop on goal, decelerating
        elseif eta > 0 and v0 == 0 then
            v = correct_amax * eta      -- handles case when near goal
        end

        v = math.min(math.max(v, -vmax), vmax) -- limit velocity to max velocity, constant velocity
        v = math.min(math.max(v, v0 - amax * dt), v0 + amax * dt) -- limit acceleration to max acceleration, accelerating

        a0[i] = (v - v0) / dt
    end

    a0:apply(function(x) if x ~= x then return 0 end end)           -- replace nan values by 0
    self.state.acc:copy(clamp(a0, -self.max_acc, self.max_acc))     -- clamp to max_acc and assign to state acceleration value
    self.converged = eta < (dt / 4)

    return eta
end

function MultiAxisTvpController2:reset()
    self.last_goal = nil
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

function MultiAxisTvpController2:generateOfflineTrajectory(start, goal, dt, start_vel)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    if start_vel ~= nil then
        self.state.vel:copy(start_vel)
    end

    local T = 2 * dt
    while T > dt / 4 do
        T = self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end

    local final_delta = goal - self.state.pos
    if final_delta:norm() >= self.convergence_threshold then
        local correction_counter = 0
        while final_delta:norm() >= self.convergence_threshold and correction_counter < 33 do
            T = self:update(goal, dt)
            result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
            counter = counter + 1
            final_delta = goal - self.state.pos
            correction_counter = correction_counter + 1
        end
    end
    assert(final_delta:norm() < self.convergence_threshold, 'Goal distance of generated TVP trajectory is too high.')
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    return result, final_delta
end

return MultiAxisTvpController2
