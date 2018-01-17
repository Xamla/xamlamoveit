local controller = require 'xamlamoveit.controller.env'
local MultiAxisTvpController = torch.class('xamlamoveit.controller.MultiAxisTvpController', controller)

local function clamp(t, min, max)
    if torch.type(t) == 'number' then
        return math.max(math.min(t, max), min)
    end
    return torch.cmin(t, max):cmax(min)
end

function math.sign(x)
    assert(type(x) == 'number')
    return x > 0 and 1 or x < 0 and -1 or 0
end

function MultiAxisTvpController:__init(dim)
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
    self.axis_scale = torch.ones(dim)
    self.longest_axis = nil
    self.slowest_axis = nil
    self.last_goal = nil
    self.convergence_threshold = 1e-4
    self.converged = true
end

function MultiAxisTvpController:update(goal, dt)
    self.state.vel = self.state.vel + self.state.acc * dt
    self.state.pos = self.state.pos + self.state.vel * dt

    local to_go = goal - self.state.pos
    self.converged = to_go:norm() < self.convergence_threshold
    if not self.last_goal or (self.last_goal - goal):norm() > self.convergence_threshold then
        self.last_goal = goal:clone()
        self.norm_max_vel = self.max_vel:clone()
        self.norm_max_acc = self.max_acc:clone()
        -- find longest axis
        local _
        _, self.longest_axis = torch.abs(to_go):max(1)

        if math.abs(to_go[self.longest_axis[1]]) > 0 then
            self.axis_scale:copy((goal - self.state.pos) / to_go[self.longest_axis[1]])
        else
            self.axis_scale:fill(1)
        end
        -- find slowest axis to reach goal
        local slowest_vel_time = torch.cdiv(torch.abs(goal - self.state.pos), self.max_vel)
        _, self.slowest_vel_axis = torch.max(slowest_vel_time, 1)

        if slowest_vel_time[self.slowest_vel_axis[1]] > 1 then
            for i = 1, self.dim do
                self.norm_max_vel[i] =
                    self.max_vel[i] * slowest_vel_time[i] / slowest_vel_time[self.slowest_vel_axis[1]]
            end
        end

        -- find slowest axis to reach norm_max_vel
        local slowest_acc_time = torch.abs(torch.cdiv(self.norm_max_vel, self.max_acc))
        local _, slowest_acc_axis = torch.max(slowest_acc_time, 1)
        if slowest_acc_time[slowest_acc_axis[1]] > 1 then
            for i = 1, self.dim do
                self.norm_max_acc[i] = self.max_acc[i] * slowest_acc_time[i] / slowest_acc_time[slowest_acc_axis[1]]
            end
        end
    end

    -- plan with longest axis with max_vel and max_acc scaled by the slowest axis
    local p0 = 0 -- current position normalized
    local p1 = to_go[self.longest_axis[1]] or 0 -- goal position
    local v0 = self.state.vel[self.longest_axis[1]] or 0 -- current velocity
    local a0  -- current acceleration
    local vmax, amax  -- max velocity
    if self.longest_axis[1] then
        vmax = self.norm_max_vel[self.longest_axis[1]] -- max velocity
        amax = self.norm_max_acc[self.longest_axis[1]] -- max acceleration
    else
        vmax = self.norm_max_vel:min()
        amax = self.norm_max_acc:min()
    end

    local v
    local eta
    local correct_amax

    eta = math.sqrt(2 * (math.abs(p1 - p0)) / amax) -- time to goal

    correct_amax = 2 * torch.abs(p1 - p0) / (eta * eta) -- correct amax
    if eta < dt * 0.5 then
        correct_amax = 0
    end

    v = math.sign(p1 - p0) * correct_amax * eta -- max velocity to stop on goal, decelerating
    v = math.min(math.max(v, -vmax), vmax) -- limit velocity to max velocity, constant velocity
    v = math.min(math.max(v, v0 - amax * dt), v0 + amax * dt) -- limit acceleration to max acceleration, accelerating

    a0 = (self.axis_scale * v - self.state.vel) / dt
    -- test nan
    a0:apply(
        function(x)
            if x ~= x then
                return 0
            end
        end
    )
    self.state.acc:copy(clamp(a0, -self.max_acc, self.max_acc))

    return eta
end

function MultiAxisTvpController:reset()
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

function MultiAxisTvpController:generateOfflineTrajectory(start, goal, dt)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    local T = 2*dt
    while T > dt/2 do
         T = self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())

    return result
end

return MultiAxisTvpController
