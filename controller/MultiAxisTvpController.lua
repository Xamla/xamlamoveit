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
    self.last_longest_axis_index = 1
    self.slowest_axis = nil
    self.last_goal = nil
    self.convergence_threshold = 1e-5
    self.converged = true
    self.max_update_steps = 2000
end

local function getVelocity(self, goal, to_go, dt)
    local function p2(t,x0,v0,a)
        return x0 + torch.cmul(v0, t) + 0.5 * torch.cmul(a, torch.cmul(t, t))
    end

    -- early stopping point and duration
    local t_stop = torch.cdiv(torch.abs(self.state.vel),self.max_acc)
    local x_stop = p2(t_stop, self.state.pos, self.state.vel, torch.cmul(torch.sign(-self.state.vel),self.max_acc))

    local d = torch.sign(goal - x_stop)

    local v = torch.cmul(d, self.max_vel)
    local a_acc = torch.cmul(d, self.max_acc)
    local a_dec = -a_acc:clone()

    -- acceleration phase
    local delta_t1 = torch.abs(torch.cdiv(v - self.state.vel, a_acc))
    local delta_x1 = p2(delta_t1, torch.zeros(self.dim), self.state.vel, a_acc)

    -- decceleration phase
    local delta_t3 = torch.abs(torch.cdiv(v, a_dec))
    local delta_x3 = p2(delta_t3, torch.zeros(self.dim), v, a_dec)

    -- cruising phase
    local delta_t2 = torch.cdiv(goal - (self.state.pos + delta_x1 + delta_x3), v)

    local T = delta_t2:clone()
    T:apply(
        function(x)
            if 0 < x then
                return x
            else
                return 0
            end
        end
    )

    T = T + torch.abs(delta_t1) + torch.abs(delta_t3) -- duration of motion

    for i = 1, self.dim do
        if delta_t2[i] < 0 then
            v[i] = d[i] * math.sqrt(d[i] * self.max_acc[i] * to_go[i] + 0.5 * self.state.vel[i] * self.state.vel[i])
            T[i] = ((d[i] * math.abs(v[i]) - self.state.vel[i]) / a_acc[i])
             + (d[i] * math.abs(v[i]) / a_dec[i])
        end
    end
    local value, plan_joint_index = T:max(1)

    local max_T = torch.ones(T:size()):fill(value[1]) -- set max duration of motion for all joints

    max_T:apply(
        function(x)
            if x ~= x then
                return 0
            else
                return x
            end
        end
    )

    -- find sync velocity
    local duration = max_T - t_stop
    local vc = v:clone()

    for i = 1, self.dim do
        if duration[i] > 0 then
            vc[i] = (goal[i] - x_stop[i]) / duration[i]
        end
    end
    v:copy(vc)
    return v, max_T
end

function MultiAxisTvpController:update(goal, dt)
    self.state.pos = self.state.pos + self.state.vel * dt + self.state.acc * dt * dt / 2
    self.state.vel = self.state.vel + self.state.acc * dt

    local to_go = goal - self.state.pos
    self.converged = to_go:gt(self.convergence_threshold):sum() > 1
    local v, T = getVelocity(self, goal, to_go, dt)

    local real_time_to_target = (torch.ceil(T / dt)) * dt
    for i = 1, self.dim do
        if math.abs(to_go[i]) < self.convergence_threshold then
        --if math.abs(real_time_to_target[i]) < dt then
            v[i] = 0
        end
    end

    a0 = (v - self.state.vel) / dt

    -- test nan
    a0:apply(
        function(x)
            if x ~= x then
                return 0
            end
        end
    )
    self.state.acc:copy(clamp(a0, -self.max_acc, self.max_acc))
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
    local max_counter = self.max_update_steps
    while (goal - self.state.pos):norm() > self.convergence_threshold and max_counter > counter do
        self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    if max_counter <= counter then
        print('not converged')
    end
    return result
end

return MultiAxisTvpController
