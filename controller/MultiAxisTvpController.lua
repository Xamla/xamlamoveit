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
    local dim = goal:size(1)

    if not self.last_goal or (self.last_goal - goal):norm() > self.convergence_threshold then
        self.last_goal = goal:clone()
        self.norm_max_vel = self.max_vel:clone()
        self.norm_max_acc = self.max_acc:clone()

        -- compute times of all phases for all axis
        local acc_times = torch.zeros(goal:size())
        local cru_times = torch.zeros(goal:size())
        local dec_times = torch.zeros(goal:size())

        for i=1,dim do
            -- calculate time to goal for each axis

            local max_vel = self.max_vel[i]
            local max_acc = self.max_acc[i]
            local distance = math.abs(to_go[i])

            local acc_time = 0
            local cru_time = 0
            local dec_time = 0

            -- compute maximum velocity that could be reached to goal (asuming inital 0 velocity)
            -- by checking how long it would take to reach mid-point of distance to go with full throttle
            local unbounded_max_v = math.sqrt(distance / max_acc) * max_acc
            --printf('unbounded_max_v: %f (max_vel: %f)', unbounded_max_v, max_vel)     -- ## debug output
            if unbounded_max_v < max_vel then
                -- we will never reach cruising velocity
                acc_time = math.sqrt(distance / max_acc)
                dec_time = acc_time
                cru_time = 0
            else
                acc_time = max_vel / max_acc         -- time for 0 -> max vel / max_vel -> 0 vel
                dec_time = acc_time

                -- distance traveled during acc & dec phase
                local acc_dec_distance = max_acc * dec_time * dec_time
                local cruise_distance = distance - acc_dec_distance
                cru_time = cruise_distance / max_vel
            end

            acc_times[i] = acc_time
            dec_times[i] = dec_time
            cru_times[i] = cru_time
        end

--[[
        -- ## debug output
        print('acc_times')
        print(acc_times)

        print('cru_times')
        print(cru_times)

        print('dec_times')
        print(dec_times)
]]

        -- compute slowest values for each phase
        local longest_acc = acc_times:max(1)[1]
        local longest_dec = dec_times:max(1)[1]
        local longest_cru = cru_times:max(1)[1]

        local total_traj_time = longest_acc + longest_cru + longest_dec

        -- now run 2nd pass over all axis and compute max_acc & max_vel values for them to reach goal in total_traj_time
        for i=1,dim do

            -- compute v_max to reach goal
            local distance = math.abs(to_go[i])
            local v_max = distance / (0.5 * longest_acc + longest_cru + 0.5 * longest_dec)

            -- compute a_max to reach v_max in longest times
            local a_acc_max = v_max / longest_acc
            --local a_dec_max = v_max / longest_dec

            self.norm_max_vel[i] = v_max
            self.norm_max_acc[i] = a_acc_max
        end
--[[
        -- ## debug output
        print('max_vel')
        print(self.max_vel)

        print('norm_max_vel')
        print(self.norm_max_vel)

        print('max_acc')
        print(self.max_acc)

        print('norm_max_acc')
        print(self.norm_max_acc)
]]
    end

    local a0 = torch.zeros(dim)  -- current acceleration
    local etas = torch.zeros(dim)

    -- compute eta
    for i=1,dim do
        local p1 = to_go[i] -- goal position
        local amax = self.norm_max_acc[i] -- max acceleration
        local eta = math.sqrt(2 * math.abs(p1) / amax) -- time to goal
        eta = math.ceil(eta / dt) * dt -- round to full dt
        etas[i] = eta
    end

    local eta = etas:max(1)[1]

    -- now compute tvp step
    for i=1,dim do

         -- plan for each axis individually
        local p1 = to_go[i] -- goal position
        local v0 = self.state.vel[i] -- current velocity

        local vmax = self.norm_max_vel[i] -- max velocity
        local amax = self.norm_max_acc[i] -- max acceleration

        --local correct_amax = amax
        local eta_ = eta + dt
        local correct_amax = 2 * torch.abs(p1) / (eta_ * eta_) -- correct amax

        --if eta <= dt then
        --    correct_amax = 0
        --end

        local v = math.sign(p1) * correct_amax * eta_ -- max velocity to stop on goal, decelerating
        v = math.min(math.max(v, -vmax), vmax) -- limit velocity to max velocity, constant velocity
        v = math.min(math.max(v, v0 - amax * dt), v0 + amax * dt) -- limit acceleration to max acceleration, accelerating
        print(v)  -- ## debug output

        a0[i] = (v - v0) / dt
    end

    -- test nan
    a0:apply(
        function(x)
            if x ~= x then
                return 0
            end
        end
    )

    self.state.acc:copy(clamp(a0, -self.max_acc, self.max_acc))
    self.converged = eta <= dt      --to_go:norm() < self.convergence_threshold
    --printf('eta: %f', eta) -- ## debug output

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
