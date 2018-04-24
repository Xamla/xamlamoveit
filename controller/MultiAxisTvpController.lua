local controller = require 'xamlamoveit.controller.env'

local MultiAxisTvpController = torch.class('xamlamoveit.controller.MultiAxisTvpController', controller)

local function clamp(t, min, max)
    if torch.type(t) == 'number' then
        return math.max(math.min(t, max), min)
    end
    return torch.cmin(t, max):cmax(min)
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
    self.last_goal = nil
    self.convergence_threshold = 1e-2
    self.goal_difference_threshold = 1e-3
    self.converged = true

    -- temporary tensors
    self.a0 = torch.zeros(dim) -- current acceleration
    self.etas = torch.zeros(dim)
end

function MultiAxisTvpController:update(goal, dt)
    local dim = self.dim
    assert(dim == goal:size(1), 'Goal element count mismatch')

    self.state.pos = self.state.pos + self.state.vel * dt + 0.5 * self.state.acc * dt * dt
    self.state.vel = self.state.vel + self.state.acc * dt

    local to_go = goal - self.state.pos

    if not self.last_goal or (self.last_goal - goal):norm() > self.goal_difference_threshold then
        assert(self.state.vel:norm() < 1e-8 and self.state.acc:norm() < 1e-8, 'Cannot handle inital non zero velocity or non zero acceleration.')
        self.last_goal = goal:clone()
        self.norm_max_vel:copy(self.max_vel)
        self.norm_max_acc:copy(self.max_acc)

        -- compute times of all phases for all axis
        local acc_times = torch.zeros(dim)
        local cru_times = torch.zeros(dim)
        local dec_times = torch.zeros(dim)

        for i = 1, dim do

            -- calculate time to goal for each axis

            local acc_time = 0
            local cru_time = 0
            local dec_time = 0

            local max_vel = self.max_vel[i]
            local max_acc = self.max_acc[i]

            local distance = math.abs(to_go[i])

            -- compute maximum velocity that could be reached to goal (asuming inital 0 velocity)
            -- by checking how long it would take to reach mid-point of distance to go with full throttle
            local unbounded_max_v = math.sqrt(distance / max_acc) * max_acc
            if unbounded_max_v < max_vel then
                -- we will never reach cruising velocity
                acc_time = math.sqrt(distance / max_acc)
                dec_time = acc_time
                cru_time = 0
            else
                acc_time = max_vel / max_acc -- time for 0 -> max vel / max_vel -> 0 vel
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

        -- compute slowest values for each phase
        local longest_acc = acc_times:max(1)[1]
        local longest_dec = dec_times:max(1)[1]
        local longest_cru = cru_times:max(1)[1]

        -- round phases to full dt
        longest_acc = math.ceil(longest_acc / dt) * dt
        longest_dec = math.ceil(longest_dec / dt) * dt
        longest_cru = math.ceil(longest_cru / dt) * dt

        --local total_traj_time = longest_acc + longest_cru + longest_dec

        -- now run 2nd pass over all axis and compute max_acc & max_vel values
        for i = 1, dim do
            -- compute v_max to reach goal
            local distance = math.abs(to_go[i])

            if distance > self.goal_difference_threshold then
                local v_max = distance / (0.5 * longest_acc + longest_cru + 0.5 * longest_dec)

                -- compute a_max to reach v_max in longest times
                local a_acc_max = v_max / longest_acc
                --local a_dec_max = v_max / longest_dec

                self.norm_max_vel[i] = v_max
                self.norm_max_acc[i] = a_acc_max
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

function MultiAxisTvpController:generateOfflineTrajectory(start, goal, dt, start_vel)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    if start_vel ~= nil then
        self.state.vel:copy(start_vel)
    end

    result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
    local T = self:update(goal, dt)
    while T > dt do
        T = self:update(goal, dt)
        counter = counter + 1
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
    end

    local final_delta = goal - self.state.pos
    if dt < 0.02 then
        -- Needed for correct convergence to convergence_threshold
        if final_delta:norm() >= self.convergence_threshold then
            local correction_counter = 0
            while final_delta:norm() >= self.convergence_threshold and correction_counter < 33 do
                T = self:update(goal, dt)
                counter = counter + 1
                result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
                final_delta = goal - self.state.pos
                correction_counter = correction_counter + 1
            end
        end
        assert(final_delta:norm() < self.convergence_threshold, 'Goal distance of generated TVP trajectory is too high.')
    end
    counter = counter + 1
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    return result, final_delta
end

return MultiAxisTvpController
