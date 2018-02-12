local controller = require 'xamlamoveit.controller.env'

local MultiAxisCppController = torch.class('xamlamoveit.controller.MultiAxisCppController', controller)

function MultiAxisCppController:__init(dim)
    self.dim = dim
    self.state = {
        pos = torch.zeros(dim),
        vel = torch.zeros(dim),
        acc = torch.zeros(dim)
    }
    self.max_vel = torch.ones(dim) * math.pi
    self.max_acc = torch.ones(dim) * math.pi / 0.5
    self.pos_gain = 2.5 * 2
    self.vel_gain = self.pos_gain / 10--1 * 0.25
    self.convergence_threshold = 1e-5
    self.converged = true
end

function MultiAxisCppController:update(goal, dt)
    local dim = self.dim
    assert(dim == goal:size(1), 'Goal element count mismatch')

    -- state update
    self.state.pos = self.state.pos + self.state.vel * dt + 0.5 * self.state.acc * dt * dt
    self.state.vel = self.state.vel + self.state.acc * dt

    -- position loop
    local to_go = goal - self.state.pos
    self.converged = torch.abs(to_go):gt(self.convergence_threshold):sum() == 0
    local vel_cmd = to_go * self.pos_gain

    -- max vel scaling
    local vel_scale = 1
    for i = 1, dim do
        vel_scale = math.min(vel_scale, 0.95 * self.max_vel[i] / math.max(self.max_vel[i], math.abs(vel_cmd[i])))
    end

    vel_cmd:mul(vel_scale)

    -- velocity loop
    local vel_error = vel_cmd - self.state.vel
    local acc_cmd = vel_error * self.vel_gain / dt

    -- max acc scaling
    local acc_scale = 1
    for i = 1, dim do
        acc_scale = math.min(acc_scale, self.max_acc[i] / math.max(self.max_acc[i], math.abs(acc_cmd[i])))
    end

    acc_cmd:mul(acc_scale)

    self.state.acc = acc_cmd
end

function MultiAxisCppController:reset()
    self.state.pos:zero()
    self.state.vel:zero()
    self.state.acc:zero()
    self.converged = true
end

local function createState(pos, vel, acc)
    local state = {
        pos = pos:clone(),
        vel = vel:clone(),
        acc = acc:clone()
    }
    return state
end

function MultiAxisCppController:generateOfflineTrajectory(start, goal, dt, start_vel)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    if start_vel ~= nil then
        self.state.vel:copy(start_vel)
    end

    local final_delta = goal - self.state.pos
    while final_delta:norm() > self.convergence_threshold do
        self:update(goal, dt)
        final_delta = goal - self.state.pos
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
        counter = counter + 1
    end

    local final_delta = goal - self.state.pos
    assert(final_delta:norm() < self.convergence_threshold, 'Goal distance of generated trajectory is too high.')
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    return result, final_delta
end

return MultiAxisCppController
