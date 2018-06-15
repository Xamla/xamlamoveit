--[[
TvpController.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
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
    self.convergence_threshold = 1e-4
    self.converged = true
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
    self.converged = max_eta < (dt / 4)

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

function TvpController:generateOfflineTrajectory(start, goal, dt, start_vel)
    local result = {}
    local counter = 1
    self:reset()
    self.state.pos:copy(start)
    if start_vel ~= nil then
        self.state.vel:copy(start_vel)
    end

    result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
    local T = 2 * dt
    while T > dt / 4 do
	counter = counter + 1
        T = self:update(goal, dt)
        result[counter] = createState(self.state.pos, self.state.vel, self.state.acc)
    end

    local final_delta = goal - self.state.pos
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
    counter = counter + 1
    result[counter] = createState(goal, self.state.vel:zero(), self.state.acc:zero())
    return result, final_delta
end

return TvpController
