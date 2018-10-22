--[[
TaskSpaceController.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local tf = require 'ros'.tf
local controller = require 'xamlamoveit.controller.env'

local TaskSpaceController,
    TvpController =
    torch.class(
    'xamlamoveit.controller.TaskSpaceController',
    'xamlamoveit.controller.MultiAxisTvpController',
    controller
)

local function poseTo6DTensor(input, solution)
    assert(torch.type(solution) == 'number', 'solution is a number [1,2]')
    solution = math.max(1, math.min(2, solution))
    local new_input
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end

    if torch.isTypeOf(input, tf.Transform) then
        new_input = torch.zeros(6)
        new_input[{{1, 3}}] = input:getOrigin()

        --RPY version
        new_input[{{4, 6}}]:copy(input:getRotation():getRPY(solution))
    end

    if torch.isTypeOf(input, torch.DoubleTensor) then
        new_input = input
    end
    assert(
        torch.isTypeOf(new_input, torch.DoubleTensor),
        string.format('Input should be of type [torch.DoubleTensor] but is of type: [%s]', torch.type(new_input))
    )
    return new_input
end

local function tensor6DToPose(vector6D)
    assert(vector6D:size(1) == 6, 'Vector should be of size 6D (offset, anglevelocities)')
    local end_pose = tf.StampedTransform()
    end_pose:setOrigin(vector6D[{{1, 3}}])
    if vector6D[{{4, 6}}]:norm() > 1e-12 then
        local end_pose_rotation = end_pose:getRotation()
        end_pose:setRotation(end_pose_rotation:setRPY(vector6D[{{4, 6}}]))
    end
    return end_pose
end

function TaskSpaceController:__init()
    local dim = 6
    TvpController.__init(self, dim)
    self.dim = dim --xyz rpy
    self.max_vel[{{1, 3}}]:fill(0.1)
    self.max_vel[{{4, 6}}]:fill(math.pi)
    self.max_acc[{{1, 3}}]:fill(0.4)
    self.max_acc[{{4, 6}}]:fill(math.pi / 2)
    self.norm_max_vel[{{1, 3}}]:fill(0.1)
    self.norm_max_vel[{{4, 6}}]:fill(math.pi)
    self.norm_max_acc[{{1, 3}}]:fill(0.4)
    self.norm_max_acc[{{4, 6}}]:fill(math.pi / 2)
end

local function transformInput(input, solution)
    local new_input
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end

    if torch.isTypeOf(input, tf.Transform) then
        new_input = poseTo6DTensor(input, solution)
    end
    if torch.isTypeOf(input, torch.DoubleTensor) then
        new_input = input
    end
    assert(
        torch.isTypeOf(new_input, torch.DoubleTensor),
        string.format('Input should be of type [torch.DoubleTensor] but is of type: [%s]', torch.type(new_input))
    )
    return new_input
end

function TaskSpaceController:update(goal, dt)
    goal = transformInput(goal)
    return TvpController.update(self, goal, dt)
end

function TaskSpaceController:getCurrentPose()
    return tensor6DToPose(self.state.pos)
end

function TaskSpaceController:setCurrentPose(pose)
    self.state.pos = transformInput(pose, self.state.pos[{{4,6}}])
end

function TaskSpaceController:generateOfflineTrajectory(start, goal, dt, start_vel)
    local start_1 = transformInput(start, 1)
    local goal_1 = transformInput(goal, 1 )
    local start_2 = transformInput(start, 2)
    local goal_2 = transformInput(goal, 2 )
    if (goal_1 - start_1 ):norm() < (goal_2 - start_2 ):norm() then
        goal = goal_1
        start = start_1
    else
        goal = goal_2
        start = start_2
    end
    if start_vel then
        start_vel = transformInput(start_vel, 1)
    end
    return TvpController.generateOfflineTrajectory(self, start, goal, dt, start_vel)
end

return TaskSpaceController
