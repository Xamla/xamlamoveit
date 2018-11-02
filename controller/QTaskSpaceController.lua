--[[
QTaskSpaceController.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local tf = require 'ros'.tf
local controller = require 'xamlamoveit.controller.env'

local QTaskSpaceController, TvpController =
    torch.class(
    'xamlamoveit.controller.QTaskSpaceController',
    'xamlamoveit.controller.MultiAxisTvpController',
    controller
)

local function poseToPositionTensor(input, angle)
    local new_input = torch.zeros(4)
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end

    if torch.isTypeOf(input, tf.Transform) then
        new_input[{{1,3}}] = input:getOrigin():clone()
        new_input[4] = angle
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

local function tensorToPose(controller, vector6D)
    print('tensorToPose')
    assert(vector6D:size(1) == 4, 'Vector should be of size 4D (offset, anglevelocities)')
    assert(controller.start_pose)
    assert(controller.goal_pose)
    local start_q = controller.start_pose:getRotation()
    local goal_q = controller.goal_pose:getRotation()
    local angle = start_q:angleShortestPath(goal_q)
    local d_angle = vector6D[4]
    local step = d_angle/angle
    local end_pose = tf.StampedTransform()
    end_pose:setOrigin(vector6D[{{1, 3}}])
    local end_pose_rotation = start_q:slerp(goal_q, step)
    end_pose:setRotation(end_pose_rotation)
    return end_pose
end

function QTaskSpaceController:__init()
    local dim = 4
    TvpController.__init(self, dim)
    self.dim = dim --xyz rpy
    self.max_vel[{{1, 3}}]:fill(0.1)
    self.max_vel[4] = math.pi
    self.max_acc[{{1, 3}}]:fill(0.4)
    self.max_acc[4] = math.pi / 2
    self.norm_max_vel[{{1, 3}}]:fill(0.1)
    self.norm_max_vel[4] = math.pi
    self.norm_max_acc[{{1, 3}}]:fill(0.4)
    self.norm_max_acc[4] = math.pi / 2
    self.start_pose = nil
    self.goal_pose = nil
    self.difference_angle = nil
end

function QTaskSpaceController:update(goal, dt)
    if torch.isTypeOf(goal, tf.StampedTransform) then
        self.goal_pose = goal:toTransform()
    elseif torch.isTypeOf(goal, tf.Transform) then
        self.goal_pose = goal
        print(self.start_pose, self.goal_pose, goal)
        self.difference_angle = self.start_pose:getRotation():angleShortestPath(self.goal_pose:getRotation())
        goal = poseToPositionTensor(goal, self.difference_angle)
    end

    return TvpController.update(self, goal, dt)
end

function QTaskSpaceController:getCurrentPose()
    return tensorToPose(self.state.pos)
end

function QTaskSpaceController:setCurrentPose(pose)
    error('not implemented')
end

function QTaskSpaceController:generateOfflineTrajectory(start, goal, dt)
    if torch.isTypeOf(goal, tf.StampedTransform) then
        self.goal_pose = goal:toTransform()
    else
        self.goal_pose = goal
    end
    if torch.isTypeOf(goal, tf.StampedTransform) then
        self.start_pose = start:toTransform()
    else
        self.start_pose = start
    end
    local start_ = torch.zeros(4)
    start_[{{1,3}}] = start:getOrigin()
    self.difference_angle = self.start_pose:getRotation():angleShortestPath(self.goal_pose:getRotation())
    goal = poseToPositionTensor(goal, self.difference_angle)
    return TvpController.generateOfflineTrajectory(self, start_, goal, dt, start_vel)
end

return controller.QTaskSpaceController
