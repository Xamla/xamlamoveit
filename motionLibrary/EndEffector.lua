--[[
EndEffector.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local tf = ros.tf
local datatypes = require 'xamlamoveit.datatypes'
local xutils = require 'xamlamoveit.xutils'
local motionLibrary = require 'xamlamoveit.motionLibrary.env'

local EndEffector = torch.class('EndEffector', motionLibrary)

function EndEffector:__init(move_group, end_effector_name, end_effector_link_name)
    self.move_group = move_group
    self.name = end_effector_name
    self.link_name = end_effector_link_name
    self.motion_service = move_group.motion_service
end

local function createTransformFromPoseMsg(msg)
    local T = tf.Transform()
    T:setOrigin({ msg.position.x, msg.position.y, msg.position.z })
    T:setRotation(tf.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
    return T
end

function EndEffector:computePose(joint_values)
    assert(joint_values ~= nil, 'Argument `joint_values` must not be nil.')
    assert(torch.isTypeOf(joint_values, datatypes.JointValues), 'Invalid argument `joint_values`: JointValues object expected.')
    local error_code, solution, error_msgs = self.motion_service:queryPose(self.move_group.name, joint_values, self.link_name)
    if error_code.val ~= 1 then
        error(error_msgs)
    end

    local T = createTransformFromPoseMsg(solution.pose)
    local stamp = solution.header.stamp
    local frame_id = solution.header.frame_id
    local result = datatypes.Pose()
    result.stampedTransform = tf.StampedTransform(T, stamp, frame_id)
    return result
end

function EndEffector:getCurrentPose()
    return self:computePose(self.move_group:getCurrentJointValues())
end

function EndEffector:getIKSolution(pose, seed_joint_values)
    assert(seed_joint_values ~= nil, 'Argument `joint_values` must not be nil.')
    assert(torch.isTypeOf(seed_joint_values, datatypes.JointValues), 'Invalid argument `seed_joint_values`: JointValues object expected.')
    local result = self:getIKSolutions({pose}, seed_joint_values)
    return result[1]
end

function EndEffector:getIKSolutions(poses, seed_joint_values)
    local seed_joint_values = seed_joint_values or self.move_group:getCurrentJointValues()
    local plan_parameters = self.move_group:getDefaultPlanParameters()
    local oks, solutions = self.motion_service:queryIK(poses, plan_parameters, seed_joint_values, self.link_name)
    local result = {}
    for i = 1, #poses do
        assert(oks[i].val == 1, string.format('Failed inverse kinematic call, index: %d', i))
        result[#result +1 ] = datatypes.JointValues(datatypes.JointSet(solutions[i]:getNames()), solutions[i]:getValues())
    end
    -- find minimum distance to existing solutions
    return result
end

function EndEffector:planMovePoseCollisionFree(target, velocity_scaling)
    local plan_parameters = self.move_group:buildPlanParameters(velocity_scaling)

    -- get current pose
    local seed = self.move_group:getCurrentJointValues()
    local goal = self:getIKSolution(target, seed)

    -- plan trajectory
    local ok, joint_trajectory, ex_plan_parameters = self.move_group:planMoveJointsCollisionFree(goal, velocity_scaling)
    return ok, joint_trajectory, ex_plan_parameters
end

function EndEffector:planMovePoseLinear(target, velocity_scaling, collision_check, acceleration_scaling)
    local plan_parameters = self.move_group:buildTaskSpacePlanParameters(self.name, velocity_scaling, acceleration_scaling, collision_check)

    -- get current pose
    local seed = self.move_group:getCurrentJointValues()
    local start = self:getCurrentPose()

    -- generate path
    local path = {start, target}

    -- plan trajectory
    local ok, joint_trajectory = self.motion_service:planMoveLinear(seed, path, plan_parameters)
    return ok, joint_trajectory, plan_parameters
end

function EndEffector:movePoseLinear(target, velocity_scaling, collision_check, acceleration_scaling)
    assert(torch.isTypeOf(target, datatypes.Pose), 'Invalid argument `target`: Pose object expected.')
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMovePoseLinear(target, velocity_scaling, collision_check, acceleration_scaling)
    assert(ok == 1, 'movePoseLinear failed')

    -- start synchronous blocking execution
    local ok, msg = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    assert(ok, 'executeTaskSpaceTrajectory failed. ' .. msg)
end

function EndEffector:movePoseLinearAsync(target, velocity_scaling, collision_check, acceleration_scaling, done_cb)
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMovePoseLinear(target, velocity_scaling, collision_check, acceleration_scaling)
    assert(ok == 1, 'movePoseLinear failed')

    local simple_action_client = self.motion_service:executeJointTrajectoryAsync(joint_trajectory, plan_parameters.collision_check, done_cb)
    return simple_action_client
end

function EndEffector:movePoseLinearSupervised(target, velocity_scaling, collision_check, accelerationScaling, done_cb)
    assert(torch.isTypeOf(target, datatypes.Pose), 'Invalid argument `target`: Pose object expected.')
    if seed then
        assert(torch.isTypeOf(seed, datatypes.JointValues), 'Invalid argument `joint_values`: JointValues object expected.')
    end
    if done_cb then
        assert(torch.type(done_cb) == 'function', 'Invalid argument `done_cb`: function expected.')
    end
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMovePoseLinear(target, velocity_scaling, collision_check, accelerationScaling)
    assert(ok == 1, 'movePoseLinear failed')

    -- start asynchronous execution
    local controller_handle = self.motion_service:executeSupervisedJointTrajectory(joint_trajectory, plan_parameters.collision_check, done_cb)
    return controller_handle
end

function EndEffector:planMovePoseLinearWaypoints(waypoints, velocity_scaling, collision_check, max_deviation, accelerationScaling)
    max_deviation = max_deviation or 0.2
    local plan_parameters = self.move_group:buildTaskSpacePlanParameters(self.name, velocity_scaling, collision_check, accelerationScaling)

    -- get current pose
    local seed = self.move_group:getCurrentJointValues()
    local start = self:getCurrentPose()

    local ok, joint_trajectory = self.motion_service:planMoveLinear(seed, waypoints, plan_parameters)
    return ok, joint_trajectory, plan_parameters
end

function EndEffector:movePoseLinearWaypoints(waypoints, velocity_scaling, collision_check, max_deviation, accelerationScaling)
    assert(#waypoints ~= 0, 'Waypoints are empty.')

    local ok, joint_trajectory, plan_parameters = self:planMovePoseLinearWaypoints(waypoints, velocity_scaling, collision_check, max_deviation, accelerationScaling)
    assert(ok == 1, 'planMovePoseLinearWaypoints failed')

    -- start synchronous blocking execution
    local ok = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    assert(ok, 'executeJointTrajectory failed.')
end

function EndEffector:movePoseCollisionFree(target, velocity_scaling)
    assert(torch.isTypeOf(target, datatypes.Pose), 'Invalid argument `target`: Pose object expected.')

    local ok, joint_trajectory, plan_parameters = self:planMovePoseCollisionFree(target, velocity_scaling)
    assert(ok == 1, 'planMovePoseLinearWaypoints failed')

    -- start synchronous blocking execution
    local ok = self.motion_service:executeJointTrajectory(joint_trajectory, false)
    assert(ok, 'executeJointTrajectory failed.')
end

function EndEffector:movePoseCollisionFreeAsync(target, velocity_scaling, done_cb)
    assert(torch.isTypeOf(target, datatypes.Pose), 'Invalid argument `target`: Pose object expected.')

    local ok, joint_trajectory, plan_parameters = self:planMovePoseCollisionFree(target, velocity_scaling)
    assert(ok == 1, 'planMovePoseCollisionFree failed')

    -- start asynchronous execution
    local simple_action_client= self.motion_service:executeJointTrajectoryAsync(joint_trajectory, false, done_cb)
    return simple_action_client
end
