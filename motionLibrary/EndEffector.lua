--[[
EndEffector.lua

Copyright (C) 2018  Xamla info@xamla.com

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
        error(error_msg)
    end

    local T = createTransformFromPoseMsg(solution.pose)
    local stamp = solution.header.stamp
    local frame_id = solution.header.frame_id
    return tf.StampedTransform(T, stamp, frame_id)
end

function EndEffector:getCurrentPose()
     return self.move_group:getCurrentPose(self.name)
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
    local ok, joint_trajectory, plan_parameters = self:planMoveLinear(target, velocity_scaling, collision_check, acceleration_scaling)
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
    local plan_parameters = self:buildTaskSpacePlanParameters(self.name, velocity_scaling, collision_check, accelerationScaling)

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
