--[[
LinearCartesianTrajectoryPlanningService.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
require 'gnuplot'
local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local Controller = require 'xamlamoveit.controller'
local Datatypes = require 'xamlamoveit.datatypes'
local Xutils = require 'xamlamoveit.xutils'
local core = require 'xamlamoveit.core'
local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))
local TvpController = Controller.QTaskSpaceController
local optimplan = require 'optimplan'
--require 'xamlamoveit.components.RosComponent'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetLinearCartesianTrajectory')
local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')

local function table_concat(dst, src)
    for i, v in ipairs(src) do
        table.insert(dst, v)
    end
    return dst
end

local function createJointValues(names, values)
    assert(
        torch.type(names) == 'table',
        string.format('Joint names should be of type table but has type %s', torch.type(names))
    )
    assert(torch.isTypeOf(values, torch.DoubleTensor))
    local joint_set = Datatypes.JointSet(names)
    local joint_values = Datatypes.JointValues(joint_set, values)
    return joint_values
end

local function getEndEffectorMoveGroupMap(kinematic_model)
    local move_group_names = kinematic_model:getJointModelGroupNames()
    local map = {}
    for k, v in pairs(move_group_names) do
        local name, suc = kinematic_model:getGroupEndEffectorName(v)
        if suc then
            map[v] = name
        end
    end
    return table.merge(map, table.swapKeyValue(map))
end

local function createPoseMsg(frame, translation, rotation)
    assert(torch.type(frame) == 'string')
    assert(torch.isTypeOf(translation, torch.DoubleTensor))
    assert(torch.isTypeOf(rotation, torch.DoubleTensor))
    local msg = ros.Message(pose_msg_spec)
    msg.pose.position.x = translation[1]
    msg.pose.position.y = translation[2]
    msg.pose.position.z = translation[3]
    msg.pose.orientation.x = rotation[1]
    msg.pose.orientation.y = rotation[2]
    msg.pose.orientation.z = rotation[3]
    msg.pose.orientation.w = rotation[4]
    msg.header.frame_id = frame
    return msg
end

local function poses2MsgArray(points)
    local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
    local result = {}
    if torch.type(points) == 'table' then
        for i, v in ipairs(points) do
            assert(
                torch.isTypeOf(v, tf.StampedTransform),
                string.format('points need to be type of tf.StampedTransform, but is of type: [%s]', torch.type(v))
            )

            local translation = v:getOrigin()
            local rotation = v:getRotation():toTensor()
            local frame = v:get_frame_id()

            table.insert(result, createPoseMsg(frame, translation, rotation))
        end
    elseif torch.isTypeOf(points, tf.StampedTransform) then
        local translation = points:getOrigin()
        local rotation = points:getRotation():toTensor()
        local frame = points:get_frame_id()

        table.insert(result, createPoseMsg(frame, translation, rotation))
    else
        error('[poses2MsgArray] unknown type of points parameter: ' .. torch.type(points))
    end
    return result
end

local function lookupPose(self, link_name, base_link_name)
    local base_link_name = base_link_name or 'base_link'
    if self.transformListener:frameExists(base_link_name) and self.transformListener:frameExists(link_name) then
        self.transformListener:waitForTransform(base_link_name, link_name, ros.Time(0), ros.Duration(0.1), true)
        return true, self.transformListener:lookupTransform(base_link_name, link_name, ros.Time(0))
    else
        return false, tf.StampedTransform()
    end
end

local function poseStampedMsg2StampedTransform(self, msg)
    local result = tf.StampedTransform()
    result:setOrigin(torch.Tensor {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z})
    result:setRotation(
        tf.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    )
    result:set_frame_id(msg.header.frame_id)
    local success = true
    if #msg.header.frame_id > 0 then
        local transform
        success, transform = lookupPose(self, msg.header.frame_id, 'world')
        if success then
            result:setData(transform:mul(result:toTransform()))
            result:set_frame_id('world')
        end
    end

    return success, result
end

local function poseTo6DTensor(input, ref)
    local ref = ref or torch.zeros(3)
    local new_input
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end
    if torch.isTypeOf(input, tf.Transform) then
        new_input = torch.zeros(6)
        new_input[{{1, 3}}] = input:getOrigin()
        --RPY version
        local rot = input:getRotation()
        local A = rot:getRPY(1)
        local B = rot:getRPY(2)
        if (A - ref):norm() < (B - ref):norm() then
            new_input[{{4, 6}}]:copy(A)
        else
            new_input[{{4, 6}}]:copy(B)
        end
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

local function tensorToPose(controller, vector4D)
    assert(vector4D:size(1) == 4, 'Vector should be of size 4D (offset, anglevelocities)')
    assert(controller.start_pose)
    assert(controller.goal_pose)
    local start_q = controller.start_pose:getRotation()
    local goal_q = controller.goal_pose:getRotation()
    local angle = controller.difference_angle
    local d_angle = vector4D[4]
    local step = d_angle / angle
    local end_pose = tf.StampedTransform()
    end_pose:setOrigin(vector4D[{{1, 3}}])
    local end_pose_rotation = start_q:slerp(goal_q, step)
    end_pose:setRotation(end_pose_rotation)
    return end_pose
end

-- calculates the weighted pseudo inverse of a matrix M
-- @param M: Matrix which needs to be inverted
-- @param W: Weight matrix (optional)
local function pseudoInverse(M, W)
    local weights = W or torch.eye(M:size(1))
    assert(M:size(1) == weights:size()[1], 'Data matrix M and weight matrix W need to have the same number of cols')
    local inv = M:t() * weights * M
    -- make it definite
    inv:add(1e-15, torch.eye(inv:size(1)))
    return torch.inverse(inv) * M:t() * weights
end

local function msgToTensor(trajectory_msg)
    local points = trajectory_msg.points
    local time, pos, vel, acc = {}, {}, {}, {}
    for i = 1, #points do
        time[i] = points[i].time_from_start:toSec()
        pos[i] = points[i].positions
        vel[i] = points[i].velocities
        if points[i].accelerations then
            acc[i] = points[i].accelerations
        end
    end
    return torch.Tensor(time), torch.cat(pos, 2), torch.cat(vel, 2)
end

local function plot(trajectory_msg, filename)
    local time, pos, vel, acc = msgToTensor(trajectory_msg)
    local g = {}
    local plot_position = true
    local plot_velocity = true
    local plot_acceleration = false
    if plot_position == nil or plot_position then
        g = {}
        gnuplot.pngfigure(string.format('%s_positions.png', filename or 'test'))
        for i = 1, pos:size(1) do
            g[#g + 1] = {string.format('pos%d', i), time, pos[{i, {}}], '-'}
        end
        gnuplot.plot(g)
        gnuplot.plotflush()
    end
    if plot_velocity == nil or plot_velocity then
        g = {}
        gnuplot.pngfigure(string.format('%s_velocities.png', filename or 'test'))
        for i = 1, pos:size(1) do
            g[#g + 1] = {string.format('vel%d', i), time, vel[{i, {}}], '-'}
        end
        gnuplot.plot(g)
        gnuplot.plotflush()
    end
    if plot_acceleration == nil or plot_acceleration then
        g = {}
        gnuplot.pngfigure(string.format('%s_accelerations.png', filename or 'test'))
        for i = 1, pos:size(1) do
            g[#g + 1] = {string.format('acc%d', i), time, acc[{{}, i}], '-'}
        end
        gnuplot.plot(g)
        gnuplot.plotflush()
    end
end

local function pose2jointTrajectory(
    self,
    seed,
    joint_names,
    poses6D,
    end_effector_name,
    collision_check,
    ik_jump_threshold,
    dt)
    if collision_check == false then
        ros.WARN('[pose2jointTrajectory]Collision checks are disabled')
    end
    local move_group = self.end_effector_map[end_effector_name]
    local result = {}
    if not move_group then
        return result, 0
    end

    local seed_joint_values = createJointValues(joint_names, seed)
    local zero_seed_joint_values = createJointValues(joint_names, torch.zeros(seed:size()))
    self.robot_state:setVariablePositions(seed, joint_names)
    self.robot_state:update()
    local traj_joint_names = self.robot_model:getGroupJointNames(move_group):totable()
    local velocity_limits = self.joint_limits.vel:select(joint_names):getValues()
    local error = error_codes.SUCCESS
    for i, pose in ipairs(poses6D) do
        --ros.INFO('set IK')
        local old_state = self.robot_state:copyJointGroupPositions(move_group)
        local suc = self.robot_state:setFromIK(move_group, pose.pos:toTensor(), 1, 0.25)
        if suc then
            self.robot_state:update()
            local colliding = false
            if collision_check then
                colliding = self.planning_scene:isStateColliding(move_group, self.robot_state, true)
            end
            if not colliding then
                local new_state = self.robot_state:copyJointGroupPositions(move_group)
                if torch.abs(old_state - new_state):gt(ik_jump_threshold):sum() > 0 then
                    ros.ERROR(
                        '[pose2jointTrajectory] Jump in IK detected. In move_group %s. Transformed %f%% of trajectory',
                        move_group,
                        100 * i / #poses6D
                    )
                    return result, error_codes.NO_IK_SOLUTION
                end
                result[i] = {}
                seed_joint_values:setValues(traj_joint_names, new_state)
                result[i].pos = seed_joint_values:getValues()
                result[i].vel = (result[i].pos - result[math.max(i - 1, 1)].pos) / dt
                local abs_vel = torch.abs(result[i].vel)
                if abs_vel:gt(velocity_limits):sum() > 0 then
                    ros.ERROR(
                        '[pose2jointTrajectory] Exceeded joint velocity limits in move_group [%s]. Transformed %f%% of trajectory',
                        move_group,
                        100 * i / #poses6D
                    )
                    --print('index', i, 'dt', dt, 'abs vel', abs_vel, 'limits', velocity_limits)
                    error = error_codes.GOAL_VIOLATES_PATH_CONSTRAINTS
                end
            else
                ros.ERROR(
                    '[pose2jointTrajectory] Collision detected in move_group %s. Transformed %f%% of trajectory',
                    move_group,
                    100 * i / #poses6D
                )

                return result, error_codes.GOAL_IN_COLLISION
            end
        else
            ros.ERROR(
                '[pose2jointTrajectory] Could not set IK solution for move_group %s. Transformed %f%% of trajectory.',
                move_group,
                100 * i / #poses6D
            )
            return result, error_codes.NO_IK_SOLUTION
        end
    end
    return result, error
end

local function getLinearPath(
    self,
    start,
    goal,
    dt,
    max_xyz_velocity,
    max_xyz_acceleration,
    max_angular_velocity,
    max_angular_acceleration)
    assert(max_xyz_velocity > 0)
    assert(max_xyz_acceleration > 0)
    assert(max_angular_velocity > 0)
    assert(max_angular_acceleration > 0)
    tic('getLinearPath')
    local suc, start = poseStampedMsg2StampedTransform(self, start)
    if suc == false then
        return {}, error_codes.FRAME_TRANSFORM_FAILURE
    end
    local suc, goal = poseStampedMsg2StampedTransform(self, goal)
    if suc == false then
        return {}, error_codes.FRAME_TRANSFORM_FAILURE
    end
    self.controller = TvpController()
    local controller = self.controller
    local taskspace_max_vel = torch.ones(4) * 0.2 --m/s
    taskspace_max_vel[4] = math.pi / 2
    local taskspace_max_acc = torch.ones(4) * 0.8 --m/s^2
    taskspace_max_acc[4] = math.pi
    if max_xyz_velocity then
        taskspace_max_vel[{{1, 3}}]:fill(max_xyz_velocity)
    end
    if max_xyz_acceleration then
        taskspace_max_acc[{{1, 3}}]:fill(max_xyz_acceleration)
    end
    if max_angular_velocity then
        taskspace_max_vel[4] = max_angular_velocity
    end
    if max_angular_acceleration then
        taskspace_max_acc[4] = max_angular_acceleration
    end
    controller.max_vel:copy(taskspace_max_vel)
    controller.max_acc:copy(taskspace_max_acc)
    tic('generateOfflineTrajectory')
    local result_4D = controller:generateOfflineTrajectory(start, goal, dt)
    local result = {}
    for i, pose in ipairs(result_4D) do
        local full_pose = tensorToPose(self.controller, pose.pos)
        result[i] = {pos = full_pose}
    end
    assert(#result > 1)
    toc('generateOfflineTrajectory')
    toc('getLinearPath')
    return result, error_codes.SUCCESS
end

local function getOptimLinearPath(
    self,
    waypoints,
    dt,
    max_xyz_velocity,
    max_xyz_acceleration,
    max_angular_velocity,
    max_angular_acceleration,
    max_deviation)
    if max_xyz_velocity <= 0 then
        ros.ERROR('[getOptimLinearPath] max_xyz_velocity needs to be greater than 0.')
        return {}, error_codes.INVALID_GOAL_CONSTRAINTS
    end
    if max_xyz_acceleration <= 0 then
        ros.ERROR('[getOptimLinearPath] max_xymax_xyz_acceleration needs to be greater than 0.')
        return {}, error_codes.INVALID_GOAL_CONSTRAINTS
    end
    if max_angular_velocity <= 0 then
        ros.ERROR('[getOptimLinearPath] max_angular_velocity needs to be greater than 0.')
        return {}, error_codes.INVALID_GOAL_CONSTRAINTS
    end
    if max_angular_acceleration <= 0 then
        ros.ERROR('[getOptimLinearPath] max_angular_acceleration needs to be greater than 0.')
        return {}, error_codes.INVALID_GOAL_CONSTRAINTS
    end
    tic('getOptimLinearPath')

    -- get waypoints as stamped transforms
    local tmp_waypoints = {}
    for i, v in ipairs(waypoints) do
        local suc, transformed = poseStampedMsg2StampedTransform(self, v)
        if suc == false then
            return {}, error_codes.FRAME_TRANSFORM_FAILURE
        end
        tmp_waypoints[#tmp_waypoints + 1] = transformed
    end

    local converter = core.XyzAngularSpaceConverter(tmp_waypoints)
    local xyzaPath = converter:getXyzAngularPath()

    local taskspace_max_vel = torch.ones(4) * 0.2 --m/s
    taskspace_max_vel[4] = math.pi / 2
    local taskspace_max_acc = torch.ones(4) * 0.8 --m/s^2
    taskspace_max_acc[4] = math.pi
    if max_xyz_velocity then
        taskspace_max_vel[{{1, 3}}]:fill(max_xyz_velocity)
    end
    if max_xyz_acceleration then
        taskspace_max_acc[{{1, 3}}]:fill(max_xyz_acceleration)
    end
    if max_angular_velocity then
        taskspace_max_vel[4] = max_angular_velocity
    end
    if max_angular_acceleration then
        taskspace_max_acc[4] = max_angular_acceleration
    end

    local valid, time, pos, vel, acc =
        optimplan.generateTrajectory(xyzaPath, taskspace_max_vel, taskspace_max_acc, max_deviation, dt)
    assert(valid)

    local result = {}
    for i = 1, time:size(1) do
        result[#result + 1] = {pos = converter:samplePose(pos[{i, {}}])}
    end
    print(tmp_waypoints[1], result[1].pos)
    toc('getOptimLinearPath')
    return result, error_codes.SUCCESS
end

local function generateTrajectory(waypoints, dt)
    local time = torch.zeros(#waypoints)
    valid = true
    local pos = torch.zeros(#waypoints, waypoints[1].pos:size(1))
    local vel = pos:clone()

    for i, point in ipairs(waypoints) do -- TODO check limits
        time[i] = dt * (i - 1)
        pos[{i, {}}]:copy(point.pos)
        vel[{i, {}}]:copy(point.vel)
    end

    return valid, time, pos, vel
end

local function queryCartesianPathServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        request.error_code.val = -2
        return true
    end
    tic('queryCartesianPathServiceHandler')
    self.planning_scene:syncPlanningScene()
    local g_path = {}
    if 2 == #request.waypoints then
        ros.INFO('only start und goal received. Controller runs with dt = %f', request.dt)
        local traj, code =
            getLinearPath(
            self,
            request.waypoints[1],
            request.waypoints[2],
            request.dt,
            request.max_xyz_velocity,
            request.max_xyz_acceleration,
            request.max_angular_velocity,
            request.max_angular_acceleration
        )
        if code < 0 then
            response.error_code.val = code
            return true
        end
        if #traj > 0 then
            table_concat(g_path, traj)
        end
    elseif #request.waypoints > 1 then
        local traj, code =
            getOptimLinearPath(
            self,
            request.waypoints,
            request.dt,
            request.max_xyz_velocity,
            request.max_xyz_acceleration,
            request.max_angular_velocity,
            request.max_angular_acceleration,
            request.max_deviation
        )
        if code < 0 then
            response.error_code.val = code
            return true
        end
        if #traj > 0 then
            table_concat(g_path, traj)
        end
    end
    ros.INFO('got taskspace trajectory. Next convert to joint trajectory')
    local waypoints, code =
        pose2jointTrajectory(
        self,
        request.seed.positions,
        request.joint_names,
        g_path,
        request.end_effector_name,
        request.collision_check,
        request.ik_jump_threshold,
        request.dt
    )

    if code < 0 and code ~= error_codes.GOAL_VIOLATES_PATH_CONSTRAINTS then
        response.error_code.val = code
        return true
    end

    local valid, time, pos, vel, acc = generateTrajectory(waypoints, request.dt)
    toc('queryCartesianPathServiceHandler')

    ros.INFO('Generated Trajectory is valid!')
    local move_group = self.end_effector_map[request.end_effector_name]
    local traj = moveit.RobotTrajectory(self.robot_model, move_group)
    local tmp_msg = traj:getRobotTrajectoryMsg()
    tmp_msg.joint_trajectory = ros.Message('trajectory_msgs/JointTrajectory')
    tmp_msg.joint_trajectory.joint_names = request.joint_names
    for i = 1, time:size(1) do
        tmp_msg.joint_trajectory.points[i] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
        tmp_msg.joint_trajectory.points[i].positions = pos[i]
        tmp_msg.joint_trajectory.points[i].velocities = vel[i]
        if acc then
            tmp_msg.joint_trajectory.points[i].accelerations = acc[i]
        end
        tmp_msg.joint_trajectory.points[i].time_from_start = ros.Duration(time[i])
    end
    ros.INFO('Check for limits')
    traj:setRobotTrajectoryMsg(self.robot_state, tmp_msg)
    traj:unwind()
    tmp_msg = traj:getRobotTrajectoryMsg()
    response.solution.joint_names = request.joint_names
    response.solution.points = tmp_msg.joint_trajectory.points
    response.error_code.val = 1
    if code == error_codes.GOAL_VIOLATES_PATH_CONSTRAINTS then
        local filename = string.format('/tmp/%d_velocity_profile', ros.Time.now():toSec())
        print('write log to:', filename)
        plot(tmp_msg.joint_trajectory, filename)
    end
    return true
end

local components = require 'xamlamoveit.components.env'
local LinearCartesianTrajectoryPlanningService, parent =
    torch.class(
    'xamlamoveit.components.LinearCartesianTrajectoryPlanningService',
    'xamlamoveit.components.RosComponent',
    components
)

function LinearCartesianTrajectoryPlanningService:__init(node_handle, joint_monitor)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.info_server = nil
    self.robot_model_loader = nil
    self.robot_model = nil
    self.planning_scene = nil
    self.robot_state = nil
    self.all_group_joint_names = nil
    self.joint_limits = {}
    self.end_effector_map = nil
    self.transformListener = nil
    self.joint_monitor = joint_monitor
    parent.__init(self, node_handle)
end

function LinearCartesianTrajectoryPlanningService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.robot_model)
    self.planning_scene:syncPlanningScene()
    self.end_effector_map = getEndEffectorMoveGroupMap(self.robot_model)
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.all_group_joint_names = self.robot_model:getJointModelGroupNames()
    self.joint_limits.pos, self.joint_limits.vel, self.joint_limits.acc = self.robot_model:getVariableBounds()
    self.variable_names = self.robot_model:getVariableNames():totable() --Note that this also includes mimic joints
    self.joint_limits.pos = createJointValues(self.variable_names, self.joint_limits.pos[{{}, 1}])
    self.joint_limits.vel = createJointValues(self.variable_names, self.joint_limits.vel[{{}, 1}])
    self.joint_limits.acc = createJointValues(self.variable_names, self.joint_limits.acc[{{}, 1}])
    self.transformListener = tf.TransformListener()
    local ready = self.joint_monitor:waitReady(2.0) -- it is not important to have the joint monitor ready at start up
    if not ready then
        ros.WARN('joint states not ready')
    else
        local ok, p = self.joint_monitor:getNextPositionsTensor()
        self.robot_state:setVariablePositions(p, self.joint_monitor:getJointNames())
        self.robot_state:update()
    end
    collectgarbage()
end

function LinearCartesianTrajectoryPlanningService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_cartesian_trajectory',
        srv_spec,
        function(request, response, header)
            return queryCartesianPathServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function LinearCartesianTrajectoryPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming LinearCartesianTrajectoryPlanningService call')
        self.callback_queue:callAvailable()
    end
    if self.joint_monitor:isReady() then
        local joints = self.joint_monitor:getPositionsTensor()
        self.robot_state:setVariablePositions(joints, self.joint_monitor:getJointNames())
        self.robot_state:update()
    end
end

function LinearCartesianTrajectoryPlanningService:onStop()
    ros.WARN('LinearCartesianTrajectoryPlanningService:onStop() NOT IMPLEMENTED')
end

function LinearCartesianTrajectoryPlanningService:onReset()
    self.info_server:shutdown()
end

function LinearCartesianTrajectoryPlanningService:onShutdown()
    self.info_server:shutdown()
end

return LinearCartesianTrajectoryPlanningService
