--- Motion client engine for xamlamoveit
-- In this class the conversion from ros.Messages to xamlamoveit.datatypes are exectued
-- @classmod xamlamoveit.motionLibrary.MotionService
--[[
MotionService.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf

local datatypes = require 'xamlamoveit.datatypes'
local PlanParameters = datatypes.PlanParameters
local TaskSpacePlanParameters = datatypes.TaskSpacePlanParameters

local ac
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local xutils = require 'xamlamoveit.xutils'
local motionLibrary = require 'xamlamoveit.motionLibrary.env'
local MotionService = torch.class('MotionService', motionLibrary)
local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))

local function getServiceConnectionLostErrorMsg(service)
    return string.format(
        'Connection lost to service: %s, Error: "%s"',
        service:getService(),
        error_codes[error_codes.SIGNAL_LOST]
    )
end

function MotionService:__init(node_handle)
    self.node_handle = node_handle
    self.global_veloctiy_scaling = 1.0
    self.global_acceleration_scaling = 1.0
    self.compute_ik_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik', 'xamlamoveit_msgs/GetIKSolution')
    self.compute_ik2_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik2', 'xamlamoveit_msgs/GetIKSolution2')
    self.move_group_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_move_group_interface', 'xamlamoveit_msgs/QueryMoveGroupInterfaces')
    self.execution_action_client = nil
    self.execution_step_action_client = nil
end

function MotionService:shutdown()
    if self.execution_action_client ~= nil then
        self.execution_action_client:shutdown()
    end
    if self.execution_step_action_client ~= nil then
        self.execution_step_action_client:shutdown()
    end
    self.compute_ik_interface:shutdown()
    self.compute_ik2_interface:shutdown()
    self.move_group_interface:shutdown()
    self.node_handle:shutdown()
end

function MotionService:getMoveGroup(move_group_name)
    return motionLibrary.MoveGroup(self, move_group_name)
end

local function poses2MsgArray(points)
    local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
    local result = {}
    if torch.type(points) == 'table' then
        for i, v in ipairs(points) do
            assert(
                torch.isTypeOf(v, datatypes.Pose),
                'points need to be type of datatypes.Pose, but is type: ',
                torch.type(v)
            )
            local msg = ros.Message(pose_msg_spec)
            msg = v:toStampedPoseMsg()
            table.insert(result, msg)
        end
    elseif torch.isTypeOf(points, datatypes.Pose) then
        local msg = ros.Message(pose_msg_spec)
        msg = points:toStampedPoseMsg()
        table.insert(result, msg)
    elseif torch.isTypeOf(points, tf.StampedTransform) then
        local msg = ros.Message(pose_msg_spec)
        msg = points:toStampedPoseMsg()
        table.insert(result, msg)
    else
        error('[poses2MsgArray] unknown type of points parameter: ' .. torch.type(points))
    end
    return result
end

local function stampedMessages2PoseArray(msgs)
    local result = {}
    for i, v in ipairs(msgs) do
        local point = datatypes.Pose()
        local position = v.pose.position
        local ori = v.pose.orientation
        point:setTranslation(torch.Tensor {position.x, position.y, position.z})
        point:setRotation(torch.Tensor {ori.x, ori.y, ori.z, ori.w})
        point.frame = v.frame_id
        table.insert(result, point)
    end
    return result
end

-- get Trajectory from service
local function queryTaskSpaceTrajectory(
    self,
    end_effector_name,
    waypoints,
    max_deviation,
    max_xyz_vel,
    max_xyz_acc,
    max_angular_vel,
    max_angular_acc,
    seed,
    ik_jump_threshold,
    collision_check,
    dt)
    local generate_trajectory_interface =
        self.node_handle:serviceClient(
        'xamlaPlanningServices/query_cartesian_trajectory',
        'xamlamoveit_msgs/GetLinearCartesianTrajectory'
    )
    local request = generate_trajectory_interface:createRequest()
    request.dt = dt > 1.0 and 1 / dt or dt
    request.max_xyz_velocity = max_xyz_vel
    request.max_xyz_acceleration = max_xyz_acc
    request.max_angular_velocity = max_angular_vel
    request.max_angular_acceleration = max_angular_acc
    request.end_effector_name = end_effector_name
    request.ik_jump_threshold = math.max(ik_jump_threshold, 0.0)
    for i, pose in ipairs(waypoints) do
        request.waypoints[i] = pose:toStampedPoseMsg()
    end
    request.seed = ros.Message('xamlamoveit_msgs/JointPathPoint')
    request.seed.positions = seed:getValues()
    request.joint_names = seed:getNames()
    request.max_deviation = max_deviation or 0.0
    request.collision_check = collision_check == nil or collision_check

    local response = nil
    if generate_trajectory_interface:exists() then
        ros.DEBUG('found service: .. %s', generate_trajectory_interface:getService())
        response = generate_trajectory_interface:call(request)
    else
        ros.INFO('could not find service: %s ', generate_trajectory_interface:getService())
    end
    local trajectory, error_code
    if response then
        error_code = response.error_code.val
        if error_code == 1 then
            trajectory = response.solution
        else
            ros.ERROR('No valid taskspace trajectory found: %d, %s', error_code, error_codes[error_code])
        end
        return error_code, trajectory
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(generate_trajectory_interface))
        return error_codes.SIGNAL_LOST
    end
end

function MotionService:queryCartesianPath(waypoints, sample_resolution)
    local compute_cartesian_path_interface =
        self.node_handle:serviceClient(
        '/xamlaPlanningServices/query_cartesian_path',
        'xamlamoveit_msgs/GetLinearCartesianPath'
    )
    local request = compute_cartesian_path_interface:createRequest()
    request.waypoints = poses2MsgArray(waypoints)
    request.num_steps = sample_resolution
    local response = compute_cartesian_path_interface:call(request)
    if response then
        local pathCartesian
        if response.error_code.val == 1 then
            pathCartesian = response.path
        else
            ros.ERROR('No valid Cartesian path found: %d', response.error_code.val)
        end
        return response.error_code.val, stampedMessages2PoseArray(pathCartesian)
    else
        return error_codes.SIGNAL_LOST
    end
end


--- Query inverse kinematic solutions one pose
-- @tparam datatypes.Pose Pose to transform to joint space
-- @tparam datatypes.PlanParameters Plan parameters which defines the limits, settings and move group name
-- @tparam datatypes.JointValues Numerical seed to control joint configuration
-- @tparam string necessary if poses are defined for end effector link
-- @tparam number Attempts to find a solution or each pose
-- @tparam ros.Duration timeout
-- @treturn number error_code
-- @treturn datatypes.JointValues inverse kinematic solutions of pose
-- @treturn string error message
function MotionService:queryIK(pose, parameters, seed_joint_values, end_effector_link, attempts, timeout)
    local seed_joint_values_tensor, seed_joint_values_names
    assert(torch.isTypeOf(parameters, datatypes.PlanParameters),
    string.format("parameters need to be of type [PlanParameters] but is of type [%s]", torch.type(parameters)))
    if torch.isTypeOf(seed_joint_values, datatypes.JointValues) then
        seed_joint_values_tensor = seed_joint_values:getValues()
        seed_joint_values_names = seed_joint_values:getNames()
    elseif torch.isTypeOf(seed_joint_values, torch.DoubleTensor) then
        seed_joint_values_tensor = seed_joint_values
        seed_joint_values_names = parameters.joint_names
    else
        error("seed_joint_values are of unsupported type")
    end
    local poses_msgs = poses2MsgArray(pose)
    local request = self.compute_ik2_interface:createRequest()
    request.group_name = parameters.move_group_name
    request.joint_names = parameters.joint_names
    request.attempts = attempts or 1
    request.timeout = timeout or ros.Duration(0.1)
    request.points = {}
    for i, msg in ipairs(poses_msgs) do
        local tmp = ros.Message('xamlamoveit_msgs/EndEffectorPoses')
        tmp.poses = {msg}
        tmp.link_names = {end_effector_link or ''}
        request.points[#request.points+1] = tmp
    end
    request.const_seed = false
    if seed_joint_values_tensor then
        request.seed.positions = seed_joint_values_tensor
        request.seed.joint_names = seed_joint_values_names
    end
    request.collision_check = true -- this is recomended to be true in all cases
    local response = self.compute_ik2_interface:call(request)
    if response then
        local solutions = {}
        for i , solution in ipairs(response.solutions)  do
            solutions[#solutions + 1] = datatypes.JointValues(datatypes.JointSet(solution.joint_names), solution.positions)
        end
        return response.error_codes,solutions
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(self.compute_ik2_interface))
        return {val = error_codes.SIGNAL_LOST}, nil, getServiceConnectionLostErrorMsg(self.compute_ik2_interface)
    end
end

--- Query all currently available move groups
-- To query the move groups the ros service with the string
-- defined in query_move_group_service is called
-- @treturn table with MoveGroup Ids (string).
-- @treturn table with instances of MoveGroupDescription. For further details please take a look into the documentation of MoveGroupDescription
function MotionService:queryAvailableMoveGroups()

    local response = self.move_group_interface:call()
    if response then
        local details = {}
        local names = {}
        for i, v in ipairs(response.move_group_interfaces) do
            details[v.name] = {
                sub_move_group_ids = v.sub_move_group_ids,
                joint_names = v.joint_names,
                end_effector_names = v.end_effector_names,
                end_effector_link_names = v.end_effector_link_names
            }
            names[i] = v.name
        end
        return names, details
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(self.move_group_interface))
        return {}, {}
    end
end

--- Query all currently available end effectors
-- To query the available end effectors the ros service with the string
-- defined in _query_move_group_service is called an only the relevant
-- information about the endeffector is filtered out
-- @treturn table with end effectors Ids (string).
-- @treturn table with instances of EndEffectorDescription. For further details please take a look into the documentation of EndEffectorDescription
function MotionService:queryAvailableEndEffectors()
    local group_names, group_details = self:queryAvailableMoveGroups()
    if #group_names ~= 0  then
        local details = {}
        local names = {}
        for i, group in ipairs(group_names) do
            for j, end_effector in ipairs(group_details[group].end_effector_names or {}) do
                details[end_effector] = {
                    move_group_name = group,
                    end_effector_link_name = group_details[group].end_effector_link_names[j],
                    joint_names = group_details[group].joint_names
                }
                names[#names + 1] = end_effector
            end
        end
        return names, details
    else
        ros.ERROR('could not find any end effectors')
        return {}, {}
    end
end


--- Query end effector limits from ros param
-- To query the end effector limits the ros param definied in
-- end_effector_limits_param is read out
-- @treturn torch.Tensor velocity limits in xyz
-- @treturn torch.Tensor acceleration limits in xyz
-- @treturn torch.Tensor angular velocity limits
-- @treturn torch.Tensor angular acceleration limits
function MotionService:queryEndEffectorLimits(name)
    local maxXYZVel = 0
    local maxXYZAcc = 0
    local maxAngularVel = 0
    local maxAngularAcc = 0
    local list = self.node_handle:getParamVariable("xamlaJointJogging/end_effector_list")
    for i, item in ipairs(list) do
        if item.name == name then
            maxXYZVel = item.taskspace_xyz_max_vel
            maxXYZAcc = item.taskspace_xyz_max_acc
            maxAngularVel = item.taskspace_angular_max_vel
            maxAngularAcc = item.taskspace_angular_max_acc
            return maxXYZVel, maxXYZAcc, maxAngularVel, maxAngularAcc
        end
    end
    return maxXYZVel, maxXYZAcc, maxAngularVel, maxAngularAcc
end


--- Query joint limits from ros param
-- To query the joint limits the ros param definied in
-- joint_limits_param + joint name + limit name is read out
-- @tparam table of strings specifying joint names
-- @treturn torch.Tensor max, min position limits
-- @treturn torch.Tensor max velocity limits
-- @treturn torch.Tensor max acceleration limits
function MotionService:queryJointLimits(joint_names)
    local max_vel = torch.zeros(#joint_names)
    local max_acc = torch.zeros(#joint_names)
    local max_min_pos = torch.zeros(#joint_names, 2)
    local nh = self.node_handle
    local root_path = 'robot_description_planning/joint_limits'
    for i, name in ipairs(joint_names) do
        local has_pos_param = string.format('/%s/%s/has_position_limits', root_path, name)
        local get_max_pos_param = string.format('/%s/%s/max_position', root_path, name)
        local get_min_pos_param = string.format('/%s/%s/min_position', root_path, name)
        local has_vel_param = string.format('/%s/%s/has_velocity_limits', root_path, name)
        local get_vel_param = string.format('/%s/%s/max_velocity', root_path, name)
        local has_acc_param = string.format('/%s/%s/has_acceleration_limits', root_path, name)
        local get_acc_param = string.format('/%s/%s/max_acceleration', root_path, name)
        if nh:getParamVariable(has_pos_param) then
            max_min_pos[i][1] = nh:getParamVariable(get_max_pos_param)
            max_min_pos[i][2] = nh:getParamVariable(get_min_pos_param)
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end
        if nh:getParamVariable(has_vel_param) then
            max_vel[i] = nh:getParamVariable(get_vel_param)
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end
        if nh:getParamVariable(has_acc_param) then
            max_acc[i] = nh:getParamVariable(get_acc_param)
        else
            max_acc[i] = max_vel[i] * 0.5
            ros.WARN('Joint: %s has no acceleration limit. Will be set to %f', name, max_acc[i])
        end
    end

    return max_min_pos, max_vel, max_acc
end


--- Computes the pose by applying forward kinematics
-- @tparam string name of the move group from which the pose is queried
-- @tparam datatypes.JointValues joint values from which the pose is calculated
-- @tparam[opt] string end effector link is necessary if end effector is not part of the move group but pose should be computed for the end effector
-- @treturn table holds error code {val = code}
-- @treturn ros.Message from type geometry_msgs/PoseStamped
-- @treturn string error message
function MotionService:queryPose(move_group_name, jointvalues, link_name)
    assert(move_group_name)
    assert(torch.isTypeOf(jointvalues, datatypes.JointValues))
    local joint_names = jointvalues:getNames()
    local move_group_pose_interface =
        self.node_handle:serviceClient('xamlaMoveGroupServices/query_fk', 'xamlamoveit_msgs/GetFKSolution')
    local request = move_group_pose_interface:createRequest()
    request.group_name = move_group_name
    request.end_effector_link = link_name or ''
    request.points[1] = ros.Message('xamlamoveit_msgs/JointPathPoint')
    request.points[1].positions = jointvalues.values
    for i, v in ipairs(joint_names) do
        request.joint_names[i] = v
    end
    local response = move_group_pose_interface:call(request)
    --check order of joint names
    if response then
        if response.error_codes[1].val == 1 then
            return response.error_codes[1], response.solutions[1], response.error_msgs[1]
        end
    end
end


--- Query the poses from joint path points by applying forward kinematics
-- @tparam string name of the move group from which the pose is queried
-- @tparam table with datatypes.JointValues. Joint path from which the poses are calculated
-- @tparam[opt] string end effector link is necessary if end effector is not part of the move group but pose should be computed for the end effector
-- @treturn table holds error code {val = code}
-- @treturn table with ros.Messages from type geometry_msgs/PoseStamped
-- @treturn table with strings holding error messages
function MotionService:queryPoses(move_group_name, jointvalues_array, link_name)
    assert(move_group_name)
    local move_group_pose_interface =
        self.node_handle:serviceClient('xamlaMoveGroupServices/query_fk', 'xamlamoveit_msgs/GetFKSolution')
    local request = move_group_pose_interface:createRequest()
    request.group_name = move_group_name
    request.end_effector_link = link_name or ''
    for j = 1, #jointvalues_array do
        assert(torch.isTypeOf(jointvalues_array[j], datatypes.JointValues))
        local joint_names = jointvalues_array[j]:getNames()
        request.points[j] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.points[j].positions = jointvalues_array[j].values
        for i, v in ipairs(joint_names) do
            request.joint_names[i] = v
        end
    end
    local response = move_group_pose_interface:call(request)
    if response then
        --check order of joint names
        return response.error_codes, response.solutions, response.error_msgs
    else
        return {val = error_codes.SIGNAL_LOST}, nil, getServiceConnectionLostErrorMsg(move_group_pose_interface)
    end
end


--- Query joint states by calling the providing ros service
-- To query the joint states the ros service with the name
-- defined in query_joint_states_service is called
-- @tparam table of strings specifying joint names
-- @treturn table field 'val' specifies the error code
-- @treturn torch.Tensor max velocity limits
-- @treturn torch.Tensor max acceleration limits
function MotionService:queryJointState(joint_names)
    local move_group_position_interface =
        self.node_handle:serviceClient(
        'xamlaMoveGroupServices/query_move_group_current_position',
        'xamlamoveit_msgs/GetCurrentJointState'
    )
    local request = move_group_position_interface:createRequest()
    for i, v in ipairs(joint_names) do
        request.joint_names[i] = v
    end
    local response = move_group_position_interface:call(request)
    --check order of joint names
    if response then
        return {val = 1}, response.current_joint_position.position
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(move_group_position_interface))
        return {val = error_codes.SIGNAL_LOST}, torch.Tensor()
    end
end


function MotionService:queryStateCollision(move_group_name, joint_names, points)
    local collision_check_interface =
        self.node_handle:serviceClient(
        '/xamlaMoveGroupServices/query_joint_position_collision_check',
        'xamlamoveit_msgs/QueryJointStateCollisions'
    )
    local request = collision_check_interface:createRequest()
    request.move_group_name = move_group_name
    request.joint_names = joint_names
    for i = 1, #points do
        request.points[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.points[i].positions = points[i]
    end
    local response
    if collision_check_interface:exists() then
        ros.DEBUG('found service: .. ' .. collision_check_interface:getService())
        response = collision_check_interface:call(request)
        if response then
            return response.success, response.in_collision, response.error_codes, response.messages
        else
            return false, false, {val = error_codes.SIGNAL_LOST}, getServiceConnectionLostErrorMsg(
                collision_check_interface
            )
        end
    else
        local error_msg = getServiceConnectionLostErrorMsg(collision_check_interface)
        ros.ERROR(error_msg)
        return false, true, {val = error_codes.SIGNAL_LOST}, {error_msg}
    end
end


-- get Path from service
local function queryJointPath(self, move_group_name, joint_names, waypoints, planning_time, planning_attempts, goal_tolerance)
    local generate_path_interface =
        self.node_handle:serviceClient('/xamlaPlanningServices/query_joint_path_parameterized', 'xamlamoveit_msgs/GetMoveItJointPathWithParameters')
    local request = generate_path_interface:createRequest()
    request.group_name = move_group_name
    request.joint_names = joint_names
    request.parameters.has_planning_attempts = planning_attempts ~= nil
    request.parameters.planning_attempts = planning_attempts or 5
    request.parameters.has_planning_time = planning_time ~= nil
    request.parameters.planning_time = planning_time or ros.Duration(10)
    request.parameters.has_goal_tolerance = goal_tolerance ~= nil
    request.parameters.goal_tolerance = goal_tolerance or 1e-5

    for i = 1, waypoints:size(2) do
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{{}, i}]
    end

    local response
    if generate_path_interface:exists() then
        response = generate_path_interface:call(request)
        ros.DEBUG('found service: .. ' .. generate_path_interface:getService())
    else
        ros.INFO('could not find service: .. ' .. generate_path_interface:getService())
    end
    if response then
        --check order of joint names
        if response.error_code.val > 0 then
            local path = torch.Tensor(#response.path, waypoints[{{}, 1}]:size(1))
            for i = 1, #response.path do
                path[i]:copy(response.path[i].positions)
            end
            return response.error_code.val, path
        else
            return response.error_code.val
        end
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(generate_path_interface))
        return error_codes.SIGNAL_LOST
    end
end


-- get Trajectory from service
local function queryJointTrajectory(self, joint_names, waypoints, max_vel, max_acc, max_deviation, dt)
    local generate_trajectory_interface =
        self.node_handle:serviceClient(
        'xamlaPlanningServices/query_joint_trajectory',
        'xamlamoveit_msgs/GetOptimJointTrajectory'
    )
    local request = generate_trajectory_interface:createRequest()
    request.max_deviation = max_deviation
    request.joint_names = joint_names
    request.dt = dt > 1.0 and 1 / dt or dt
    request.max_velocity = max_vel
    request.max_acceleration = max_acc
    for i = 1, waypoints:size(1) do
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{i, {}}]
    end
    local response = nil
    if generate_trajectory_interface:exists() then
        ros.DEBUG('found service: .. %s', generate_trajectory_interface:getService())
        response = generate_trajectory_interface:call(request)
    else
        ros.INFO('could not find service: %s ', generate_trajectory_interface:getService())
    end
    local trajectory, error_code
    if response then
        error_code = response.error_code.val
        if error_code == 1 then
            trajectory = response.solution
        else
            ros.ERROR('No valid joint trajectory found: %d, %s', error_code, error_codes[error_code])
        end
        return error_code, trajectory
    else
        ros.ERROR(getServiceConnectionLostErrorMsg(generate_trajectory_interface))
        return error_codes.SIGNAL_LOST
    end
end


--- Trajectory which should be executed supervised
-- @tparam ros.Message
-- @tparam boolean If True check for collision while executing
-- @tparam[opt] function callback.
-- @treturn motionLibrary.SteppedMotionClient handle
function MotionService:executeSupervisedJointTrajectory(traj, check_collision, done_cb)
    if self.execution_step_action_client == nil then
        self.execution_step_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/StepwiseMoveJ', 'moveJ_step_action', self.node_handle)
        self.execution_step_action_client:waitForServer(ros.Duration(2.5))
    elseif not self.execution_step_action_client:isServerConnected() then
        self.execution_step_action_client:shutdown()
        self.execution_step_action_client = nil
        self.execution_step_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/StepwiseMoveJ', 'moveJ_step_action', self.node_handle)
        self.execution_step_action_client:waitForServer(ros.Duration(2.5))
    end
    local action_client = self.execution_step_action_client
    ros.spinOnce()
    assert(action_client:isServerConnected(), 'could not reach moveJ_step_action')

    local g = action_client:createGoal()
    g.trajectory.joint_names = traj.joint_names
    g.trajectory.points = traj.points
    g.check_collision = check_collision or false

    action_client:sendGoal(g, done_cb)

    local controller_handle = motionLibrary.SteppedMotionClient(action_client)

    return controller_handle
end

--- Executes a joint trajectory asynchronous
-- calles action
-- @tparam ros.Message
-- @tparam boolean If True check for collision while executing
-- @tparam[opt] function callback.
-- @treturn ros.ActionLib.SimpleActionClient handle
function MotionService:executeJointTrajectoryAsync(traj, check_collision, done_cb)
    if self.execution_action_client == nil then
        ros.WARN('Action Client does not exist yet')
        self.execution_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
        self.execution_action_client:waitForServer(ros.Duration(1))
    elseif not self.execution_action_client:isServerConnected() then
        ros.WARN('Action Client not reachable recreating client')
        self.execution_action_client:shutdown()
        self.execution_action_client = nil
        self.execution_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
        self.execution_action_client:waitForServer(ros.Duration(1))
    end

    local action_client = self.execution_action_client

    ros.spinOnce()
    assert(action_client:isServerConnected(), 'could not reach moveJ_action')

    local g = action_client:createGoal()
    g.trajectory.joint_names = traj.joint_names
    g.trajectory.points = traj.points
    g.check_collision = check_collision or false

    action_client:sendGoal(g, done_cb)
    return action_client
end


--- Executes a joint trajectory
-- calles action client
-- @tparam ros.Message
-- @tparam boolean If True check for collision while executing
-- @treturn boolean True if successfully executed trajectory
-- @treturn string error message
function MotionService:executeJointTrajectory(traj, check_collision)
    if self.execution_action_client == nil then
        self.execution_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
        self.execution_action_client:waitForServer(ros.Duration(2.5))
    elseif not self.execution_action_client:isServerConnected() then
        self.execution_action_client:shutdown()
        self.execution_action_client = nil
        self.execution_action_client =
            actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
        self.execution_action_client:waitForServer(ros.Duration(2.5))
    end
    local action_client = self.execution_action_client
    local g = action_client:createGoal()
    g.trajectory.joint_names = traj.joint_names
    g.trajectory.points = traj.points
    g.check_collision = check_collision or false

    if action_client:isServerConnected() then
        local state, state_msg = action_client:sendGoalAndWait(g)
        local answere = action_client:getResult()
        ros.INFO('moveJ_action returned: %s, [%d]', state_msg, state)
        if answere == nil then
            answere = {result = error_codes.SIGNAL_LOST}
        end
        if state == SimpleClientGoalState.SUCCEEDED then
            return true, state_msg
        else
            return false, string.format('%s, Result: %s (%d)', state_msg, error_codes[answere.result], answere.result)
        end
    else
        ros.ERROR('could not reach moveJ_action')
        return false, 'could not reach moveJ_action'
    end
end

local function planMoveP_1(self, start, goal, parameters)
    ros.INFO('planMoveP_1')
    assert(torch.isTypeOf(start, datatypes.Pose))
    assert(torch.isTypeOf(goal, datatypes.Pose))
    assert(
        torch.isTypeOf(parameters, datatypes.PlanParameters),
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local js_error_code, seed = self:queryJointState(parameters.joint_names)
    if js_error_code ~= 1 then
        return false
    end
    local ik_error_codes, start = self:queryIK(start, parameters, seed)
    if ik_error_codes then
        if ik_error_codes[1].val ~= 1 then
            return false
        end
    else
        return false
    end
    ik_error_codes, goal = self:queryIK(goal, parameters, start.solutions[1])
    if ik_error_codes then
        if ik_error_codes[1].val ~= 1 then
            ros.ERROR('goal suc ', error_codes[ik_error_codes[1].val])
            return false
        end
    else
        return false
    end
    return self:planMoveJoints(torch.cat(start[1]:getValues(), goal[1]:getValues(), 2):t(), parameters)
end

local function planMoveP_2(self, waypoints, parameters)
    ros.INFO('planMoveP_2')
    assert(torch.type(waypoints) == 'table')
    assert(
        torch.isTypeOf(parameters, datatypes.PlanParameters),
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local result = {}
    local js_error_code, seed = self:queryJointState(parameters.joint_names)

    if js_error_code.val ~= 1 then
        ros.ERROR('Error code: ' .. error_codes[js_error_code[1].val])
        return false
    end
    for i, v in ipairs(waypoints) do
        local suc, start = self:queryIK(v, parameters, seed)
        if suc then
            if suc[1].val ~= 1 then
                ros.ERROR('Error code: ' .. error_codes[suc[1].val])
                return false
            end
        else
            return false
        end
        table.insert(result, start[1]:getValues())
        seed = start[1]
    end
    return self:planMoveJoints(torch.cat(result, 2):t(), parameters)
end

function MotionService:planMoveP(_1, _2, _3)
    ros.INFO('planMoveP')
    if torch.isTypeOf(_1, datatypes.Pose) then
        return planMoveP_1(self, _1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return planMoveP_2(self, _1, _2)
    end
end

local function planJointPath_1(self, start, goal, parameters)
    assert(torch.isTypeOf(start, torch.DoubleTensor))
    assert(torch.isTypeOf(goal, torch.DoubleTensor))
    assert(torch.isTypeOf(parameters, datatypes.PlanParameters))
    assert(start:size(1) == goal:size(1))
    return queryJointPath(self, parameters.move_group_name, parameters.joint_names, torch.cat({start, goal}, 2))
end

local function planJointPath_2(self, waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.isTypeOf(parameters, datatypes.PlanParameters))
    return queryJointPath(self, parameters.move_group_name, parameters.joint_names, torch.cat(waypoints, 2))
end

function MotionService:planJointPath(_1, _2, _3)
    if torch.isTypeOf(_1, torch.DoubleTensor) then
        return planJointPath_1(self, _1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return planJointPath_2(self, _1, _2)
    end
end

function MotionService:planCartesianPath(waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.isTypeOf(parameters, datatypes.PlanParameters))
    local error_code, path = self:queryCartesianPath(waypoints, #waypoints)
    return error_code, path
end

--- Plans trajectory from a joint path
-- @tparam table of joint values the trajectory must reach
-- @tparam datatypes.PlanParameters plan parameters which defines the limits, settings and move group name
-- @treturn number error_code (SUCCEEDED = 1)
-- @tretrun ros.Message trajectory_msgs/JointTrajectory. Planned joint trajectory which reach the poses defined in path under the constraints of parameters
function MotionService:planMoveJoints(path, parameters)
    ros.DEBUG('planMoveJoints')
    assert(torch.isTypeOf(path, torch.DoubleTensor))
    assert(torch.isTypeOf(parameters, datatypes.PlanParameters))
    assert(
        path:size(2) == #parameters.joint_names,
        string.format(
            'path dim and number of provided joint names do not match: %d vs %d',
            path:size(2),
            #parameters.joint_names
        )
    )

    local error_code, trajectory =
        queryJointTrajectory(
        self,
        parameters.joint_names,
        path,
        parameters.max_velocity * self.global_veloctiy_scaling,
        parameters.max_acceleration * self.global_acceleration_scaling,
        parameters.max_deviation,
        parameters.dt or 0.008
    )
    return error_code, trajectory
end


--- Create PlanParameters from user defined and/or queried inputs
-- @tparam string name of move group
-- @tparam table of strings holding joint names
-- @tparam[opt] torch.tensor max position joint limits
-- @tparam[opt] torch.tensor min position joint limits
-- @tparam[opt] torch.tensor max veloctiy joint limits
-- @tparam[opt] torch.tensor max acceleration joint limits
-- @tparam[opt] number maximal deviation from waypoints
-- @tparam[opt] boolean check for collision if True
-- @treturn datatypes.PlanParameters Instance of plan parameters with automatically queried and/or user defined values
function MotionService:getDefaultPlanParameters(
    move_group_name,
    joint_names,
    max_positions,
    min_positions,
    max_velocity,
    max_acceleration,
    max_deviation,
    collision_check,
    dt)
    local max_min_pos
    if not max_velocity and not max_acceleration then
        max_min_pos, max_velocity, max_acceleration = self:queryJointLimits(joint_names)
        max_positions = max_min_pos[{{}, 1}]:clone()
        min_positions = max_min_pos[{{}, 2}]:clone()
    end
    return PlanParameters.new(
        move_group_name,
        joint_names,
        max_positions,
        min_positions,
        max_velocity,
        max_acceleration,
        collision_check,
        max_deviation,
        dt
    )
end


--- Plans trajectory with linear movements from a cartesian path
-- @tparam table of datatypes.Pose
-- @tparam datatypes.TaskSpacePlanParameters plan parameters which defines the limits, settings and move group name
-- @treturn number error_code (SUCCEEDED = 1)
-- @tretrun ros.Message trajectory_msgs/JointTrajectory. Planned joint trajectory which reach the poses defined in path under the constraints of parameters
function MotionService:planMoveLinear(joint_value_seed, path, parameters)
    ros.DEBUG('planMoveLinear')
    assert(torch.isTypeOf(joint_value_seed, datatypes.JointValues))
    assert(
        torch.type(path) == 'table',
        string.format('[MotionService:planMoveLinear] path is not of type [table] bus [%s]', torch.type(path))
    )
    assert(
        torch.isTypeOf(parameters, datatypes.TaskSpacePlanParameters),
        string.format(
            'Input argument exception: parameters need to be from type [TaskSpacePlanParameters] but is from type %s',
            torch.type(parameters)
        )
    )

    local error_code, trajectory =
        queryTaskSpaceTrajectory(
        self,
        parameters.end_effector_name,
        path,
        parameters.max_deviation,
        parameters.max_xyz_velocity * self.global_veloctiy_scaling,
        parameters.max_xyz_acceleration * self.global_acceleration_scaling,
        parameters.max_angular_velocity * self.global_veloctiy_scaling,
        parameters.max_angular_acceleration * self.global_acceleration_scaling,
        joint_value_seed,
        parameters.ik_jump_threshold,
        parameters.collision_check,
        parameters.dt
    )
    return error_code, trajectory
end

function MotionService:getDefaultTaskSpacePlanParameters(
    end_effector_name,
    max_deviation,
    max_xyz_velocity,
    max_xyz_acceleration,
    max_angular_velocity,
    max_angular_acceleration,
    collision_check,
    ik_jump_threshold,
    dt)
    return TaskSpacePlanParameters.new(
        end_effector_name or 'EE_manipulator',
        max_deviation or 0.0,
        max_xyz_velocity or 0.01,
        max_xyz_acceleration or 0.04,
        max_angular_velocity or math.rad(1.5),
        max_angular_acceleration or math.rad(1.5) * 4,
        collision_check or true,
        ik_jump_threshold or 0.1,
        dt or 0.01
    )
end

function MotionService:emergencyStop(enable)
    assert(toch.type(enable) == 'boolean')
    local enable = enable == nil or enable
    local set_emergency_stop = self.node_handle:serviceClient('EmergencySTOP/set_emergency_stop', 'std_srvs/SetBool')
    local g = set_emergency_stop:createRequest()
    g.data = enable
    local response = set_emergency_stop:call(g)
    if response then
        return response.success, response.message
    else
        return error_codes.SIGNAL_LOST, getServiceConnectionLostErrorMsg(set_emergency_stop)
    end
end

return motionLibrary.MotionService
