local torch = require 'torch'
local ros = require 'ros'
local PlanParameters = require 'xamlamoveit.components.PlanParameters'
local datatypes = require 'xamlamoveit.components.datatypes'

local ac
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local motionLibrary = require 'xamlamoveit.motionLibrary.env'
local MotionService = torch.class('MotionService', motionLibrary)

function MotionService:__init(node_handle)
    self.node_handle = node_handle
    self.global_veloctiy_scaling = 1.0
    self.global_acceleration_scaling = 1.0
    local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
    self.compute_ik_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik', srv_spec)
    self.execution_action_client = nil
end

function MotionService:shutdown()
    if  self.execution_action_client ~= nil then
        self.execution_action_client:shutdown()
    end

    self.compute_ik_interface:shutdown()
    self.node_handle:shutdown()
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
            msg.pose.position.x = v.translation[1]
            msg.pose.position.y = v.translation[2]
            msg.pose.position.z = v.translation[3]
            msg.pose.orientation.x = v.rotation[1]
            msg.pose.orientation.y = v.rotation[2]
            msg.pose.orientation.z = v.rotation[3]
            msg.pose.orientation.w = v.rotation[4]
            msg.header.frame_id = v.frame
            table.insert(result, msg)
        end
    elseif torch.isTypeOf(points, datatypes.Pose) then
        local msg = ros.Message(pose_msg_spec)
        msg.pose.position.x = points.translation[1]
        msg.pose.position.y = points.translation[2]
        msg.pose.position.z = points.translation[3]
        msg.pose.orientation.x = points.rotation[1]
        msg.pose.orientation.y = points.rotation[2]
        msg.pose.orientation.z = points.rotation[3]
        msg.pose.orientation.w = points.rotation[4]
        msg.header.frame_id = points.frame or ''
        table.insert(result, msg)
    else
        error('[poses2MsgArray] unknown type of points parameter: ' .. torch.type(points))
    end
    return result
end

function MotionService:queryCartesianPath(waypoints, sample_resolution)
    local compute_cartesian_path_interface =
        self.node_handle:serviceClient(
        '/xamlaPlanningServices/query_cartesian_path',
        'xamlamoveit_msgs/GetLinearCartesianPath'
    )
    local request = compute_cartesian_path_interface:createRequest()
    request.waypoints = poses2MsgArray(waypoints)
    --print(request.points)
    request.num_steps = sample_resolution
    local response = compute_cartesian_path_interface:call(request)
    return response
end

function MotionService:queryIK(pose, parameters, seed_joint_values, end_effector_link, attempts, timeout)
    --local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
    --local compute_ik_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik', srv_spec)
    local request = self.compute_ik_interface:createRequest()
    request.group_name = parameters.move_group_name
    request.joint_names = parameters.joint_names
    request.end_effector_link = end_effector_link or ''
    request.attempts = attempts or 5
    request.timeout = timeout or ros.Duration(0.5)
    request.points = poses2MsgArray(pose)
    request.const_seed = false
    if seed_joint_values then
        request.seed.positions = seed_joint_values
    end
    request.collision_check = true -- gemeint ist hier die self collision die sollte immer on sein
    local response = self.compute_ik_interface:call(request)
    return response.error_codes, response.solutions
end

--get Avalable move groups
function MotionService:queryAvailableMovegroups()
    local move_group_interface =
        self.node_handle:serviceClient(
        'xamlaMoveGroupServices/query_move_group_interface',
        'xamlamoveit_msgs/QueryMoveGroupInterfaces'
    )
    local response = move_group_interface:call()
    local details = {}
    local names = {}
    for i, v in ipairs(response.move_group_interfaces) do
        details[v.name] = {
            sub_move_group_ids = v.sub_move_group_ids,
            joint_names = v.joint_names,
            end_effector_names = v.end_effector_names
        }
        names[i] = v.name
    end
    return names, details
end

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

-- get current Position of movegroup
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
    if response.error_codes[1].val == 1 then
        return response.error_codes[1], response.solutions[1], response.error_msgs[1]
    end
end

-- get current Position of movegroup
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
        request.points[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.points[i].positions = jointvalues_array[i].values
        for i, v in ipairs(joint_names) do
            request.joint_names[i] = v
        end
    end
    local response = move_group_pose_interface:call(request)
    --check order of joint names
    return response.error_codes, response.solutions, response.error_msgs
end

-- get current Position of movegroup
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
    return response.current_joint_position.position
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
        print('queryStateCollision', points[i])
        request.points[i].positions = points[i]
    end
    --print(request)
    local response
    if collision_check_interface:exists() then
        ros.INFO('found service: .. ' .. collision_check_interface:getService())
        response = collision_check_interface:call(request)
        --print(response)
        return response.success, response.in_collision, response.error_codes, response.messages
    else
        ros.INFO('could not find service: .. ' .. collision_check_interface:getService())
    end
    return false
end

-- get Path from service
local function queryJointPath(self, move_group_name, joint_names, waypoints)
    local generate_path_interface =
        self.node_handle:serviceClient('xamlaPlanningServices/query_joint_path', 'xamlamoveit_msgs/GetMoveItJointPath')
    local request = generate_path_interface:createRequest()
    request.group_name = move_group_name
    request.joint_names = joint_names
    for i = 1, waypoints:size(2) do
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{{}, i}]
    end

    local response
    if generate_path_interface:exists() then
        response = generate_path_interface:call(request)
        ros.INFO('found service: .. ' .. generate_path_interface:getService())
    else
        ros.INFO('could not find service: .. ' .. generate_path_interface:getService())
    end

    --check order of joint names
    if response.error_code.val > 0 then
        local path = torch.Tensor(#response.path, waypoints[{{}, 1}]:size(1))
        for i = 1, #response.path do
            path[i]:copy(response.path[i].positions)
        end
        return true, path
    else
        return false
    end
end

-- get Trajectory from service
local function queryJointTrajectory(self, joint_names, waypoints, max_vel, max_acc, max_deviation, dt)
    ros.INFO("queryJointTrajectory")
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
    ros.WARN('using Sampling dt: ' .. request.dt)
    for i = 1, waypoints:size(1) do
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{i, {}}]
    end
    local response = nil
    if generate_trajectory_interface:exists() then
        ros.INFO('found service: ..' .. generate_trajectory_interface:getService())
        response = generate_trajectory_interface:call(request)
    else
        ros.INFO('could not find service: ..' .. generate_trajectory_interface:getService())
    end

    return response
end

function MotionService:executeJointTrajectoryAsync(traj, check_collision, cancelToken)
    if  self.execution_action_client ~= nil then
        self.execution_action_client:shutdown()
        self.execution_action_client = nil
    end
    self.execution_action_client = actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
    local action_client =  self.execution_action_client
    local g = action_client:createGoal()
    g.trajectory.joint_names = traj.joint_names
    g.trajectory.points = traj.points
    g.check_collision = check_collision or false
    cancelToken.done = false
    local function action_done(state, result)
        ros.INFO('actionDone')
        ros.INFO('Finished with states: %s (%d)', SimpleClientGoalState[state], state)
        ros.INFO('Result:\n%s', result)
        cancelToken.done = true
    end

    local function action_active()
        ros.INFO('MotionService Action_active')
    end

    local function action_feedback(feedback)
        ros.INFO('Action_feedback')
    end

    ros.spinOnce()
    action_client:waitForServer(ros.Duration(2.5))
    if action_client:isServerConnected() then
        action_client:sendGoal(g, action_done, action_active, action_feedback)
        return true, action_client
    else
        ros.ERROR('could not reach moveJ_action')
        return false
    end
end

function MotionService:executeJointTrajectory(traj, check_collision)
    local cancelToken = {done = false}
    local dt = ros.Rate(25)
    if self:executeJointTrajectoryAsync(traj, check_collision, cancelToken) then
        while ros.ok() and cancelToken.done == false do
            ros.spinOnce()
            dt:sleep()
        end
        return true
    else
        return false
    end
end

local function planMoveCartesian_1(self, start, goal, parameters)
    ros.INFO("planMoveCartesian_1")
    assert(torch.isTypeOf(start, datatypes.Pose))
    assert(torch.isTypeOf(goal, datatypes.Pose))
    assert(
        torch.type(parameters) == 'PlanParameters',
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local seed = self:queryJointState(parameters.joint_names)
    local suc, start = self:queryIK(start, parameters, seed)
    if suc[1].val ~= 1 then
        print('start suc ', suc)
        return false
    end
    suc, goal = self:queryIK(goal, parameters, start.solutions[1])
    if suc[1].val ~= 1 then
        print('goal suc ', suc.error_code.val)
        return false
    end
    return self:planMoveJoint(torch.cat(start[1].positions, goal[1].positions,2):t(), parameters)
end

local function planMoveCartesian_2(self, waypoints, parameters)
    ros.INFO("planMoveCartesian_2")
    assert(torch.type(waypoints) == 'table')
    assert(
        torch.type(parameters) == 'PlanParameters',
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local result = {}
    local seed = self:queryJointState(parameters.joint_names)
    for i, v in ipairs(waypoints) do
        local suc, start = self:queryIK(v, parameters, seed)
        if suc[1].val ~= 1 then
            print('start suc ', suc)
            return false
        end
        table.insert(result, start[1].positions)
        seed = start[1].positions
    end
    return self:planMoveJoint(torch.cat(result,2):t(), parameters)
end

function MotionService:planMoveCartesian(_1, _2, _3)
    ros.INFO("planMoveCartesian")
    if torch.isTypeOf(_1, datatypes.Pose) then
        return planMoveCartesian_1(self, _1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return planMoveCartesian_2(self, _1, _2)
    end
end

--IJointPath Plan(JointValues start, JointValues goal, PlanParameters parameters);

local function planJointPath_1(self, start, goal, parameters)
    assert(torch.isTypeOf(start, torch.DoubleTensor))
    assert(torch.isTypeOf(goal, torch.DoubleTensor))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(start:size(1) == goal:size(1))
    return queryJointPath(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        torch.cat({start, goal}, 2)
    )
end

local function planJointPath_2(self, waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.type(parameters) == 'PlanParameters')
    return queryJointPath(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        torch.cat(waypoints, 2)
    )
end

function MotionService:planJointPath(_1, _2, _3)
    if torch.isTypeOf(_1, torch.DoubleTensor) then
        return planJointPath_1(self, _1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return planJointPath_2(self, _1, _2)
    end
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

function MotionService:planCartesianPath(waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.type(parameters) == 'PlanParameters')
    local pathCartesian
    local res = self:queryCartesianPath(waypoints, #waypoints)
    if res.error_code.val == 1 then
        pathCartesian = res.path
    else
        ros.ERROR("No valid Cartesian path found: %d", res.error_code.val)
    end
    return res.error_code.val, stampedMessages2PoseArray(pathCartesian)
end

--IJointTrajectory PlanMoveJoint(IJointPath path, PlanParameters parameters);
function MotionService:planMoveJoint(path, parameters)
    ros.INFO("planMoveJoint")
    assert(torch.isTypeOf(path, torch.DoubleTensor))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(
        path:size(2) == #parameters.joint_names,
        string.format(
            'path dim and number of provided joint names do not match: %d vs %d',
            path:size(2),
            #parameters.joint_names
        )
    )
    local trajectory
    local res =
        queryJointTrajectory(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        path,
        parameters.max_velocity * self.global_veloctiy_scaling,
        parameters.max_acceleration * self.global_acceleration_scaling,
        0.0,
        parameters.dt or 0.008
    )
    if res.error_code.val == 1 then
        trajectory = res.solution
    else
        ros.ERROR("No valid joint trajectory found: %d", res.error_code.val)
    end
    return res.error_code.val, trajectory
end

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
        max_positions = max_min_pos[{1, {}}]:clone()
        min_positions = max_min_pos[{2, {}}]:clone()
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

function MotionService:emergencyStop(enable)
    assert(toch.type(enable) == 'boolean')
    local enable = enable == nil or enable
    local set_emergency_stop = self.node_handle:serviceClient('EmergencySTOP/set_emergency_stop', 'std_srvs/SetBool')
    local g = set_emergency_stop:createRequest()
    g.data = enable
    local response = set_emergency_stop:call(g)
    return response.success, response.message
end

return MotionService
