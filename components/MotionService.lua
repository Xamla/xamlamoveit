local torch = require 'torch'
local ros = require 'ros'
local PlanParameters = require 'xamlamoveit.components.PlanParameters'
local datatypes = require 'xamlamoveit.components.datatypes'

local ac
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local components = require 'xamlamoveit.components.env'
local MotionService = torch.class('MotionService', components)

function MotionService:__init(node_handle)
    self.node_handle = node_handle
    self.global_veloctiy_scaling = 1.0
    self.global_acceleration_scaling = 1.0
    local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
    self.compute_ik_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik', srv_spec)
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
        print("else")
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

function MotionService:query_cartesian_path(
    move_group_name,
    joint_names,
    end_effector_link,
    waypoints,
    sampleResolution,
    max_deviation,
    check_collision)
    local compute_cartesian_path_interface =
        self.node_handle:serviceClient(
        '/xamlaPlanningServices/query_cartesian_path',
        'xamlamoveit_msgs/GetLinearCartesianPath'
    )
    local request = compute_cartesian_path_interface:createRequest()
    request.waypoints = poses2MsgArray(waypoints)
    --print(request.points)
    request.num_steps = sampleResolution
    local response = compute_cartesian_path_interface:call(request)
    return response
end

local function query_ik_call(self, pose, parameters, seed_joint_values, end_effector_link)
    --local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
    --local compute_ik_interface = self.node_handle:serviceClient('xamlaMoveGroupServices/query_ik', srv_spec)
    local request = self.compute_ik_interface:createRequest()
    request.group_name = parameters.move_group_name
    request.joint_names = parameters.joint_names
    request.end_effector_link = end_effector_link or ''
    request.points = poses2MsgArray(pose)
    if seed_joint_values then
        request.seed.positions = seed_joint_values
    end
    request.check_collision = true
    print(request)
    local response = self.compute_ik_interface:call(request)
    return response.error_code.val, response.solution
end

--get Avalable move groups
function MotionService:query_available_movegroups()
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

function MotionService:query_joint_limits(joint_names)
    local max_vel = torch.zeros(#joint_names)
    local max_acc = torch.zeros(#joint_names)
    local nh = self.node_handle
    local root_path = 'robot_description_planning/joint_limits'
    for i, name in ipairs(joint_names) do
        local has_vel_param = string.format('/%s/%s/has_velocity_limits', root_path, name)
        local get_vel_param = string.format('/%s/%s/max_velocity', root_path, name)
        local has_acc_param = string.format('/%s/%s/has_acceleration_limits', root_path, name)
        local get_acc_param = string.format('/%s/%s/max_acceleration', root_path, name)
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

    return max_vel, max_acc
end

-- get current Position of movegroup
function MotionService:query_current_pose(move_group_name, jointvalues, link_name)
    assert(move_group_name)
    assert(torch.isTypeOf(jointvalues, datatypes.JointValues))
    local joint_names = jointvalues:getNames()
    local move_group_pose_interface =
        self.node_handle:serviceClient('xamlaMoveGroupServices/query_fk', 'xamlamoveit_msgs/GetFKSolution')
    local request = move_group_pose_interface:createRequest()
    request.group_name = move_group_name
    request.end_effector_link = link_name or ''
    request.point.positions = jointvalues.values
    for i, v in ipairs(joint_names) do
        request.joint_names[i] = v
    end
    local response = move_group_pose_interface:call(request)
    --check order of joint names
    if response.error_code.val == 1 then
        return response.solution
    end
end

-- get current Position of movegroup
function MotionService:query_joint_state(joint_names)
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

-- get Path from service
local function query_joint_path(self, move_group_name, joint_names, waypoints, num_steps, max_deviation, with_moveit)
    local generate_path_interface =
        self.node_handle:serviceClient('xamlaPlanningServices/query_joint_path', 'xamlamoveit_msgs/GetOptimJointPath')
    local with_moveit = with_moveit or false
    local request = generate_path_interface:createRequest()
    request.max_deviation = max_deviation
    request.group_name = move_group_name
    request.joint_names = joint_names
    if num_steps > 1.0 then
        request.num_steps = num_steps
    else
        request.num_steps = math.ceil(waypoints:size(2) * num_steps)
    end
    request.moveit_adaptive_plan = with_moveit
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
        local path = torch.Tensor(num_steps, waypoints[{{}, 1}]:size(1))
        for i = 1, num_steps do
            path[i]:copy(response.path[i].positions)
        end
        return true, path
    else
        return false
    end
end

-- get Trajectory from service
local function query_joint_trajectory(self, move_group_name, joint_names, waypoints, max_vel, max_acc, max_deviation, dt)
    local generate_trajectory_interface =
        self.node_handle:serviceClient(
        'xamlaPlanningServices/query_joint_trajectory',
        'xamlamoveit_msgs/GetOptimJointTrajectory'
    )
    local request = generate_trajectory_interface:createRequest()
    request.max_deviation = max_deviation
    request.group_name = move_group_name
    request.joint_names = joint_names
    request.dt = dt
    request.max_velocity = max_vel
    request.max_acceleration = max_acc

    for i = 1, waypoints:size(1) do
        print("query_joint_trajectory waypoint index: ", i)
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{i, {}}]
    end
    local response = nil
    if generate_trajectory_interface:exists() then
        response = generate_trajectory_interface:call(request)
        ros.INFO('found service: ..' .. generate_trajectory_interface:getService())
    else
        ros.INFO('could not find service: ..' .. generate_trajectory_interface:getService())
    end

    return response
end

function MotionService:executeJointTrajectoryAsync(traj, cancelToken)
    local action_client = actionlib.SimpleActionClient('xamlamoveit_msgs/moveJ', 'moveJ_action', self.node_handle)
    local velocity = velocity or 0.5
    local acc = acc or 0.5
    local g = action_client:createGoal()
    g.trajectory.joint_names = traj.joint_names
    g.trajectory.points = traj.points
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
    --print(g)
    print('send', g)
    action_client:waitForServer(ros.Duration(1.5))
    if action_client:isServerConnected() then
        action_client:sendGoal(g, action_done, action_active, action_feedback)
        return true
    else
        ros.ERROR('could not reach moveJ_action')
        return false
    end
end

function MotionService:executeJointTrajectory(traj)
    local cancelToken = {done = false}
    local dt = ros.Rate(25)
    if self:executeJointTrajectoryAsync(traj, cancelToken) then
        while ros.ok() and cancelToken.done == false do
            ros.spinOnce()
            dt:sleep()
        end
        return true
    else
        ros.ERROR('could not reach moveJ_action')
        return false
    end
end

--IJointTrajectory PlanCollisionFree(Pose start, Pose goal, PlanParameters parameters);

local function planCollisionFreeCartesianPath_1(self, start, goal, parameters)
    assert(torch.isTypeOf(start, datatypes.Pose))
    assert(torch.isTypeOf(goal, datatypes.Pose))
    assert(
        torch.type(parameters) == 'PlanParameters',
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local seed = self:query_joint_state(parameters.joint_names)
    local suc, start = query_ik_call(self, start, parameters, seed)
    if suc ~= 1 then
        print('start suc ', suc)
        return false
    end
    suc, goal = query_ik_call(self, goal, parameters, start[1].positions)
    if suc ~= 1 then
        print('goal suc ', suc)
        return false
    end
    print('start', start[1].positions)
    print('goal', goal[1].positions)
    return self:planCollisionFreeJointPath(start[1].positions, goal[1].positions, parameters)
end

local function planCollisionFreeCartesianPath_2(self, waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(
        torch.type(parameters) == 'PlanParameters',
        'Wrong data type should be PlanParameters but is: ' .. torch.type(parameters)
    )
    local result = {}
    local seed = self:query_joint_state(parameters.joint_names)
    for i, v in ipairs(waypoints) do
        local suc, start = query_ik_call(self, v, parameters, seed)
        if suc ~= 1 then
            print('start suc ', suc)
            return false
        end
        table.insert(result, start[1].positions)
        seed = start[1].positions
    end
    return self:planCollisionFreeJointPath(result, parameters)
end

function MotionService:planCollisionFreeCartesianPath(_1, _2, _3)
    if torch.isTypeOf(_1,  datatypes.Pose) then
        return planCollisionFreeCartesianPath_1(self, _1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return planCollisionFreeCartesianPath_2(self, _1, _2)
    end
end

--IJointPath PlanCollisionFree(JointValues start, JointValues goal, PlanParameters parameters);
function MotionService:planCollisionFreeJointPath(_1, _2, _3)
    if torch.isTypeOf(_1, torch.DoubleTensor) then
        return self:planCollisionFreeJointPath_1(_1, _2, _3)
    elseif torch.type(_1) == 'table' then
        return self:planCollisionFreeJointPath_2(_1, _2)
    end
end

function MotionService:planCollisionFreeJointPath_1(start, goal, parameters)
    assert(torch.isTypeOf(start, torch.DoubleTensor))
    assert(torch.isTypeOf(goal, torch.DoubleTensor))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(start:size(1) == goal:size(1))
    return query_joint_path(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        torch.cat({start, goal}, 2),
        parameters.sampleResolution,
        0.0,
        false
    )
end

function MotionService:planCollisionFreeJointPath_2(waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.type(parameters) == 'PlanParameters')
    return query_joint_path(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        torch.cat(waypoints, 2),
        parameters.sampleResolution,
        0.0,
        false
    )
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
--IJointTrajectory PlanMoveCartesian(ICartesianPath path, PlanParameters parameters);
function MotionService:planMoveCartesian(waypoints, parameters)
    assert(torch.type(waypoints) == 'table')
    assert(torch.type(parameters) == 'PlanParameters')
    local pathCartesian
    local res =
        self:query_cartesian_path(
        parameters.move_group_name,
        parameters.joint_names,
        nil,
        waypoints,
        parameters.sampleResolution,
        0.0,
        parameters.collisionCheck
    )
    if res.error_code.val == 1 then
        pathCartesian = res.path
    end
    return res.error_code.val, stampedMessages2PoseArray(pathCartesian)
end

--IJointTrajectory PlanMoveJoint(IJointPath path, PlanParameters parameters);
function MotionService:planMoveJoint(path, parameters)
    assert(torch.isTypeOf(path, torch.DoubleTensor))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(path:size(2) == #parameters.joint_names)
    local trajectory
    local res =
        query_joint_trajectory(
        self,
        parameters.move_group_name,
        parameters.joint_names,
        path,
        parameters.maxVelocity * self.global_veloctiy_scaling,
        parameters.maxAcceleration * self.global_acceleration_scaling,
        0.0,
        parameters.sampleResolution or 0.008
    )
    if res.error_code.val == 1 then
        trajectory = res.solution
    end
    return res.error_code.val, trajectory
end

function MotionService:getDefaultPlanParameters(move_group_name, joint_names, max_velocity, max_acceleration)
    return PlanParameters.new(move_group_name, joint_names, true, max_velocity, max_acceleration)
end

function MotionService:emergencyStop(enable)
    assert(toch.type(enable) == 'boolean')
    local enable = enable or true
    local trigger_emergency_stop =
        self.node_handle:serviceClient('EmergencySTOP/query_emergency_stop', 'std_srvs/SetBool')
    local g = trigger_emergency_stop:createRequest()
    g.data = enable
    local response = trigger_emergency_stop:call(g)
    return response.success, response.message
end

return MotionService
