local torch = require 'torch'
local ros = require 'ros'
local PlanParameters = require 'xamlamoveit.components.PlanParameters'

local ac
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState

local components = require 'xamlamoveit.components.env'
local MotionService = torch.class('MotionService', components)

function MotionService:__init(node_handle)
    self.node_handle = node_handle
    self.global_veloctiy_scaling = 1.0
    self.global_acceleration_scaling = 1.0
end

local function query_ik_call()
end

local function create_ik_request(group_name, robot_state, avoid_collisions, ik_link_names, poses_stamped)
    local robot_state_msg
    if robot_state then
        robot_state_msg = robot_state:toRobotStateMsg()
    end
    local ik_spec = ros.MsgSpec('moveit_msgs/PositionIKRequest')
    local req_msg = ros.Message(ik_spec)
    req_msg.group_name = group_name
    req_msg.avoid_collisions = avoid_collisions
    if robot_state_msg then
        req_msg.robot_state = robot_state_msg
        print(torch.type(robot_state_msg))
    end

    if #ik_link_names == 1 then
        local poses = {}
        for i, k in ipairs(poses_stamped) do
            table.insert(poses, k:toStampedPoseMsg())
        end
        req_msg.ik_link_names = ik_link_names
        req_msg.pose_stamped_vector = poses
    else
        req_msg.ik_link_name = ik_link_names[1]
        req_msg.pose_stamped = poses_stamped[1]:toStampedPoseMsg()
        req_msg.timeout = ros.Duration(0.1)
        req_msg.attempts = 1
    end
    return req_msg
end

--get Avalable move groups
function MotionService:query_available_movegroups()
    local move_group_interface =
        self.node_handle:serviceClient(
        '/xamlaservices/query_move_group_interface',
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
function MotionService:query_joint_state(joint_names)
    local move_group_position_interface =
        self.node_handle:serviceClient(
        '/xamlaservices/query_move_group_current_position',
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
        self.node_handle:serviceClient('/xamlaservices/query_joint_path', 'xamlamoveit_msgs/GetOptimJointPath')
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
    --print(request)
    local response = generate_path_interface:call(request)

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
        '/xamlaservices/query_joint_trajectory',
        'xamlamoveit_msgs/GetOptimJointTrajectory'
    )
    local request = generate_trajectory_interface:createRequest()
    request.max_deviation = max_deviation
    request.group_name = move_group_name
    request.joint_names = joint_names
    request.dt = dt
    request.max_velocity = max_vel
    request.max_acceleration = max_acc

    for i = 1, waypoints:size(2) do
        request.waypoints[i] = ros.Message('xamlamoveit_msgs/JointPathPoint')
        request.waypoints[i].positions = waypoints[{{}, i}]
    end

    local response = generate_trajectory_interface:call(request)

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
        ros.INFO('Action_active')
    end

    local function action_feedback(feedback)
        ros.INFO('Action_feedback')
    end

    ros.spinOnce()
    --print(g)
    --print('send', g)
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
function MotionService:planCollisionFreeCartesianPath(start, goal, parameters)
    assert(torch.isTypeOf(start, tf.Transform))
    assert(torch.isTypeOf(goal, tf.Transform))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(start:size(1) == goal:size(1))
    error('not tested')
    local suc, start = self:query_ik_call(start, PlanParameters)
    if suc == false then
        return false
    end
    suc, goal = self:query_ik_call(goal)
    if suc == false then
        return false
    end
    return self:query_cartesian_path(
        parameters.move_group_name,
        parameters.joint_names,
        {[1] = start, [2] = goal},
        parameters.sampleResolution,
        0.0,
        true
    )
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

--IJointTrajectory PlanMoveCartesian(ICartesianPath path, PlanParameters parameters);
--IJointTrajectory PlanMoveJoint(IJointPath path, PlanParameters parameters);
function MotionService:planMoveJoint(path, parameters)
    assert(torch.isTypeOf(path, torch.DoubleTensor))
    assert(torch.type(parameters) == 'PlanParameters')
    assert(path:size(1) == #parameters.joint_names)
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
        self.node_handle:serviceClient('/EmergencySTOP/query_emergency_stop', 'std_srvs/SetBool')
    local g = trigger_emergency_stop:createRequest()
    g.data = enable
    local response = trigger_emergency_stop:call(g)
    return response.success, response.message
end

return MotionService
