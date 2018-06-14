local ros = require 'ros'
local tf = ros.tf
local datatypes = require 'xamlamoveit.datatypes'
local xutils = require 'xamlamoveit.xutils'
local motionLibrary = require 'xamlamoveit.motionLibrary.env'

local MoveGroup = torch.class('MoveGroup', motionLibrary)

function MoveGroup:__init(motion_service, move_group_name)
    assert(motion_service ~= nil, 'Argument `motion_service` must not be nil.')
    self.motion_service = motion_service

    local names, details = motion_service:queryAvailableMoveGroups()
    self.name = move_group_name or names[1]
    self.details = details[self.name]
    assert(self.details ~= nil, string.format('Move group with name \'%s\' not found.', self.name or '<nil>'))
    self.joint_set = datatypes.JointSet(self.details.joint_names)
    self.default_plan_parameters = motion_service:getDefaultPlanParameters(self.name, self.joint_set.joint_names)

    self.velocity_scaling = 1
    self.collision_check = false
    local end_effectors = {}
    for i,end_effector_name in ipairs(self.details.end_effector_names) do
        end_effectors[end_effector_name] = motionLibrary.EndEffector(self, end_effector_name, self.details.end_effector_link_names[i])
    end
    self.end_effectors = end_effectors
    self.default_end_effector_name = #self.details.end_effector_names > 0 and self.details.end_effector_names[1]   -- use first end effector as default
    local maxXYZVel, maxXYZAcc, maxAngularVel, maxAngularAcc = self.motion_service:queryEndEffectorLimits(self.default_end_effector_name)
    self.default_task_space_plan_parameters = self.motion_service:getDefaultTaskSpacePlanParameters(self.default_end_effector_name, 0.0, maxXYZVel, maxXYZAcc, maxAngularVel, maxAngularAcc)
end

function MoveGroup:getEndEffectorNames()
    return self.details.end_effector_names
end

function MoveGroup:getEndEffector(end_effector_name)
    end_effector_name = end_effector_name or self.default_end_effector_name
    return self.end_effectors[end_effector_name]
end

function MoveGroup:getCurrentPose(end_effector_name)
    local ee = self:getEndEffector(end_effector_name)
    return ee:getCurrentPose(), ee.link_name
end

function MoveGroup:setDefaultEndEffector(name)
    assert(self.end_effectors[name], string.format('Specified end effector \'%s\' does not exist in move group \'%s\'.', name, self.name))
    self.default_end_effector_name = name
end

function MoveGroup:getVelocityScaling()
    return self.velocity_scaling
end

function MoveGroup:setVelocityScaling(value)
    assert(type(value) == 'number', 'Invalid type of argument `value`. Number expected.')
    assert(value > 0 and value <= 1, 'Argument `value` must lie in range (0-1).')
    self.velocity_scaling = value
end

function MoveGroup:getJointNames()
    return self.joint_set.joint_names
end

function MoveGroup:getCurrentJointValues()
    local joint_names = self:getJointNames()
    local error_code, joint_values = self.motion_service:queryJointState(joint_names)
    assert(error_code.val == 1, string.format('Could not aquire current joint state. [%d]', error_code.val))
    return datatypes.JointValues(self.joint_set, joint_values)
end

function MoveGroup:getDefaultPlanParameters()
    return self.default_plan_parameters
end

function MoveGroup:getDefaultTaskSpacePlanParameters(end_effector_name)
    assert(torch.type(end_effector_name) == 'string', 'Please specify your end effector which should be moved.')
    return self.motion_service:getDefaultTaskSpacePlanParameters(end_effector_name)
end

local function apply(dst, src)
    assert(src ~= nil, 'Source table must not be nil.')
    assert(dst ~= nil, 'Destination table must not be nil.')
    for k, v in pairs(src) do
        dst[k] = src[k]
    end
end

function MoveGroup:buildTaskSpacePlanParameters(end_effector_name, velocity_scaling, acceleration_scaling, collision_check, max_deviation, ik_jump_threshold, dt)
    -- check input or use member values if arguments where not provided
    if end_effector_name == nil then
        end_effector_name = self.default_end_effector_name
    end
    if velocity_scaling == nil then
        velocity_scaling = self.velocity_scaling
    else
        assert(type(velocity_scaling) == 'number', 'Invalid type of argument `velocity_scaling`. Number expected.')
        assert(velocity_scaling > 0 and velocity_scaling <= 1, 'Argument `velocity_scaling` must lie in range (0-1).')
    end

    if acceleration_scaling == nil then
        acceleration_scaling = self.velocity_scaling
    else
        assert(type(acceleration_scaling) == 'number', 'Invalid type of argument `acceleration_scaling`. Number expected.')
        assert(acceleration_scaling > 0 and acceleration_scaling <= 1, 'Argument `acceleration_scaling` must lie in range (0-1).')
    end
    if collision_check == nil then
        collision_check = self.collision_check
    end

    -- start with default plan parameters
    local t = self:getDefaultTaskSpacePlanParameters(end_effector_name):toTable()
    t.collision_check = collision_check
    if max_deviation then
        t.max_deviation = max_deviation
    end

    if ik_jump_threshold then
        t.ik_jump_threshold = ik_jump_threshold
    end

    if dt then
        t.dt = dt
    end

    if t.max_xyz_velocity ~= nil then
        t.max_xyz_velocity = t.max_xyz_velocity * acceleration_scaling
    end
    if t.max_xyz_acceleration ~= nil then
        t.max_xyz_acceleration = t.max_xyz_acceleration * acceleration_scaling
    end

    if t.max_angular_velocity ~= nil then
        t.max_angular_velocity = t.max_angular_velocity * velocity_scaling
    end
    if t.max_angular_acceleration ~= nil then
        t.max_angular_acceleration = t.max_angular_acceleration * velocity_scaling
    end

    -- create empty result object and apply values
    local result = datatypes.TaskSpacePlanParameters()
    result:fromTable(t)
    return result
end

function MoveGroup:buildPlanParameters(velocity_scaling, collision_check, max_deviation)
    -- check input or use member values if arguments where not provided
    if velocity_scaling == nil then
        velocity_scaling = self.velocity_scaling
    else
        assert(type(velocity_scaling) == 'number', 'Invalid type of argument `velocity_scaling`. Number expected.')
        assert(velocity_scaling > 0 and velocity_scaling <= 1, 'Argument `velocity_scaling` must lie in range (0-1).')
    end
    if collision_check == nil then
        collision_check = self.collision_check
    end

    -- start with default plan parameters
    local t = self:getDefaultPlanParameters():toTable()
    t.collision_check = collision_check
    if max_deviation then
        t.max_deviation = max_deviation
    end

    if t.max_velocity ~= nil then
        t.max_velocity = t.max_velocity * velocity_scaling
    end
    if t.max_acceleration ~= nil then
        t.max_acceleration = t.max_acceleration * velocity_scaling
    end

    -- create empty result object and apply values
    local result = datatypes.PlanParameters()
    apply(result, t)
    return result
end

function MoveGroup:planMoveJoints(target, velocity_scaling, collision_check)
    local plan_parameters = self:buildPlanParameters(velocity_scaling, collision_check)

    -- get current pose
    local joint_names = target:getNames()
    local start = self:getCurrentJointValues()
    local start_joints = start.values
    local end_joints = target.values

    -- generate joint path
    local joint_path = torch.DoubleTensor(2, start_joints:size(1))
    joint_path[{1,{}}] = start_joints
    joint_path[{2,{}}] = end_joints

    -- plan trajectory
    local ok, joint_trajectory = self.motion_service:planMoveJoint(joint_path, plan_parameters)
    return ok, joint_trajectory, plan_parameters
end

function MoveGroup:moveJoints(target, velocity_scaling, collision_check)
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMoveJoints(target, velocity_scaling, collision_check)
    assert(ok == 1, 'planMoveJoints failed')

    -- start synchronous blocking execution
    local ok, msg = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    assert(ok, 'executeJointTrajectory failed. ' .. msg)
end

function MoveGroup:moveJointsAsync(target, velocity_scaling, collision_check, done_cb)
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMoveJoints(target, velocity_scaling, collision_check)
    assert(ok == 1, 'planMoveJoints failed')

    local simple_action_client = self.motion_service:executeJointTrajectoryAsync(joint_trajectory, plan_parameters.collision_check, done_cb)
    return simple_action_client
end

function MoveGroup:moveJointsSupervised(target, velocity_scaling, collision_check)
    -- plan trajectory
    local ok, joint_trajectory, plan_parameters = self:planMoveJoints(target, velocity_scaling, collision_check)
    assert(ok == 1, 'planMoveJoints failed')

    -- start asynchronous execution
    local controller_handle = self.motion_service:executeSupervisedJointTrajectory(joint_trajectory, plan_parameters.collision_check, done_cb)
    return controller_handle
end

function MoveGroup:planMoveJointsWaypoints(waypoints, velocity_scaling, collision_check, max_deviation)
    max_deviation = max_deviation or 0.2
    local plan_parameters = self:buildPlanParameters(velocity_scaling, collision_check, max_deviation)

    -- get current pose
    local joint_names = waypoints[1]:getNames()
    local start = self:getCurrentJointValues()
    local start_joints = start.values

    -- generate joint path
    local joint_path = torch.DoubleTensor(1 + #waypoints, start_joints:size(1))
    joint_path[{1,{}}] = start_joints
    for i,p in ipairs(waypoints) do
        joint_path[{i+1,{}}] = p.values
    end

    -- plan trajectory
    local ok, joint_trajectory = self.motion_service:planMoveJoint(joint_path, plan_parameters)
    return ok, joint_trajectory, plan_parameters
end

function MoveGroup:moveJWaypoints(waypoints, velocity_scaling, collision_check, max_deviation)
    if #waypoints == 0 then
        return
    end

    local ok, joint_trajectory, plan_parameters = self:planMoveJointsWaypoints(waypoints, velocity_scaling, collision_check, max_deviation)
    assert(ok == 1, 'planMoveJointsWaypoints failed')

    -- start synchronous blocking execution
    local ok = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    assert(ok, 'executeJointTrajectory failed.')
end

function MoveGroup:moveJWaypointsAsync(waypoints, velocity_scaling, collision_check, max_deviation, done_cb)
    if #waypoints == 0 then
        return nil
    end

    local ok, joint_trajectory, plan_parameters = self:planMoveJointsWaypoints(waypoints, velocity_scaling, collision_check, max_deviation)
    assert(ok == 1, 'planMoveJointsWaypoints failed')

    local simple_action_client = self.motion_service:executeJointTrajectoryAsync(joint_trajectory, plan_parameters.collision_check, done_cb)
    return simple_action_client
end
