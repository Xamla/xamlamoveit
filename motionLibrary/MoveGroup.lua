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
        end_effectors[end_effector_name] = motionLibrary.EndEffector(self, end_effector_name)
    end
    self.end_effectors = end_effectors
    self.default_end_effector_name = #self.details.end_effector_names > 0 and self.details.end_effector_names[1]   -- use first end effector as default
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
    return ee:getCurrentPose(), ee.name
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
    local joint_values = self.motion_service:queryJointState(joint_names)
    return datatypes.JointValues(self.joint_set, joint_values)
end

function MoveGroup:getDefaultPlanParameters()
    return self.default_plan_parameters
end

local function apply(dst, src) 
    assert(src ~= nil, 'Source table must not be nil.')
    assert(dst ~= nil, 'Destination table must not be nil.')
    for k, v in pairs(src) do
        dst[k] = src[k]
    end
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

function MoveGroup:moveJ(target, velocity_scaling, collision_check)
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
    local success, joint_trajectory = self.motion_service:planMoveJoint(joint_path, plan_parameters)
    assert(success == 1, 'planMoveJoint() failed')

    -- start synchronous blocking execution
    local result = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    print(result)
end

function MoveGroup:moveWaypointList(waypoints, velocity_scaling, collision_check)
    if #waypoints == 0 then
        return
    end

    local plan_parameters = self:buildPlanParameters(velocity_scaling, collision_check, 0.2)

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
    local success, joint_trajectory = self.motion_service:planMoveJoint(joint_path, plan_parameters)
    assert(success == 1, 'planMoveJoint() failed')

    -- start synchronous blocking execution
    local result = self.motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
    print(result)
end
