--[[
JointPathPlanningService.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local errorCodes = require 'xamlamoveit.core.ErrorCodes'
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetMoveItJointPath')
local srv_with_parameter_spec = ros.SrvSpec('xamlamoveit_msgs/GetMoveItJointPathWithParameters')
local set_float_spec = ros.SrvSpec('xamlamoveit_msgs/SetFloat')
local get_float_spec = ros.SrvSpec('xamlamoveit_msgs/GetFloat')

local PARAMETERS = {
    [1] = 'planning_time',
    [2] = 'goal_tolerance',
    [3] = 'planning_attemts'
}

local function findString(my_string, collection)
    local index = -1
    if torch.type(collection) == 'table' then
        index = table.indexof(collection, my_string)
    elseif torch.type(collection) == 'std.StringVector' then
        index = table.indexof(collection:totable(), my_string)
    else
        error('unknown type: ' .. torch.type(collection))
    end

    return index > -1, index
end

local function table_concat(dst, src)
    for i, v in ipairs(src) do
        table.insert(dst, v)
    end
    return dst
end

local function checkMoveGroupName(self, name)
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    ros.DEBUG('available move_groups:\n%s', tostring(all_group_joint_names))
    for k, v in pairs(all_group_joint_names) do
        if name == v then
            return true
        end
    end
    ros.ERROR('could not find move_group: ' .. name)
    return false
end

local function initializeMoveGroup(self, group_id, velocity_scaling)
    local group_id = group_id or 'manipulator'
    if checkMoveGroupName(self, group_id) then
        local velocity_scaling = velocity_scaling or 0.5
        ros.INFO('connection with movegroup: ' .. group_id)
        local manipulator = moveit.MoveGroupInterface(group_id)

        manipulator:setMaxVelocityScalingFactor(velocity_scaling)
        manipulator:setGoalTolerance(self.moveit_parameters.goal_tolerance)
        manipulator:setPlanningTime(self.moveit_parameters.planning_time)

        -- ask move group for current state
        manipulator:startStateMonitor(0.008)
        local cs = manipulator:getCurrentState()
        manipulator:setStartStateToCurrentState()
        local currentPose = manipulator:getCurrentPose():toTensor()
        printf('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
        return manipulator
    end
end

local function getMoveitPath(self, group_name, joint_names, waypoints)
    local num_steps = waypoints:size(1)
    ros.INFO("getMoveitPath with %d via points", num_steps)
    local manipulator = self.manipulators[group_name]
    manipulator:setGoalTolerance(self.moveit_parameters.goal_tolerance)
    manipulator:setPlanningTime(self.moveit_parameters.planning_time)
    manipulator:setNumPlanningAttempts(self.moveit_parameters.planning_attemts)
    local plannedwaypoints = {}
    local robot_state = manipulator:getCurrentState()
    robot_state:fromRobotStateMsg(self.plan_scene:getCurrentState():toRobotStateMsg(true))
    for i = 1, num_steps do
        local k = math.min(i + 1, num_steps)
        if i < k then
            robot_state:setVariablePositions(waypoints[i], joint_names)
            manipulator:setStartState(robot_state)
            if not manipulator:setJointValueTarget(waypoints[k]) then
                ros.ERROR('Setting joint value target failed.\n waypoint %d, Joints: %s',k, waypoints[k])
                return false, nil, robot_state:getVariableNames():totable()
            end
            xutils.tic('moveit plan request')
            local s, p = manipulator:plan()
            if s ~= 1 then
                ros.ERROR('Moveit Planning failed: [%d] %s',s, errorCodes[s])
                return false, nil, robot_state:getVariableNames():totable()
            end
            local positions, velocities, accelerations, efforts = p:convertTrajectoyMsgToTable(p:getTrajectoryMsg())
            plannedwaypoints[i] = positions
            xutils.toc('moveit plan request')
        end
    end
    local result = {}
    for i, v in ipairs(plannedwaypoints) do
        table_concat(result, v)
    end
    -- make sure to have the goal exact in plan
    result[#result + 1 ] = waypoints[num_steps]
    return true, result, robot_state:getVariableNames():totable()
end

local function queryJointPathServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        response.error_code.val = -2
        return true
    end
    if request.waypoints[1].positions:nDimension() < 1 then
        response.error_code.val = -2
        return true
    end

    local dim = request.waypoints[1].positions:size(1)
    if dim ~= #request.joint_names then
        response.error_code.val = -2
        return true
    end

    local waypoints = torch.Tensor(#request.waypoints, dim)
    for i, v in ipairs(request.waypoints) do
        if v.positions:nDimension() < 1 then
            response.error_code.val = -2
            return false
        end
        waypoints[i]:copy(v.positions)
    end

    local moveit_plan_success

    ros.INFO('using moveit')
    local joint_names_m
    moveit_plan_success, waypoints, joint_names_m = getMoveitPath(self, request.group_name, request.joint_names, waypoints)
    if moveit_plan_success then
        ros.INFO('moveit plan succeeded generated %d waypoints', #waypoints)
        response.error_code.val = 1
        local spec = ros.MsgSpec('xamlamoveit_msgs/JointPathPoint')
        for i, v in ipairs(waypoints) do
            response.path[i] = ros.Message(spec)
            response.path[i].positions = v
        end
    else
        response.error_code.val = -2
    end


    response.joint_names = joint_names_m
    return true
end

local function queryJointPathServiceHandlerWithParameters(self, request, response, header)
    local tmp = {planning_time = 10, goal_tolerance = 1E-5, planning_attemts = 5}
    tmp.planning_attempts = self.moveit_parameters.planning_attempts
    tmp.goal_tolerance = self.moveit_parameters.goal_tolerance
    tmp.planning_time = self.moveit_parameters.planning_time
    if request.parameters.has_planning_time then
        self.moveit_parameters.planning_attempts =  request.parameters.planning_time
    end
    if request.parameters.has_goal_tolerance then
        self.moveit_parameters.goal_tolerance = request.parameters.goal_tolerance
    end
    if request.parameters.has_planning_time then
        self.moveit_parameters.planning_time = request.parameters.planning_time:toSec()
    end
    local suc = queryJointPathServiceHandler(self, request, response, header)
    self.moveit_parameters.planning_attempts = tmp.planning_attempts
    self.moveit_parameters.goal_tolerance = tmp.goal_tolerance
    self.moveit_parameters.planning_time = tmp.planning_time
    return suc
end

local function getParameterNames(self, request, response, header)
    response.result = PARAMETERS
    response.success = true
    return true
end


local function setParameterServiceHandler(self, request, response, header, parameter_name)
    local suc, index = findString(parameter_name, PARAMETERS)
    response.success = suc
    ros.INFO('Set parameter %s to %f', PARAMETERS[index], request.data)
    self.moveit_parameters[PARAMETERS[index]] = request.data
    return true
end


local function getParameterServiceHandler(self, request, response, header, parameter_name)
    local suc, index = findString(parameter_name, PARAMETERS)
    response.success = suc
    local value = self.moveit_parameters[PARAMETERS[index]]
    ros.INFO('Get parameter %s to %f', PARAMETERS[index], value)
    response.data = value
    return true
end


local components = require 'xamlamoveit.components.env'
local JointPathPlanningService,
    parent =
    torch.class('xamlamoveit.components.JointPathPlanningService', 'xamlamoveit.components.RosComponent', components)

function JointPathPlanningService:__init(node_handle, joint_monitor, robot_model)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = robot_model
    self.robot_model_loader = nil
    self.robot_state = nil
    self.plan_scene = nil
    self.info_server = nil
    self.set_planning_time_server = nil
    self.get_planning_time_server = nil
    self.set_goal_tolerance_server = nil
    self.get_goal_tolerance_server = nil
    self.info_server_with_parameter = nil
    self.joint_monitor = joint_monitor
    self.manipulators = {}
    self.moveit_parameters = {planning_time = 10, goal_tolerance = 1E-5, planning_attemts = 5}
    parent.__init(self, node_handle)
end

function JointPathPlanningService:onInitialize()
    if not self.robot_model then
        self.robot_model_loader = moveit.RobotModelLoader('robot_description')
        self.robot_model = self.robot_model_loader:getModel()
    end
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.plan_scene = moveit.PlanningScene(self.robot_model)
    self.plan_scene:syncPlanningScene()
    local ready = self.joint_monitor:waitReady(2.0) -- it is not important to have the joint monitor ready at start up
    if not ready then
        ros.WARN('joint states not ready')
    else
        local ok, p = self.joint_monitor:getNextPositionsTensor()
        self.robot_state:setVariablePositions(
            p,
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
    end
    local move_group_names = self.robot_model:getJointModelGroupNames()
    for i, v in ipairs(move_group_names) do
        self.manipulators[v] = initializeMoveGroup(self, v)
    end
end

function JointPathPlanningService:onStart()
    self.info_server_with_parameter =
        self.node_handle:advertiseService(
        'query_joint_path_parameterized',
        srv_with_parameter_spec,
        function(request, response, header)
            return queryJointPathServiceHandlerWithParameters(self, request, response, header)
        end,
        self.callback_queue
    )
    self.info_server =
        self.node_handle:advertiseService(
        'query_joint_path',
        srv_spec,
        function(request, response, header)
            return queryJointPathServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
    self.set_planning_time_server =
        self.node_handle:advertiseService(
        'set_planning_time',
        set_float_spec,
        function(request, response, header)
            return setParameterServiceHandler(self, request, response, header, 'planning_time')
        end,
        self.callback_queue
    )
    self.get_planning_time_server =
        self.node_handle:advertiseService(
        'get_planning_time',
        get_float_spec,
        function(request, response, header)
            return getParameterServiceHandler(self, request, response, header, 'planning_time')
        end,
        self.callback_queue
    )
    self.set_goal_tolerance_server =
        self.node_handle:advertiseService(
        'set_goal_tolerance',
        set_float_spec,
        function(request, response, header)
            return setParameterServiceHandler(self, request, response, header, 'goal_tolerance')
        end,
        self.callback_queue
    )
    self.get_goal_tolerance_server =
        self.node_handle:advertiseService(
        'get_goal_tolerance',
        get_float_spec,
        function(request, response, header)
            return getParameterServiceHandler(self, request, response, header, 'goal_tolerance')
        end,
        self.callback_queue
    )
end

function JointPathPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.DEBUG('[!] incoming JointPathPlanningService call')
        self.callback_queue:callAvailable()
    end
    if self.joint_monitor:isReady() then
        local joints = self.joint_monitor:getPositionsTensor()

        self.robot_state:setVariablePositions(
            joints,
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
    end
end

function JointPathPlanningService:onStop()
    ros.WARN('JointPathPlanningService:onStop() NOT IMPLEMENTED')
end

function JointPathPlanningService:onReset()
    self.info_server:shutdown()
end

function JointPathPlanningService:onShutdown()
    self.info_server:shutdown()
end

return JointPathPlanningService
