local ros = require 'ros'
local moveit = require 'moveit'
local optimplan = require 'optimplan'
--require 'xamlamoveit.components.RosComponent'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetOptimJointPath')

local function checkMoveGroupName(self, name)
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    ros.INFO('available move_groups:\n%s', tostring(all_group_joint_names))
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
        manipulator:setGoalTolerance(1E-5)
        manipulator:setPlanningTime(2.0)

        -- ask move group for current state
        manipulator:startStateMonitor(0.008)
        local cs = manipulator:getCurrentState()
        manipulator:setStartStateToCurrentState()
        local currentPose = manipulator:getCurrentPose():toTensor()
        print('Current robot pose:')
        print(currentPose)

        printf('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
        return manipulator
    end
end

local function getMoveitPath(self, group_name, joint_names, waypoints)
    local manipulator = initializeMoveGroup(self, group_name)
    local num_steps = waypoints:size(1)
    local plannedwaypoints = {}
    local robot_state = manipulator:getCurrentState()
    for i = 1, num_steps - 1 do
        local k = math.min(i + 1, num_steps)
        if i < k then
            robot_state:setVariablePositions(waypoints[i], joint_names)
            manipulator:setStartState(robot_state)
            if not manipulator:setJointValueTarget(waypoints[k]) then
                ros.ERROR('Setting joint value target failed.\n Joints: %s', waypoints[k])
                return false
            end
            local s, p = manipulator:plan()
            if s == 0 then
                ros.ERROR('Moveit Planning failed')
                return false
            end
            local positions, velocities, accelerations, efforts = p:convertTrajectoyMsgToTable(p:getTrajectoryMsg())
            plannedwaypoints[i] = positions
        end
    end
    result = {}
    for i, v in ipairs(plannedwaypoints) do
        result[i] = torch.cat(v, 2)
    end
    return true, torch.cat(result, 2):t()
end

local function generatePath(waypoints, MAX_DEVIATION)
    local MAX_DEVIATION = MAX_DEVIATION or 1e-6
    MAX_DEVIATION = MAX_DEVIATION < 1e-6 and 1e-6 or MAX_DEVIATION

    local path = optimplan.Path(waypoints, MAX_DEVIATION)
    local suc, split, scip = path:analyse()
    waypoints = path.waypoints
    if not suc and #scip > 0 then
        local indeces = torch.Storage(waypoints:size(1)):fill(1)
        for i, v in ipairs(scip) do
            indeces[v] = 0
        end
        waypoints = waypoints[{indeces, {}}]
        path = optimplan.Path(waypoints, MAX_DEVIATION)
        suc, split, scip = path:analyse()
        assert(#scip == 0)
    end
    return path
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

    local moveit_plan_success = true

    if request.moveit_adaptive_plan then
        moveit_plan_success, waypoints = getMoveitPath(self, request.group_name, request.joint_names, waypoints)
    end

    if moveit_plan_success then
        local path = generatePath(waypoints, response.max_deviation)
        local dist = path:getLength()
        local num_steps = request.num_steps
        local vel = dist / (num_steps - 1)
        --print(request)
        ros.WARN('NumSteps = %f', num_steps)
        for i = 0, num_steps - 1 do
            response.path[i + 1] = ros.Message('xamlamoveit_msgs/JointPathPoint')
            response.path[i + 1].positions = path:getConfig(vel * i)
        end
        ros.WARN('NumSteps = %d', #response.path)
        response.error_code.val = 1
    else
        response.error_code.val = -2
    end
    return true
end

local components = require 'xamlamoveit.components.env'
local JointPathPlanningService,
    parent =
    torch.class('xamlamoveit.components.JointPathPlanningService', 'xamlamoveit.components.RosComponent', components)

function JointPathPlanningService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.robot_state = nil
    self.info_server = nil
    parent.__init(self, node_handle)
end

function JointPathPlanningService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
end

function JointPathPlanningService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_joint_path',
        srv_spec,
        function(request, response, header)
            return queryJointPathServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function JointPathPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming JointPathPlanningService call')
        self.callback_queue:callAvailable()
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