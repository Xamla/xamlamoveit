local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetMoveItJointPath')

local function table_concat(dst, src)
    for i, v in ipairs(src) do
        table.insert(dst, v)
    end
    return dst
end

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
        printf('MoveIt! initialization done. Current end effector link: "%s"', manipulator:getEndEffectorLink())
        return manipulator
    end
end

local function getMoveitPath(self, group_name, joint_names, waypoints)
    ros.INFO("getMoveitPath")
    local manipulator = initializeMoveGroup(self, group_name)
    local num_steps = waypoints:size(1)
    local plannedwaypoints = {}
    local robot_state = manipulator:getCurrentState()
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
            if s == 0 then
                ros.ERROR('Moveit Planning failed')
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
        ros.INFO('moveit plan succeeded')
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
        ros.DEBUG('[!] incoming JointPathPlanningService call')
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
