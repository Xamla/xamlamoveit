local ros = require 'ros'
local moveit = require 'moveit'
local optimplan = require 'optimplan'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetOptimJointTrajectory')
local xutils = require 'xamlamoveit.xutils'

local function generateSimpleTvpTrajectory(waypoints, max_velocities, max_accelerations, dt)
    local start = waypoints[{1, {}}]
    local goal = waypoints[{2, {}}]
    local dim = goal:size(1)
    local controller = require 'xamlamoveit.controller'.MultiAxisTvpController(dim)
    controller.max_vel:copy(max_velocities)
    controller.max_acc:copy(max_accelerations)
    local result = controller:generateOfflineTrajectory(start, goal, dt)
    local positions = torch.zeros(#result, dim)
    local velocities = torch.zeros(#result, dim)
    local accelerations = torch.zeros(#result, dim)
    local time = {}
    for i = 1, #result do
        time[i] = dt * i
        positions[{i, {}}]:copy(result[i].pos)
        velocities[{i, {}}]:copy(result[i].vel)
        accelerations[{i, {}}]:copy(result[i].acc)
    end
    time = torch.Tensor(time)
    return time, positions, velocities, accelerations
end



local function checkPathLength(waypoints)
    assert(torch.isTypeOf(waypoints, torch.DoubleTensor), "wrong type")
    local length = 0
    for i = 1, waypoints:size(1) do
        local j = math.min(i + 1, waypoints:size(1))
        length = (waypoints[j] - waypoints[i]):norm() + length
    end
    return length
end

local function queryJointTrajectoryServiceHandler(self, request, response, header)
    xutils.tic('queryJointTrajectoryServiceHandler')
    if #request.waypoints < 2 then
        ros.ERROR('Only one waypoint detected. Abort Trajectory generation.')
        response.error_code.val = -2
        return true
    end
    if request.waypoints[1].positions:nDimension() < 1 then
        ros.ERROR('Waypoints have wrong Dimensions')
        response.error_code.val = -2
        return true
    end
    local dim = request.waypoints[1].positions:size(1)
    local waypoints = torch.Tensor(#request.waypoints, dim)
    for i, v in ipairs(request.waypoints) do
        if v.positions:nDimension() < 1 then
            response.error_code.val = -2
            return false
        end
        waypoints[i]:copy(v.positions)
    end

    local valid, time, pos, vel, acc
    if waypoints:size(1) > 2 then
        if checkPathLength(waypoints) < 1e-6 then
            ros.INFO('waypoints seem to be all equal')
            response.solution.joint_names = request.joint_names
            response.solution.points[1] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
            response.solution.points[1].positions = request.waypoints[1].positions
            response.solution.points[1].velocities = torch.zeros(dim)
            response.solution.points[1].accelerations = torch.zeros(dim)
            response.solution.points[1].time_from_start = ros.Duration(0)
            response.error_code.val = 1
            return true
        end
        xutils.tic('generateTrajectory')
        valid,
            time,
            pos,
            vel,
            acc =
            optimplan.generateTrajectory(
            waypoints,
            request.max_velocity,
            request.max_acceleration,
            request.max_deviation,
            request.dt
        )
        xutils.toc('generateTrajectory')
    elseif waypoints:size(1) == 2 then
        xutils.tic('generateSimpleTvpTrajectory')
        valid = true
        time,
            pos,
            vel,
            acc = generateSimpleTvpTrajectory(waypoints, request.max_velocity, request.max_acceleration, request.dt)
        xutils.toc('generateSimpleTvpTrajectory')
    else
        valid = false
    end

    if not valid then
        ros.ERROR('Generated Trajectory is not valid!')
        response.error_code.val = -2
        xutils.toc('queryJointTrajectoryServiceHandler')
        return true
    else
        ros.INFO('Generated Trajectory is valid!')
    end

    response.solution.joint_names = request.joint_names
    for i = 1, time:size(1) do
        response.solution.points[i] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
        response.solution.points[i].positions = pos[i]
        response.solution.points[i].velocities = vel[i]
        response.solution.points[i].accelerations = acc[i]
        response.solution.points[i].time_from_start = ros.Duration(time[i])
    end
    response.error_code.val = 1
    xutils.toc('queryJointTrajectoryServiceHandler')
    --print(response)
    return true
end

local components = require 'xamlamoveit.components.env'
local JointTrajectoryPlanningService,
    parent =
    torch.class(
    'xamlamoveit.components.JointTrajectoryPlanningService',
    'xamlamoveit.components.RosComponent',
    components
)

function JointTrajectoryPlanningService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.robot_state = nil
    self.info_server = nil
    parent.__init(self, node_handle)
end

function JointTrajectoryPlanningService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.all_group_joint_names = self.robot_model:getJointModelGroupNames()
end

function JointTrajectoryPlanningService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_joint_trajectory',
        srv_spec,
        function(request, response, header)
            return queryJointTrajectoryServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function JointTrajectoryPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.DEBUG('[!] incoming JointTrajectoryPlanningService call')
        self.callback_queue:callAvailable()
    end
end

function JointTrajectoryPlanningService:onStop()
end

function JointTrajectoryPlanningService:onShutdown()
    self.info_server:shutdown()
end

return JointTrajectoryPlanningService
