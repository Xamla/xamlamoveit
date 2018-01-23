local ros = require 'ros'
local moveit = require 'moveit'
local optimplan = require 'optimplan'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetOptimJointTrajectory')

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

local function sample(traj, dt)
    ros.INFO('Resample Trajectory with dt = %f', dt)
    local time, pos, vel, acc
    if type(traj) == 'table' then
        time, pos, vel, acc = {}, {}, {}, {}
        for i, v in ipairs(traj) do
            time[i], pos[i], vel[i], acc[i] = v:sample(dt or 0.01)
        end

        for i = 2, #time do
            local j = i - 1
            time[i]:add(time[j][time[j]:size(1)])
        end
        time = torch.cat(time, 1)
        pos = torch.cat(pos, 1)
        acc = torch.cat(acc, 1)
        vel = torch.cat(vel, 1)
    else
        time, pos, vel, acc = traj:sample(dt or 0.01)
    end
    ros.INFO('Trajectory num points = %d, duration %s', time:size(1), tostring(time[time:size(1)]))
    return time, pos, vel, acc
end

local function generateTrajectory(waypoints, max_velocities, max_accelerations, max_deviation, dt)
    max_deviation = max_deviation or 1e-5
    max_deviation = max_deviation < 1e-5 and 1e-5 or max_deviation

    local time_step = dt or 0.008
    time_step = math.max(time_step, 0.001)
    ros.INFO('generateTrajectory from waypoints with max dev: %08f, dt %08f', max_deviation, time_step)
    local path = {}
    path[1] = optimplan.Path(waypoints, max_deviation)
    local suc, split, skip = path[1]:analyse()
    waypoints = path[1].waypoints:clone()

    if not suc and #skip > 0 then
        ros.INFO('skipping %d points split %d ', #skip, #split)
        local indeces = torch.ByteTensor(waypoints:size(1)):fill(1)
        for i, v in ipairs(skip) do
            indeces[v] = 0
        end
        local newIndeces = {}
        for i = 1, indeces:size(1) do
            if indeces[i] == 1 then
                newIndeces[#newIndeces + 1] = i
            end
        end
        ros.INFO('newIndeces %d points', #newIndeces)
        waypoints = waypoints:index(1, torch.LongTensor(newIndeces)):clone()
        path[1] = optimplan.Path(waypoints, max_deviation)
        waypoints = path[1].waypoints:clone()
        suc, split, skip = path[1]:analyse()
        if (#skip > 0) then
            ros.WARN('check max deviation parameter... can propably be reduced')
        end
    end
    if not suc and #split > 0 then
        ros.INFO('splitting plan')
        path = {}
        local start_index = 1
        for i, v in ipairs(split) do
            if start_index < v then
                path[#path + 1] = optimplan.Path(waypoints[{{start_index, v}, {}}], max_deviation)
            end
            start_index = v
        end
        print(start_index, waypoints:size(1))
        path[#path + 1] = optimplan.Path(waypoints[{{start_index, waypoints:size(1)}, {}}], max_deviation)
    end
    local trajectory = {}
    local valid = true

    ros.INFO('generating trajectory from %d path segments, with dt = %f', #path, time_step)
    local str = {[1] = 'st', [2] = 'nd', [3] = 'th'}
    for i = 1, #path do
        ros.INFO('generating trajectory from %d%s path segments', i, str[math.min(i, 3)])
        trajectory[i] = optimplan.Trajectory(path[i], max_velocities, max_accelerations, time_step)
        trajectory[i]:outputPhasePlaneTrajectory()
        if not trajectory[i]:isValid() then
            valid = false
        else
            ros.INFO('Generation of Trajectories: %d%s', i / #path * 100, '%')
        end
    end
    local time, pos, vel, acc
    if valid then
        time, pos, vel, acc = sample(trajectory, dt)
    end
    return valid, time, pos, vel, acc
end

local function queryJointTrajectoryServiceHandler(self, request, response, header)
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

    local valid,  time, pos, vel, acc
    if waypoints:size(1)>2 then
      valid,  time, pos, vel, acc =
        generateTrajectory(waypoints, request.max_velocity, request.max_acceleration, request.max_deviation, request.dt)
    elseif waypoints:size(1) == 2 then
        valid = true
        time, pos, vel, acc = generateSimpleTvpTrajectory(waypoints, request.max_velocity, request.max_acceleration, request.dt)
    else
        valid = false
    end

    if not valid then
        ros.ERROR('Generated Trajectory is not valid!')
        response.error_code.val = -2
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
        ros.INFO('[!] incoming JointTrajectoryPlanningService call')
        self.callback_queue:callAvailable()
    end
end

function JointTrajectoryPlanningService:onStop()
end

function JointTrajectoryPlanningService:onShutdown()
    self.info_server:shutdown()
end

return JointTrajectoryPlanningService
