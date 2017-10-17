local ros = require 'ros'
local moveit = require 'moveit'
local optimplan = require 'optimplan'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetOptimJointTrajectory')

local function generateTrajectory(waypoints, maxVelocities, maxAccelerations, MAX_DEVIATION, dt)
    MAX_DEVIATION = MAX_DEVIATION or 1e-4
    MAX_DEVIATION = MAX_DEVIATION < 1e-4 and 1e-4 or MAX_DEVIATION

    local TIME_STEP = dt
    ros.INFO("generateTrajectory from waypoints with max dev: %08f, dt %08f", MAX_DEVIATION, TIME_STEP)
    local path = {}
    path[1] = optimplan.Path(waypoints, MAX_DEVIATION)
    local suc, split, scip = path[1]:analyse()
    waypoints = path[1].waypoints
    if not suc and #scip > 0 then
        local indeces = torch.ByteTensor(waypoints:size(1)):fill(1)
        for i, v in ipairs(scip) do
            indeces[v] = 0
        end
        local newIndeces = {}
        for i = 1, indeces:size(1) do
            if indeces[i ] == 1 then
                newIndeces[#newIndeces+1] = i
            end
        end
        waypoints = waypoints:index(1,torch.LongTensor(newIndeces))

        path[1] = optimplan.Path(waypoints, MAX_DEVIATION)
        suc, split, scip = path[1]:analyse()
        if(#scip > 0) then
            ros.WARN("check max deviation parameter... can propably be reduced")
        end
    end
    if not suc and #split > 0 then
        ros.INFO("splitting plan")
        path = {}
        local startI = 1
        for i, v in ipairs(split) do
            path[#path + 1] = optimplan.Path(waypoints[{{startI, v}, {}}], MAX_DEVIATION)
            startI = v
        end
        path[#path + 1] = optimplan.Path(waypoints[{{startI, waypoints:size(1)}, {}}], MAX_DEVIATION)
    end
    local trajectory = {}
    local valid = true

    for i = 1, #path do
        trajectory[i] = optimplan.Trajectory(path[i], maxVelocities, maxAccelerations, TIME_STEP)
        trajectory[i]:outputPhasePlaneTrajectory()
        if not trajectory[i]:isValid() then
            valid = false
        else
            ros.INFO("Generation of Trajectories: %d%s", i/#path*100, '%')
        end
    end
    return trajectory, valid
end

local function sample(traj, dt)
    ros.INFO("Resample Trajectory with dt = %f",dt)
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
    ros.INFO("Trajectory num points = %d, duration %s",time:size(1),tostring(time[time:size(1)]))
    return time, pos, vel, acc
end

local function queryJointTrajectoryServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        ros.ERROR("Only one waypoint detected. Abort Trajectory generation.")
        response.error_code.val = -2
        return true
    end
    if request.waypoints[1].positions:nDimension() < 1 then
        ros.ERROR("Waypoints have wrong Dimensions")
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
    local traj,
        valid =
        generateTrajectory(waypoints, request.max_velocity, request.max_acceleration, request.max_deviation, request.dt)

    if not valid then
        ros.ERROR("Generated Trajectory is not valid!")
        response.error_code.val = -2
        return true
    else
        ros.INFO("Generated Trajectory is valid!")
    end

    local time, pos, vel, acc = sample(traj, request.dt)print(time:size(1))
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
