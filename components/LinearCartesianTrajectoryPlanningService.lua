local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local optimplan = require 'optimplan'
local Controller = require 'xamlamoveit.controller'
local Datatypes = require 'xamlamoveit.datatypes'
local Xutils = require 'xamlamoveit.xutils'
local error_codes = require 'xamlamoveit.core.ErrorCodes'.error_codes
error_codes = table.merge(error_codes, table.swapKeyValue(error_codes))
local TvpController = Controller.TaskSpaceController
--require 'xamlamoveit.components.RosComponent'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetLinearCartesianTrajectory')
local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')

local function table_concat(dst, src)
    for i, v in ipairs(src) do
        table.insert(dst, v)
    end
    return dst
end

local function createJointValues(names, values)
    assert(
        torch.type(names) == 'table',
        string.format('Joint names should be of type table but has type %s', torch.type(names))
    )
    assert(torch.isTypeOf(values, torch.DoubleTensor))
    local joint_set = Datatypes.JointSet(names)
    local joint_values = Datatypes.JointValues(joint_set, values)
    return joint_values
end

local function getEndEffectorMoveGroupMap(kinematic_model)
    local move_group_names = kinematic_model:getJointModelGroupNames()
    local map = {}
    for k, v in pairs(move_group_names) do
        local name, suc = kinematic_model:getGroupEndEffectorName(v)
        if suc then
            map[v] = name
        end
    end
    return table.merge(map, table.swapKeyValue(map))
end

local function createPoseMsg(frame, translation, rotation)
    assert(torch.type(frame) == 'string')
    assert(torch.isTypeOf(translation, torch.DoubleTensor))
    assert(torch.isTypeOf(rotation, torch.DoubleTensor))
    local msg = ros.Message(pose_msg_spec)
    msg.pose.position.x = translation[1]
    msg.pose.position.y = translation[2]
    msg.pose.position.z = translation[3]
    msg.pose.orientation.x = rotation[1]
    msg.pose.orientation.y = rotation[2]
    msg.pose.orientation.z = rotation[3]
    msg.pose.orientation.w = rotation[4]
    msg.header.frame_id = frame
    return msg
end

local function poses2MsgArray(points)
    local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
    local result = {}
    if torch.type(points) == 'table' then
        for i, v in ipairs(points) do
            print(v)
            assert(
                torch.isTypeOf(v, tf.StampedTransform),
                string.format('points need to be type of tf.StampedTransform, but is of type: [%s]', torch.type(v))
            )

            local translation = v:getOrigin()
            local rotation = v:getRotation():toTensor()
            local frame = v:get_frame_id()

            table.insert(result, createPoseMsg(frame, translation, rotation))
        end
    elseif torch.isTypeOf(points, tf.StampedTransform) then
        local translation = points:getOrigin()
        local rotation = points:getRotation():toTensor()
        local frame = points:get_frame_id()

        table.insert(result, createPoseMsg(frame, translation, rotation))
    else
        error('[poses2MsgArray] unknown type of points parameter: ' .. torch.type(points))
    end
    return result
end

local function poseStampedMsg2StampedTransform(msg)
    local result = tf.StampedTransform()
    result:setOrigin(torch.Tensor {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z})
    result:setRotation(
        tf.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    )
    return result
end

local function poseTo6DTensor(input)
    local new_input
    if torch.isTypeOf(input, tf.StampedTransform) then
        input = input:toTransform()
    end

    if torch.isTypeOf(input, tf.Transform) then
        new_input = torch.zeros(6)
        new_input[{{1, 3}}] = input:getOrigin()
        new_input[{{4, 6}}] = input:getRotation():getAxis() * input:getRotation():getAngle()
    end
    if torch.isTypeOf(input, torch.DoubleTensor) then
        new_input = input
    end
    assert(
        torch.isTypeOf(new_input, torch.DoubleTensor),
        string.format('Input should be of type [torch.DoubleTensor] but is of type: [%s]', torch.type(new_input))
    )
    return new_input
end

local function tensor6DToPose(vector6D)
    assert(vector6D:size(1) == 6, 'Vector should be of size 6D (offset, anglevelocities)')
    local end_pose = tf.StampedTransform()
    end_pose:setOrigin(vector6D[{{1, 3}}])
    if vector6D[{{4, 6}}]:norm() > 1e-12 then
        local end_pose_rotation = end_pose:getRotation()
        end_pose:setRotation(end_pose_rotation:setRotation(vector6D[{{4, 6}}], vector6D[{{4, 6}}]:norm()))
    end
    return end_pose
end

local function convertTwist2Pose(twists)
    local result = {}
    for i, twist in ipairs(twists) do
        result[i] = tensor6DToPose(twist.pos)
        result[i]:set_frame_id('world')
    end
    return result
end

local function pose2jointTrajectory(
    self,
    seed,
    joint_names,
    poses6D,
    end_effector_name,
    max_deviation,
    collision_check,
    ik_jump_threshold)
    local code = error_codes.SUCCESS
    if collision_check == false then
        ros.WARN('[pose2jointTrajectory]Collision checks are disabled')
    end
    local move_group = self.end_effector_map[end_effector_name]
    local result = {}
    if not move_group then
        return result, 0
    end

    self.robot_state:setVariablePositions(seed, joint_names)
    self.robot_state:update()
    for i, pose in ipairs(poses6D) do
        --ros.INFO('set IK')
        local old_state = self.robot_state:copyJointGroupPositions(move_group)
        local suc = self.robot_state:setFromIK(move_group, tensor6DToPose(pose.pos):toTensor())
        if suc then
            self.robot_state:update()
            local colliding = false
            if collision_check then
                colliding = self.planning_scene:isStateColliding(move_group, self.robot_state, true)
            end
            if not colliding then
                local new_state = self.robot_state:copyJointGroupPositions(move_group)
                if torch.abs(old_state - new_state):gt(ik_jump_threshold):sum() > 0 then
                    ros.ERROR(
                        '[pose2jointTrajectory] Jump in IK detected. In move_group %s. Transformed %f%% of trajectory',
                        move_group,
                        100 * i / #poses6D
                    )
                    return result, error_codes.PLANNING_FAILED
                end
                local jac = self.robot_state:getJacobian(move_group)
                local tmp =
                    createJointValues(
                    self.robot_model:getGroupJointNames(move_group):totable(),
                    new_state
                )
                result[i] = {}
                result[i].pos = tmp:select(joint_names):getValues()
                result[i].vel = torch.inverse(jac) * poseTo6DTensor(pose.vel)
            else
                ros.ERROR(
                    '[pose2jointTrajectory] Collision detected in move_group %s. Transformed %f%% of trajectory',
                    move_group,
                    100 * i / #poses6D
                )

                return result, error_codes.GOAL_IN_COLLISION
            end
        else
            ros.ERROR(
                '[pose2jointTrajectory] Could not set IK solution for move_group %s. Transformed %f%% of trajectory',
                move_group,
                100 * i / #poses6D
            )
            return result, error_codes.NO_IK_SOLUTION
        end
    end
    return result, 1
end

local function getLinearPath(
    start,
    goal,
    dt,
    max_xyz_velocity,
    max_xyz_accelaration,
    max_angular_velocity,
    max_angular_accelaration)
    tic('getLinearPath')
    local start = poseStampedMsg2StampedTransform(start)
    local goal = poseStampedMsg2StampedTransform(goal)
    local controller = TvpController()
    local taskspace_max_vel = torch.ones(6) * 0.2 --m/s
    taskspace_max_vel[{{4, 6}}]:fill(math.pi / 2)
    local taskspace_max_acc = torch.ones(6) * 0.8 --m/s^2
    taskspace_max_acc[{{4, 6}}]:fill(math.pi)
    if max_xyz_velocity then
        taskspace_max_vel[{{1, 3}}]:fill(max_xyz_velocity)
    end
    if max_xyz_accelaration then
        taskspace_max_acc[{{1, 3}}]:fill(max_xyz_accelaration)
    end
    if max_angular_velocity then
        taskspace_max_vel[{{4, 6}}]:fill(max_angular_velocity)
    end
    if max_angular_accelaration then
        taskspace_max_acc[{{4, 6}}]:fill(max_angular_accelaration)
    end
    controller.max_vel:copy(taskspace_max_vel)
    controller.max_acc:copy(taskspace_max_acc)
    print(start, goal, controller.max_vel, controller.max_acc)
    local result = controller:generateOfflineTrajectory(start, goal, dt or 0.01)
    assert(#result > 1)
    toc('getLinearPath')
    return result
end

local function generateTrajectory(waypoints, dt)
    local time_step = dt or 0.008
    time_step = math.max(time_step, 0.001)

    local time = torch.zeros(#waypoints)
    valid = true
    local pos = torch.zeros(#waypoints, 6)
    local vel = pos:clone()

    for i, point in ipairs(waypoints) do -- TODO check limits
        time[i] = dt * i
        pos[{i, {}}]:copy(point.pos)
        vel[{i, {}}]:copy(point.vel)
    end

    return valid, time, pos, vel
end

local function queryCartesianPathServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        request.error_code.val = -2
        return true
    end
    tic('generateTrajectory')
    self.planning_scene:syncPlanningScene()
    local g_path = {}
    if 2 == #request.waypoints then
        ros.INFO('only start und goal received. Controller runs with dt = %f', request.dt)
        local traj =
            getLinearPath(
            request.waypoints[1],
            request.waypoints[2],
            request.dt,
            request.max_xyz_velocity,
            request.max_xyz_accelaration,
            request.max_angular_velocity,
            request.max_angular_accelaration
        )
        if #traj > 0 then
            table_concat(g_path, traj)
        end
    else
        for i, v in ipairs(request.waypoints) do
            local k = math.min(#request.waypoints, i + 1)
            local w = request.waypoints[k]
            local traj =
                getLinearPath(
                v,
                w,
                request.dt,
                request.max_xyz_velocity,
                request.max_xyz_accelaration,
                request.max_angular_velocity,
                request.max_angular_accelaration
            )
            if #traj > 0 then
                table_concat(g_path, traj)
            end
        end
    end
    ros.INFO('got taskspace trajectory. Next convert to joint trajectory')
    local dim = request.seed.positions:size(1)
    local waypoints,
        code =
        pose2jointTrajectory(
        self,
        request.seed.positions,
        request.joint_names,
        g_path,
        request.end_effector_name,
        request.max_deviation,
        request.collision_check,
        request.ik_jump_threshold
    )
    if code < 0 then
        response.error_code.val = code
        return true
    end

    local valid, time, pos, vel, acc = generateTrajectory(waypoints, request.dt)
    toc('generateTrajectory')

    ros.INFO('Generated Trajectory is valid!')


    response.solution.joint_names = request.joint_names
    for i = 1, time:size(1) do
        response.solution.points[i] = ros.Message('trajectory_msgs/JointTrajectoryPoint')
        response.solution.points[i].positions = pos[i]
        response.solution.points[i].velocities = vel[i]
        if acc then
            response.solution.points[i].accelerations = acc[i]
        else
            response.solution.points[i].accelerations = vel[i]:zero()
        end
        response.solution.points[i].time_from_start = ros.Duration(time[i])
    end

    response.error_code.val = 1
    return true
end

local components = require 'xamlamoveit.components.env'
local LinearCartesianTrajectoryPlanningService,
    parent =
    torch.class(
    'xamlamoveit.components.LinearCartesianTrajectoryPlanningService',
    'xamlamoveit.components.RosComponent',
    components
)

function LinearCartesianTrajectoryPlanningService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.info_server = nil
    self.robot_model = nil
    self.planning_scene = nil
    self.robot_state = nil
    self.all_group_joint_names = nil
    self.joint_limits = {}
    self.end_effector_map = nil
    parent.__init(self, node_handle)
end

function LinearCartesianTrajectoryPlanningService:onInitialize()
    local robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.robot_model)
    self.planning_scene:syncPlanningScene()
    self.end_effector_map = getEndEffectorMoveGroupMap(self.robot_model)
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.all_group_joint_names = self.robot_model:getJointModelGroupNames()
    self.joint_limits.pos, self.joint_limits.vel, self.joint_limits.acc = self.robot_model:getVariableBounds()
    self.variable_names = self.robot_model:getActiveJointNames():totable()
    self.joint_limits.pos = createJointValues(self.variable_names, self.joint_limits.pos[{{}, 1}])
    self.joint_limits.vel = createJointValues(self.variable_names, self.joint_limits.vel[{{}, 1}])
    self.joint_limits.acc = createJointValues(self.variable_names, self.joint_limits.acc[{{}, 1}])
    collectgarbage()
end

function LinearCartesianTrajectoryPlanningService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_cartesian_trajectory',
        srv_spec,
        function(request, response, header)
            return queryCartesianPathServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function LinearCartesianTrajectoryPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming LinearCartesianTrajectoryPlanningService call')
        self.callback_queue:callAvailable()
    end
end

function LinearCartesianTrajectoryPlanningService:onStop()
    ros.WARN('LinearCartesianTrajectoryPlanningService:onStop() NOT IMPLEMENTED')
end

function LinearCartesianTrajectoryPlanningService:onReset()
    self.info_server:shutdown()
end

function LinearCartesianTrajectoryPlanningService:onShutdown()
    self.info_server:shutdown()
end

return LinearCartesianTrajectoryPlanningService