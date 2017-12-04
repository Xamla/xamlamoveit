local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'

function math.sign(x)
    assert(type(x) == 'number')
    return x > 0 and 1 or x < 0 and -1 or 0
end

local publisherPointPositionCtrl

local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
-- JointPosition --
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local cartesian_pose_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
local feedback_spec = ros.MsgSpec('xamlamoveit_msgs/ControllerState')

local function queryJointLimits(node_handle, joint_names, namespace)
    namespace = namespace or node_handle:getNamespace()
    local max_vel = torch.zeros(#joint_names)
    local max_acc = torch.zeros(#joint_names)
    local nh = node_handle
    local root_path = string.format('%s/joint_limits', namespace) -- robot_description_planning
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

---
--@param desired joint angle position
local function sendPositionCommand(q_des, q_dot, group)
    --TODO find correct publisher for corresponding joint value set controllers
    ros.INFO('sendPositionCommand to: ' .. publisherPointPositionCtrl:getTopic())
    local m = ros.Message(joint_pos_spec)
    local mPoint = ros.Message(joint_point_spec)
    local names = std.StringVector()

    group:getActiveJoints(names)

    m.joint_names = {}
    for ii = 1, q_des:size(1) do
        m.joint_names[ii] = names[ii]
    end
    mPoint.positions:set(q_des)
    mPoint.velocities:set(q_dot) --TODO this is probably not optimal.
    mPoint.time_from_start = ros.Time.now() - BEGIN_EXECUTION
    m.points = {mPoint}

    publisherPointPositionCtrl:publish(m)
end

local function sendFeedback(self, group)
    --TODO find correct publisher for corresponding joint value set controllers
    ros.INFO('sendFeedback to: ' .. self.publisher_feedback:getTopic())
    local m = ros.Message(feedback_spec)
    local names = std.StringVector()
    self.move_group:getActiveJoints(names)
    m.joint_distance:set(torch.zeros(#names))
    m.cartesian_distance:set(torch.zeros(6))
    m.error_code = 1
    m.converged = self.converged
    self.publisher_feedback:publish(m)
end

local controller = require 'xamlamoveit.controller.env'
local JoggingControllerOpenLoop = torch.class('xamlamoveit.controller.JoggingControllerOpenLoop', controller)

function JoggingControllerOpenLoop:__init(node_handle, move_group, ctr_name, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    end
    self.FIRSTPOINT = true
    self.dt_monitor = core.MonitorBuffer(100, 1)
    self.nh = node_handle
    self.move_group = move_group or error('move_group should not be nil')
    self.state = move_group:getCurrentState()
    self.lastCommandJointPositons = self.state:copyJointGroupPositions(move_group:getName()):clone()
    self.current_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.target_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.converged = false
    self.joint_monitor = core.JointMonitor(move_group:getActiveJoints():totable())
    self.time_last = ros.Time.now()

    local ready = false
    while not ready and ros.ok() do
        ready = self.joint_monitor:waitReady(0.01)
        ros.ERROR('joint states not ready')
    end

    print(self.joint_monitor:getNextPositionsTensor())

    if torch.type(dt) == 'number' or torch.type(dt) == 'nil' then
        self.dt = ros.Duration(dt)
    elseif torch.type(dt) == 'nil' then
        self.dt = ros.Duration(0.01)
    elseif torch.type(dt) == 'ros.Duration' then
        self.dt = dt
    else
        error('dt has unsupported type')
    end

    self.ctrl = planning.MoveitPlanning(node_handle, move_group, dt)
    self.start_time = ros.Time.now()
    BEGIN_EXECUTION = ros.Time.now()

    self.controller_name = ctr_name or 'pos_based_pos_traj_controller'

    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.kinematic_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.kinematic_model)
    self.lock_client = core.LeasedBaseLockClient(node_handle)
    self.resource_lock = nil
    self.subscriber_pose_goal = nil
    self.subscriber_posture_goal = nil
    self.publisher_feedback = nil
    self.mode = 0
    self.singularity_threshold = 3.5
    self.opt = {stiffness = 1.0, damping = 0.2}
    self.max_vel = nil
    self.max_acc = nil
    self.speed_scaling = 0.25 -- 25% of the max constraints allowed for jogging
end

function JoggingControllerOpenLoop:getInTopic()
    return {self.subscriber_pose_goal:getTopic(), self.subscriber_posture_goal:getTopic()}
end

function JoggingControllerOpenLoop:getOutTopic()
    return string.format('/%s/joint_command', self.controller_name)
end

local function clamp(t, min, max)
    return torch.cmin(t, max):cmax(min)
end

--- calculates the weighted pseudoInverse of M
-- @param M: Matrix which needs to be inversed
-- @param W: weight Matrix. (Optional)
local function pseudoInverse(M, W)
    local weights = W or torch.eye(M:size(1))
    assert(M:size(1) == weights:size()[1], 'Data matrix M and weight matrix W need to have the same number of cols')
    local inv = M:t() * weights * M
    -- make it definite
    inv:add(1e-15, torch.eye(inv:size(1)))
    return torch.inverse(inv) * M:t() * weights
end

local function checkConditionNumber(jacobian, singularity_threshold)
    local e = torch.eig(jacobian)
    local eigenValues = torch.abs(e:select(2, 1))
    local cn = eigenValues:max() / eigenValues:min()
    return cn <= singularity_threshold, cn
end

local function msg2StampedTransform(msg)
    local pose = tf.StampedTransform()
    pose:setOrigin(torch.Tensor {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z})
    pose:setRotation(
        torch.Tensor {
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        }
    )
    pose:set_stamp(msg.header.stamp)
    pose:set_frame_id( msg.header.frame_id)
    return pose
end
function JoggingControllerOpenLoop:getPoseGoal()
    local newMessage = false
    local pose = nil
    while self.subscriber_pose_goal:hasMessage() do
        ros.WARN('NEW POSE MESSAGE RECEIVED')
        newMessage = true
        local msg = self.subscriber_pose_goal:read()
        pose = msg2StampedTransform(msg)
    end
    return newMessage, pose
end

function JoggingControllerOpenLoop:getPostureGoal()
    local newMessage = false
    local msg = nil
    local joint_names = {}
    local joint_posture = nil
    while self.subscriber_posture_goal:hasMessage() do
        ros.WARN('NEW POSTURE MESSAGE RECEIVED')
        newMessage = true
        msg = self.subscriber_posture_goal:read()
        joint_names = msg.joint_names
        joint_posture = msg.points[1].positions
    end
    return newMessage, joint_posture, joint_names
end

function JoggingControllerOpenLoop:getCurrentPose()
    return self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
end

local function satisfiesBounds(self, positions)
    local state = self.state:clone()
    state:setVariablePositions(positions, self.joint_monitor:getJointNames())
    state:update()
    local collisions = self.planning_scene:checkSelfCollision(state)
    if state:satisfiesBounds(0.0) then
        if collisions then
            ros.ERROR('Self Collision detected')
            return false, 'Self Collision detected!!'
        end
    else
        state:enforceBounds()
        state:update()
        positions:copy(state:copyJointGroupPositions(self.move_group:getName()):clone())
        collisions = self.planning_scene:checkSelfCollision(state)
        if not collisions then
            ros.WARN('Target position is out of bounds!!')
            return true, 'Target position is out of bounds!!'
        else
            return false, 'Self Collision detected!!'
        end
    end
    return true, 'Success'
end

--@param desired joint angle position
function JoggingControllerOpenLoop:isValid(q_des, q_curr) -- avoid large jumps in posture values and check if q_des tensor is valid
    local diff = 2
    if q_des:nDimension() > 0 then
        ros.DEBUG('q_des checked')
        if satisfiesBounds(self, q_des) then
            ros.DEBUG('satisfiesBounds')
            if q_curr then
                diff = torch.norm(q_curr - q_des)
            end
        end
    end
    ros.DEBUG(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 1
end

function JoggingControllerOpenLoop:connect(pose_topic, joint_topic)
    local pose_topic = pose_topic or 'goal_pose'
    local joint_topic = joint_topic or 'goal_posture'
    self.subscriber_pose_goal = self.nh:subscribe(pose_topic, cartesian_pose_spec, 1)
    self.subscriber_posture_goal = self.nh:subscribe(joint_topic, joint_pos_spec, 1)
    self.publisher_feedback = self.nh:publisher('feedback', feedback_spec, 1)
    self:getNewRobotState()
    self.max_vel, self.max_acc = queryJointLimits(self.nh, self.joint_monitor:getJointNames())
    self.max_vel = self.max_vel * self.speed_scaling
    self.max_acc = self.max_acc * self.speed_scaling
    return true
end

function JoggingControllerOpenLoop:shutdown()
    self.subscriber_pose_goal:shutdown()
    self.subscriber_posture_goal:shutdown()
end

function JoggingControllerOpenLoop:tracking(q_dot, duration)
    ros.INFO('tracking')
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    duration = duration or ros.Time.now() - BEGIN_EXECUTION
    local group = self.move_group
    local state = self.state:clone()

    local q_des = self.lastCommandJointPositons + q_dot

    if not self.converged then
        if self:isValid(q_des, self.lastCommandJointPositons) then
            if not publisherPointPositionCtrl then
                local myTopic = string.format('/%s/joint_command', self.controller_name)
                ros.WARN(myTopic)
                publisherPointPositionCtrl = self.nh:advertise(myTopic, joint_pos_spec)
            end
            if self.FIRSTPOINT then
                sendPositionCommand(self.lastCommandJointPositons, q_dot:zero(), group, duration)
                self.FIRSTPOINT = false
                ros.DEBUG('FIRSTPOINT')
            else
                ros.DEBUG('GoGoGo')
                sendPositionCommand(q_des, q_dot, group, duration)
            end
            self.lastCommandJointPositons:copy(q_des)
        else
            --os.exit()
            if publisherPointPositionCtrl then
                publisherPointPositionCtrl:shutdown()
            end
            publisherPointPositionCtrl = nil
            ros.ERROR('command is not valid!!!')
        end
    else
        self.lastCommandJointPositons = state:copyJointGroupPositions(group:getName()):clone()
    end
    if self.debug then
        ros.DEBUG('q_curr')
        ros.DEBUG(tostring(self.lastCommandJointPositons))
        ros.DEBUG('q_des')
        ros.DEBUG(tostring(q_des))
        ros.DEBUG('q_dot')
        ros.DEBUG((tostring(q_dot)))
    end

    if q_dot:norm() < 1e-12 then
        self.converged = true
        --BEGIN_EXECUTION = ros.Time.now()
        self.FIRSTPOINT = true
    else
        self.converged = false
        self.FIRSTPOINT = false
    end
end

function targetTransformation(target_frame, offset, rotation_rpy)
    local tmp_offset_tf = tf.Transform():setOrigin(offset)
    local rotation = tmp_offset_tf:getRotation()

    local curr_pose_inv = target_frame:clone():inverse()
    local curr_pose = target_frame:clone()
    tmp_offset_tf:fromTensor(curr_pose:toTensor() * tmp_offset_tf:toTensor() * curr_pose_inv:toTensor())

    return tmp_offset_tf:getOrigin(), rotation_rpy
end

function JoggingControllerOpenLoop:getStep(D_force, D_torques, timespan)
    local D_torques = D_torques or torch.zeros(3)

    local K, D = self.opt.stiffness, self.opt.damping
    --stiffness and damping
    local s = timespan or ros.Duration(1.0)
    local offset = -D_force / (K + D * s:toSec())
    local x_rot_des = (-D_torques / (K + D * s:toSec())) -- want this in EE KO
    local suc

    offset, x_rot_des = targetTransformation(self.current_pose, offset, x_rot_des)

    local vel6D = torch.DoubleTensor(6)
    vel6D[{{1, 3}}]:copy(offset)
    vel6D[{{4, 6}}]:copy(x_rot_des)
    local q_dot_des, jacobian = self:getQdot(vel6D / self.dt:toSec())

    ros.DEBUG(string.format('-------------> time: %d', s:toSec()))
    ros.DEBUG(string.format('-------------> D_force %s', tostring(D_force)))
    ros.DEBUG(string.format('-------------> q_dot_des %s', tostring(q_dot_des)))

    return q_dot_des, jacobian
end

function JoggingControllerOpenLoop:getQdot(vel6D)
    local jac = self.state:getJacobian(self.move_group:getName())

    local inv_jac = pseudoInverse(jac)
    local valid, cn = checkConditionNumber(jac, self.singularity_threshold)
    if not valid then
        ros.WARN(string.format('detected ill-conditioned Jacobian. Robot is close to singularity slowing down.'))
        return inv_jac * (vel6D * self.dt:toSec() * self.singularity_threshold / cn), jac
    end

    return inv_jac * (vel6D * self.dt:toSec()), jac
end

function JoggingControllerOpenLoop:getNewRobotState()
    local p, l = self.joint_monitor:getNextPositionsTensor()
    self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
    self.state:update()
    self.lastCommandJointPositons:copy(p)
    self.current_pose = self:getCurrentPose()
end

function JoggingControllerOpenLoop:update()
    -- Handle feedback
    local mode = self.mode
    if self.converged then
        mode = 0
        self:getNewRobotState()
    else
        self.state:setVariablePositions(self.lastCommandJointPositons, self.joint_monitor:getJointNames())
        self.state:update()
        self.current_pose = self:getCurrentPose()
    end

    -- Handle goals
    local new_pose_message, pose_goal = self:getPoseGoal()
    local new_posture_message, posture_goal, joint_names = self:getPostureGoal()
    local newMessege = new_pose_message or new_posture_message
    -- Decide which goal is valid
    if mode == 0 and new_pose_message then
        mode = 1
    elseif mode == 0 and new_posture_message then
        mode = 2
    end

    local deltaPosition = torch.zeros(3)
    local deltaRPY = torch.zeros(3)

    if mode == 1 then
        deltaPosition = pose_goal:getOrigin() - self.current_pose:getOrigin()
        deltaRPY = (pose_goal:getRotation() - self.current_pose:getRotation()):getRPY()
        self.target_pose = pose_goal:clone()
    elseif mode == 2 then
        local tmp_state = self.state:clone()
        tmp_state:setVariablePositions(posture_goal, joint_names)
        tmp_state:update()
        local tmp_pose = tmp_state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
        deltaPosition = tmp_pose:getOrigin() - self.current_pose:getOrigin()
        deltaRPY = (tmp_pose:getRotation() - self.current_pose:getRotation()):getRPY()
        self.target_pose = tmp_pose:clone()
    end

    -- only reset the timer if almost zero changes are applied to avoid rapid accellarations
    if (torch.norm(deltaPosition) + torch.norm(deltaRPY)) < 1e-3 then
        ros.DEBUG('Reset start time')
        self.start_time = ros.Time.now()
    end

    local curr_time = ros.Time.now()
    local q_dot = self:getStep(deltaPosition, deltaRPY, curr_time - self.start_time)
    if self.dt:toSec() > 0.15 then
        ros.WARN('dt is to large !! dt = %f', self.dt:toSec())
        newMessege = false
        q_dot:zero()
    end

    if newMessege == true then
        ros.INFO('new Messege')
        if self.resource_lock == nil then
            ros.INFO('lock resources')
            self.resource_lock = self.lock_client:lock(self.state:getVariableNames():totable())
        else
            local dur = ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
            if dur > (self.resource_lock.expiration - ros.Time.now()) then
                self.resource_lock =
                    self.lock_client:lock(self.state:getVariableNames():totable(), self.resource_lock.id)
            end
        end

        if self.resource_lock.success then
            self:tracking(q_dot:clone(), self.dt)
        else
            ros.WARN('could not lock resources')
            self.resource_lock = nil
        end
    else
        if self.resource_lock ~= nil then
            local dur = ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
            if dur > (self.resource_lock.expiration - ros.Time.now()) then
                print(self.resource_lock)
                self.lock_client:release(self.resource_lock)
                self.resource_lock = nil
            end
        end
    end
    sendFeedback(self)
end

function JoggingControllerOpenLoop:setSpeed(speed)
    ros.INFO(string.format('Set speed to %s', speed))
end

function JoggingControllerOpenLoop:reset(timeout)
    self.state = self.move_group:getCurrentState()
    self.joint_monitor:shutdown()
    self.joint_monitor = core.JointMonitor(self.move_group:getActiveJoints():totable())
    self.time_last = ros.Time.now()
    return self.joint_monitor:waitReady(timeout or ros.Duration(0.1))
end

function JoggingControllerOpenLoop:setMoveGroupInterface(name)
    self.move_group = moveit.MoveGroupInterface(name)
    local ready = self:reset(ros.Duration(0.01))
    local move_group_name = self.move_group:getName()
    if ready and (move_group_name == name) then
        return true, 'Successfully changed movegroup.'
    else
        if move_group_name ~= name then
            return false, string.Format('Could not set moveGroup with name: %s. ', name)
        end
        if not ready then
            return false, 'joint state is not available in time.'
        end
    end
end

return JoggingControllerOpenLoop
