local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'
local datatypes = require 'xamlamoveit.components.datatypes'
local tvpController = require 'xamlamoveit.controller.TvpController'
local transformListener
function math.sign(x)
    assert(type(x) == 'number')
    return x > 0 and 1 or x < 0 and -1 or 0
end

local publisherPointPositionCtrl = {}

local joint_point_spec = ros.MsgSpec('trajectory_msgs/JointTrajectoryPoint')
-- JointPosition --
local joint_pos_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local cartesian_pose_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
local feedback_spec = ros.MsgSpec('xamlamoveit_msgs/ControllerState')

local function lookupPose(linkName, baseLinkName)
    return transformListener:lookupTransform(baseLinkName or 'base_link', linkName, ros.Time(0))
end

local function transformVector(target_frame, frame_id, input)
    local transform = tf.StampedTransform()
    transform = lookupPose(target_frame, frame_id)
    local end_vector = torch.ones(4, 1)
    end_vector[{{1, 3}, 1}]:copy(input)
    local origin = torch.zeros(4, 1)
    origin[4] = 1
    local output = (transform:toTensor() * end_vector) - (transform:toTensor() * origin)
    return target_frame, output[{{1, 3}, 1}]:squeeze()
end

local function createJointValues(names, values)
    local joint_set = datatypes.JointSet(names)
    local joint_values = datatypes.JointValues(joint_set, values)
    return joint_values
end

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

local function isSubset(A, B)
    for ia, a in ipairs(A) do
        if table.indexof(B, a) == -1 then
            return false
        end
    end
    return true
end

local function isSimilar(A, B)
    if #A == #B then
        return isSubset(A, B)
    else
        return false
    end
    return true
end

---
--@param desired joint angle position
local function sendPositionCommand(self, q_des, q_dot, names)
    for i, v in ipairs(self.controller_list) do
        if isSimilar(names, v.joints) then
            local myTopic = string.format('/%s/joint_command', v.name)
            ros.WARN(myTopic)
            if publisherPointPositionCtrl[v.name] == nil then
                publisherPointPositionCtrl[v.name] = self.nh:advertise(myTopic, joint_pos_spec)
            end
            local m = ros.Message(joint_pos_spec)
            local mPoint = ros.Message(joint_point_spec)

            m.joint_names = {}
            for ii = 1, q_des:size(1) do
                m.joint_names[ii] = names[ii]
            end
            mPoint.positions:set(q_des)
            mPoint.velocities:set(q_dot) --TODO this is probably not optimal.
            mPoint.time_from_start = ros.Time.now() - BEGIN_EXECUTION
            m.points = {mPoint}
            publisherPointPositionCtrl[v.name]:publish(m)
            ros.INFO('sendPositionCommand to: ' .. publisherPointPositionCtrl[v.name]:getTopic())
            return
        end
    end
    --TODO find correct publisher for corresponding joint value set controllers
end

local function sendFeedback(self, group)
    --TODO find correct publisher for corresponding joint value set controllers
    ros.INFO('sendFeedback to: ' .. self.publisher_feedback:getTopic())
    local m = self.publisher_feedback:createMessage()
    local names = std.StringVector()
    self.move_group:getActiveJoints(names)
    m.joint_distance:set(torch.zeros(#names))
    m.cartesian_distance:set(torch.zeros(6))
    m.error_code = 1
    m.converged = self.controller.converged
    self.publisher_feedback:publish(m)
end

local controller = require 'xamlamoveit.controller.env'
local JoggingControllerOpenLoop = torch.class('xamlamoveit.controller.JoggingControllerOpenLoop', controller)

function JoggingControllerOpenLoop:__init(node_handle, move_group, ctr_list, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    end
    self.FIRSTPOINT = true
    self.dt_monitor = core.MonitorBuffer(100, 1)
    self.nh = node_handle
    self.move_group = move_group or error('move_group should not be nil')
    self.state = move_group:getCurrentState()
    self.joint_set = datatypes.JointSet(move_group:getActiveJoints():totable())
    self.joint_monitor = core.JointMonitor(self.joint_set.joint_names)
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

    self.controller_list = ctr_list

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
    self.max_speed_scaling = 0.25 -- 25% of the max constraints allowed for jogging
    self.speed_scaling = 1.0
    self.lastCommandJointPositions =
        createJointValues(
        self.joint_set.joint_names,
        self.joint_monitor:getPositionsOrderedTensor(self.joint_set.joint_names)
    )
    self.current_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.target_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.timeout = ros.Duration(10.5)
    self.controller = tvpController.new(#self.joint_set.joint_names)
    transformListener = tf.TransformListener()
end

function JoggingControllerOpenLoop:getInTopic()
    return self.subscriber_pose_goal:getTopic()
    -- self.subscriber_posture_goal:getTopic()
end

function JoggingControllerOpenLoop:getOutTopic()
    return string.format('/%s/joint_command', self.controller_list[1].name)
end

local function clamp(t, min, max)
    if torch.type(t) == 'number' then
        return math.max(math.min(t, max), min)
    end
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
        tf.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    )
    pose:set_stamp(msg.header.stamp)
    pose:set_frame_id(msg.header.frame_id)
    return pose
end

function JoggingControllerOpenLoop:getPoseGoal()
    local newMessage = false
    local pose = nil
    local msg = nil
    while self.subscriber_pose_goal:hasMessage() and ros.ok() do
        ros.WARN('NEW POSE MESSAGE RECEIVED')
        msg = self.subscriber_pose_goal:read()
    end
    if msg then
        pose = msg2StampedTransform(msg)
        newMessage = true
    end
    if newMessage then
        ros.INFO('getPoseGoal')
        print(pose)
    end
    return newMessage, pose
end

function JoggingControllerOpenLoop:getPostureGoal()
    local newMessage = false
    local msg = nil
    local joint_names = {}
    local joint_posture = self.lastCommandJointPositions:clone()
    while self.subscriber_posture_goal:hasMessage() and ros.ok() do
        ros.WARN('NEW POSTURE MESSAGE RECEIVED')
        msg = self.subscriber_posture_goal:read()
    end
    if msg then
        newMessage = true
        local joint_set = datatypes.JointSet(msg.joint_names)

        if msg.points[1].positions:nElement() > 0 then
            joint_posture = datatypes.JointValues(joint_set, msg.points[1].positions)
        elseif msg.points[1].velocities:nElement() > 0 then
            local vel = msg.points[1].velocities
            for i, v in ipairs(joint_set.joint_names) do
                vel[i] = math.sign(vel[i]) * 0.01 * self.speed_scaling
            end
            local q_dot = datatypes.JointValues(joint_set, vel)
            joint_posture:add(q_dot)
        end
    end
    return newMessage, joint_posture
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
        --positions:copy(state:copyJointGroupPositions(self.move_group:getName()):clone())
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
        ros.INFO('q_des checked')
        if satisfiesBounds(self, q_des) then
            ros.INFO('satisfiesBounds')
            if q_curr then
                diff = torch.norm(q_curr - q_des)
            end
        else
            ros.INFO('does not satisfy bounds')
            print(q_des)
        end
    else
        ros.INFO('q_des not checked')
        print(q_des)
    end
    ros.WARN(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 2
end

function JoggingControllerOpenLoop:connect(joint_topic, pose_topic)
    local pose_topic = pose_topic or 'goal_pose'
    local joint_topic = joint_topic or 'goal_posture'
    self.subscriber_pose_goal = self.nh:subscribe(pose_topic, cartesian_pose_spec, 1)
    self.subscriber_posture_goal = self.nh:subscribe(joint_topic, joint_pos_spec, 1)
    self.publisher_feedback = self.nh:advertise('feedback', feedback_spec, 1)
    self:getNewRobotState()
    self.max_vel,
        self.max_acc = queryJointLimits(self.nh, self.joint_monitor:getJointNames(), '/robot_description_planning')
    self.max_vel = self.max_vel * self.max_speed_scaling
    self.max_acc = self.max_acc --* self.max_speed_scaling
    self.controller.max_vel = self.max_vel
    self.controller.max_acc = self.max_acc
    return true
end

function JoggingControllerOpenLoop:shutdown()
    self.subscriber_pose_goal:shutdown()
    self.subscriber_posture_goal:shutdown()
end

local function scaleJointVelocity(vel, max_vel)
    local abs_vel = torch.abs(vel)
    local ratio = torch.cdiv(abs_vel, max_vel)
    local violates = ratio[abs_vel:gt(max_vel)]
    local scaling = 1
    if violates:nElement() > 0 then
        scaling = math.min(violates:min(), 1.0)
    end
    vel:mul(scaling)
end

function JoggingControllerOpenLoop:tracking(q_des, duration)
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    duration = duration or ros.Time.now() - BEGIN_EXECUTION
    local state = self.state:clone()
    --scaleJointVelocity(q_dot, self.max_vel)

    if self:isValid(q_des.values, self.lastCommandJointPositions.values) then
        self.controller:update(q_des.values, self.dt:toSec())
        sendPositionCommand(self, self.controller.state.pos, self.controller.state.vel, q_des:getNames(), duration)
        self.lastCommandJointPositions:setValues(q_des:getNames(), q_des.values)
    else
        for i, v in ipairs(publisherPointPositionCtrl) do
            v:shutdown()
        end
        publisherPointPositionCtrl = {}
        ros.ERROR('command is not valid!!!')
    end
end

function JoggingControllerOpenLoop:getStep(D_force, D_torques, timespan)
    local D_torques = D_torques or torch.zeros(3)

    local K, D = self.opt.stiffness, self.opt.damping
    --stiffness and damping
    local s = timespan or ros.Duration(1.0)
    local offset = -D_force / (K + D * s:toSec())
    local x_rot_des = (-D_torques / (K + D * s:toSec())) -- want this in EE KO
    local suc

    --offset, x_rot_des = targetTransformation(self.current_pose, offset, x_rot_des)

    local vel6D = torch.DoubleTensor(6)
    vel6D[{{1, 3}}]:copy(offset)
    vel6D[{{4, 6}}]:copy(x_rot_des)
    local q_dot_des, jacobian = self:getQdot(vel6D * self.dt:toSec())

    ros.DEBUG(string.format('-------------> time: %d', s:toSec()))
    ros.DEBUG(string.format('-------------> D_force %s', tostring(D_force)))
    ros.DEBUG(string.format('-------------> q_dot_des %s', tostring(q_dot_des)))

    return q_dot_des, jacobian
end

function JoggingControllerOpenLoop:getQdot(vel6D)
    local jac = self.state:getJacobian(self.move_group:getName())
    local joint_names = self.state:getVariableNames():totable()
    local inv_jac = pseudoInverse(jac)
    local valid, cn = checkConditionNumber(inv_jac, self.singularity_threshold)
    local q_dot = createJointValues(joint_names, inv_jac * vel6D)
    if not valid then
        ros.DEBUG('detected ill-conditioned Jacobian. Robot is close to singularity slowing down. CN: %f', cn)
        q_dot = createJointValues(joint_names, (inv_jac * vel6D))
    end
    return q_dot, jac
end

function JoggingControllerOpenLoop:getNewRobotState()
    local p, l = self.joint_monitor:getNextPositionsTensor()
    self.state:setVariablePositions(p, self.joint_monitor:getJointNames())
    self.state:update()
    self.lastCommandJointPositions:setValues(self.joint_monitor:getJointNames(), p)
    self.current_pose = self:getCurrentPose()
end

function JoggingControllerOpenLoop:update()
    -- Handle feedback
    local status_msg = 'OK'
    local q_dot = self.lastCommandJointPositions:clone()
    q_dot.values:zero()
    if self.controller.converged then
        --self.controller:reset()
        --self.controller.state.pos:copy(self.lastCommandJointPositions.values)
        self.mode = 0
        self:getNewRobotState()
        self.target_pose = self.current_pose:clone()
    else
        self.state:setVariablePositions(
            self.lastCommandJointPositions.values,
            self.lastCommandJointPositions:getNames()
        )
        self.state:update()
        self.current_pose = self:getCurrentPose()
    end
    local curr_time = ros.Time.now()
    -- Handle goals
    local new_pose_message, pose_goal = self:getPoseGoal()

    local new_posture_message, posture_goal = self:getPostureGoal()
    -- Decide which goal is valid
    if self.mode < 2 and new_pose_message then
        self:getNewRobotState()
        local world_link_name = 'world'
        local link_name = self.move_group:getEndEffectorLink()
        self.mode = 1
        self.start_time = ros.Time.now()
        self.target_pose = pose_goal:clone()

        local rel_pose = self.target_pose:mul(self.current_pose:inverse())
        local pos_frame_id, deltaPosition = transformVector(world_link_name, link_name , rel_pose:getOrigin())
        local ax_frame_id,
            deltaAxis = transformVector(world_link_name, link_name, rel_pose:getRotation():getAxisAngle())
        q_dot = self:getStep(deltaPosition, deltaAxis, curr_time - self.start_time)
        self.controller.converged = false
    elseif (self.mode == 0 or self.mode == 2) and new_posture_message then
        self.mode = 2
        self.start_time = ros.Time.now()
        local tmp_state = self.state:clone()
        print('posture_goal ', posture_goal:getNames(), posture_goal.values)
        q_dot = posture_goal:clone()
        q_dot:sub(self.lastCommandJointPositions)
        self.controller.converged = false
    end

    if self.timeout:toSec() < (ros.Time.now() - self.start_time):toSec() then
        self.lastCommandJointPositions.values:copy(self.controller.state.pos)
        self.controller.state.vel:zero()
        self.controller.state.acc:zero()
        q_dot.values:zero()
    end

    if not self.controller.converged then
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
            local q_des = self.lastCommandJointPositions:clone()
            q_des:add(q_dot)
            self:tracking(q_des)
            sendFeedback(self)
        else
            ros.WARN('could not lock resources')
            self.resource_lock = nil
        end
        ros.INFO('NOT CONVERGED')
    else
        self.mode = 0
        if self.resource_lock ~= nil then
            self.lock_client:release(self.resource_lock)
            self.resource_lock = nil
        end
    end

    return true, status_msg
end

function JoggingControllerOpenLoop:setSpeed(speed)
    ros.INFO('Set speed to %s', speed)
end

function JoggingControllerOpenLoop:reset(timeout)
    self.state = self.move_group:getCurrentState()
    self.joint_monitor:shutdown()
    self.time_last = ros.Time.now()
    self.joint_set = datatypes.JointSet(self.move_group:getActiveJoints():totable())
    self.joint_monitor = core.JointMonitor(self.joint_set.joint_names)
    self.current_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    self.target_pose = self.state:getGlobalLinkTransform(self.move_group:getEndEffectorLink())
    if self.joint_monitor:waitReady(timeout or ros.Duration(0.1)) then
        self.lastCommandJointPositions =
            createJointValues(
            self.joint_set.joint_names,
            self.joint_monitor:getNextPositionsOrderedTensor(timeout or ros.Duration(0.1), self.joint_set.joint_names)
        )
        self.controller = tvpController.new(#self.joint_set.joint_names)
        self.controller.max_vel = self.max_vel
        self.controller.max_acc = self.max_acc
        self.controller.state.pos:copy(self.lastCommandJointPositions.values)
        return true
    end
    return false
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
