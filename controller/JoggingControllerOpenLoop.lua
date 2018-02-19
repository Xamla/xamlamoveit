local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'
local datatypes = require 'xamlamoveit.datatypes'
--local tvpController = require 'xamlamoveit.controller.MultiAxisTvpController2'
local tvpController = require 'xamlamoveit.controller.MultiAxisCppController'
local xutils = require 'xamlamoveit.xutils'
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
local cartesian_twist_spec = ros.MsgSpec('geometry_msgs/TwistStamped')
local feedback_spec = ros.MsgSpec('xamlamoveit_msgs/ControllerState')

local function lookupPose(link_name, base_link_name)
    local base_link_name = base_link_name or 'base_link'
    if transformListener:frameExists(base_link_name) and transformListener:frameExists(link_name) then
        transformListener:waitForTransform(base_link_name, link_name, ros.Time(0), ros.Duration(0.1), true)
        return true, transformListener:lookupTransform(base_link_name, link_name, ros.Time(0))
    else
        return false, tf.StampedTransform()
    end
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

---
--@param desired joint angle position
local function sendPositionCommand(self, q_des, q_dot, names)
    for i, v in ipairs(self.controller_list) do
        if table.isSimilar(names, v.joints) then
            local current_id = v.name
            local myTopic = string.format('/%s/joint_command', v.name)
            --ros.WARN(myTopic)
            if publisherPointPositionCtrl[v.name] == nil then
                publisherPointPositionCtrl[v.name] = self.nh:advertise(myTopic, joint_pos_spec)
                if publisherPointPositionCtrl[v.name]:getNumSubscribers() == 0 then
                    publisherPointPositionCtrl[v.name]:shutdown()
                    publisherPointPositionCtrl[v.name] = nil
                    --ros.WARN('Special case detected. Subscriber on %s is not available. Searching in namespace for alternatives.', myTopic)
                    local ns = string.split(v.name, '/')
                    myTopic = string.format('/%s/joint_command', ns[1])
                    publisherPointPositionCtrl[ns[1]] = self.nh:advertise(myTopic, joint_pos_spec)
                    if publisherPointPositionCtrl[ns[1]]:getNumSubscribers() >= 0 then
                        --ros.INFO('found alternative topic at: %s', myTopic)
                        current_id = ns[1]
                    else
                        publisherPointPositionCtrl[ns[1]]:shutdown()
                        publisherPointPositionCtrl[ns[1]] = nil
                        myTopic = string.format('/%s/joint_command', v.name)
                        publisherPointPositionCtrl[v.name] = self.nh:advertise(myTopic, joint_pos_spec)
                        current_id = v.name
                    end
                end
            end

            local m = ros.Message(joint_pos_spec)
            local mPoint = ros.Message(joint_point_spec)

            m.joint_names = {}
            for ii = 1, q_des:size(1) do
                m.joint_names[ii] = names[ii]
            end
            mPoint.positions:set(q_des)
            mPoint.velocities:set(q_dot)
            mPoint.time_from_start = self.dt
            m.points = {mPoint}
            publisherPointPositionCtrl[current_id]:publish(m)
            ros.DEBUG('sendPositionCommand to: ' .. publisherPointPositionCtrl[current_id]:getTopic())
            return
        end
    end
    --TODO find correct publisher for corresponding joint value set controllers
end

local function sendFeedback(self)
    ros.DEBUG('sendFeedback to: ' .. self.publisher_feedback:getTopic())

    local rel_pose = self.target_pose:mul(self.current_pose:inverse())

    local tmp = torch.zeros(6)
    tmp[{{1, 3}}]:copy(rel_pose:getOrigin())
    tmp[{{4, 6}}]:copy(rel_pose:getRotation():getAxisAngle())
    self.feedback_message.cartesian_distance:set(tmp)
    self.feedback_message.converged = self.controller.converged
    self.publisher_feedback:publish(self.feedback_message)
end

local function resetGoals(self)
    self.goals = {pose_goal = nil, posture_goal = nil, twist_goal = nil}
end

local controller = require 'xamlamoveit.controller.env'
local JoggingControllerOpenLoop = torch.class('xamlamoveit.controller.JoggingControllerOpenLoop', controller)

function JoggingControllerOpenLoop:__init(node_handle, joint_monitor, move_group, ctr_list, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    end
    self.FIRSTPOINT = true
    self.dt_monitor = core.MonitorBuffer(100, 1)
    self.nh = node_handle
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.kinematic_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.kinematic_model)
    self.lock_client = core.LeasedBaseLockClient(node_handle)

    self.joint_monitor = joint_monitor

    self.move_groups = {}
    self.curr_move_group_name = nil
    if move_group then
        self.move_groups[move_group:getName()] = move_group
        self.curr_move_group_name = move_group:getName()
    else
        error('move_group should not be nil')
    end
    self.state = move_group:getCurrentState()
    self.time_last = ros.Time.now()
    self.joint_set = datatypes.JointSet(move_group:getActiveJoints():totable())

    self.lastCommandJointPositions =
        createJointValues(
        self.joint_set.joint_names,
        self.joint_monitor:getPositionsOrderedTensor(self.joint_set.joint_names)
    )
    self.controller = tvpController.new(#self.joint_set.joint_names)

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

    self.controller_list = ctr_list

    self.goals = {pose_goal = nil, posture_goal = nil, twist_goal = nil}

    self.resource_lock = nil
    self.subscriber_pose_goal = nil
    self.subscriber_posture_goal = nil
    self.subscriber_twist_goal = nil
    self.publisher_feedback = nil
    self.mode = 0
    self.opt = {stiffness = 10.0, damping = 2.0}
    self.max_vel = nil
    self.max_acc = nil
    self.max_speed_scaling = 0.75 --50% of the max constraints allowed for jogging
    self.speed_scaling = 1.0
    self.command_distance_threshold = 0.05 --m
    self.command_rotation_threshold = math.rad(0.5)
    self.joint_step_width = math.rad(0.5)
    local link_name = self.move_groups[self.curr_move_group_name]:getEndEffectorLink()
    if #link_name > 0 then
        self.current_pose = self.state:getGlobalLinkTransform(link_name)
        self.target_pose = self.state:getGlobalLinkTransform(link_name)
    else
        self.current_pose = tf.Transform()
        self.target_pose = tf.Transform()
    end
    self.timeout = ros.Duration(1.0)

    transformListener = tf.TransformListener()
    self.feedback_message = nil
end

function JoggingControllerOpenLoop:setTimeout(value)
    if type(value) == 'number' then
        if value < 0 then
            error('Invalid argument: positive Number expected')
        else
            self.timeout = ros.Duration(value)
        end
    elseif not torch.isTypeOf(value, ros.Duration) then
        error('Invalid argument: Instance of ros.Duration expected')
    else
        self.timeout = value
    end
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

function JoggingControllerOpenLoop:getTwistGoal()
    local newMessage = false
    local twist = nil
    local msg = nil
    local success = true
    while self.subscriber_twist_goal:hasMessage() and ros.ok() do
        ros.DEBUG('NEW TWIST MESSAGE RECEIVED')
        msg = self.subscriber_twist_goal:read()
    end
    if msg then
        twist = torch.Tensor(6)
        twist[1] = msg.twist.linear.x
        twist[2] = msg.twist.linear.y
        twist[3] = msg.twist.linear.z
        twist[4] = msg.twist.angular.x
        twist[5] = msg.twist.angular.y
        twist[6] = msg.twist.angular.z
        if #msg.header.frame_id > 0 then
            local transform
            success, transform = lookupPose(msg.header.frame_id, 'world')
            if success then
                twist[{{1, 3}}] = transform:getBasis() * twist[{{1, 3}}]
                twist[{{4, 6}}] = transform:getBasis() * twist[{{4, 6}}]
                twist:mul(self.speed_scaling)
            else
                twist:zero()
            end
        end
        newMessage = true
    end
    return newMessage, twist, success
end

function JoggingControllerOpenLoop:getPoseGoal()
    local newMessage = false
    local pose = nil
    local msg = nil
    while self.subscriber_pose_goal:hasMessage() and ros.ok() do
        ros.DEBUG('NEW POSE MESSAGE RECEIVED')
        msg = self.subscriber_pose_goal:read()
    end
    if msg then
        pose = msg2StampedTransform(msg)
        newMessage = true
    end
    return newMessage, pose
end

function JoggingControllerOpenLoop:getPostureGoal()
    local newMessage = false
    local msg = nil
    local joint_posture = self.lastCommandJointPositions:clone()
    while self.subscriber_posture_goal:hasMessage() and ros.ok() do
        ros.DEBUG('NEW POSTURE MESSAGE RECEIVED')
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
                vel[i] = math.sign(vel[i]) * self.joint_step_width * self.speed_scaling
            end
            local q_dot = datatypes.JointValues(joint_set, vel)
            joint_posture:add(q_dot)
        end
    end
    return newMessage, joint_posture
end

function JoggingControllerOpenLoop:getCurrentPose()
    local link_name = self.move_groups[self.curr_move_group_name]:getEndEffectorLink()
    if #link_name > 0 then
        return self.state:getGlobalLinkTransform(link_name)
    else
        ros.WARN('no pose available since no link_name for EndEffector exists.')
        return tf.Transform()
    end
end

local function satisfiesBounds(self, positions, joint_names)
    local state = self.state:clone()
    state:setVariablePositions(positions, joint_names)
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
            return false, 'Target position is out of bounds!!'
        else
            return false, 'Self Collision detected!!'
        end
    end
    return true, 'Success'
end

--@param desired joint angle position
function JoggingControllerOpenLoop:isValid(q_des, q_curr, joint_names) -- avoid large jumps in posture values and check if q_des tensor is valid
    local diff = 2
    if q_des:nDimension() > 0 then
        ros.DEBUG('q_des checked')
        if satisfiesBounds(self, q_des, joint_names) then
            ros.DEBUG('satisfiesBounds')
            if q_curr then
                diff = torch.norm(q_curr - q_des)
            end
        else
            ros.INFO('does not satisfy bounds')
            self.feedback_message.error_code = -2 --SELFCOLLISION
        end
    else
        --ros.INFO('q_des not checked')
        self.feedback_message.error_code = -1 --INVALID_IK
    end
    --ros.WARN(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 2
end

function JoggingControllerOpenLoop:setSpeedScaling(value)
    self.speed_scaling = math.max(0.001, math.min(1, value))
    self.controller.max_vel = self.max_vel * self.speed_scaling
    self.controller.max_acc = self.max_acc
end

function JoggingControllerOpenLoop:connect(joint_topic, pose_topic, twist_topic)
    local pose_topic = pose_topic or 'goal_pose'
    local joint_topic = joint_topic or 'goal_posture'
    local twist_topic = twist_topic or 'goal_twist'

    self.subscriber_pose_goal = self.nh:subscribe(pose_topic, cartesian_pose_spec, 1)
    self.subscriber_posture_goal = self.nh:subscribe(joint_topic, joint_pos_spec, 1)
    self.subscriber_twist_goal = self.nh:subscribe(twist_topic, cartesian_twist_spec, 1)
    self.publisher_feedback = self.nh:advertise('feedback', feedback_spec, 1)
    self.feedback_message = self.publisher_feedback:createMessage()
    self:getNewRobotState()
    self.max_vel,
        self.max_acc =
        queryJointLimits(self.nh, self.lastCommandJointPositions:getNames(), '/robot_description_planning')
    self.max_vel = self.max_vel * self.max_speed_scaling
    self.max_acc = self.max_acc --* self.max_speed_scaling
    return true
end

function JoggingControllerOpenLoop:shutdown()
    self.subscriber_pose_goal:shutdown()
    self.subscriber_posture_goal:shutdown()
    self.subscriber_twist_goal:shutdown()
    self.publisher_feedback:shutdown()
end

function JoggingControllerOpenLoop:tracking(q_des, duration)
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    duration = ros.Duration(0.0)
    local state = self.state:clone()

    if self:isValid(q_des.values, self.lastCommandJointPositions.values, self.lastCommandJointPositions:getNames()) then
        assert(
            self.controller.state.pos:size(1) == q_des.values:size(1),
            string.format('inconsistent size: %dx%d', self.controller.state.pos:size(1), q_des.values:size(1))
        )
        self.controller:update(q_des.values, self.dt:toSec())
        sendPositionCommand(self, self.controller.state.pos, self.controller.state.vel, q_des:getNames(), duration)
        self.lastCommandJointPositions:setValues(q_des:getNames(), q_des.values)
    else
        for i, v in pairs(publisherPointPositionCtrl) do
            v:shutdown()
            v = nil
        end
        publisherPointPositionCtrl = {}
        ros.ERROR('command is not valid!!!')
    end
    self.feedback_message.joint_distance:set(q_des.values - self.controller.state.pos)
end


function JoggingControllerOpenLoop:getStep(D_force, D_torques, timespan)
    local D_torques = D_torques or torch.zeros(3)

    local K, D = self.opt.stiffness, self.opt.damping
    --stiffness and damping
    local s = timespan or ros.Duration(0.0)
    local offset = -D_force / (K + D * s:toSec())
    local x_rot_des = (-D_torques / (K + D * s:toSec())) -- want this in EE KO
    local suc

    local vel6D = torch.DoubleTensor(6)
    vel6D[{{1, 3}}]:copy(offset)
    vel6D[{{4, 6}}]:copy(x_rot_des)
    local q_dot_des, jacobian = self:getQdot(vel6D * self.dt:toSec())

    ros.DEBUG(string.format('-------------> time: %d', s:toSec()))
    ros.DEBUG(string.format('-------------> D_force %s', tostring(D_force)))
    ros.DEBUG(string.format('-------------> q_dot_des %s', tostring(q_dot_des)))

    return q_dot_des, jacobian
end

function JoggingControllerOpenLoop:getNewRobotState()
    local names = self.lastCommandJointPositions:getNames()
    local ok, p, l = self.joint_monitor:getNextPositionsOrderedTensor(ros.Duration(0.1), names)
    assert(ok, '[getNewRobotState] exceeded timeout for next robot joint state.')
    if #names ~= p:size(1) then
        ros.ERROR('Miss match: ')
        print(names, p)
        return
    end
    self.state:setVariablePositions(p, names)
    self.state:update()
    self.lastCommandJointPositions:setValues(names, p)
    self.current_pose = self:getCurrentPose()
end

local function tryLock(self)
    if self.resource_lock == nil then
        ros.DEBUG('lock resources')
        self.resource_lock = self.lock_client:lock(self.state:getVariableNames():totable())
        ros.DEBUG('locked resources')
    else
        local dur = ros.Duration((self.resource_lock.expiration:toSec() - self.resource_lock.created:toSec()) / 2)
        if dur > (self.resource_lock.expiration - ros.Time.now()) then
            self.resource_lock = self.lock_client:lock(self.state:getVariableNames():totable(), self.resource_lock.id)
        end
    end

    if self.resource_lock.success then
        return true
    else
        ros.WARN('could not lock resources')
        self.resource_lock = nil
        return false
    end
end

local function transformPose2PostureTarget(self, pose_goal, joint_names)
    local world_link_name = 'world'
    local link_name = self.move_groups[self.curr_move_group_name]:getEndEffectorLink()
    local posture_goal
    if #link_name > 0 then
        self.start_time = ros.Time.now()
        self.target_pose = pose_goal:clone()
        local state = self.state:clone()
        state:setFromIK(self.move_groups[self.curr_move_group_name]:getName(), pose_goal:toTransform())
        posture_goal =
            createJointValues(state:getVariableNames():totable(), state:getVariablePositions()):select(joint_names)
    else
        ros.WARN("Cannot uses this move group: '%s' since it has no EndEffector Link. ")
    end
    return posture_goal
end

function JoggingControllerOpenLoop:update()
    -- Handle feedback
    self.feedback_message.error_code = 1
    local status_msg = 'OK'
    local q_dot = self.lastCommandJointPositions:clone()
    q_dot.values:zero()
    if self.controller.converged then
        self.mode = 0
        self:getNewRobotState()
        --self.target_pose = self.current_pose:clone()
        self.controller:reset()
        self.controller.state.pos:copy(self.lastCommandJointPositions.values)
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

    local new_twist_message, twist_goal, transformed_successful = self:getTwistGoal()

    if self.dt:toSec() * 2 < (ros.Time.now() - self.start_time):toSec() then
        ros.DEBUG('reset goals')
        resetGoals(self)
    end

    -- Decide which goal is valid
    if (self.mode < 2 and new_pose_message) or (self.mode == 1 and self.goals.pose_goal) then
        if pose_goal ~= nil then
            self.goals.pose_goal = pose_goal:clone()
        elseif self.goals.pose_goal then
            pose_goal = self.goals.pose_goal:clone()
        end
        local rel_poseAB = pose_goal:clone()
        rel_poseAB = rel_poseAB:mul(self.current_pose:inverse())
        if rel_poseAB:getOrigin():norm() > self.command_distance_threshold then
            local off = rel_poseAB:getOrigin()
            off:div(off:norm()):mul(self.command_distance_threshold)
            rel_poseAB:setOrigin(off)
        end
        local tmp = tf.StampedTransform(rel_poseAB:mul(self.current_pose))
        local posture_tmp_goal = transformPose2PostureTarget(self,tmp , q_dot:getNames())
        if posture_tmp_goal and self.lastCommandJointPositions then
            q_dot = posture_tmp_goal - self.lastCommandJointPositions
            self.mode = 1
        else
            ros.ERROR('transformation from pose to posture failed')
        end

        if q_dot.values:norm() > 1 then
            ros.ERROR('detected jump in IK. ')
            self.feedback_message.error_code = -1
            q_dot.values:zero()
        end
        self.controller.converged = false
    elseif ((self.mode == 0 or self.mode == 2) and new_posture_message) or (self.mode == 2 and self.goals.posture_goal) then
        if posture_goal ~= nil then
            self.goals.posture_goal = posture_goal:clone()
        elseif self.goals.posture_goal then
            posture_goal = self.goals.posture_goal:clone()
        end
        self.mode = 2
        self.start_time = ros.Time.now()
        local tmp_state = self.state:clone()
        --print('posture_goal ', posture_goal:getNames(), posture_goal.values)
        q_dot = posture_goal:clone()
        q_dot:sub(self.lastCommandJointPositions)
        self.controller.converged = false
    elseif ((self.mode == 0 or self.mode == 3) and new_twist_message) or (self.mode == 3 and self.goals.twist_goal)  then
        if twist_goal ~= nil then
            self.goals.twist_goal = twist_goal:clone()
        elseif self.goals.twist_goal then
            twist_goal = self.goals.twist_goal:clone()
        end
        local rel_tmp_pose = tf.StampedTransform()
        local dt = self.dt:toSec()

        local quaternion = rel_tmp_pose:getRotation()
        if (twist_goal[{{4, 6}}]):norm() > 0 then
            quaternion:setRotation(twist_goal[{{4, 6}}] * dt, (twist_goal[{{4, 6}}] * dt):norm())
        end
        rel_tmp_pose:setRotation(quaternion)
        rel_tmp_pose:setOrigin(twist_goal[{{1, 3}}])

        local rel_poseAB = rel_tmp_pose:clone()
        if rel_poseAB:getOrigin():norm() > self.command_distance_threshold * dt * 2 then
            --print(rel_poseAB:toTensor())
            local off = rel_poseAB:getOrigin()
            off:div(off:norm()):mul(self.command_distance_threshold * dt * 2 )
            rel_poseAB:setOrigin(off)
        end
        local tmp = tf.StampedTransform(rel_poseAB:mul(self.current_pose))
        tmp:setOrigin(rel_poseAB:getOrigin() + self.current_pose:getOrigin() )
        tmp:setRotation(rel_poseAB:getRotation() * self.current_pose:getRotation())
        local posture_tmp_goal = transformPose2PostureTarget(self, tmp, q_dot:getNames())

        if posture_tmp_goal and self.lastCommandJointPositions then
            q_dot = posture_tmp_goal - self.lastCommandJointPositions
            self.mode = 3
        else
            ros.ERROR('transformation from pose to posture failed')
        end

        local rel_poseAB = self.target_pose:mul(self.current_pose:inverse())
        if q_dot.values:norm() > 1 or rel_poseAB:getOrigin():norm() > self.command_distance_threshold then
            ros.ERROR('detected jump in IK. ')
            self.feedback_message.error_code = -1
            q_dot.values:zero()
        end

        if transformed_successful == false then
            self.feedback_message.error_code = -18 --INVALID_LINK_NAME
        end

        self.controller.converged = false
    end

    local timeout_b = false
    if self.timeout:toSec() < (ros.Time.now() - self.start_time):toSec() then
        self.lastCommandJointPositions.values:copy(self.controller.state.pos)
        self.controller:reset()
        self.controller.state.pos:copy(self.lastCommandJointPositions.values)
        q_dot.values:zero()
        timeout_b = true
        ros.DEBUG('timeout')
    end

    local q_des = self.lastCommandJointPositions:clone()
    if not self.controller.converged and not timeout_b then
        q_des:add(q_dot)
    else
        self.mode = 0
    end
    ros.DEBUG('Locking')
    if tryLock(self) then
        self:tracking(q_des)
    end
    ros.DEBUG('Feedback')
    sendFeedback(self)
    return true, status_msg
end

function JoggingControllerOpenLoop:reset()
    ros.DEBUG('resetting ....')
    if publisherPointPositionCtrl then
        for i, v in pairs(publisherPointPositionCtrl) do
            ros.INFO('shutting down publisher for topic: %s', v:getTopic())
            v:shutdown()
            v = nil
        end
    end
    publisherPointPositionCtrl = {}
    local move_group = self:getCurrentMoveGroup()
    --self.state = move_group:getCurrentState()
    self.time_last = ros.Time.now()
    self.joint_set = datatypes.JointSet(move_group:getActiveJoints():totable())
    ros.INFO('Start Joint Monitor with %d joint names', #self.joint_set.joint_names)
    local link_name = move_group:getEndEffectorLink()
    if #link_name > 0 then
        ros.INFO('Found link_name: %s', link_name)
        self.current_pose = self.state:getGlobalLinkTransform(link_name)
        self.target_pose = self.state:getGlobalLinkTransform(link_name)
    else
        ros.WARN('no end effector')
        self.current_pose = tf.Transform()
        self.target_pose = tf.Transform()
    end
    local ok, p = self.joint_monitor:getNextPositionsOrderedTensor(0.1, self.joint_set.joint_names)
    assert(ok, '[reset] exceeded timeout for next robot joint state.')
    self.lastCommandJointPositions =
        createJointValues(
        self.joint_set.joint_names,
        p
    )
    self.max_vel, self.max_acc = queryJointLimits(self.nh, self.joint_set.joint_names, '/robot_description_planning')
    self.max_vel = self.max_vel * self.max_speed_scaling
    self.max_acc = self.max_acc --* self.max_speed_scaling

    self.mode = 0
    self.controller = tvpController.new(#self.joint_set.joint_names)
    self.controller.max_vel = self.max_vel
    self.controller.max_acc = self.max_acc
    self.controller.state.pos:copy(self.lastCommandJointPositions.values)
    resetGoals(self)
    ros.INFO('resetting finished successfully')
    return true
end

function JoggingControllerOpenLoop:releaseResources()
    if self.resource_lock ~= nil then
        self.lock_client:release(self.resource_lock)
        self.resource_lock = nil
    end
end

function JoggingControllerOpenLoop:setMoveGroupInterface(name)
    if not self.move_groups[name] then
        local tmp_move_group = moveit.MoveGroupInterface(name)
        self.move_groups[tmp_move_group:getName()] = tmp_move_group
    end
    local last_move_group_name = self.curr_move_group_name
    self.curr_move_group_name = name
    local ready = self:reset()
    if ready then
        --self:releaseResources()
        ros.INFO('Triggered reset with movegroup name: %s', self.move_groups[self.curr_move_group_name]:getName())
        return true, string.format('Successfully changed movegroup to %s', self.curr_move_group_name)
    else
        self.curr_move_group_name = last_move_group_name
        return false, string.format('Could not set moveGroup with name: %s. ', name)
    end
end

function JoggingControllerOpenLoop:getCurrentMoveGroup()
    return self.move_groups[self.curr_move_group_name]
end

return JoggingControllerOpenLoop
