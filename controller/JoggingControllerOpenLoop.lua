--[[
JoggingControllerOpenLoop.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local tf = ros.tf
local planning = require 'xamlamoveit.planning'
local core = require 'xamlamoveit.core'
local datatypes = require 'xamlamoveit.datatypes'
local tvpController = require 'xamlamoveit.controller.MultiAxisTvpController2'
local tvpPoseController = require 'xamlamoveit.controller.MultiAxisTvpController2'
local xutils = require 'xamlamoveit.xutils'

local error_codes = {
    OK = 1,
    INVALID_IK = -1,
    SELF_COLLISION = -2,
    SCENE_COLLISION = -3,
    FRAME_TRANSFORM_FAILURE = -4,
    IK_JUMP_DETECTED = -5,
    CLOSE_TO_SINGULARITY = -6,
    JOINT_LIMITS_VIOLATED = -7,
    INVALID_LINK_NAME = -8,
    TASK_SPACE_JUMP_DETECTED = -9
}

local jogging_node_tracking_states = {
    IDLE = 0,
    POSTURE = 2,
    POSE = 1,
    TWIST = 3
}

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
    local max_min_pos = torch.zeros(#joint_names, 2)
    local nh = node_handle
    local root_path = string.format('%s/joint_limits', namespace) -- robot_description_planning
    for i, name in ipairs(joint_names) do
        local has_pos_param = string.format('/%s/%s/has_position_limits', root_path, name)
        local get_max_pos_param = string.format('/%s/%s/max_position', root_path, name)
        local get_min_pos_param = string.format('/%s/%s/min_position', root_path, name)
        local has_vel_param = string.format('/%s/%s/has_velocity_limits', root_path, name)
        local get_vel_param = string.format('/%s/%s/max_velocity', root_path, name)
        local has_acc_param = string.format('/%s/%s/has_acceleration_limits', root_path, name)
        local get_acc_param = string.format('/%s/%s/max_acceleration', root_path, name)
        local param_has_pos = nh:getParamVariable(has_pos_param)
        if param_has_pos then
            max_min_pos[i][1] = nh:getParamVariable(get_max_pos_param)
            max_min_pos[i][2] = nh:getParamVariable(get_min_pos_param)
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end

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
    return max_min_pos, max_vel, max_acc
end

local function queryTaskSpaceLimits(self, name)
    local name = name or ''
    local max_vel = torch.zeros(3)
    local max_acc = torch.zeros(3)
    local nh = self.nh
    local success = true
    local joint_max, joint_min, position_max, position_min, rotation_max, rotation_min
    position_max = 0.2
    position_min = 0.001
    rotation_max = math.rad(5)
    rotation_min = math.rad(0.1)
    joint_max = math.rad(5)
    joint_min = math.rad(0.1)

    local parameters = nh:getParamVariable('end_effector_list')
    if parameters == nil then
        ros.WARN_THROTTLE('noEE', 1, 'no end_effector_list set for node')
        return false
    end
    local index =
        table.findIndex(
        parameters,
        function(x)
            return x.name == name
        end
    )
    if parameters[index] == nil then
        ros.WARN_THROTTLE(name, 1, 'no end_effector_list set for %s', name)
        return false
    end

    parameters = parameters[index]
    if parameters.has_taskspace_xyz_vel_limit == true then
        max_vel:fill(parameters.taskspace_xyz_max_vel)
        print(1.1 * parameters.taskspace_xyz_max_vel * self.dt:toSec(), position_max)
        position_max = math.min(1.1 * parameters.taskspace_xyz_max_vel, position_max)
        position_min = math.min(position_max / 10, position_min)
    else
        --m/s
        max_vel:fill(0.2)
    end
    if parameters.has_taskspace_xyz_acc_limit == true then
        max_acc:fill(parameters.taskspace_xyz_max_acc)
    elseif parameters.has_taskspace_xyz_vel_limit == true then
        max_acc = max_vel * 2.0
    end

    if parameters.has_position_step == true then
        position_max = parameters.position_step_max
        position_min = parameters.position_step_min
    end
    if parameters.has_rotation_step == true then
        rotation_max = parameters.rotation_step_max
        rotation_min = parameters.rotation_step_min
    end
    if parameters.has_jointspace_step == true then
        joint_max = parameters.jointspace_step_max
        joint_min = parameters.jointspace_step_min
    end

    self:setStepWidthModel(joint_max, joint_min, position_max, position_min, rotation_max, rotation_min)
    return success, max_vel, max_acc
end

local function createPublisher(self, names)
    for i, v in ipairs(self.controller_list) do
        if table.isSimilar(names, v.joints) then
            self.current_id = v.name
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
                        self.current_id = ns[1]
                        return
                    else
                        publisherPointPositionCtrl[ns[1]]:shutdown()
                        publisherPointPositionCtrl[ns[1]] = nil
                        myTopic = string.format('/%s/joint_command', v.name)
                        publisherPointPositionCtrl[v.name] = self.nh:advertise(myTopic, joint_pos_spec)
                        self.current_id = v.name
                        return
                    end
                end
            end
        end
    end
    for i, v in ipairs(self.controller_list) do
        if table.isSubset(names, v.joints) then
            self.current_id = v.name
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
                        self.current_id = ns[1]
                        return
                    else
                        publisherPointPositionCtrl[ns[1]]:shutdown()
                        publisherPointPositionCtrl[ns[1]] = nil
                        myTopic = string.format('/%s/joint_command', v.name)
                        publisherPointPositionCtrl[v.name] = self.nh:advertise(myTopic, joint_pos_spec)
                        self.current_id = v.name
                        return
                    end
                end
            end
        end
    end
    --TODO find correct publisher for corresponding joint value set controllers
end

---
--@param desired joint angle position
local function sendPositionCommand(self, q_des, q_dot, names, dt)
    assert(torch.isTypeOf(dt, ros.Duration))

    local m = ros.Message(joint_pos_spec)
    local mPoint = ros.Message(joint_point_spec)

    m.joint_names = {}
    for ii = 1, q_des:size(1) do
        m.joint_names[ii] = names[ii]
    end
    mPoint.positions:set(q_des)
    mPoint.velocities:set(q_dot)
    mPoint.time_from_start = ros.Duration()
    m.points = {mPoint}
    if publisherPointPositionCtrl[self.current_id] then
        --ros.INFO('sendPositionCommand to: ' .. publisherPointPositionCtrl[self.current_id]:getTopic())
        publisherPointPositionCtrl[self.current_id]:publish(m)
    else
        ros.WARN('could not find topic at: %s', self.current_id)
    end
end

local function sendFeedback(self, changing)
    ros.DEBUG('sendFeedback to: ' .. self.publisher_feedback:getTopic())

    local rel_pose = self.target_pose:mul(self.current_pose:inverse())

    local tmp = torch.zeros(6)
    tmp[{{1, 3}}]:copy(rel_pose:getOrigin())
    tmp[{{4, 6}}]:copy(rel_pose:getRotation():getAxisAngle())
    tmp:apply(
        function(x)
            if x ~= x then
                return 0
            end
        end
    ) -- replace nan values by 0

    -- if last error was not ok
    if self.feedback_message.error_code == error_codes.OK and self.last_error_code ~= error_codes.OK then
        -- override ok status with last error if no new valid command was set
        if self.controller.converged or changing < 1e-10 then
            self.feedback_message.error_code = self.last_error_code
        else
            -- store error code if last valid command request produced an error
            self.last_error_code = self.feedback_message.error_code
        end
    else
        -- store error code if last valid command request produced an error
        self.last_error_code = self.feedback_message.error_code
    end


    self.feedback_message.cartesian_distance:set(tmp)
    self.feedback_message.converged = self.controller.converged
    self.feedback_message.self_collision_check_enabled = not self.no_self_collision_check
    self.feedback_message.scene_collision_check_enabled = not self.no_scene_collision_check
    self.feedback_message.joint_limits_check_enabled = not self.no_joint_limits_check
    self.publisher_feedback:publish(self.feedback_message)
end

local function resetGoals(self)
    self.goals = {pose_goal = nil, posture_goal = nil, twist_goal = nil}
end

local function setStopGoals(self)
    if self.goals.pose_goal then
        self.goals.pose_goal:setOrigin(self.goals.pose_goal:getOrigin() * 0 / 0)
    end
    if self.goals.posture_goal then
        self.goals.posture_goal = nil
    end
    if self.goals.twist_goal then
        self.goals.twist_goal:zero()
    end
end

local function getEndEffectorMoveGroupMap(self)
    local move_group_names = self.kinematic_model:getJointModelGroupNames()
    local map = {}
    local linkmap = {}
    for k, v in pairs(move_group_names) do
        local name, suc = self.kinematic_model:getGroupEndEffectorName(v)
        local sub_move_group_ids = self.kinematic_model:getJointModelSubGroupNames(v)
        local attached_end_effectors = {} --self.kinematic_model:getAttachedEndEffectorNames(v)
        if suc then
            local link_name, suc2 = self.kinematic_model:getEndEffectorLinkName(name)
            if name ~= nil and name ~= '' then
                map[name] = {v}
                linkmap[name] = link_name
            end
            for i, aee in ipairs(attached_end_effectors) do
                link_name, suc2 = self.kinematic_model:getEndEffectorLinkName(aee)
                if suc2 then
                    ros.INFO('GroupName: %s EndEffectorName: %s LinkName: %s', v, aee, link_name)
                    if map[aee] == nil then
                        map[aee] = {}
                    end
                    table.insert(map[aee], v)
                    linkmap[aee] = link_name
                end
            end
        end
    end
    return map, linkmap
end

local controller = require 'xamlamoveit.controller.env'
local determinant = controller.determinant
local detectNan = controller.detectNan
local JoggingControllerOpenLoop = torch.class('xamlamoveit.controller.JoggingControllerOpenLoop', controller)

function JoggingControllerOpenLoop:__init(node_handle, joint_monitor, move_group, ctr_list, dt, debug)
    self.debug = debug or false
    if self.debug then
        ros.console.set_logger_level(nil, ros.console.Level.Debug)
    end
    self.nh = node_handle
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.kinematic_model = self.robot_model_loader:getModel()
    self.end_effector_to_move_group_map, self.end_effector_to_link_map = getEndEffectorMoveGroupMap(self)
    self.planning_scene = moveit.PlanningScene(self.kinematic_model)
    self.lock_client = core.LeaseBasedLockClient(node_handle)
    self.current_id = ''
    self.joint_monitor = joint_monitor

    self.move_groups = {}
    self.curr_move_group_name = nil
    self.curr_end_effector_name = nil
    if move_group then
        self.move_groups[move_group:getName()] = move_group
        self.curr_move_group_name = move_group:getName()
        local key =
            table.findFirst(
            self.end_effector_to_move_group_map,
            function(x)
                return 0 <
                    table.findIndex(
                        x,
                        function(x, ix)
                            return x == self.curr_move_group_name
                        end
                    )
            end
        )
        self.curr_end_effector_name = key
    else
        error('move_group should not be nil')
    end
    self.state = move_group:getCurrentState()
    self.time_last = ros.Time.now()
    self.joint_set = datatypes.JointSet(move_group:getActiveJoints():totable())
    local ok, values = self.joint_monitor:getNextPositionsOrderedTensor(ros.Duration(0.5), self.joint_set.joint_names)
    self.lastCommandJointPositions = createJointValues(self.joint_set.joint_names, values)
    self.controller = tvpController.new(#self.joint_set.joint_names)
    self.taskspace_controller = tvpPoseController.new(3)
    self.dt = nil
    self:setDeltaT(dt)
    self.start_time = ros.Time.now()
    self.last_command_send_time = ros.Time.now()
    self.start_cool_down_time = ros.Time.now()

    self.controller_list = ctr_list

    self.goals = {pose_goal = nil, posture_goal = nil, twist_goal = nil}

    self.resource_lock = nil
    self.subscriber_pose_goal = nil
    self.subscriber_posture_goal = nil
    self.subscriber_twist_goal = nil
    self.publisher_feedback = nil
    self.mode = jogging_node_tracking_states.IDLE
    self.joint_limits = nil
    self.taskspace_max_vel = torch.ones(3) * 0.2 --m/s
    self.taskspace_max_acc = torch.ones(3) * 0.8 --m/s^2

    self.max_speed_scaling = 0.85 --85% of the max constraints allowed for jogging
    self.speed_scaling = 1.0
    --(joint_max, joint_min, position_max, position_min, rotation_max, rotation_min)
    self.step_width_model = nil
    self:setStepWidthModel(math.rad(5), math.rad(0.1), 0.2, 0.001, math.rad(5), math.rad(0.1))
    self.command_distance_threshold, self.command_rotation_threshold = self.step_width_model.taskspace.scaling_fkt(1)
    self.joint_step_width = self.step_width_model.joint_space.scaling_fkt(1)
    self.current_pose = nil --tf.Transform()
    self.target_pose = nil -- tf.Transform()
    self.timeout = ros.Duration(0.25)
    self.cool_down_timeout = ros.Duration(0.3)
    self.synced = false
    self.no_self_collision_check = false
    self.no_scene_collision_check = false
    self.no_joint_limits_check = false

    transformListener = tf.TransformListener()
    self.feedback_message = nil
    self.last_error_code = error_codes.OK
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
    return string.format('/%s/joint_command', self.current_id)
end

local function clamp(t, min, max)
    if torch.type(t) == 'number' then
        return math.max(math.min(t, max), min)
    end
    return torch.cmin(t, max):cmax(min)
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
            else
                ros.ERROR('Twist message is invalid. lookupPose failed to find transformation')
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
        if #msg.joint_names < 1 then
            ros.WARN('received posture message without joint names specified')
            return false, nil
        elseif msg.joint_names[1] == '' then
            ros.WARN('received posture message without joint names specified')
            return false, nil
        end
        newMessage = true
        local joint_set = datatypes.JointSet(msg.joint_names)

        if msg.points[1].positions:nElement() > 0 then
            joint_posture = datatypes.JointValues(joint_set, msg.points[1].positions)
        elseif msg.points[1].velocities:nElement() > 0 then
            local vel = msg.points[1].velocities
            for i, v in ipairs(joint_set.joint_names) do
                vel[i] = math.sign(vel[i]) * self.joint_step_width
            end
            local q_dot = datatypes.JointValues(joint_set, vel)
            if q_dot.joint_set:isSubset(joint_posture.joint_set) then
                joint_posture:add(q_dot)
            else
                ros.WARN('jogging is not configure to receive this joint set: ' .. tostring(q_dot.joint_set))
            end
        end
    end
    return newMessage, joint_posture
end

function JoggingControllerOpenLoop:getCurrentPose()
    local link_name
    if self.curr_end_effector_name then
        link_name = self.end_effector_to_link_map[self.curr_end_effector_name]
    else
        link_name = self.move_groups[self.curr_move_group_name]:getEndEffectorLink()
    end
    if link_name and #link_name > 0 then
        return self.state:getGlobalLinkTransform(link_name)
    else
        ros.WARN('no pose available since no link_name for EndEffector exists.')
        return tf.Transform()
    end
end

local function satisfiesBounds(self, positions, joint_names)
    local state = self.state:clone()
    local tmp_joint_values = datatypes.JointValues(datatypes.JointSet(joint_names), positions)
    state:setVariablePositions(positions, joint_names)
    state:update()
    self.planning_scene:syncPlanningScene()
    local self_collisions = false
    if not self.no_self_collision_check then
        self_collisions = self.planning_scene:checkSelfCollision(state)
    end

    local is_state_colliding = false
    if not self.no_scene_collision_check then
        is_state_colliding = self.planning_scene:isStateColliding(nil, state, true)
    end

    local in_joint_limits = true
    if not self.no_joint_limits_check then
        in_joint_limits = self.joint_limits:satisfiesBounds(tmp_joint_values)
    end

    if in_joint_limits then
        if is_state_colliding or self_collisions then
            ros.ERROR('Collision detected')
            self:getFullRobotState()
            if self_collisions then
                self.feedback_message.error_code = error_codes.SELF_COLLISION
            else
                self.feedback_message.error_code = error_codes.SCENE_COLLISION
            end
            return false, 'Collision detected'
        end
    else
        state:enforceBounds()
        state:update()
        --positions:copy(state:copyJointGroupPositions(self.move_group:getName()):clone())
        self_collisions = not self.no_self_collision_check and self.planning_scene:checkSelfCollision(state)
        is_state_colliding =
            not self.no_scene_collision_check and self.planning_scene:isStateColliding(nil, state, true)
        self.feedback_message.error_code = error_codes.JOINT_LIMITS_VIOLATED
        if not (self_collisions or is_state_colliding) then
            ros.WARN('Target position is out of bounds')
            return false, 'Target position is out of bounds'
        else
            if self_collisions then
                self.feedback_message.error_code = error_codes.SELF_COLLISION
            else
                self.feedback_message.error_code = error_codes.SCENE_COLLISION
            end
            self:getFullRobotState()
            ros.WARN('Target position is out of bounds and Self Collision detected')
            return false, 'Target position is out of bounds and Self Collision detected'
        end
    end
    return true, 'Success'
end

--@param desired joint angle position
function JoggingControllerOpenLoop:isValid(q_des, q_curr, joint_names) -- avoid large jumps in posture values and check if q_des tensor is valid
    local diff = 2
    local success = true
    if q_des:nDimension() > 0 then
        ros.DEBUG('q_des checked')
        if satisfiesBounds(self, q_des, joint_names) then
            ros.DEBUG('satisfiesBounds')
            if q_curr then
                diff = torch.norm(q_curr - q_des)
            end
        else
            success = false
        end
    else
        --ros.INFO('q_des not checked')
        self.feedback_message.error_code = error_codes.INVALID_IK
        success = false
    end
    --ros.WARN(string.format('Difference between q_des and q_curr diff = %f', diff))
    return diff < 2 and success
end

function JoggingControllerOpenLoop:setStepWidthModel(
    joint_max,
    joint_min,
    position_max,
    position_min,
    rotation_max,
    rotation_min)
    local model = {joint_space = {}, taskspace = {position = {}, rotation = {}}}
    if (joint_max < joint_min) then
        ros.WARN('joint max and min values are not valid. specified min %f, max %f.', joint_min, joint_max)
        return self.step_width_model
    end

    if (position_max < position_min) then
        ros.WARN(
            'Cartesian postion step max and min values are not valid. specified min %f, max %f.',
            position_min,
            position_max
        )
        return self.step_width_model
    end

    if (rotation_max < rotation_min) then
        ros.WARN(
            'Cartesian rotation step max and min values are not valid. specified min %f, max %f.',
            rotation_min,
            rotation_max
        )
        return self.step_width_model
    end
    --joint_posture
    model.joint_space.min = joint_min
    model.joint_space.max = math.max(joint_max, math.rad(5))
    model.joint_space.scaling_fkt = function(x)
        return (joint_max - joint_min) * x + joint_min
    end
    --xyz model
    model.taskspace.position.min = position_min
    model.taskspace.position.max = position_max
    model.taskspace.position.scaling_fkt = function(x)
        return (position_max - position_min) * x + position_min
    end
    --rotation model
    model.taskspace.rotation.min = rotation_min
    model.taskspace.rotation.max = rotation_max
    model.taskspace.rotation.scaling_fkt = function(x)
        return (rotation_max - rotation_min) * x + rotation_min
    end
    model.taskspace.scaling_fkt = function(x)
        return model.taskspace.position.scaling_fkt(x), model.taskspace.rotation.scaling_fkt(x)
    end
    self.step_width_model = model
    return model
end

function JoggingControllerOpenLoop:setSpeedScaling(value)
    self.speed_scaling = math.max(0.001, math.min(1, value))

    self.joint_step_width = self.step_width_model.joint_space.scaling_fkt(value)

    self.command_distance_threshold, self.command_rotation_threshold =
        self.step_width_model.taskspace.scaling_fkt(self.speed_scaling)
    ros.INFO('Speed scaling %f, command threshold %f', self.speed_scaling, self.command_distance_threshold)
    self.controller.max_vel:copy(self.joint_limits:getMaxVelocities(self.joint_set:getNames()) * self.speed_scaling)
    self.controller.max_acc:copy(self.joint_limits:getMaxAccelerations(self.joint_set:getNames()))

    self.taskspace_controller.max_vel:copy(self.taskspace_max_vel * self.speed_scaling)
    self.taskspace_controller.max_acc:copy(self.taskspace_max_acc)
end

function JoggingControllerOpenLoop:connect(joint_topic, pose_topic, twist_topic)
    assert(self.step_width_model, 'Step width model was not configured please use setStepWidthModel')
    local pose_topic = pose_topic or 'goal_pose'
    local joint_topic = joint_topic or 'goal_posture'
    local twist_topic = twist_topic or 'goal_twist'

    self.subscriber_pose_goal = self.nh:subscribe(pose_topic, cartesian_pose_spec, 1)
    self.subscriber_posture_goal = self.nh:subscribe(joint_topic, joint_pos_spec, 1)
    self.subscriber_twist_goal = self.nh:subscribe(twist_topic, cartesian_twist_spec, 1)
    self.publisher_feedback = self.nh:advertise('feedback', feedback_spec, 1)
    self.feedback_message = self.publisher_feedback:createMessage()

    self:reset()
    return true
end

function JoggingControllerOpenLoop:shutdown()
    self.subscriber_pose_goal:shutdown()
    self.subscriber_posture_goal:shutdown()
    self.subscriber_twist_goal:shutdown()
    self.publisher_feedback:shutdown()
end

local function tensor6DToPose(vector6D)
    assert(vector6D:size(1) == 6, 'Vector should be of size 6D (offset, anglevelocities)')
    local end_pose = tf.Transform()
    end_pose:setOrigin(vector6D[{{1, 3}}])
    if vector6D[{{4, 6}}]:norm() > 1e-12 then
        local end_pose_rotation = end_pose:getRotation()
        end_pose:setRotation(end_pose_rotation:setRotation(vector6D[{{4, 6}}], vector6D[{{4, 6}}]:norm()))
    end
    return end_pose
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

local function getPostureForFullStop(cntr, q_desired, dt, msg)
    local q_actual = cntr.state.pos
    local delta = q_desired - q_actual

    local minTime = torch.cdiv(delta, cntr.max_vel):abs():max()
    local trunc_dt = 2 * dt
    if torch.abs(delta):gt(1e-3):sum() > 0 and minTime > trunc_dt then
        -- truncate goal
        ros.DEBUG('truncate goal: %f, %s', trunc_dt / minTime, msg or '')
        q_desired = q_actual + delta * math.min(1, trunc_dt / minTime)
    end
    return q_desired
end

local function getPoseForFullStop(cntr, pose_desired, dt, msg)
    local msg = msg or 'PoseForFullStop'
    if torch.isTypeOf(dt, ros.Duration) then
        dt = dt:toSec()
    end
    if not pose_desired then
        return
    end
    pose_desired = poseTo6DTensor(pose_desired)
    local result = pose_desired:clone()
    result[{{1, 3}}] = getPostureForFullStop(cntr, pose_desired[{{1, 3}}], dt / 4, msg)
    return tensor6DToPose(result)
end

function JoggingControllerOpenLoop:tracking(q_des, duration)
    if type(duration) == 'number' then
        duration = ros.Duration(duration)
    end

    if not self.no_joint_limits_check then
        if not self.joint_limits:satisfiesBounds(q_des) then
            q_des = self.joint_limits:clamp_positions(q_des)
        end
    end

    if self:isValid(q_des.values, self.lastCommandJointPositions.values, self.lastCommandJointPositions:getNames()) then
        assert(
            self.controller.state.pos:size(1) == q_des.values:size(1),
            string.format('inconsistent size: %dx%d', self.controller.state.pos:size(1), q_des.values:size(1))
        )
        local eta = self.controller:update(q_des.values, self.dt:toSec())
        ros.DEBUG_THROTTLE('Controller_eta', 0.1, 'eta: %f', eta)
        local state_joint_values = self.lastCommandJointPositions:clone()
        state_joint_values:setValues(self.lastCommandJointPositions:getNames(), self.controller.state.pos)
        if not self.no_joint_limits_check then
            if self.joint_limits:satisfiesBounds(state_joint_values) then
                sendPositionCommand(self, state_joint_values:getValues(), self.controller.state.vel * 0, state_joint_values:getNames(), duration)
            end
        else
            sendPositionCommand(self, state_joint_values:getValues(), self.controller.state.vel * 0, state_joint_values:getNames(), duration)
        end
        self.lastCommandJointPositions:setValues(q_des:getNames(), q_des.values)
    else
        setStopGoals(self)
        ros.ERROR('command is not valid!!!')
    end
    self.feedback_message.joint_distance:set(q_des.values - self.controller.state.pos)
end

function JoggingControllerOpenLoop:getFullRobotState()
    local names = self.joint_monitor:getJointNames()
    local p = self.joint_monitor:getPositionsOrderedTensor(names)
    self.state:setVariablePositions(p, names)
    self.state:update()
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
    local status = error_codes.OK
    local world_link_name = 'world'
    local link_name
    if self.curr_end_effector_name then
        link_name = self.end_effector_to_link_map[self.curr_end_effector_name]
    else
        link_name = self.move_groups[self.curr_move_group_name]:getEndEffectorLink()
    end
    local posture_goal
    if link_name and #link_name > 0 then
        local dt = self.dt:toSec()
        if detectNan(pose_goal:getOrigin()) then
            return self.lastCommandJointPositions, error_codes.OK
        end
        pose_goal:setOrigin(
            getPostureForFullStop(self.taskspace_controller, pose_goal:getOrigin(), dt, 'position control')
        )
        self.target_pose = pose_goal:clone()
        self.taskspace_controller:update(pose_goal:getOrigin(), dt)

        local current_rotation = self.current_pose:getRotation()
        local target_rotation = pose_goal:getRotation()
        local to_go = (pose_goal:getOrigin() - self.current_pose:getOrigin()):norm()
        if to_go > 0 then
            local step = (self.current_pose:getOrigin() - self.taskspace_controller.state.pos):norm() / to_go
            local result_rotation = current_rotation:slerp(target_rotation, step)
            self.target_pose:setOrigin(self.taskspace_controller.state.pos)
            self.target_pose:setRotation(result_rotation)
        end

        local state = self.state:clone()
        local attempts, timeout = 1, self.dt:toSec() * 2
        local suc = (not detectNan(self.target_pose:getOrigin())) and
            state:setFromIK(
            self.move_groups[self.curr_move_group_name]:getName(),
            self.target_pose:toTransform(),
            attempts,
            timeout,
            false,
            link_name
        )

        if suc then
            state:update()
            local jac = state:getJacobian(self.move_groups[self.curr_move_group_name]:getName())
            local det = determinant(jac * jac:t())

            if det < 1e-8 then
                status = error_codes.CLOSE_TO_SINGULARITY
            end

            local tmp_target = poseTo6DTensor(self.target_pose:clone():mul(self.current_pose:inverse()))
            posture_goal =
                createJointValues(state:getVariableNames():totable(), state:getVariablePositions()):select(joint_names)

            -- Check if pose motion direction is corresponding to the joint posture motion direction (handles IK jumps)
            local tmp_q = posture_goal:clone()
            local tmp_q_dot = posture_goal:clone()
            tmp_q_dot = (tmp_q_dot - self.lastCommandJointPositions) * self.dt:toSec()
            tmp_q:add(tmp_q_dot)
            state:setVariablePositions(tmp_q:getValues(), tmp_q:getNames())
            state:update()
            local cmd_curr = poseTo6DTensor(state:getGlobalLinkTransform(link_name):mul(self.current_pose:inverse()))

            if tmp_target:norm() > 1e-3 then
                if 1 - torch.dot(tmp_target / tmp_target:norm(), cmd_curr / cmd_curr:norm()) > 1e-4 then
                    ros.ERROR('[transformPose2PostureTarget] Directions do not match')
                    return self.lastCommandJointPositions, error_codes.TASK_SPACE_JUMP_DETECTED
                end
            end
        else
            ros.ERROR('[transformPose2PostureTarget] Could not set IK solution.')
            return self.lastCommandJointPositions, error_codes.FRAME_TRANSFORM_FAILURE
        end
    else
        ros.WARN(
            "[transformPose2PostureTarget] Cannot uses this move group: '%s' since it has no EndEffector Link. ",
            self.curr_move_group_name or '?'
        )
        return self.lastCommandJointPositions, error_codes.INVALID_LINK_NAME
    end
    return posture_goal, status
end

function JoggingControllerOpenLoop:update()
    local timeout_b = false
    -- Handle feedback
    self.feedback_message.error_code = error_codes.OK
    local status_msg = 'OK'
    local q_dot = self.lastCommandJointPositions:clone()
    q_dot.values:zero()

    local curr_time = ros.Time.now()
    -- Handle goals
    local new_pose_message, pose_goal = self:getPoseGoal()

    local new_posture_message, posture_goal = self:getPostureGoal()

    local new_twist_message, twist_goal, transformed_successful = self:getTwistGoal()

    --update state
    if
        self.controller.converged == true and
            self.cool_down_timeout:toSec() < (curr_time - self.start_cool_down_time):toSec()
     then
        --sync with real world

        self.mode = jogging_node_tracking_states.IDLE
        if self.synced == false or new_pose_message or new_posture_message or new_twist_message then
            self.synced = true
            local jointPositionsBeforeSync = self.lastCommandJointPositions:clone()
            self:getNewRobotState()
            local diff = self.lastCommandJointPositions - jointPositionsBeforeSync
            if diff.values:norm() < 1e-3 then
                self.taskspace_controller:reset()
                self.taskspace_controller.state.pos:copy(self.current_pose:getOrigin())
                self.controller:reset()
                self.controller.state.pos:copy(self.lastCommandJointPositions.values)
            else
                ros.WARN(
                    'could not sync with controller state since last command is to far away from current robto state.'
                )
                self.start_cool_down_time = ros.Time.now()
                self.lastCommandJointPositions = jointPositionsBeforeSync
                self.state:setVariablePositions(
                    self.lastCommandJointPositions.values,
                    self.lastCommandJointPositions:getNames()
                )
                self.state:update()
                self.current_pose = self:getCurrentPose()
                self.synced = false
            end
        end
    else
        --self.taskspace_controller.state.pos:copy(self.current_pose:getOrigin()) --feedback current position to xyz controller
        self.synced = false
        --open loop control. Use state from controllers
        self.state:setVariablePositions(
            self.lastCommandJointPositions.values,
            self.lastCommandJointPositions:getNames()
        )
        self.state:update()
        self.current_pose = self:getCurrentPose()
    end

    local stop_received = false
    -- Decide which goal is valid
    if
        ((self.mode == jogging_node_tracking_states.IDLE or self.mode == jogging_node_tracking_states.POSE) and
            new_pose_message) or
            (self.mode == jogging_node_tracking_states.POSE and self.goals.pose_goal)
     then
        --set pose
        local isNan = false
        if pose_goal then
            isNan = detectNan(pose_goal:getOrigin())
        end

        if pose_goal ~= nil and isNan == false then
            self.goals.pose_goal = pose_goal:clone()
        elseif self.goals.pose_goal then
            pose_goal = self.goals.pose_goal:clone()
        end

        if (new_pose_message and isNan) or self.mode == jogging_node_tracking_states.IDLE then
            if self.mode ~= jogging_node_tracking_states.IDLE then
                ros.INFO('[Pose] Received stop signal')
                stop_received = true
                if detectNan(pose_goal:getOrigin()) then
                    pose_goal = self.current_pose:clone()
                end
            end
            pose_goal:setRotation(self.current_pose:getRotation())
            self.goals.pose_goal = nil
        end
        local rel_poseAB = pose_goal:clone()
        rel_poseAB = rel_poseAB:mul(self.current_pose:inverse())
        local tmp = tf.StampedTransform(rel_poseAB:mul(self.current_pose))
        local posture_tmp_goal, err_code = transformPose2PostureTarget(self, tmp, q_dot:getNames())
        if posture_tmp_goal and self.lastCommandJointPositions then
            q_dot = posture_tmp_goal - self.lastCommandJointPositions
            self.mode = jogging_node_tracking_states.POSE
            self.feedback_message.error_code = err_code
            stop_received = true
            if err_code == error_codes.OK or err_code == error_codes.CLOSE_TO_SINGULARITY then
                stop_received = false
            end
        end

        if torch.abs(q_dot.values):gt(math.pi / 2):sum() > 1 then
            ros.ERROR('detected jump in IK.')
            self.feedback_message.error_code = error_codes.IK_JUMP_DETECTED
            q_dot.values:zero()
        end
        if not stop_received then
            self.controller.converged = false
        else
            self.mode = jogging_node_tracking_states.IDLE
        end
    elseif
        ((self.mode == jogging_node_tracking_states.IDLE or self.mode == jogging_node_tracking_states.POSTURE) and
            new_posture_message) or
            (self.mode == jogging_node_tracking_states.POSTURE and self.goals.posture_goal)
     then
        -- set posture
        if posture_goal ~= nil then
            self.goals.posture_goal = posture_goal:clone()
        elseif self.goals.posture_goal then
            posture_goal = self.goals.posture_goal:clone()
        end
        self.mode = jogging_node_tracking_states.POSTURE

        q_dot = posture_goal:clone()
        q_dot:sub(self.lastCommandJointPositions)
        if q_dot.values:norm() < 1e-10 then
            self.goals.posture_goal = nil
            q_dot.values:zero()
            self.mode = jogging_node_tracking_states.IDLE
            stop_received = true
        else
            self.controller.converged = false
        end
    elseif
        ((self.mode == jogging_node_tracking_states.IDLE or self.mode == jogging_node_tracking_states.TWIST) and
            new_twist_message) or
            (self.mode == jogging_node_tracking_states.TWIST and self.goals.twist_goal)
     then
        --set twist
        if twist_goal ~= nil then
            self.goals.twist_goal = twist_goal:clone()
        elseif self.goals.twist_goal ~= nil then
            twist_goal = self.goals.twist_goal:clone()
        end
        if (new_twist_message and twist_goal:norm() < 1e-12) or self.mode == jogging_node_tracking_states.IDLE then
            if self.mode ~= jogging_node_tracking_states.IDLE then
                ros.INFO('[twist] Received stop signal')
                stop_received = true
            end
            self.goals.twist_goal = nil
        end
        local rel_tmp_pose = tf.StampedTransform()
        local dt = self.dt:toSec()

        local pos_offset = twist_goal[{{1, 3}}] * self.command_distance_threshold * dt
        local rot_offset = twist_goal[{{4, 6}}] * self.command_rotation_threshold * dt

        local quaternion = rel_tmp_pose:getRotation()
        if (rot_offset):norm() > 0 then
            quaternion:setRotation(rot_offset, (rot_offset):norm())
        end
        rel_tmp_pose:setRotation(quaternion)
        rel_tmp_pose:setOrigin(pos_offset)

        local rel_poseAB = rel_tmp_pose:clone()
        local tmp = tf.StampedTransform()
        tmp:setOrigin(rel_poseAB:getOrigin() + self.current_pose:getOrigin())
        tmp:setRotation(rel_poseAB:getRotation() * self.current_pose:getRotation())
        local posture_tmp_goal, err_code = transformPose2PostureTarget(self, tmp, q_dot:getNames())

        if posture_tmp_goal and self.lastCommandJointPositions then
            q_dot = posture_tmp_goal - self.lastCommandJointPositions
            if self.mode == jogging_node_tracking_states.IDLE then
                q_dot.values:zero()
            end
            self.mode = jogging_node_tracking_states.TWIST
            self.feedback_message.error_code = err_code
            stop_received = true
            if err_code == error_codes.OK or err_code == error_codes.CLOSE_TO_SINGULARITY then
                stop_received = false
            end
        end

        if torch.abs(q_dot.values):gt(math.pi / 2):sum() > 1 then
            ros.ERROR(
                '[twist] detected jump in IK. In %d number of joints threshold was exceeded.',
                torch.abs(q_dot.values):gt(math.pi / 2):sum()
            )
            self.feedback_message.error_code = error_codes.IK_JUMP_DETECTED
            q_dot.values:zero()
        end

        if transformed_successful == false then
            self.feedback_message.error_code = error_codes.INVALID_LINK_NAME
        end
        if not stop_received then
            self.controller.converged = false
        else
            self.mode = jogging_node_tracking_states.IDLE
        end
    end

    --Handle command timeout and reset controllers (initiate stop of robot)
    if new_pose_message or new_posture_message or new_twist_message then
        self.start_time = ros.Time.now()
        self.start_cool_down_time = ros.Time.now()
    end

    if self.timeout:toSec() < (ros.Time.now() - self.start_time):toSec() then
        if self.controller.converged == false and self.mode ~= jogging_node_tracking_states.IDLE then
            ros.ERROR_THROTTLE('JointJoggingTIMEOUT1', 0.1, 'TIMEOUT occured while tracking target. Set Stop targets')
            setStopGoals(self)
        else
            resetGoals(self)
        end
        self.lastCommandJointPositions.values:copy(self.controller.state.pos)
        self.controller:reset()
        self.controller.state.pos:copy(self.lastCommandJointPositions.values)
        self.taskspace_controller:reset()
        self.taskspace_controller.state.pos:copy(self.current_pose:getOrigin())
        q_dot.values:zero()
        timeout_b = true
    end

    -- prepare tracking target
    local q_des = self.lastCommandJointPositions:clone()
    if self.controller.converged == false and timeout_b == false then
        q_des:add(q_dot)
        q_des.values = getPostureForFullStop(self.controller, q_des.values, self.dt:toSec(), 'joint control')
        ros.DEBUG('Locking')
    else
        if timeout_b == true then
            self.mode = jogging_node_tracking_states.IDLE
        end
    end
    ros.DEBUG_THROTTLE('TrackingMode', 1, string.format('[JoggingControllerOpenLoop] Control Mode: %d', self.mode))

    if tryLock(self) then
        self:tracking(q_des, (ros.Time.now() - self.last_command_send_time))
    end

    ros.DEBUG('Feedback')
    sendFeedback(self, q_dot.values:norm())
    return true, status_msg
end

function JoggingControllerOpenLoop:reset()
    ros.DEBUG('resetting ....')
    while self.cool_down_timeout:toSec() > (ros.Time.now() - self.start_cool_down_time):toSec() do
        ros.INFO_THROTTLE('coolDown', 1, '[JoggingControllerOpenLoop] need to cool down first')
        ros.spinOnce()
        self.dt:sleep()
    end
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

    self.lastCommandJointPositions =
        createJointValues(
        self.joint_set.joint_names,
        self.joint_monitor:getPositionsOrderedTensor(self.joint_set.joint_names)
    )
    self.controller = tvpController.new(#self.joint_set.joint_names)
    createPublisher(self, self.joint_set.joint_names)
    self:getNewRobotState()
    self:getFullRobotState()
    self.target_pose = self.current_pose:clone()

    local max_min_pos, max_vel, max_acc =
        queryJointLimits(self.nh, self.joint_set.joint_names, '/robot_description_planning')
    max_vel = max_vel * self.max_speed_scaling

    self.joint_limits =
        datatypes.JointLimits(self.joint_set, max_min_pos[{{}, 1}], max_min_pos[{{}, 2}], max_vel, max_acc)

    local key =
        table.findFirst(
        self.end_effector_to_move_group_map,
        function(x)
            return 0 <
                table.findIndex(
                    x,
                    function(xx, ix)
                        return xx == self.curr_move_group_name
                    end
                )
        end
    )
    self.curr_end_effector_name = key

    local success, taskspace_max_vel, taskspace_max_acc = queryTaskSpaceLimits(self, self.curr_end_effector_name)
    if success == true then
        self.taskspace_max_vel = taskspace_max_vel
        self.taskspace_max_acc = taskspace_max_acc
    else
        ros.WARN(
            'No taskspace limits found in namespace using default configuration. [%s:%s]',
            self.curr_move_group_name,
            self.curr_end_effector_name
        )
    end

    self.mode = jogging_node_tracking_states.IDLE
    self.controller = tvpController.new(#self.joint_set:getNames())
    self.controller.max_vel:copy(self.joint_limits:getMaxVelocities(self.joint_set:getNames()))
    self.controller.max_acc:copy(self.joint_limits:getMaxAccelerations(self.joint_set:getNames()))
    self.controller.state.pos:copy(self.lastCommandJointPositions.values)
    self.taskspace_controller:reset()
    self.taskspace_controller.state.pos:copy(self.current_pose:getOrigin())
    self:setSpeedScaling(self.speed_scaling)
    self:setSelfCollisionChecksState(true)
    self:setSceneCollisionChecksState(true)
    self:setJointLimitsChecks(true)
    resetGoals(self)
    self.synced = true
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
        self.curr_end_effector_name = nil
        --self:releaseResources()
        ros.INFO('Triggered reset with movegroup name: %s', self.move_groups[self.curr_move_group_name]:getName())
        return true, string.format('Successfully changed movegroup to %s', self.curr_move_group_name)
    else
        self.curr_move_group_name = last_move_group_name
        return false, string.format('Could not set moveGroup with name: %s. ', name)
    end
end

function JoggingControllerOpenLoop:setEndEffector(name)
    assert(name)
    local suc, msg = true, ''
    local key
    for k, v in pairs(self.end_effector_to_move_group_map) do
        if k == name then
            key = k
        end
    end

    if key == nil then
        return false, 'could not find EndEffector definition'
    end

    local index =
        table.indexof(
        self.end_effector_to_move_group_map[key],
        function(x)
            return x == self.curr_move_group_name
        end
    )
    if index < 0 then
        index = 1
    end

    local move_group_name = self.end_effector_to_move_group_map[key][index]
    if move_group_name then
        suc, msg = self:setMoveGroupInterface(move_group_name)
        if suc then
            self.curr_end_effector_name = name
        else
            self.curr_end_effector_name = nil
        end
    end
    return suc, msg
end

function JoggingControllerOpenLoop:getCurrentEndEffector()
    return self.curr_end_effector_name
end

function JoggingControllerOpenLoop:getCurrentMoveGroup()
    return self.move_groups[self.curr_move_group_name]
end

function JoggingControllerOpenLoop:setSelfCollisionChecksState(check)
    assert(
        torch.type(check) == 'boolean',
        '[JoggingControllerOpenLoop:setSelfCollisionChecksState] Argument needs to be a boolean'
    )
    self.no_self_collision_check = not check
    if check == false then
        self.no_scene_collision_check = true
    end
end

function JoggingControllerOpenLoop:setSceneCollisionChecksState(check)
    assert(
        torch.type(check) == 'boolean',
        '[JoggingControllerOpenLoop:setSceneCollisionChecksState] Argument needs to be a boolean'
    )
    self.no_scene_collision_check = not check
    if check == true then
        self.no_self_collision_check = false
    end
end

function JoggingControllerOpenLoop:setJointLimitsChecks(check)
    assert(
        torch.type(check) == 'boolean',
        '[JoggingControllerOpenLoop:setJointLimitsChecks] Argument needs to be a boolean'
    )
    self.no_joint_limits_check = not check
end

function JoggingControllerOpenLoop:getSelfCollisionChecksState()
    return not self.no_self_collision_check
end

function JoggingControllerOpenLoop:getSceneCollisionChecksState()
    return not self.no_scene_collision_check
end

function JoggingControllerOpenLoop:getJointLimitsChecks()
    return not self.no_joint_limits_check
end

local function validateTimeDuration(dt)
    if torch.type(dt) == 'number' then
        dt = ros.Duration(dt)
    elseif torch.type(dt) == 'nil' then
        dt = ros.Duration(0.01)
    elseif torch.isTypeOf(dt, ros.Rate) then
        dt = ros.Duration(dt:expectedCycleTime())
    elseif torch.type(dt, ros.Duration) then
        -- do nothing
    else
        error('dt has unsupported type')
    end
    return dt
end

function JoggingControllerOpenLoop:setDeltaT(dt)
    self.dt = validateTimeDuration(dt)
    ros.DEBUG('frequenc change: %f', self.dt:toSec())
end

return JoggingControllerOpenLoop
