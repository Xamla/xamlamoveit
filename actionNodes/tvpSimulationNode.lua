#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local printf = xutils.printf
local xtable = xutils.Xtable

local xamlacontroller = xamlamoveit.controller
local TvpController = xamlacontroller.TvpController

local actionlib = ros.actionlib
local ActionServer = actionlib.ActionServer
local GoalStatus = actionlib.GoalStatus

local joint_sensor_spec = ros.MsgSpec('sensor_msgs/JointState')
local joint_msg = ros.Message(joint_sensor_spec)

local jointtrajmsg_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local subscriber

local GenerativeSimulationWorker = require 'xamlamoveit.xutils.GenerativeSimulationWorker'

-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
    SUCCESSFUL = 0,
    INVALID_GOAL = -1,
    INVALID_JOINTS = -2,
    OLD_HEADER_TIMESTAMP = -3,
    PATH_TOLERANCE_VIOLATED = -4,
    GOAL_TOLERANCE_VIOLATED = -5
}

--[[
local config = {
    {
        name = '',
        ns = '/sda10d',
        group = 0,
        joints = {
            'arm_left_joint_1_s',
            'arm_left_joint_2_l',
            'arm_left_joint_3_e',
            'arm_left_joint_4_u',
            'arm_left_joint_5_r',
            'arm_left_joint_6_b',
            'arm_left_joint_7_t',
            'arm_right_joint_1_s',
            'arm_right_joint_2_l',
            'arm_right_joint_3_e',
            'arm_right_joint_4_u',
            'arm_right_joint_5_r',
            'arm_right_joint_6_b',
            'arm_right_joint_7_t',
            'torso_joint_b1'
        }
    },
    {
        name = 'sda10d_r1_controller',
        ns = '/sda10d',
        group = 0,
        joints = {
            'arm_left_joint_1_s',
            'arm_left_joint_2_l',
            'arm_left_joint_3_e',
            'arm_left_joint_4_u',
            'arm_left_joint_5_r',
            'arm_left_joint_6_b',
            'arm_left_joint_7_t'
        }
    },
    {
        name = 'sda10d_r2_controller',
        ns = '/sda10d',
        group = 1,
        joints = {
            'arm_right_joint_1_s',
            'arm_right_joint_2_l',
            'arm_right_joint_3_e',
            'arm_right_joint_4_u',
            'arm_right_joint_5_r',
            'arm_right_joint_6_b',
            'arm_right_joint_7_t'
        }
    },
    {
        name = 'sda10d_b1_controller',
        ns = '/sda10d',
        group = 2,
        joints = {'torso_joint_b1'}
    }
}
]]
local config = {}
local node_handle, sp, worker

local function initSetup(ns)
    ros.init(ns)
    node_handle = ros.NodeHandle('~')
    --service_queue = ros.CallbackQueue()

    sp = ros.AsyncSpinner() -- background job
    sp:start()
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
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

local new_message = false
local seq = 1
local joint_name_collection = {}
local last_command_joint_position = {}
local controller = {}
local action_server = {}
local feedback_buffer_pos = {}
local feedback_buffer_vel = {}

local function decodeJointTrajectoryMsg(trajectory)
    local point_count = #trajectory.points
    local time = torch.zeros(point_count) -- convert trajectory to internal tensor format
    local pos = torch.zeros(point_count, #trajectory.joint_names)
    local vel = torch.zeros(point_count, #trajectory.joint_names)
    local acc = torch.zeros(point_count, #trajectory.joint_names)
    local has_velocity = true
    local has_acceleration = true

    for i = 1, point_count do
        local pt = trajectory.points[i]
        time[i] = pt.time_from_start:toSec()
        pos[i]:copy(pt.positions)

        if pt.velocities ~= nil and pt.velocities:nElement() > 0 then
            vel[i]:copy(pt.velocities)
        else
            has_velocity = false
        end

        if pt.accelerations ~= nil and pt.accelerations:nElement() > 0 then
            acc[i]:copy(pt.accelerations)
        else
            has_acceleration = false
        end
    end

    if not has_acceleration then
        acc = nil
    end

    if not has_velocity then
        vel = nil
    end

    return time, pos, vel, acc
end

local function moveJAction_serverGoal(goal_handle, q_buffer, qd_buffer, target_joint_names)
    ros.INFO('moveJAction_serverGoal')
    local g = goal_handle:getGoal()
    if not isSubset(g.goal.trajectory.joint_names, target_joint_names) then
        ros.ERROR('not correct set of joints for this group')
        return
    end
    -- decode trajectory
    local time, pos, vel, acc = decodeJointTrajectoryMsg(g.goal.trajectory)
    local traj = {
        time = time,
        pos = pos,
        vel = vel,
        acc = acc,
        goalHandle = goal_handle,
        goal = g,
        q_buffer = q_buffer,
        qd_buffer = qd_buffer,
        joint_names = g.goal.trajectory.joint_names,
        state_joint_names = joint_name_collection,
        accept = function()
            if goal_handle:getGoalStatus().status == GoalStatus.PENDING then
                goal_handle:setAccepted('Starting trajectory execution')
                return true
            else
                ros.WARN('Status of queued trajectory is not pending but %d.', goal_handle:getGoalStatus().status)
                return false
            end
        end,
        proceed = function()
            if goal_handle:getGoalStatus().status == GoalStatus.ACTIVE then
                return true
            else
                ros.WARN('Goal status of current trajectory no longer ACTIVE (actual: %d).', goal_handle:getGoalStatus().status)
                return false
            end
        end,
        abort = function(self, msg)
            goal_handle:setAborted(nil, msg or 'Error')
        end,
        completed = function()
            local r = goal_handle:createResult()
            r.error_code = worker.errorCodes.SUCCESSFUL
            goal_handle:setSucceeded(r, 'Completed')
        end
    }
    if traj.pos:nElement() == 0 then -- empty trajectory
        local r = goal_handle:createResult()
        r.error_code = TrajectoryResultStatus.SUCCESSFUL
        goal_handle:setSucceeded(r, 'Completed (nothing to do)')
        ros.WARN('Received empty FollowJointTrajectory request (goal: %s).', goal_handle:getGoalID().id)
    else
        worker:doTrajectoryAsync(traj) -- queue for processing
        ros.INFO('Trajectory queued for execution (goal: %s).', goal_handle:getGoalID().id)
    end
end

local function FollowJointTrajectory_Cancel(goalHandle)
    ros.INFO('FollowJointTrajectory_Cancel')

    -- check if trajectory is in trajectoryQueue
    local i =
        table.findIndex(
        worker.trajectoryQueue,
        function(x)
            return x.goalHandle == goalHandle
        end
    )
    if i > 0 then
        -- entry found, simply remove from queue
        table.remove(worker.trajectoryQueue, i)
        goalHandle:setCanceled(nil, 'Canceled')
    elseif worker.currentTrajectory ~= nil and worker.currentTrajectory.goalHandle == goalHandle then
        worker:cancelCurrentTrajectory('Canceled')
    end
end

function jointCommandCb(msg, header)
    if #msg.points > 0 then
        for igroup, group in ipairs(config) do
            for i, name in ipairs(msg.joint_names) do
                local index = table.indexof(joint_name_collection, name)
                if index > -1 then
                    last_command_joint_position[index] = msg.points[1].positions[i]
                end
            end
        end
        seq = seq + 1
        new_message = true
    end
end

local function initControllers(delay, dt)
    local offset = math.ceil(delay / dt:toSec())

    config = node_handle:getParamVariable('/move_group/controller_list')
    local start_time = ros.Time.now()
    local current_time = ros.Time.now()
    local attemts = 0
    while config == nil do
        attemts = attemts + 1
        ros.WARN('no controller specified in "/move_group/controller_list". Retry in 5sec')
        while current_time:toSec() - start_time:toSec() < 5 do
            current_time = ros.Time.now()
            sys.sleep(0.01)
        end
        start_time = ros.Time.now()
        current_time = ros.Time.now()
        config = node_handle:getParamVariable('/move_group/controller_list')

        if not ros.ok() then
            return -1, 'Ros is not ok'
        end

        if attemts > 5 then
            return -2, 'Reached max attempts'
        end
    end
    local ns
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(joint_name_collection, vv) == -1 then
                joint_name_collection[#joint_name_collection + 1] = vv
            end
        end

        if #v.name > 0 then
            action_server[v.name] = ActionServer(node_handle, string.format('%s/joint_trajectory_action', v.name), 'control_msgs/FollowJointTrajectory')
        else
            action_server[v.name] = ActionServer(node_handle, 'joint_trajectory_action', 'control_msgs/FollowJointTrajectory')
        end
        ns = string.split(action_server[v.name].node:getNamespace(), '/')
    end
    controller = TvpController(#joint_name_collection)
    feedback_buffer_pos = xutils.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_pos.offset = offset
    feedback_buffer_vel = xutils.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_vel.offset = offset
    last_command_joint_position = torch.ones(#joint_name_collection) * 1.3

    for i, v in ipairs(config) do
        action_server[v.name]:registerGoalCallback(
            function(gh)
                moveJAction_serverGoal(gh, feedback_buffer_pos, feedback_buffer_vel, v.joints)
            end
        )
        action_server[v.name]:registerCancelCallback(FollowJointTrajectory_Cancel)
        action_server[v.name]:start()
    end

    subscriber = node_handle:subscribe(string.format('/%s/joint_command', ns[1]), jointtrajmsg_spec, 1)
    subscriber:registerCallback(jointCommandCb)

    worker = GenerativeSimulationWorker.new(ros.NodeHandle(string.format('/%s',ns[1])))
    return 0, 'Success'
end

local function shutdownAction_server()
    for i, v in pairs(action_server) do
        v:shutdown()
    end
end

local cmd = torch.CmdLine()
cmd:option('-delay', 0.150, 'Feedback delay time')
cmd:option('-frequency', 0.008, 'Node cycle time')

local parameter = xutils.parseRosParametersFromCommandLine(arg,cmd) or {}
initSetup(parameter["__name"]) -- TODO

local joint_state_publisher = node_handle:advertise('/joint_states', joint_sensor_spec, 1)

local function sendJointState(position, velocity, joint_names, sequence)
    local m = ros.Message(joint_sensor_spec)
    m.header.seq = sequence
    m.header.stamp = ros.Time.now()
    m.name = joint_names
    m.position:set(position)
    m.velocity:set(velocity)
    joint_state_publisher:publish(m)
end

local function simulation(delay, dt)
    local sim_seq = 1
    local value, succ = node_handle:getParamDouble(string.format('%s/feedback_delay', node_handle:getNamespace()))
    if succ then
        delay = value
    end
    value, succ = node_handle:getParamDouble('frequency')
    if succ then
        dt = value
    end
    dt = ros.Duration(dt or 0.008)
    local offset = math.ceil(delay / dt:toSec())
    local err, msg = initControllers(delay, dt)
    if err < 0 then
        ros.ERROR('Could not initialize controller. ' .. msg)
        return
    end
    while ros.ok() do
        controller:update(last_command_joint_position, dt:toSec())
        feedback_buffer_pos:add(controller.state.pos)
        feedback_buffer_vel:add(controller.state.vel)

        local pos = feedback_buffer_pos:getPastIndex(offset)
        local vel = feedback_buffer_vel:getPastIndex(offset)

        if pos then
            sendJointState(pos, vel, joint_name_collection,sim_seq)
        end

        sim_seq = sim_seq + 1
        dt:sleep()
        worker:spin()
        ros.spinOnce()
    end
    shutdownAction_server()
end




simulation(parameter.delay, parameter.frequency)
shutdownSetup()
