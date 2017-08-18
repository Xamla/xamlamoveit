#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local controller = require 'xamlamoveit.controller'
local TvpController = controller.TvpController
local GenerativeSimulationWorker = require 'xamlamoveit.xutils.GenerativeSimulationWorker'
local MonitorBuffer = require 'xamlamoveit.xutils.MonitorBuffer'

local actionlib = ros.actionlib
local ActionServer = actionlib.ActionServer
local GoalStatus = actionlib.GoalStatus

local joint_sensor_spec = ros.MsgSpec('sensor_msgs/JointState')
local joint_msg = ros.Message(joint_sensor_spec)

local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local printf = xutils.printf
local xtable = xutils.Xtable

-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
    SUCCESSFUL = 0,
    INVALID_GOAL = -1,
    INVALID_JOINTS = -2,
    OLD_HEADER_TIMESTAMP = -3,
    PATH_TOLERANCE_VIOLATED = -4,
    GOAL_TOLERANCE_VIOLATED = -5
}

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

local nodehandle, sp, worker
local function initSetup(ns)
    ros.init(ns)
    nodehandle = ros.NodeHandle('~')
    service_queue = ros.CallbackQueue()

    sp = ros.AsyncSpinner() -- background job
    sp:start()
    worker = GenerativeSimulationWorker.new(nodehandle)
end

local function shutdownSetup()
    sp:stop()
    ros.shutdown()
end

local function isSubset(A, B)
    for ia, a in ipairs(A) do
        if table.indexof(B,a)==-1 then
            return false
        end
    end
    return true
end

local function isSimilar(A, B)
    if #A == #B then
        return isSubset(A,B)
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
    local function copyMapped(dst, src, map)
        for k, v in pairs(map) do
            dst[k] = src[v]
        end
    end

    local pointCount = #trajectory.points
    local time = torch.zeros(pointCount) -- convert trajectory to internal tensor format
    local pos = torch.zeros(pointCount, #trajectory.joint_names)
    local vel = torch.zeros(pointCount, #trajectory.joint_names)
    local acc = torch.zeros(pointCount, #trajectory.joint_names)
    local hasVelocity = true
    local hasAcceleration = true

    for i = 1, pointCount do
        local pt = trajectory.points[i]
        time[i] = pt.time_from_start:toSec()
        pos[i]:copy(pt.positions)

        if pt.velocities ~= nil and pt.velocities:nElement() > 0 then
            vel[i]:copy(pt.velocities)
        else
            hasVelocity = false
        end

        if pt.accelerations ~= nil and pt.accelerations:nElement() > 0 then
            acc[i]:copy(pt.accelerations)
        else
            hasAcceleration = false
        end
    end

    if not hasAcceleration then
        acc = nil
    end

    return time, pos, vel, acc
end

local function moveJAction_serverGoal(goal_handle, q_buffer, qd_buffer, target_joint_names)
    ros.INFO('moveJAction_serverGoal')
    local g = goal_handle:getGoal()
    if not isSubset(g.goal.trajectory.joint_names,target_joint_names) then
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

local function initControllers(delay, dt)
    local offset = math.ceil(delay / dt:toSec())
    config = nodehandle:getParamVariable("/move_group/controller_list")
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(joint_name_collection, vv) == -1 then
                joint_name_collection[#joint_name_collection + 1] = vv
            end
        end

        if #v.name > 0 then
            action_server[v.name] = ActionServer(nodehandle, string.format('%s/joint_trajectory_action', v.name), 'control_msgs/FollowJointTrajectory')
        else
            action_server[v.name] = ActionServer(nodehandle, 'joint_trajectory_action', 'control_msgs/FollowJointTrajectory')
        end
    end

    controller = TvpController(#joint_name_collection)
    feedback_buffer_pos = MonitorBuffer.new(offset + 1, #joint_name_collection)
    feedback_buffer_pos.offset = offset
    feedback_buffer_vel = MonitorBuffer.new(offset + 1, #joint_name_collection)
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
end

local function shutdownAction_server()
    for i, v in pairs(action_server) do
        v:shutdown()
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

initSetup('sda10d')

local jointtrajmsg_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local subscriber = nodehandle:subscribe('joint_command', jointtrajmsg_spec, 1)
subscriber:registerCallback(jointCommandCb)
local joint_state_publisher = nodehandle:advertise('/joint_states', joint_sensor_spec, 1)

local function sendJointState(position, velocity, joint_names, seq)
    local m = ros.Message(joint_sensor_spec)
    m.header.seq = seq
    m.header.stamp = ros.Time.now()
    m.name = joint_names
    m.position:set(position)
    joint_state_publisher:publish(m)
end

local function simulation(delay, dt)
    local seq = 1
    local delay = delay or 0.15
    local dt = ros.Duration(dt or 0.008)
    local offset = math.ceil(delay / dt:toSec())
    initControllers(delay, dt)

    while ros.ok() do
        controller:update(last_command_joint_position, dt:toSec())
        feedback_buffer_pos:add(controller.state.pos)
        feedback_buffer_vel:add(controller.state.vel)

        local pos = feedback_buffer_pos:getPastIndex(offset)
        local vel = feedback_buffer_vel:getPastIndex(offset)

        if pos then
            sendJointState(pos, vel, joint_name_collection, seq)
        end

        seq = seq + 1
        dt:sleep()
        worker:spin()
        ros.spinOnce()
    end
    shutdownAction_server()
end

--local cmd = torch.CmdLine()
--cmd:option('-delay', 0.150, 'Feedback delay time')
--cmd:option('-cycleTime', 0.008, 'Node cycle time')
--local params = cmd:parse(arg)

simulation(0.150, 0.008)
shutdownSetup()
