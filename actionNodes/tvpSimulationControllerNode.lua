#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local xamla_sysmon = require 'xamla_sysmon'
local xamlamoveit = require 'xamlamoveit'
local core = xamlamoveit.core
local xutils = xamlamoveit.xutils
local printf = xutils.printf
local xtable = xutils.Xtable

local xamlacontroller = xamlamoveit.controller
local Controller = xamlacontroller.TvpController

local jointtrajmsg_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_sensor_spec = ros.MsgSpec('sensor_msgs/JointState')
local joint_msg = ros.Message(joint_sensor_spec)

local subscriber

-- http://docs.ros.org/fuerte/api/control_msgs/html/msg/FollowJointTrajectoryResult.html
local TrajectoryResultStatus = {
    SUCCESSFUL = 0,
    INVALID_GOAL = -1,
    INVALID_JOINTS = -2,
    OLD_HEADER_TIMESTAMP = -3,
    PATH_TOLERANCE_VIOLATED = -4,
    GOAL_TOLERANCE_VIOLATED = -5
}
local config = {}
local node_handle, sp, worker

local function initSetup(ns, param)
    ros.init(ns, nil, param)
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
local last_command_joint_velocity = {}
local last_command_time = ros.Time.now()
local controller = {}
local feedback_buffer_pos = {}
local feedback_buffer_vel = {}

local function jointCommandCb(msg, header)
    if #msg.points > 0 then
        for igroup, group in ipairs(config) do
            for i, name in ipairs(msg.joint_names) do
                local index = table.indexof(joint_name_collection, name)
                if index > -1 then
                    --print(msg.points[1])
                    last_command_joint_position[index] = msg.points[1].positions[i]
                    if msg.points[1].velocities:nDimension() > 0 then
                        last_command_joint_velocity[index] = msg.points[1].velocities[i]
                    else
                        last_command_joint_velocity[index] = 0
                    end
                end
            end
        end
        seq = seq + 1
        new_message = true
        last_command_time = ros.Time.now()
        ros.DEBUG('jointCommandCb')
    end
end

local error_state = false


local function checkParameterForAvailability(topic, wait_duration)
    wait_duration = wait_duration or ros.Duration(1.0)
    local value
    local counter = 0
    while value == nil and ros.ok() do
        if counter % 10 == 1 then
            ros.WARN('%s not available trying again in 1 sec', topic)
        end
        wait_duration:sleep()
        value = node_handle:getParamVariable(topic)
        ros.spinOnce()
        counter = 1 + counter
    end
    return true, value
end

local function queryJointLimits(joint_names, namespace)
    namespace = namespace or node_handle:getNamespace()
    local nh = node_handle
    local root_path = string.format('%s/joint_limits', namespace) -- robot_description_planning
    local suc, value = checkParameterForAvailability( root_path )
    if suc == false then
        return
    end

    local max_vel = torch.zeros(#joint_names)
    local max_acc = torch.zeros(#joint_names)
    local max_min_pos = torch.zeros(#joint_names, 2)

    for i, name in ipairs(joint_names) do
        local has_pos_param = value[name].has_position_limits
        local has_vel_param = value[name].has_velocity_limits
        local has_acc_param = value[name].has_acceleration_limits

        if has_pos_param then
            max_min_pos[i][1] = value[name].max_position
            max_min_pos[i][2] = value[name].min_position
        else
            ros.WARN('Joint: %s has no position limit', name)
        end

        if has_vel_param then
            max_vel[i] = value[name].max_velocity
        else
            ros.WARN('Joint: %s has no velocity limit', name)
        end
        if has_acc_param then
            max_acc[i] = value[name].max_acceleration
        else
            max_acc[i] = max_vel[i] * 0.5
            ros.WARN('Joint: %s has no acceleration limit. Will be set to %f', name, max_acc[i])
        end
    end
    return max_vel, max_acc, max_min_pos
end

local function initControllers(delay, dt)
    local offset = math.ceil(delay / dt:toSec())

    config = node_handle:getParamVariable(string.format('%s/controller_list', node_handle:getNamespace()))
    local start_time = ros.Time.now()
    local current_time = ros.Time.now()
    local attemts = 0
    while config == nil do
        attemts = attemts + 1
        ros.WARN('no controller specified in "%s/controller_list". Retry in 5sec', node_handle:getNamespace())
        while current_time:toSec() - start_time:toSec() < 5 do
            current_time = ros.Time.now()
            sys.sleep(0.01)
        end
        start_time = ros.Time.now()
        current_time = ros.Time.now()
        config = node_handle:getParamVariable('controller_list')

        if not ros.ok() then
            return -1, 'Ros is not ok'
        end

        if attemts > 5 then
            return -2, 'Reached max attempts'
        end
    end
    local ns = ''
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(joint_name_collection, vv) == -1 then
                joint_name_collection[#joint_name_collection + 1] = vv
            end
        end
        ns = string.split(v.name, '/')
    end
    controller = Controller(#joint_name_collection)
    feedback_buffer_pos = core.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_pos.offset = offset
    feedback_buffer_vel = core.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_vel.offset = offset
    last_command_joint_position = torch.zeros(#joint_name_collection)
    last_command_joint_velocity = last_command_joint_position:clone():zero()
    for i = 1, offset + 1 do
        feedback_buffer_pos:add(last_command_joint_position)
    end
    local max_min_positions
    controller.max_vel,
        controller.max_acc,
        max_min_positions = queryJointLimits(joint_name_collection, node_handle:getNamespace())
    local soft_max_vel, soft_max_acc, soft_max_min_positions = queryJointLimits(joint_name_collection, '/robot_description_planning')
    last_command_joint_position =
        xutils.clamp(last_command_joint_position, max_min_positions[{{}, 2}], max_min_positions[{{}, 1}])
    last_command_joint_position =
        xutils.clamp(last_command_joint_position, soft_max_min_positions[{{}, 2}], soft_max_min_positions[{{}, 1}])
    controller.state.pos:copy(last_command_joint_position)
    subscriber = node_handle:subscribe(string.format('/%s/joint_command', ns[1]), jointtrajmsg_spec, 1)
    subscriber:registerCallback(jointCommandCb)

    return 0, 'Success'
end

local cmd = torch.CmdLine()
cmd:option('-delay', 0.150, 'Feedback delay time in s')
cmd:option('-frequency', 0.008, 'Node cycle time in s')

local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
initSetup(parameter['__name'], parameter)

local joint_state_publisher
local sim_seq = 1
local offset

local function sendJointState(position, velocity, joint_names, sequence)
    local m = ros.Message(joint_sensor_spec)
    m.header.seq = sequence
    m.header.stamp = ros.Time.now()
    --m.frame_id = "global"
    m.name = joint_names
    m.position:set(position)
    m.velocity:set(velocity)
    joint_state_publisher:publish(m)
end

local function init(delay, dt)
    joint_state_publisher = node_handle:advertise('/joint_states', joint_sensor_spec, 1)

    local value, succ = node_handle:getParamDouble(string.format('%s/feedback_delay', node_handle:getNamespace()))
    if succ then
        delay = value
    end
    value, succ = node_handle:getParamDouble('frequency')
    if succ then
        dt = value
    end
    dt = ros.Rate((1 / dt) or 125)
    offset = math.ceil(delay / dt:expectedCycleTime():toSec())
    local err, msg = initControllers(delay, dt:expectedCycleTime())

    if err < 0 then
        ros.ERROR('Could not initialize controller. ' .. msg)
        return
    end

    return dt
end

local function simulation(delay, dt)
    sim_seq = 1
    local heartbeat = xamla_sysmon.Heartbeat.new()
    heartbeat:start(node_handle, 1) --1 [Hz]
    heartbeat:updateStatus(heartbeat.GO, '')
    heartbeat:publish()
    local sysmon_watch = xamla_sysmon.Watch.new(node_handle, 3.0)
    dt = init(delay, dt)
    local initialized = true

    local timeout_recovery_counter = 0
    local timeout_error = false
    local pos, vel
    dt:reset()
    local global_state_summary = sysmon_watch:getGlobalStateSummary()
    local dependencies_are_no_go = global_state_summary.no_go and not global_state_summary.only_secondary_error

    -- main simulation loop
    while ros.ok() do
        -- check if we are running with an acceptable rate
        if not timeout_error and dt:expectedCycleTime():toSec() * 2 <= dt:cycleTime():toSec() then
            local err =
                string.format(
                'exeeded cycleTime delay: expected :%f actual: %f, diff: %f',
                dt:expectedCycleTime():toSec(),
                dt:cycleTime():toSec(),
                dt:expectedCycleTime():toSec() - dt:cycleTime():toSec()
            )
            if timeout_recovery_counter >= 200 then
                ros.WARN(err)
            --heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, err)
            end
            timeout_recovery_counter = 0
            timeout_error = true
        end
        if timeout_error == true and timeout_recovery_counter >= 200 then
            timeout_error = false
        --heartbeat:updateStatus(heartbeat.GO, '')
        end

        global_state_summary = sysmon_watch:getGlobalStateSummary()
        dependencies_are_no_go = global_state_summary.no_go and not global_state_summary.only_secondary_error
        error_state =
            dependencies_are_no_go and
            (heartbeat:getStatus() == heartbeat.GO or heartbeat:getStatus() == heartbeat.SECONDARY_ERROR)
        if error_state == false then
            if initialized == false then
                sim_seq = 1
                xutils.tic('Initialize')
                ros.INFO('Reinitializing...')
                heartbeat:updateStatus(heartbeat.GO, '')
                last_command_joint_position:copy(controller.state.pos) -- begin with current controller state
                --controller.state.pos:copy(last_command_joint_position)
                initialized = true
                xutils.toc('Initialize')
                dt:reset()
            end

            -- main update controller
            controller:update(last_command_joint_position, dt:expectedCycleTime():toSec())
            --controller:update(last_command_joint_position, ros.Time.now():toSec() - last_command_time:toSec())

            ros.DEBUG('latency: %f', ros.Time.now():toSec() - last_command_time:toSec())
        else
            if initialized then
                ros.ERROR('error state')
                initialized = false
            end
            if heartbeat:getStatus() == heartbeat.GO then
                heartbeat:updateStatus(heartbeat.SECONDARY_ERROR, '')
            end

            -- stay at current position
            controller:update(controller.state.pos, dt:expectedCycleTime():toSec())
        end

        feedback_buffer_pos:add(controller.state.pos)
        feedback_buffer_vel:add(controller.state.vel)
        pos = feedback_buffer_pos:getPastIndex(offset)
        vel = feedback_buffer_vel:getPastIndex(offset)
        if pos then
            sendJointState(pos, vel, joint_name_collection, sim_seq)
        end
        ros.spinOnce()

        sim_seq = sim_seq + 1

        timeout_recovery_counter = timeout_recovery_counter + 1
        heartbeat:publish()
        dt:sleep()
    end
    joint_state_publisher:shutdown()
end

simulation(parameter.delay, parameter.frequency)
shutdownSetup()
