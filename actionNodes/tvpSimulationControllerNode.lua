#!/usr/bin/env th
local torch = require 'torch'
local ros = require 'ros'
local xamlamoveit = require 'xamlamoveit'
local xutils = xamlamoveit.xutils
local printf = xutils.printf
local xtable = xutils.Xtable

local xamlacontroller = xamlamoveit.controller
local Controller = xamlacontroller.TvpController

local jointtrajmsg_spec = ros.MsgSpec('trajectory_msgs/JointTrajectory')
local joint_sensor_spec = ros.MsgSpec('sensor_msgs/JointState')
local joint_msg = ros.Message(joint_sensor_spec)

local subscriber

local xamla_sysmon = require 'xamla_sysmon'

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
local function updateSystemState(msg, header)
    if msg.system_status ~= 0 and error_state == false then
        error_state = true
    elseif msg.system_status == 0 and error_state == true then
        error_state = false
    end
end

local function query_joint_limits(joint_names, namespace)
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
        config = node_handle:getParamVariable(string.format('%s/controller_list', node_handle:getNamespace()))

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
    feedback_buffer_pos = xutils.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_pos.offset = offset
    feedback_buffer_vel = xutils.MonitorBuffer(offset + 1, #joint_name_collection)
    feedback_buffer_vel.offset = offset
    last_command_joint_position = torch.ones(#joint_name_collection) * 1.3
    last_command_joint_velocity = last_command_joint_position:clone():zero()
    for i = 1, offset + 1 do
        feedback_buffer_pos:add(last_command_joint_position)
    end
    controller.max_vel, controller.max_acc = query_joint_limits(joint_name_collection, node_handle:getNamespace())
    controller.state.pos:copy(last_command_joint_position)
    subscriber = node_handle:subscribe(string.format('/%s/joint_command', ns[1]), jointtrajmsg_spec, 1)
    subscriber:registerCallback(jointCommandCb)

    return 0, 'Success'
end

local cmd = torch.CmdLine()
cmd:option('-delay', 0.150, 'Feedback delay time in ms')
cmd:option('-frequency', 0.008, 'Node cycle time in ms')

local parameter = xutils.parseRosParametersFromCommandLine(arg, cmd) or {}
initSetup(parameter['__name']) -- TODO

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
    heartbeat:start(node_handle, 0.5) --[Hz]
    heartbeat:updateStatus(heartbeat.GO, '')
    heartbeat:publish()
    local system_state_subscriber =
        node_handle:subscribe(
        '/xamla_sysmon/system_status',
        'xamla_sysmon_msgs/SystemStatus',
        1,
        {'udp', 'tcp'},
        {tcp_nodelay = true}
    )
    dt = init(delay, dt)
    local initialized = true
    system_state_subscriber:registerCallback(updateSystemState)

    local timeout_recovery_counter = 0
    local timeout_error = false
    local pos, vel
    dt:reset()
    while ros.ok() do
        if not timeout_error and dt:expectedCycleTime():toSec() * 2 <= dt:cycleTime():toSec() then
            local err =
                string.format(
                'exeeded cycleTime delay: expected :%f actual: %f, diff: %f',
                dt:expectedCycleTime():toSec() * 2,
                dt:cycleTime():toSec(),
                dt:expectedCycleTime():toSec() * 2 - dt:cycleTime():toSec()
            )
            if timeout_recovery_counter >= 200 then
                heartbeat:updateStatus(heartbeat.INTERNAL_ERROR, err)
            end
            timeout_recovery_counter = 0
            timeout_error = true
        end
        if timeout_error == true and timeout_recovery_counter >= 200 then
            timeout_error = false
            heartbeat:updateStatus(heartbeat.GO, '')
        end
        if error_state == false then
            if initialized == false then
                sim_seq = 1
                xutils.tic('Initialize')
                ros.INFO('Reinizialise')
                dt = init()
                heartbeat:updateStatus(heartbeat.GO, '')
                controller.state.pos:copy(last_command_joint_position)
                initialized = true
                xutils.toc('Initialize')
                dt:reset()
            end
            controller:update(last_command_joint_position, dt:expectedCycleTime():toSec())
            --controller:update(last_command_joint_position, ros.Time.now():toSec() - last_command_time:toSec())
            ros.DEBUG('latency: %f', ros.Time.now():toSec() - last_command_time:toSec())
        else
            if initialized then
                ros.ERROR('error state')
                initialized = false
            end
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
    system_state_subscriber:shutdown()
end

simulation(parameter.delay, parameter.frequency)
shutdownSetup()
