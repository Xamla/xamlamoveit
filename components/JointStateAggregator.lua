--[[
JointStateAggregator.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local JointMonitor = require 'xamlamoveit.core'.JointMonitor
local components = require 'xamlamoveit.components.env'

local JointStateAggregator, parent =
    torch.class('xamlamoveit.components.JointStateAggregator', 'xamlamoveit.components.RosComponent', components)

local joint_state_spec = ros.MsgSpec('sensor_msgs/JointState')

local function sendJointState(self, position, names)
    ros.DEBUG('sendPositionCommand')
    local m = ros.Message(joint_state_spec)
    m.header.seq = self.seq
    m.header.stamp = ros.Time.now()
    m.name = names
    m.position = position

    self.joint_state_publisher:publish(m)
    self.seq = self.seq + 1
end

local function toDuration(value, default_value)
    if not value then
        return default_value
    elseif type(value) == 'number' then
        if value < 0 then
            return nil
        else
            return ros.Duration(value)
        end
    elseif not torch.isTypeOf(value, ros.Duration) then
        error('Invalid argument: Instance of ros.Duration expected')
    end
end

function JointStateAggregator:__init(node_handle, timeout)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.joint_state_publisher = nil
    self.joint_monitor = nil
    self.last_joint_state = nil
    self.joint_names = {}
    self.timeout = toDuration(timeout, ros.Duration(0.1))
    self.feedback_topic = 'aggregated_joint_state'
    self.seq = 1
    parent.__init(self, node_handle)
end

local function getControllerConfig(self)
    local config = self.node_handle:getParamVariable(string.format('%s/controller_list', self.node_handle:getNamespace()))
    --local config = self.node_handle:getParamVariable('/move_group/controller_list')
    local start_time = ros.Time.now()
    local current_time = ros.Time.now()
    local attempts = 0
    while config == nil do
        attempts = attempts + 1
        ros.WARN('no controller specified in "%s/controller_list". Retry in 5sec', self.node_handle:getNamespace())
        while current_time:toSec() - start_time:toSec() < 5 do
            current_time = ros.Time.now()
            sys.sleep(0.01)
        end
        start_time = ros.Time.now()
        current_time = ros.Time.now()
        config = self.node_handle:getParamVariable(string.format('%s/controller_list', self.node_handle:getNamespace()))

        if not ros.ok() then
            return -1, 'Ros is not ok'
        end

        if attempts > 5 then
            return -2, 'Reached max attempts'
        end
    end
    for i, v in ipairs(config) do
        for ii, vv in ipairs(v.joints) do
            if table.indexof(self.joint_names, vv) == -1 then
                self.joint_names[#self.joint_names + 1] = vv
            end
        end
    end
end

function JointStateAggregator:onInitialize()
    getControllerConfig(self)
    if #self.joint_names>0 then
        self.joint_monitor = JointMonitor.new(self.joint_names, self.timeout:toSec())
        local ready = self.joint_monitor:waitReady(20.0)
        local once = true
        while not ready and ros.ok() do
            if once then
                ros.ERROR('joint states not ready')
                once = false
            end
            ready = self.joint_monitor:waitReady(20.0)
        end
        if not ros.ok() then
            self:shutdown()
            return
        end
        ros.INFO('joint states ready')
        local ok
        ok, self.last_joint_state = self.joint_monitor:getNextPositionsTensor(0.5)
        self.joint_names = self.joint_monitor:getJointNames()
    end
end

function JointStateAggregator:onStart()
    self.joint_state_publisher = self.node_handle:advertise(self.feedback_topic, joint_state_spec)
end

function JointStateAggregator:onProcess()
    local ok, p = true, nil
    if #self.joint_names > 0 then
        ok, p = self.joint_monitor:getNextPositionsTensor(0.1)
    end
    if not ok then
        ros.ERROR_THROTTLE('JointStateAggregator:onProcess', 1, '[onProcess] exceeded timeout for next robot joint state!')
    end
    if self.last_joint_state then
        self.last_joint_state:copy(p)
    else
        if p then
            self.last_joint_state = p:clone()
        end
    end
    if self.last_joint_state then
        sendJointState(self, self.last_joint_state, self.joint_names)
    end
end

function JointStateAggregator:onStop()
    self.joint_state_publisher:shutdown()
end

function JointStateAggregator:onReset()
    self.seq = 1
    self.joint_monitor:shutdown()
end

return JointStateAggregator
