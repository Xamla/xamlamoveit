local torch = require 'torch'
local ros = require 'ros'

local xutils = require 'xamlamoveit.xutils.env'
local JointMonitor = torch.class('JointMonitor', xutils)

local function jointStatesCb(self, msg, header)
    for i, name in ipairs(msg.name) do
        local js = self.joint_status[name]
        if js then
            js[1] = msg.position[i]
            js[2] = msg.header.stamp
            js[3] = self.seq
        end
    end
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

function JointMonitor:__init(joint_names, joint_timeout, joint_states_topic)
    if type(joint_names) ~= 'table' then
        error("Missing argument: 'joint_names'")
    end

    self.joint_timeout = toDuration(joint_timeout, ros.Duration(0.1))
    self.seq = 1
    self.last_seq = self.seq
    self.joint_states_topic = joint_states_topic or '/joint_states'
    self.nh = ros.NodeHandle()
    self.joint_names = joint_names

    self.joint_status = {}
    for i, name in ipairs(joint_names) do
        self.joint_status[name] = {nil, ros.Time(0)}
    end

    -- subscribe to joint states topic
    self.joint_state_sub = self.nh:subscribe(self.joint_states_topic, 'sensor_msgs/JointState', 16)
    self.joint_state_sub:registerCallback(
        function(msg, header)
            jointStatesCb(self, msg, header)
        end
    )
end

local function updated(self, seq)
    for k, v in pairs(self.joint_status) do
        if v[1] and v[3] <= seq then
            return false
        end
    end
    return true
end

-- waits until all joints have seen new values
function JointMonitor:waitForNextState(timeout)
    local start_seq = self.last_seq
    if not updated(self, start_seq) then
        timeout = toDuration(timeout)

        local wait_duration = ros.Duration(0.0001)
        local start = ros.Time.now()
        while not updated(self, start_seq) and ros.ok() do
            if timeout ~= nil then
                local elapsed = ros.Time.now() - start
                if elapsed > timeout then
                    return false
                end
            end
            ros.spinOnce()
            wait_duration:sleep()
        end
    end

    self.last_seq = self.seq

    return true
end

-- wait to receive initial state
function JointMonitor:waitReady(timeout)
    timeout = toDuration(timeout)

    local wait_duration = ros.Duration(0.01)
    local start = ros.Time.now()
    while not self:isReady() do
        local elapsed = ros.Time.now() - start
        if (not ros.ok()) or ((timeout ~= nil) and (elapsed > timeout)) then
            ros.WARN(string.format('Joint states not available during: %s sec', tostring(elapsed)))
            return false
        end
        ros.spinOnce()
        wait_duration:sleep()
    end
    return true
end

function JointMonitor:isReady()
    for k, v in pairs(self.joint_status) do
        if not v[1] then
            return false
        end
    end
    return true
end

-- Get joint names
function JointMonitor:getJointNames()
    return self.joint_names
end

function JointMonitor:getPositions(target_joint_names)
    local target_joint_names = target_joint_names or self.joint_names
    local now = ros.Time.now()
    local pos = {}
    local max_latency = ros.Duration(0)
    for i, v in ipairs(self.joint_names) do
        if table.indexof(target_joint_names,v) > -1 then
            local latency = now - self.joint_status[v][2]
            if latency > max_latency then
                max_latency = latency
            end
            if latency > self.joint_timeout then
                ros.ERROR(string.format("joint timeout: '%s' was last updated %fs ago.", v, latency:toSec()))
            end
            pos[#pos + 1] = self.joint_status[v][1]
            assert(pos[#pos] ~= nil, string.format("Position value of joint '%s' is invalid.", v))
        end
    end
    return pos, max_latency
end

-- Get joint position values in the order of the joint_names list used to initialize the JointMonitor.
function JointMonitor:getPositionsUnchecked()
    local pos = {}
    for i, v in ipairs(self.joint_names) do
        pos[i] = self.joint_status[v][1]
    end
    return pos
end

function JointMonitor:getPositionsTensor(target_joint_names)
    local p, l = self:getPositions(target_joint_names)
    return torch.DoubleTensor({p}):squeeze(1), l
end

function JointMonitor:getNextPositions(timeout)
    self:waitForNextState(timeout)
    return self:getPositions()
end

function JointMonitor:getNextPositionsTensor(timeout,target_joint_names)
    self:waitForNextState(timeout)
    return self:getPositionsTensor(target_joint_names)
end

function JointMonitor:getTimestamps()
    local last_update = {}
    for i, v in ipairs(self.joint_names) do
        last_update[i] = self.joint_status[v][2]
    end
    return last_update
end

-- Get max joint latency and list of individual joint latencies in the joint_names order used
-- to initialze the JointMonitor.
function JointMonitor:getLatency()
    local latency_values = {}
    local now = ros.Time.now()

    for i, v in ipairs(self.joint_names) do
        latency_values[i] = (now - self.joint_status[v][2]):toSec()
    end
    local max_latency = math.max(unpack(latency_values))
    return max_latency, latency_values
end

function JointMonitor:shutdown()
    self.joint_state_sub:shutdown()
end
