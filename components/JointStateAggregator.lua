local ros = require 'ros'
local moveit = require 'moveit'
local JointMonitor = require 'xamlamoveit.core'.JointMonitor
local components = require 'xamlamoveit.components.env'
local JointStateAggregator,
    parent =
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
    self.robot_model = nil
    self.robot_model_loader = nil
    self.robot_state = nil
    self.joint_state_publisher = nil
    self.joint_monitor = nil
    self.last_joint_state = nil
    self.joint_names = nil
    self.timeout = toDuration(timeout, ros.Duration(0.1))
    self.feedback_topic = 'aggregated_joint_state'
    self.seq = 1
    parent.__init(self, node_handle)
end

function JointStateAggregator:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.joint_monitor = JointMonitor.new(self.robot_model:getVariableNames():totable(), self.timeout:toSec())
    local ready = self.joint_monitor:waitReady(2.1)
    if ready then
        self.last_joint_state = self.joint_monitor:getPositionsTensor()
        self.joint_names = self.joint_monitor:getJointNames()
    else
        ros.ERROR('joint states not ready')
    end
end

function JointStateAggregator:onStart()
    self.joint_state_publisher = self.node_handle:advertise(self.feedback_topic, joint_state_spec)
end

function JointStateAggregator:onProcess()
    self.last_joint_state:copy(self.joint_monitor:getPositionsTensor())
    sendJointState(self, self.last_joint_state, self.joint_names)
end

function JointStateAggregator:onStop()
    self.joint_state_publisher:shutdown()
end

function JointStateAggregator:onReset()
    self.seq = 1
    self.joint_monitor:shutdown()
end

return JointStateAggregator
