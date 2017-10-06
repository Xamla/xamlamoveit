local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetCurrentJointState')

local function queryJointPositionServiceHandler(self, request, response, header)
    local joint_names = request.joint_names
    local wait_duration = ros.Duration(0.1)
    local joints = self.joint_monitor:getNextPositionsTensor(wait_duration, joint_names)
    --local joints = self.robot_state:getVariablePositions(joint_names):clone()
    response.current_joint_position.header.stamp = ros.Time.now()
    response.current_joint_position.name = joint_names
    response.current_joint_position.position = joints
    return joints:size(1) == #joint_names
end

local components = require 'xamlamoveit.components.env'
local PositionStateInfoService,
    parent =
    torch.class('xamlamoveit.components.PositionStateInfoService', 'xamlamoveit.components.RosComponent', components)

function PositionStateInfoService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.planning_scene = nil
    self.joint_monitor = nil
    self.robot_state = nil
    self.info_server = nil
    parent.__init(self, node_handle)
end

function PositionStateInfoService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.robot_model)
    self.planning_scene:syncPlanningScene()

    self.joint_monitor = xutils.JointMonitor(self.robot_model:getVariableNames():totable())
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)

    local ready = false

    ready = self.joint_monitor:waitReady(2.1)
    ros.ERROR('joint states not ready')
    if ready then
        self.robot_state:setVariablePositions(
            self.joint_monitor:getNextPositionsTensor(),
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
    end
end

function PositionStateInfoService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_move_group_current_position',
        srv_spec,
        function(request, response, header)
            return queryJointPositionServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function PositionStateInfoService:onProcess()
    self.robot_state:setVariablePositions(
        self.joint_monitor:getNextPositionsTensor(),
        self.joint_monitor:getJointNames()
    )
    self.robot_state:update()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming PositionStateInfoService call')
        self.callback_queue:callAvailable()
    end
end

function PositionStateInfoService:onStop()
    self.info_server:shutdown()
end

function PositionStateInfoService:onReset()
    self.joint_monitor:shutdown()
end

return PositionStateInfoService
