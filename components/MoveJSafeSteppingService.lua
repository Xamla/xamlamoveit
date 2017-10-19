local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local LeasedBaseLockClient = xutils.LeasedBaseLockClient
local ms = require 'xamlamoveit.components.MotionService'

local function queryMoveJSafeSteppingServiceHandler(self, request, response, header)
    if not self.current_path then
        self.path_index = 1
        self.current_goal = request.point
        self.current_path = nil
        response.success = true
    else
        response.success = false
        response.error = 'Joint Path is already set'
    end
    return true
end

local function queryNextPathPointHandler(self, request, response, header)
    if self.path_index > 0 then
        self.path_index = self.path_index + 1
        response.success = #self.current_path >= self.path_index
        response.message =
            string.format('Path index increased to %d from %d points in path', self.path_index, #self.current_path)
    else
        response.success = false
        response.message = 'No path loaded'
    end
end

local set_bool_spec = ros.SrvSpec('std_srvs/SetBool')
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/SetJointPath')
local trigger_srv_spec = ros.SrvSpec('std_srvs/Trigger')
local joint_point_spec = ros.MsgSpec("trajectory_msgs/JointTrajectoryPoint")

local components = require 'xamlamoveit.components.env'
local MoveJSafeSteppingService,
    parent =
    torch.class('xamlamoveit.components.MoveJSafeSteppingService', 'xamlamoveit.components.RosComponent', components)

function MoveJSafeSteppingService:__init(nh)
    self.node_handle = nh
    self.callback_queue = ros.CallbackQueue()
    self.trigger_callback_queue = ros.CallbackQueue()
    self.robot_model_loader = nil
    self.robot_model = nil
    self.resource_names = nil
    self.info_server = nil
    self.trigger_server = nil
    self.current_path = nil
    self.path_index = -1
    self.lock_client = nil
    self.lock = nil
    self.motion_service = nil
    parent.__init(self, nh)
end

function MoveJSafeSteppingService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    local robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.resource_names = robot_state:getVariableNames():totable()
    self.lock_client = LeasedBaseLockClient(self.node_handle)
    self.motion_service = ms.new(self.node_handle)
    --TODO feedback
end

function MoveJSafeSteppingService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'set_joint_path',
        srv_spec,
        function(request, response, header)
            return queryMoveJSafeSteppingServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
    self.trigger_server =
        self.node_handle:advertiseService(
        'trigger_next_point',
        trigger_srv_spec,
        function(request, response, header)
            return queryNextPathPointHandler(self, request, response, header)
        end,
        self.trigger_callback_queue
    )
    self.publisher = self.node_handle:advertise('topic_to_controller', joint_point_spec, 1)
end

function MoveJSafeSteppingService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming MoveJSafeSteppingService call')
        self.callback_queue:callAvailable()
    end
    if not self.trigger_callback_queue:isEmpty() then
        ros.INFO('[!] incoming trigger call for MoveJSafeSteppingService')
        self.trigger_callback_queue:callAvailable()
    end
    if self.path_index > 0 then
        if self.lock then
            self.lock = self.lock_client:lock(self.lock.resources, self.lock.id)
        else
            self.lock = self.lock_client:lock(self.resource_names)
        end
        --lock all resources
        if self.path_index == 1 then
        --Check if robot is at given start
        end
    else
        if self.lock then
            self.lock_client:release(self.lock.resources, self.lock.id)
            self.lock = nil
        end
    end
end

function MoveJSafeSteppingService:onReset()
    self.current_path = nil
    self.path_index = -1
    if self.lock then
        self.lock_client:release(self.lock.resources, self.lock.id)
        self.lock = nil
    end
end

function MoveJSafeSteppingService:onStop()
    self.info_server:shutdown()
    self.trigger_server:shutdown()
end

function MoveJSafeSteppingService:onShutdown()
    self.info_server:shutdown()
    self.trigger_server:shutdown()
    self.lock_client:shutdown()
end

return MoveJSafeSteppingService
