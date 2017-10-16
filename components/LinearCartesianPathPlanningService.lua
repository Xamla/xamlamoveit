local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local optimplan = require 'optimplan'
--require 'xamlamoveit.components.RosComponent'
local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetLinearCartesianPath')
local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')

local function table_concat(dst, src)
    for i, v in ipairs(src) do
        table.insert(dst, v)
    end
    return dst
end

local function poseStampedMsg2StampedTransform(msg)
    local result = tf.StampedTransform()
    result:setOrigin(torch.Tensor {msg.pose.position.x, msg.pose.position.x, msg.pose.position.z})
    result:setRotation(
        tf.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    )
end
local function getLinearPath(start, goal)
    local pose = ros.Messege(pose_msg_spec)
    local start = poseStampedMsg2StampedTransform(start)
    local goal = poseStampedMsg2StampedTransform(goal)
    for i = 1, trajectory:size(1) do
        pose:setOrigin(start:getOrigin():clone())
        pose:setRotation(start:getRotation():slerp(goal:getRotation(), (i - 1) / trajectory:size(1)))
    end
    return true
end

local function queryCartesianPathServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        request.error_code.val = -2
        return true
    end
    local g_path = response.path
    for i, v in ipairs(request.waypoints) do
        local k = math.min(#request.waypoints, i + 1)
        local w = request.waypoints[k]
        local path = getLinearPath(v, w)
        if #path > 0 then
            table_concat(g_path, path)
        end
    end
    return true
end

local components = require 'xamlamoveit.components.env'
local LinearCartesianPathPlanningService,
    parent =
    torch.class(
    'xamlamoveit.components.LinearCartesianPathPlanningService',
    'xamlamoveit.components.RosComponent',
    components
)

function LinearCartesianPathPlanningService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.info_server = nil
    parent.__init(self, node_handle)
end

function LinearCartesianPathPlanningService:onInitialize()
end

function LinearCartesianPathPlanningService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_cartesian_path',
        srv_spec,
        function(request, response, header)
            return queryCartesianPathServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function LinearCartesianPathPlanningService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming LinearCartesianPathPlanningService call')
        self.callback_queue:callAvailable()
    end
end

function LinearCartesianPathPlanningService:onStop()
    ros.WARN('LinearCartesianPathPlanningService:onStop() NOT IMPLEMENTED')
end

function LinearCartesianPathPlanningService:onReset()
    self.info_server:shutdown()
end

function LinearCartesianPathPlanningService:onShutdown()
    self.info_server:shutdown()
end

return LinearCartesianPathPlanningService
