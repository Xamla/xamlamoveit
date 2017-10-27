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

local function createPoseMsg(frame, translation, rotation)
    assert(torch.type(frame) == 'string')
    assert(torch.isTypeOf(translation, torch.DoubleTensor))
    assert(torch.isTypeOf(rotation, torch.DoubleTensor))
    local msg = ros.Message(pose_msg_spec)
    msg.pose.position.x = translation[1]
    msg.pose.position.y = translation[2]
    msg.pose.position.z = translation[3]
    msg.pose.orientation.x = rotation[1]
    msg.pose.orientation.y = rotation[2]
    msg.pose.orientation.z = rotation[3]
    msg.pose.orientation.w = rotation[4]
    msg.header.frame_id = frame
    return msg
end

local function poses2MsgArray(points)
    local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
    local result = {}
    if torch.type(points) == 'table' then
        for i, v in ipairs(points) do
            assert(
                torch.isTypeOf(v, tf.StampedTransform),
                'points need to be type of datatypes.Pose, but is type: ',
                torch.type(v)
            )

            local translation = v:getOrigin()
            local rotation = v:getRotation():toTensor()
            local frame = v:get_frame_id()

            table.insert(result, createPoseMsg(frame, translation, rotation))
        end
    elseif torch.isTypeOf(points, tf.StampedTransform) then
        local translation = points:getOrigin()
        local rotation = points:getRotation():toTensor()
        local frame = points:get_frame_id()

        table.insert(result, createPoseMsg(frame, translation, rotation))
    else
        error('[poses2MsgArray] unknown type of points parameter: ' .. torch.type(points))
    end
    return result
end

local function poseStampedMsg2StampedTransform(msg)
    local result = tf.StampedTransform()
    result:setOrigin(torch.Tensor {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z})
    result:setRotation(
        tf.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    )
    return result
end

local function getLinearPath(start, goal, num_samples)
    local start = poseStampedMsg2StampedTransform(start)
    local goal = poseStampedMsg2StampedTransform(goal)
    local pose = start:clone()
    local result = {}
    local direction = goal:getOrigin() - start:getOrigin()
    for i = 1, num_samples do
        pose:setOrigin(start:getOrigin() + direction * (i - 1) / num_samples)
        pose:setRotation(start:getRotation():slerp(goal:getRotation(), (i - 1) / num_samples))
        result[i] = pose:clone()
    end
    return result
end

local function queryCartesianPathServiceHandler(self, request, response, header)
    if #request.waypoints < 2 then
        request.error_code.val = -2
        return true
    end
    local g_path = response.path
    if request.num_steps == #request.waypoints and request.num_steps == 2 then
        local path = getLinearPath(request.waypoints[1], request.waypoints[2], request.num_steps)
        if #path>0 then
            table_concat(g_path, poses2MsgArray(path))
        end
    else
        for i, v in ipairs(request.waypoints) do
            local k = math.min(#request.waypoints, i + 1)
            local w = request.waypoints[k]
            local path = getLinearPath(v, w, request.num_steps)
            if #path > 0 then
                table_concat(g_path, poses2MsgArray(path))
            end
        end
    end
    response.error_code.val = 1
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
