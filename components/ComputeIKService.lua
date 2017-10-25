local ros = require 'ros'
local moveit = require 'moveit'
local optimplan = require 'optimplan'
require 'xamlamoveit.xutils'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')

local function create_ik_request(group_name, robot_state, avoid_collisions, poses_stamped, ik_link_names)
    local ik_link_names = ik_link_names or {}
    local robot_state_msg
    if robot_state then
        print(torch.type(robot_state), robot_state)
        robot_state_msg = robot_state:toRobotStateMsg()
    end
    local ik_spec = ros.MsgSpec('moveit_msgs/PositionIKRequest')
    local req_msg = ros.Message(ik_spec)
    req_msg.group_name = group_name
    req_msg.avoid_collisions = avoid_collisions
    if robot_state_msg then
        req_msg.robot_state = robot_state_msg
    end

    if #poses_stamped == 1 then
        local poses = {}
        for i, k in ipairs(poses_stamped) do
            table.insert(poses, k)
        end
        req_msg.ik_link_names = ik_link_names
        req_msg.pose_stamped_vector = poses
    else
        if #ik_link_names > 0 then
            req_msg.ik_link_name = ik_link_names[1]
        end
        req_msg.pose_stamped = poses_stamped[1]
        req_msg.timeout = ros.Duration(0.1)
        req_msg.attempts = 5
    end
    return req_msg
end

local function queryIKServiceHandler(self, request, response, header)
    local r_state
    if request.seed then
        self.robot_state:setVariablePositions(
            query_joint_state(self, self.robot_state:getVariableNames():totable()),
            self.robot_state:getVariableNames()
        )
        r_state = self.robot_state:setVariablePositions(request.seed, request.joint_names)
        print(r_state:getVariablePositions())
    end

    ros.DEBUG('query Group Joint Names for: ' .. request.group_name)
    local known_joint_names = self.robot_model:getGroupJointNames(request.group_name)
    ros.DEBUG('query Group EndEffector Names for: ' .. request.group_name)

    local target_link = request.end_effector_link
    local ik_req =
        create_ik_request(request.group_name, r_state, request.collision_check, {request.points[1]}, {target_link})
    local ik_res = self.service_client:call(ik_req)

    if ik_res.error_code.val ~= 1 then
        response.error_code = ik_res.error_code
        return true
    end
    if not self.robot_state:fromRobotStateMsg(ik_res.solution) then
        response.error_code.val = -17 -- INVALID_ROBOT_STATE
    end
    local point_msg = ros.Message('xamlamoveit_msgs/JointPathPoint')
    local state = self.robot_state:getVariablePositions()
    local names = self.robot_state:getVariableNames():totable()
    local res = torch.DoubleTensor(#request.joint_names)
    for i, v in ipairs(request.joint_names) do
        local index = table.indexof(names, v)
        print(index)
        res[i] = state[index]
    end
    point_msg.positions = res
    response.solution[1] = point_msg

    response.error_code = ik_res.error_code
    return true
end

local components = require 'xamlamoveit.components.env'
local ComputeIKService,
    parent = torch.class('xamlamoveit.components.ComputeIKService', 'xamlamoveit.components.RosComponent', components)

function ComputeIKService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.robot_state = nil
    self.info_server = nil
    parent.__init(self, node_handle)
end

function ComputeIKService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)
    self.service_client = self.node_handle:serviceClient('/compute_ik', 'moveit_msgs/GetPositionIK')
end

function ComputeIKService:onStart()
    local timeout = ros.Duration(10)
    local ok = self.service_client:waitForExistence(timeout)
    if not ok then
        ros.ERROR('could not reach ik service!')
        error('could not reach ik service!')
    end
    self.info_server =
        self.node_handle:advertiseService(
        'query_ik',
        srv_spec,
        function(request, response, header)
            return queryIKServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function ComputeIKService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming ComputeIKService call')
        self.callback_queue:callAvailable()
    end
end

function ComputeIKService:onStop()
    ros.WARN('ComputeIKService:onStop() NOT IMPLEMENTED')
end

function ComputeIKService:onReset()
    self.info_server:shutdown()
    self.service_client:shutdown()
end

function ComputeIKService:onShutdown()
    self.info_server:shutdown()
    self.service_client:shutdown()
end

return ComputeIKService
