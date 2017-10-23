local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetCurrentJointState')
local ik_srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
local fk_srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetFKSolution')

local function create_ik_request(group_name, robot_state, avoid_collisions, poses_stamped, ik_link_names)
    local ik_link_names = ik_link_names or {}
    local robot_state_msg
    if robot_state then
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
    local r_state = self.robot_state:clone()
    if request.seed then
        r_state:setVariablePositions(request.seed.positions, request.joint_names)
        r_state:update()
    end

    ros.DEBUG('query Group Joint Names for: ' .. request.group_name)
    local known_joint_names = self.robot_model:getGroupJointNames(request.group_name)
    ros.DEBUG('query Group EndEffector Names for: ' .. request.group_name)

    local target_link = request.end_effector_link
    local ik_req =
        create_ik_request(request.group_name, r_state, request.collision_check, {request.points[1]}, {target_link})
    local ik_res = self.ik_service_client:call(ik_req)

    if ik_res.error_code.val ~= 1 then
        response.error_code = ik_res.error_code
        return true
    end
    if not r_state:fromRobotStateMsg(ik_res.solution) then
        response.error_code.val = -17 -- INVALID_ROBOT_STATE
    end
    local point_msg = ros.Message('xamlamoveit_msgs/JointPathPoint')
    local state = r_state:getVariablePositions()
    local names = r_state:getVariableNames():totable()
    local res = torch.DoubleTensor(#request.joint_names)
    for i, v in ipairs(request.joint_names) do
        local index = table.indexof(names, v)
        res[i] = state[index]
    end
    point_msg.positions = res
    response.solution[1] = point_msg

    response.error_code = ik_res.error_code
    return true
end

local function queryFKServiceHandler(self, request, response, header)
    local r_state = self.robot_state:clone()
    r_state:setVariablePositions(request.point.positions, request.joint_names)
    r_state:update()
    local ee_names = self.robot_model:getGroupEndEffectorName(request.group_name)
    local pose
    if ee_names ~= '' then
        local ee_link_name = self.robot_model:getEndEffectorLinkName(ee_names)
        print("getEndEffectorLinkName", ee_link_name)
        pose = r_state:getGlobalLinkTransform(ee_link_name)
        print(pose)
    else
        response.error_code.val = -2
        return true
    end
    if pose then
        local position = pose:getOrigin()
        local quaternion = pose:getRotation():toTensor()
        response.solution.pose.position.x = position[1]
        response.solution.pose.position.y = position[2]
        response.solution.pose.position.z = position[3]
        response.solution.pose.orientation.x = quaternion[1]
        response.solution.pose.orientation.y = quaternion[2]
        response.solution.pose.orientation.z = quaternion[3]
        response.solution.pose.orientation.w = quaternion[4]
        response.error_code.val = 1
        print("ik query successfull")
    else
        response.error_code.val = -3
    end
    return true
end

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
    self.ik_callback_queue = ros.CallbackQueue()
    self.fk_callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.planning_scene = nil
    self.joint_monitor = nil
    self.robot_state = nil
    self.info_server = nil
    self.fk_info_server = nil
    self.ik_info_server = nil
    self.ik_service_client = nil
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

    if ready then
        self.robot_state:setVariablePositions(
            self.joint_monitor:getNextPositionsTensor(),
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
    else
        ros.ERROR('joint states not ready')
    end
    self.ik_service_client = self.node_handle:serviceClient('/compute_ik', 'moveit_msgs/GetPositionIK')
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
    self.ik_info_server =
        self.node_handle:advertiseService(
        'query_ik',
        ik_srv_spec,
        function(request, response, header)
            return queryIKServiceHandler(self, request, response, header)
        end,
        self.ik_callback_queue
    )
    self.fk_info_server =
        self.node_handle:advertiseService(
        'query_fk',
        fk_srv_spec,
        function(request, response, header)
            return queryFKServiceHandler(self, request, response, header)
        end,
        self.fk_callback_queue
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

    if not self.ik_callback_queue:isEmpty() then
        ros.INFO('[!] incoming ComputeIKService call')
        self.ik_callback_queue:callAvailable()
    end

    if not self.fk_callback_queue:isEmpty() then
        ros.INFO('[!] incoming ComputeFKService call')
        self.fk_callback_queue:callAvailable()
    end
end

function PositionStateInfoService:onStop()
    self.info_server:shutdown()
    self.fk_info_server:shutdown()
    self.ik_info_server:shutdown()
    self.ik_service_client:shutdown()
end

function PositionStateInfoService:onReset()
    self.joint_monitor:shutdown()
end

return PositionStateInfoService
