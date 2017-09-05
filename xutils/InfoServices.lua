local torch = require 'torch'
local ros = require 'ros'
local moveit = require 'moveit'

local xutils = require 'xamlamoveit.xutils.env'
local InfoServices = torch.class('InfoServices', xutils)

local service_queue, ik_service_queue, joint_position_queue

local info_server, current_joint_position_info_server

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryMoveGroupInterfaces')
local cj_srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetCurrentJointState')

local msg_spec = ros.MsgSpec('xamlamoveit_msgs/MoveGroupInterfaceDescription')

local robot_model_loader, robot_model

local wait_duration = ros.Duration(0.01)

function queryServiceHandler(request, response, header)
    robot_model_loader = moveit.RobotModelLoader('robot_description')
    robot_model = robot_model_loader:getModel()

    local all_EE_parent_group_names, all_EE_parent_link_names = robot_model:getEndEffectorParentGroups()
    local all_group_joint_names = robot_model:getJointModelGroupNames()
    print("print(all_EE_parent_link_names)")
    print(all_EE_parent_link_names)

    for k, v in pairs(all_group_joint_names) do
        local l = ros.Message(msg_spec)
        l.name = v
        l.sub_move_group_ids = robot_model:getJointModelSubGroupNames(v)
        l.joint_names = robot_model:getGroupJointNames(v)
        l.end_effector_names = robot_model:getGroupEndEffectorNames(v)
        table.insert(response.move_group_interfaces, l)
    end
    return true
end

function queryJointPositionServiceHandler(self, request, response, header)
    local joint_names = request.joint_names
    local joints = self.joint_monitor:getNextPositionsTensor(wait_duration,joint_names)
    response.current_joint_position.header.stamp = ros.Time.now()
    response.current_joint_position.name = joint_names
    response.current_joint_position.position = joints
    return joints:size(1) == #joint_names
end

function InfoServices:__init(node_handle)
    service_queue = ros.CallbackQueue()
    joint_position_queue = ros.CallbackQueue()
    self.info_server =
    node_handle:advertiseService('/query_move_group_interface', srv_spec, queryServiceHandler, service_queue)
    self.current_joint_position_info_server =
    node_handle:advertiseService(
        '/query_move_group_current_position',
        cj_srv_spec,
        function(request, response, header)
            return queryJointPositionServiceHandler(self, request, response, header)
        end,
        joint_position_queue
    )
    robot_model_loader = moveit.RobotModelLoader('robot_description')
    robot_model = robot_model_loader:getModel()

    self.joint_monitor = xutils.JointMonitor(robot_model:getVariableNames():totable())
    local ready = false
    while not ready and ros.ok() do
        ready = self.joint_monitor:waitReady(0.1)
        ros.ERROR('joint states not ready')
    end
end

function InfoServices:spin()
    if not service_queue:isEmpty() then
        ros.INFO('[!] incoming service call')
        service_queue:callAvailable()
    end
    if not joint_position_queue:isEmpty() then
        ros.INFO('[!] incoming joint position service call')
        joint_position_queue:callAvailable()
    end
end

function InfoServices:start()
end

function InfoServices:shutdown()
    self.info_server:shutdown()
    self.current_joint_position_info_server:shutdown()
end
