local ros = require 'ros'
local moveit = require 'moveit'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryMoveGroupInterfaces')
local msg_spec = ros.MsgSpec('xamlamoveit_msgs/MoveGroupInterfaceDescription')

local function queryServiceHandler(self, request, response, header)
    print('print(all_EE_parent_link_names)')
    print(self.all_EE_parent_link_names)

    for k, v in pairs(self.all_group_joint_names) do
        local l = ros.Message(msg_spec)
        l.name = v
        l.sub_move_group_ids = self.robot_model:getJointModelSubGroupNames(v)
        l.joint_names = self.robot_model:getGroupJointNames(v)
        l.end_effector_names = self.robot_model:getGroupEndEffectorNames(v)
        table.insert(response.move_group_interfaces, l)
    end
    return true
end

local components = require 'xamlamoveit.components.env'
local MoveGroupInfoNodeService,
    parent = torch.class('MoveGroupInfoNodeService', 'xamlamoveit.components.RosComponent', components)

function MoveGroupInfoNodeService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model_loader = nil
    self.robot_model = nil
    self.all_EE_parent_group_names = {}
    self.all_EE_parent_link_names = {}
    self.all_group_joint_names = {}
    parent.__init(self, node_handle)
end

function MoveGroupInfoNodeService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.all_EE_parent_group_names, self.all_EE_parent_link_names = self.robot_model:getEndEffectorParentGroups()
    self.all_group_joint_names = self.robot_model:getJointModelGroupNames()
end

function MoveGroupInfoNodeService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_move_group_interface',
        srv_spec,
        function(request, response, header)
            return queryServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function MoveGroupInfoNodeService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming service call')
        self.callback_queue:callAvailable()
    end
end

function MoveGroupInfoNodeService:onStop()
end

function MoveGroupInfoNodeService:onShutdown()
    self.info_server:shutdown()
end

return MoveGroupInfoNodeService
