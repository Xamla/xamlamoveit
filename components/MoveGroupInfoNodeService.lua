local ros = require "ros"
local moveit = require "moveit"

local srv_spec = ros.SrvSpec("xamlamoveit_msgs/QueryMoveGroupInterfaces")
local msg_spec = ros.MsgSpec("xamlamoveit_msgs/MoveGroupInterfaceDescription")

local function queryServiceHandler(self, request, response, header)
    for k, v in pairs(self.all_group_joint_names) do
        local l = ros.Message(msg_spec)
        l.name = v
        l.sub_move_group_ids = self.robot_model:getJointModelSubGroupNames(v)
        l.joint_names = self.robot_model:getGroupJointNames(v)
        local test = {} --self.robot_model:getAttachedEndEffectorNames(v)
        l.end_effector_names = {}
        local link_name = self.robot_model:getEndEffectorLinkName(v)
        if #link_name > 0 then
            table.insert(l.end_effector_names, link_name)
        end
        if #test > 0 then
            for ii, vv in pairs(test) do
                link_name = vv
                if #link_name > 0 then
                    local index = table.indexof(l.end_effector_names, link_name)
                    if index < 0 then
                        table.insert(l.end_effector_names, link_name)
                    end
                end
            end
        end
        table.insert(response.move_group_interfaces, l)
    end
    return true
end

local components = require "xamlamoveit.components.env"
local MoveGroupInfoNodeService,
    parent = torch.class("MoveGroupInfoNodeService", "xamlamoveit.components.RosComponent", components)

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

local function setLimits2ParmeterServer(self, pos_lim, vel_lim, acc_lim)
    local root_path = "robot_description_planning/joint_limits"
    local nh = self.node_handle
    for j, name in ipairs(self.variable_names) do
        local has_pos_param = string.format("/%s/%s/has_position_limits", root_path, name)
        local get_max_pos_param = string.format("/%s/%s/max_position", root_path, name)
        local get_min_pos_param = string.format("/%s/%s/min_position", root_path, name)
        local has_vel_param = string.format("/%s/%s/has_velocity_limits", root_path, name)
        local get_vel_param = string.format("/%s/%s/max_velocity", root_path, name)
        local has_acc_param = string.format("/%s/%s/has_acceleration_limits", root_path, name)
        local get_acc_param = string.format("/%s/%s/max_acceleration", root_path, name)
        local i = self.robot_model:getVariableIndex(name) --+ 1
        local param_has_pos = nh:getParamVariable(has_pos_param)
        if param_has_pos == nil or param_has_pos == false then
            nh:setParamBool(has_pos_param, true)
            ros.WARN("Joint: %s has no position limit", name)
            nh:setParamDouble(get_max_pos_param, pos_lim[i][1])
            nh:setParamDouble(get_min_pos_param, pos_lim[i][2])
        else
            ros.INFO("Joint: %s has position limit", name)
        end

        local param_has_vel = nh:getParamVariable(has_vel_param)
        if param_has_vel == nil or param_has_vel == false then
            local value = torch.min(torch.abs(vel_lim[{i, {}}]))
            if value < math.huge then
                nh:setParamBool(has_vel_param, true)
                ros.WARN("Joint: %s has no velocity limit", name)
                nh:setParamDouble(get_vel_param, torch.min(torch.abs(vel_lim[{i, {}}])))
            end
        else
            ros.INFO("Joint: %s has velocity limit", name)
        end

        local param_has_acc = nh:getParamVariable(has_acc_param)
        if param_has_acc == nil or param_has_acc == false then
            local value = torch.min(torch.abs(acc_lim[{i, {}}]))
            if value == math.huge and nh:getParamVariable(has_vel_param) == true then
                value = nh:getParamDouble(get_vel_param) * 0.5
                ros.WARN("override acc limit")
            else
                ros.INFO("no override", value, math.huge, param_has_vel)
            end

            if value < math.huge then
                nh:setParamBool(has_acc_param, true)
                ros.WARN("Joint: %s has no acceleration limit", name)
                nh:setParamDouble(get_acc_param, value)
            end
        else
            ros.INFO("Joint: %s has acceleration limit", name)
        end
    end
end

function MoveGroupInfoNodeService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader("robot_description")
    self.robot_model = self.robot_model_loader:getModel()
    self.all_EE_parent_group_names, self.all_EE_parent_link_names = self.robot_model:getEndEffectorParentGroups()
    self.all_group_joint_names = self.robot_model:getJointModelGroupNames()
    local pos_lim, vel_lim, acc_lim = self.robot_model:getVariableBounds()
    self.variable_names = self.robot_model:getActiveJointNames()
    setLimits2ParmeterServer(self, pos_lim, vel_lim, acc_lim)
end

function MoveGroupInfoNodeService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        "query_move_group_interface",
        srv_spec,
        function(request, response, header)
            return queryServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function MoveGroupInfoNodeService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.DEBUG("[!] incoming service call")
        self.callback_queue:callAvailable()
    end
end

function MoveGroupInfoNodeService:onStop()
end

function MoveGroupInfoNodeService:onShutdown()
    self.info_server:shutdown()
end

return MoveGroupInfoNodeService
