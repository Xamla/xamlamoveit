local ros = require 'ros'
local moveit = require 'moveit'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryJointStateCollisions')

local function checkMoveGroupName(self, name)
    local all_group_joint_names = self.robot_model:getJointModelGroupNames()
    ros.INFO('available move_groups:\n%s', tostring(all_group_joint_names))
    for k, v in pairs(all_group_joint_names) do
        if name == v then
            return true
        end
    end
    ros.ERROR('could not find move_group: ' .. name)
    return false
end

local function isSubset(A, B)
    for ia, a in ipairs(A) do
        if table.indexof(B, a) == -1 then
            return false
        end
    end
    return true
end

local function isSimilar(A, B)
    if #A == #B then
        return isSubset(A, B)
    else
        return false
    end
end

local function checkJointNames(self, move_group_name, joint_names)
    local ori_joint_names = self.robot_model:getGroupJointNames(move_group_name)
    return isSimilar(ori_joint_names, joint_names)
end

local function queryCollisionCheckServiceHandler(self, request, response, header)
    if not checkMoveGroupName(self, request.move_group_name) then
        response.success = false
        response.message = string.format('Move_group_name: %s is not known', request.move_group_name)
        return true
    end

    if not checkJointNames(self, request.move_group_name, request.joint_names) then
        response.success = false
        response.message = string.format('Joint names are not valid for this movegroup: %s', request.move_group_name)
        return true
    end

    if #request.joint_names ~= request.positions:size(1) then
        response.success = false
        response.message =
            string.format(
            'Joint names have not the same length as we have joint values: %d vs. %d',
            #request.joint_names,
            request.positions:size(1)
        )
        return true
    end

    local robot_state = moveit.RobotState.createFromModel(self.robot_model)
    robot_state:setVariablePositions(request.positions, request.joint_names)
    robot_state:update()
    self.plan_scene:syncPlanningScene()
    local is_state_colliding = self.plan_scene:isStateColliding(request.move_group_name, robot_state, true)
    local has_self_collision = self.plan_scene:checkSelfCollision(robot_state)
    response.in_collision = is_state_colliding or has_self_collision
    if is_state_colliding then
        response.message = 'joint state is colliding with environment'
    elseif has_self_collision then
        response.message = 'joint state has self collisions'
    else
        response.message = 'State is valid.'
    end

    response.success = true
    return true
end

local components = require 'xamlamoveit.components.env'
local JointPositionCollisionCheckService,
    parent =
    torch.class(
    'xamlamoveit.components.JointPositionCollisionCheckService',
    'xamlamoveit.components.RosComponent',
    components
)

function JointPositionCollisionCheckService:__init(node_handle)
    self.node_handle = node_handle
    self.callback_queue = ros.CallbackQueue()
    self.robot_model = nil
    self.robot_model_loader = nil
    self.robot_state = nil
    self.info_server = nil
    parent.__init(self, node_handle)
end

function JointPositionCollisionCheckService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.plan_scene = moveit.PlanningScene(self.robot_model)
    self.plan_scene:syncPlanningScene()
end

function JointPositionCollisionCheckService:onStart()
    self.info_server =
        self.node_handle:advertiseService(
        'query_joint_position_collision_check',
        srv_spec,
        function(request, response, header)
            return queryCollisionCheckServiceHandler(self, request, response, header)
        end,
        self.callback_queue
    )
end

function JointPositionCollisionCheckService:onProcess()
    if not self.callback_queue:isEmpty() then
        ros.INFO('[!] incoming JointPositionCollisionCheckService call')
        self.callback_queue:callAvailable()
    end
end

function JointPositionCollisionCheckService:onStop()
    ros.WARN('JointPositionCollisionCheckService:onStop() NOT IMPLEMENTED')
end

function JointPositionCollisionCheckService:onReset()
    self.info_server:shutdown()
end

function JointPositionCollisionCheckService:onShutdown()
    self.info_server:shutdown()
end

return JointPositionCollisionCheckService
