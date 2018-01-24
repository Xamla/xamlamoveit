local ros = require 'ros'
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/QueryJointStateCollisions')

local codes = {
    STATE_VALID = 1,
    STATE_SELF_COLLISION = -1,
    STATE_SCENE_COLLISION = -2,
    INVALID_JOINTS = -11,
    INVALID_MOVE_GROUP = -12
}

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

local function checkJointNames(self, move_group_name, joint_names)
    local ori_joint_names = self.robot_model:getGroupJointNames(move_group_name)
    return table.isSimilar(ori_joint_names, joint_names)
end

local function queryCollisionCheckServiceHandler(self, request, response, header)
    if not checkMoveGroupName(self, request.move_group_name) then
        response.success = false
        response.messages[1] = string.format('Move_group_name: %s is not known', request.move_group_name)
        response.error_codes[1] = codes.INVALID_MOVE_GROUP
        return true
    end

    if not checkJointNames(self, request.move_group_name, request.joint_names) then
        response.success = false
        response.messages[1] =
            string.format('Joint names are not valid for this movegroup: %s', request.move_group_name)
        response.error_codes[1] = codes.INVALID_MOVE_GROUP
        return true
    end

    local robot_state = moveit.RobotState.createFromModel(self.robot_model)
    robot_state:enforceBounds() -- nur auf joint model ebene nicht im state
    robot_state:update()
    response.error_codes:resize(#request.points):zero()
    print(response)
    for i = 1, #request.points do
        if #request.joint_names ~= request.points[i].positions:size(1) then
            response.success = false
            response.messages[i] =
                string.format(
                'Joint names have not the same length as we have joint values: %d vs. %d',
                #request.joint_names,
                request.points[i].positions:size(1)
            )
            response.error_codes[i] = codes.INVALID_JOINTS
            return true
        end

        ros.INFO('set state')
        robot_state:setVariablePositions(request.points[i].positions, request.joint_names)
        ros.INFO('update state')
        robot_state:update()
        print(robot_state:getVariablePositions())
        ros.INFO('sync scene')
        self.plan_scene:syncPlanningScene()
        ros.INFO('checks selfcollision')
        local is_state_colliding = self.plan_scene:isStateColliding(request.move_group_name, robot_state, true)
        ros.INFO('checks scene collision')
        local has_self_collision = self.plan_scene:checkSelfCollision(robot_state)
        ros.INFO('set collision')
        response.in_collision[i] = is_state_colliding or has_self_collision
        if is_state_colliding then
            response.messages[i] = 'joint state is colliding'
            response.error_codes[i] = codes.STATE_SCENE_COLLISION
        elseif has_self_collision then
            response.messages[i] = 'joint state has self collisions'
            ros.INFO('set self collision: ' .. response.messages[i])
            response.error_codes[i] = codes.STATE_SELF_COLLISION
        else
            response.messages[i] = 'State is valid.'
            ros.INFO('No collision: ' .. response.messages[i])
            response.error_codes[i] = codes.STATE_VALID
        end
        response.success = true
        print(response)
    end
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
        ros.DEBUG('[!] incoming JointPositionCollisionCheckService call')
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
