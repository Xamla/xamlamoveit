--[[
PositionStateInfoService.lua

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]
local ros = require 'ros'
local moveit = require 'moveit'
local core = require 'xamlamoveit.core'

local srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetCurrentJointState')
local ik_srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetIKSolution')
local fk_srv_spec = ros.SrvSpec('xamlamoveit_msgs/GetFKSolution')

local errorCodes = require 'xamlamoveit.core.ErrorCodes'.error_codes
errorCodes = table.merge(errorCodes, table.swapKeyValue(errorCodes))

local function createIKRequest(
    group_name,
    robot_state,
    avoid_collisions,
    poses_stamped,
    ik_link_names,
    attempts,
    timeout)
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
        req_msg.timeout = timeout
        req_msg.attempts = attempts
    else
        if #ik_link_names > 0 then
            req_msg.ik_link_name = ik_link_names[1]
        end
        req_msg.pose_stamped = poses_stamped[1]
        req_msg.timeout = timeout
        req_msg.attempts = attempts
    end
    return req_msg
end

local function queryIKServiceHandler(self, request, response, header)
    local r_state = self.robot_state:clone()
    local last_good
    ros.DEBUG(tostring(request))
    if request.seed then
        r_state:setVariablePositions(request.seed.positions, request.joint_names)
        r_state:update()
        last_good = request.seed.positions:clone()
    end

    local known_joint_names = self.robot_model:getGroupJointNames(request.group_name)
    ros.DEBUG('query Group EndEffector Names for: ' .. request.group_name)

    local target_link = request.end_effector_link
    local attempts = request.attempts or 1
    local timeout = request.timeout or ros.Duration(0.1)
    if timeout == ros.Duration(0) then
        timeout = ros.Duration(0.1)
    end
    for point_index = 1, #request.points do
        local ik_req =
            createIKRequest(
            request.group_name,
            r_state,
            request.collision_check,
            {request.points[point_index]},
            {target_link},
            attempts,
            timeout
        )
        local ik_res = self.ik_service_client:call(ik_req)
        response.error_codes[point_index] = ik_res.error_code

        if not r_state:fromRobotStateMsg(ik_res.solution) then
            response.error_codes[point_index] = ros.Message('moveit_msgs/MoveItErrorCodes')
            response.error_codes[point_index].val = errorCodes.INVALID_ROBOT_STATE
        end
        local point_msg = ros.Message('xamlamoveit_msgs/JointPathPoint')
        local state = r_state:getVariablePositions()
        local names = r_state:getVariableNames():totable()
        local res = torch.DoubleTensor(#request.joint_names)
        for i, v in ipairs(request.joint_names) do
            local index = table.indexof(names, v)
            res[i] = state[index]
        end
        if response.error_codes[point_index].val == errorCodes.SUCCESS then
            point_msg.positions = res
            response.solutions[point_index] = point_msg
            last_good:copy(res)
        end
        if request.const_seed == true then
            r_state:setVariablePositions(request.seed.positions, request.joint_names)
            last_good = request.seed.positions:clone()
        else
            r_state:setVariablePositions(last_good, request.joint_names)
        end
    end
    return true
end

local pose_msg_spec = ros.MsgSpec('geometry_msgs/PoseStamped')
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

local function queryFKServiceHandler(self, request, response, header)
    local r_state = self.robot_state:clone()
    local group_name = request.group_name
    local ee_link_name = request.end_effector_link

    if ee_link_name == nil or ee_link_name == '' then
      local ee_names = self.robot_model:getGroupEndEffectorName(group_name)
      if ee_names == '' then
        response.error_codes[1] = ros.Message('moveit_msgs/MoveItErrorCodes')
        response.error_msgs[1] = ''
        response.error_codes[1].val = errorCodes.INVALID_GROUP_NAME
        response.error_msgs[1] = 'MoveGroup name is empty!'
        return true
      end
      ee_link_name = self.robot_model:getEndEffectorLinkName(ee_names)
    end

    for i = 1, #request.points do
        response.error_codes[i] = ros.Message('moveit_msgs/MoveItErrorCodes')
        response.solutions[i] = ros.Message('geometry_msgs/PoseStamped')
        response.error_codes[i].val = 1
        if #request.joint_names == request.points[i].positions:size(1) then
            r_state:setVariablePositions(request.points[i].positions, request.joint_names)
            r_state:update()
            local pose = r_state:getGlobalLinkTransform(ee_link_name)
            if pose then
                local position = pose:getOrigin()
                local quaternion = pose:getRotation():toTensor()
                response.solutions[i] = createPoseMsg('', position, quaternion)
                response.error_codes[i].val = math.min(1, response.error_codes[i].val)
                response.error_msgs[i] = string.format('Found solution for ee_link_name: %s', ee_link_name)
            else
                response.error_codes[i].val = errorCodes.INVALID_LINK_NAME
                response.error_msgs[i].data = string.format('INVALID_LINK_NAME: %s', ee_link_name)
            end
        else
            response.error_codes[i].val = errorCodes.INVALID_ROBOT_STATE
            response.error_msgs[i] =
                string.format(
                'Number of joint names and vector size do not match: %d vs. %d',
                #request.joint_names,
                request.points[i].positions:size(1)
            )
        end
    end
    return true
end

local function queryJointPositionServiceHandler(self, request, response, header)
    local joint_names = request.joint_names
    local wait_duration = ros.Duration(0.1)
    local ok, joints = self.joint_monitor:getNextPositionsTensor(wait_duration, joint_names)
    --local joints = self.robot_state:getVariablePositions(joint_names):clone()
    response.current_joint_position.header.stamp = ros.Time.now()
    response.current_joint_position.name = joint_names
    response.current_joint_position.position = joints
    return joints:size(1) == #joint_names and ok
end

local components = require 'xamlamoveit.components.env'
local PositionStateInfoService,
    parent =
    torch.class('xamlamoveit.components.PositionStateInfoService', 'xamlamoveit.components.RosComponent', components)

function PositionStateInfoService:__init(node_handle, joint_monitor)
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
    self.joint_monitor = joint_monitor
    parent.__init(self, node_handle)
end

function PositionStateInfoService:onInitialize()
    self.robot_model_loader = moveit.RobotModelLoader('robot_description')
    self.robot_model = self.robot_model_loader:getModel()
    self.planning_scene = moveit.PlanningScene(self.robot_model)
    self.planning_scene:syncPlanningScene()
    self.robot_state = moveit.RobotState.createFromModel(self.robot_model)

    local ready = self.joint_monitor:waitReady(2.0) -- it is not important to have the joint monitor ready at start up
    if not ready then
        ros.WARN('joint states not ready')
    else
        local ok, p = self.joint_monitor:getNextPositionsTensor()
        self.robot_state:setVariablePositions(
            p,
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
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
    local state_info_call = not self.callback_queue:isEmpty()
    local ik_info_call = not self.ik_callback_queue:isEmpty()
    local fk_info_call = not self.fk_callback_queue:isEmpty()
    if not (state_info_call or ik_info_call or fk_info_call) then
        return
    end
    if self.joint_monitor:isReady() then
        local joints, latency = self.joint_monitor:getPositionsTensor()
        --assert(ok, 'exceeded timeout for next robot joint state.')
        self.robot_state:setVariablePositions(
            joints,
            self.joint_monitor:getJointNames()
        )
        self.robot_state:update()
    end
    if state_info_call then
        ros.INFO('[!] incoming PositionStateInfoService call')
        self.callback_queue:callAvailable()
    end

    if ik_info_call then
        ros.INFO('[!] incoming ComputeIKService call')
        self.ik_callback_queue:callAvailable()
    end

    if fk_info_call then
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
