--[[
WorldViewClient.lua

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
local ADD_JOINTS_SERVER_ADDRESS = '/rosvita/world_view/add_joint_posture'
local GET_JOINTS_SERVER_ADDRESS = '/rosvita/world_view/get_joint_posture'
local QUERY_JOINTS_POSTURES_SERVER_ADDRESS = '/rosvita/world_view/query_joint_postures'
local ADD_POSE_SERVER_ADDRESS = '/rosvita/world_view/add_pose'
local GET_POSE_SERVER_ADDRESS = '/rosvita/world_view/get_pose'
local QUERY_POSES_SERVER_ADDRESS = '/rosvita/world_view/query_poses'
local UPDATE_JOINTS_SERVER_ADDRESS = '/rosvita/world_view/update_joint_posture'
local UPDATE_POSE_SERVER_ADDRESS = '/rosvita/world_view/update_pose'
local REMOVE_SERVER_ADDRESS = '/rosvita/world_view/remove_element'
local ADD_FOLDER_SERVER_ADDRESS = '/rosvita/world_view/add_folder'

local ros = require 'ros'
local datatypes = require 'xamlamoveit.datatypes'
local rosvita = require 'xamlamoveit.rosvita.env'

local joint_values_point_spec = ros.MsgSpec('xamlamoveit_msgs/JointValuesPoint')

local get_joint_posture_spec = ros.SrvSpec('xamlamoveit_msgs/GetJointPostureWorldView')
local set_joint_posture_spec = ros.SrvSpec('xamlamoveit_msgs/SetJointPostureWorldView')
local update_joint_posture_spec = ros.SrvSpec('xamlamoveit_msgs/UpdateJointPostureWorldView')
local get_pose_spec = ros.SrvSpec('xamlamoveit_msgs/GetPoseWorldView')
local set_pose_spec = ros.SrvSpec('xamlamoveit_msgs/SetPoseWorldView')
local update_pose_spec = ros.SrvSpec('xamlamoveit_msgs/UpdatePoseWorldView')
local remove_element_spec = ros.SrvSpec('xamlamoveit_msgs/RemoveElementWorldView')
local create_folder_spec = ros.SrvSpec('xamlamoveit_msgs/CreateFolderWorldView')
local query_poses_spec = ros.SrvSpec('xamlamoveit_msgs/QueryPosesWorldView')
local query_joint_postures_spec = ros.SrvSpec('xamlamoveit_msgs/QueryJointValuesWorldView')

local WorldViewClient = torch.class('xamlamoveit.rosvita.WorldViewClient', rosvita)

function WorldViewClient:__init(node_handle)
    self.node_handle = node_handle
    self.set_joint_posture = self.node_handle:serviceClient(ADD_JOINTS_SERVER_ADDRESS, set_joint_posture_spec)
    self.get_joint_posture = self.node_handle:serviceClient(GET_JOINTS_SERVER_ADDRESS, get_joint_posture_spec)
    self.query_joint_postures =
        self.node_handle:serviceClient(QUERY_JOINTS_POSTURES_SERVER_ADDRESS, query_joint_postures_spec)
    self.set_pose = self.node_handle:serviceClient(ADD_POSE_SERVER_ADDRESS, set_pose_spec)
    self.get_pose = self.node_handle:serviceClient(GET_POSE_SERVER_ADDRESS, get_pose_spec)
    self.query_poses = self.node_handle:serviceClient(QUERY_POSES_SERVER_ADDRESS, query_poses_spec)
    self.update_joint_posture = self.node_handle:serviceClient(UPDATE_JOINTS_SERVER_ADDRESS, update_joint_posture_spec)
    self.update_pose = self.node_handle:serviceClient(UPDATE_POSE_SERVER_ADDRESS, update_pose_spec)
    self.remove_element = self.node_handle:serviceClient(REMOVE_SERVER_ADDRESS, remove_element_spec)
    self.create_folder = self.node_handle:serviceClient(ADD_FOLDER_SERVER_ADDRESS, create_folder_spec)
end

function WorldViewClient:getJointPosture(element_path)
    assert(torch.type(element_path) == 'string', 'element_path should be a string')
    assert(element_path ~= '', 'element_path should not be an empty string')
    local request = self.get_joint_posture:createRequest()
    request.element_path = element_path
    if self.get_joint_posture:isValid() then
        local responds = self.get_joint_posture:call(request)
        if responds then
            if responds.success then
                local joint_set = datatypes.JointSet(responds.point.joint_names)
                local result = datatypes.JointValues(joint_set, responds.point.positions)
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:addJointValues(display_name, element_path, point, transient)
    assert(
        torch.isTypeOf(point, datatypes.JointValues),
        'Should be xamlamoveit.datatypes.JointValues but is:' .. torch.type(point)
    )
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    element_path = element_path or ''
    transient = transient or false
    local request = self.set_joint_posture:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    request.point = ros.Message(joint_values_point_spec)
    request.point.positions = point:getValues()
    request.point.joint_names = point:getNames()

    if self.set_joint_posture:isValid() then
        local responds = self.set_joint_posture:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
    end
    return false, 'service not valid'
end

local function poseFromPoseStampedMsg(msg)
    local result = datatypes.Pose()
    local trans = torch.DoubleTensor({msg.pose.position.x, msg.pose.position.y, msg.pose.position.z})
    local rot =
        torch.DoubleTensor {
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    }

    local frame_id = msg.header.frame_id
    result:setTranslation(trans)
    result:setRotation(rot)
    result:setFrame(rot)
    return result
end

function WorldViewClient:getPose(element_path)
    assert(torch.type(element_path) == 'string', 'element_path should be a string')
    assert(element_path ~= '', 'element_path should not be an empty string')
    local request = self.get_pose:createRequest()
    request.element_path = element_path
    if self.get_pose:isValid() then
        local responds = self.get_pose:call(request)
        if responds then
            if responds.success then
                local result = poseFromPoseStampedMsg(responds.point)
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:addPose(display_name, element_path, point, transient)
    assert(torch.isTypeOf(point, datatypes.Pose), 'Should be xamlamoveit.datatypes.Pose but is:' .. torch.type(point))
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    element_path = element_path or ''
    transient = transient or false
    local request = self.set_pose:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    request.point = point:toStampedPoseMsg()

    if self.set_pose:isValid() then
        local responds = self.set_pose:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
    end
    return false, 'service not valid'
end

function WorldViewClient:updatePose(display_name, element_path, point, transient)
    assert(torch.isTypeOf(point, datatypes.Pose), 'Should be xamlamoveit.datatypes.Pose but is:' .. torch.type(point))
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    element_path = element_path or ''
    transient = transient or false
    local request = self.update_pose:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    request.point = point:toStampedPoseMsg()

    if self.update_pose:isValid() then
        local responds = self.update_pose:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
    end
    return false, 'service not valid'
end

function WorldViewClient:queryPoses(prefix, folder_path, recursive)
    assert(torch.type(prefix) == 'string', 'element_path should be a string')
    folder_path = folder_path or ''
    recursive = recursive or false
    local request = self.query_poses:createRequest()
    request.prefix = prefix
    request.folder_path = folder_path
    request.recursive = recursive

    if self.query_poses:isValid() then
        local responds = self.query_poses:call(request)
        if responds then
            if responds.success then
                local result = {}
                for i, v in ipairs(responds.points) do
                    result[#result + 1] = poseFromPoseStampedMsg(v)
                end
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:queryJointValues(prefix, folder_path, recursive)
    assert(torch.type(prefix) == 'string', 'element_path should be a string')
    folder_path = folder_path or ''
    recursive = recursive or false
    local request = self.query_joint_postures:createRequest()
    request.prefix = prefix
    request.folder_path = folder_path
    request.recursive = recursive

    if self.query_joint_postures:isValid() then
        local responds = self.query_joint_postures:call(request)
        if responds then
            if responds.success then
                local result = {}
                for i, v in ipairs(responds.points) do
                    local joint_set = datatypes.JointSet(v.joint_names)
                    result[#result + 1] = datatypes.JointValues(joint_set, v.positions)
                end
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:updateJointValues(display_name, element_path, point, transient)
    assert(
        torch.isTypeOf(point, datatypes.JointValues),
        'Should be xamlamoveit.datatypes.JointValues but is:' .. torch.type(point)
    )
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    element_path = element_path or ''
    transient = transient or false
    local request = self.update_joint_posture:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    request.point = ros.Message(joint_values_point_spec)
    request.point.positions = point:getValues()
    request.point.joint_names = point:getNames()

    if self.update_joint_posture:isValid() then
        local responds = self.update_joint_posture:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
    end
    return false, 'service not valid'
end

function WorldViewClient:addFolder(display_name, element_path, transient)
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    local element_path = element_path or ''
    transient = transient or false
    local request = self.create_folder:createRequest()
    request.folder_path = element_path
    request.folder_name = display_name
    request.transient = transient

    if self.create_folder:isValid() then
        local responds = self.create_folder:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
    end
    return false, 'service not valid'
end

local function removeElement(self, element_path)
    assert(torch.type(element_path) == 'string', 'element_path should be a string but is: ' .. torch.type(element_path))
    assert(element_path ~= '', 'element_path should not be an empty string')
    local request = self.remove_element:createRequest()
    request.element_path = element_path
    if self.remove_element:isValid() then
        local responds = self.remove_element:call(request)
        if responds then
            return responds.success, responds.error
        end
    end
    return false, 'service not valid'
end

function WorldViewClient:removePose(display_name, element_path)
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    local element_path = element_path or ''
    local path = paths.concat('/', element_path, display_name)
    return removeElement(self, path)
end

function WorldViewClient:removeFolder(display_name, element_path)
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    local element_path = element_path or ''
    local path = paths.concat('/', element_path, display_name)
    return removeElement(self, path)
end

function WorldViewClient:removeJointValues(display_name, element_path)
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    local element_path = element_path or ''
    local path = paths.concat('/', element_path, display_name)
    return removeElement(self, path)
end

function disposeRos(self, unit)
    if self[unit] then
        self[unit]:shutdown()
        self[unit] = nil
    end
end

function WorldViewClient:shutdown()
    disposeRos(self, 'get_joint_posture')
    disposeRos(self, 'set_joint_posture')
    disposeRos(self, 'update_joint_posture')
    disposeRos(self, 'get_pose')
    disposeRos(self, 'set_pose')

    disposeRos(self, 'update_pose')
    disposeRos(self, 'remove_element')
    disposeRos(self, 'create_folder')
    disposeRos(self, 'query_poses')
    disposeRos(self, 'query_joint_postures')

    assert(self.update_pose == nil)
    assert(self.query_joint_postures == nil)
    assert(self.set_pose == nil)
end

return WorldViewClient
