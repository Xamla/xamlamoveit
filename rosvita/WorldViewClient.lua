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

local UPDATE_CARTESIANPATH_SERVER_ADDRESS = '/rosvita/world_view/update_cartesianpath'
local SET_CARTESIANPATH_SERVER_ADDRESS = '/rosvita/world_view/set_cartesianpath'
local GET_CARTESIANPATH_SERVER_ADDRESS = '/rosvita/world_view/get_cartesianpath'
local QUERY_CARTESIANPATH_SERVER_ADDRESS = '/rosvita/world_view/query_cartesianpath'

local SET_COLLISIONOBJECT_SERVER_ADDRESS = '/rosvita/world_view/set_collisionobject'
local GET_COLLISIONOBJECT_SERVER_ADDRESS = '/rosvita/world_view/get_collisionobject'
local UPDATE_COLLISIONOBJECT_SERVER_ADDRESS = '/rosvita/world_view/update_collisionobject'
local QUERY_COLLISIONOBJECT_SERVER_ADDRESS = '/rosvita/world_view/query_collisionobject'

local ros = require 'ros'
local datatypes = require 'xamlamoveit.datatypes'
local CollisionPrimitive = datatypes.CollisionPrimitive
local CollisionPrimitiveKind = CollisionPrimitive.CollisionPrimitiveKind
local CollisionObject = datatypes.CollisionObject
local rosvita = require 'xamlamoveit.rosvita.env'

local joint_values_point_spec = ros.MsgSpec('xamlamoveit_msgs/JointValuesPoint')
local cartesian_path_spec = ros.MsgSpec('xamlamoveit_msgs/CartesianPath')

local solidprimitive_spec = ros.MsgSpec('shape_msgs/SolidPrimitive')
local collisionobject_spec = ros.MsgSpec('moveit_msgs/CollisionObject')
local objecttype_spec = ros.MsgSpec('object_recognition_msgs/ObjectType')

local plane_spec = ros.MsgSpec('shape_msgs/Plane')
local header_spec = ros.MsgSpec('std_msgs/Header')
local pose_spec = ros.MsgSpec('geometry_msgs/Pose')

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
local query_cartesianpath_spec = ros.SrvSpec('xamlamoveit_msgs/QueryCartesianPathWorldView')
local set_cartesianpath_spec = ros.SrvSpec('xamlamoveit_msgs/SetCartesianPathWorldView')
local get_cartesianpath_spec = ros.SrvSpec('xamlamoveit_msgs/GetCartesianPathWorldView')

local set_collisionobject_spec = ros.SrvSpec('xamlamoveit_msgs/SetCollisionObjectWorldView')
local get_collisionobject_spec = ros.SrvSpec('xamlamoveit_msgs/GetCollisionObjectWorldView')
local query_collisionobject_spec = ros.SrvSpec('xamlamoveit_msgs/QueryCollisionObjectWorldView')

local function poseToPoseMsg(pose)
    local msg = ros.Message(pose_spec)
    local trans = pose:getTranslation()
    local ori = pose:getRotation()
    msg.position.x = trans[1]
    msg.position.y = trans[2]
    msg.position.z = trans[3]
    msg.orientation.x = ori[1]
    msg.orientation.y = ori[2]
    msg.orientation.z = ori[3]
    msg.orientation.w = ori[4]
    return msg
end

local function poseFromPoseMsg(msg)
    local result = datatypes.Pose()
    local trans = torch.DoubleTensor({msg.position.x, msg.position.y, msg.position.z})
    local rot =
        torch.DoubleTensor {
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    }

    result:setTranslation(trans)
    result:setRotation(rot)
    return result
end

local function poseFromPoseStampedMsg(msg)
    print(msg)
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
    result:setFrame(frame_id)
    return result
end

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

    self.update_cartesianpath =
        self.node_handle:serviceClient(UPDATE_CARTESIANPATH_SERVER_ADDRESS, set_cartesianpath_spec)
    self.set_cartesianpath = self.node_handle:serviceClient(SET_CARTESIANPATH_SERVER_ADDRESS, set_cartesianpath_spec)
    self.get_cartesianpath = self.node_handle:serviceClient(GET_CARTESIANPATH_SERVER_ADDRESS, get_cartesianpath_spec)
    self.query_cartesianpaths =
        self.node_handle:serviceClient(QUERY_CARTESIANPATH_SERVER_ADDRESS, query_cartesianpath_spec)

    self.set_collisionobject =
        self.node_handle:serviceClient(SET_COLLISIONOBJECT_SERVER_ADDRESS, set_collisionobject_spec)
    self.get_collisionobject =
        self.node_handle:serviceClient(GET_COLLISIONOBJECT_SERVER_ADDRESS, get_collisionobject_spec)
    self.update_collisionobject =
        self.node_handle:serviceClient(UPDATE_COLLISIONOBJECT_SERVER_ADDRESS, set_collisionobject_spec)
    self.query_collisionobject =
        self.node_handle:serviceClient(QUERY_COLLISIONOBJECT_SERVER_ADDRESS, query_collisionobject_spec)
end

local function ToCollisionPlaneMessage(primitive)
    assert(primitive:getKind() == CollisionPrimitiveKind.Plane, 'wrong Type')
    local plane = ros.Message(plane_spec)
    plane.coef = primitive:getParameters()
    return plane
end

local function ToCollisionPrimitiveMessage(primitive)
    local kind, info = primitive:getKind()
    local solidPrimitive = ros.Message(solidprimitive_spec)
    assert(kind ~= CollisionPrimitiveKind.Plane)

    solidPrimitive.type = kind

    solidPrimitive.dimensions:set(primitive:getParameters())
    return solidPrimitive
end

local function ToCollisionObjectMessage(collision_object)
    local collision_object_message = ros.Message(collisionobject_spec)
    local frame_id = collision_object:getFrame()
    local primitive_poses = {}
    local plane_poses = {}
    local plane_messages = {}
    local primitive_messages = {}

    for i, obj in ipairs(collision_object:getPrimitives()) do
        if obj:getKind() == CollisionPrimitiveKind.Plane then
            plane_messages[#plane_messages + 1] = ToCollisionPlaneMessage(obj)
            plane_poses[#plane_poses + 1] = obj:getPose():toStampedPoseMsg()
        else
            primitive_messages[#primitive_messages + 1] = ToCollisionPrimitiveMessage(obj)
            primitive_poses[#primitive_poses + 1] = poseToPoseMsg(obj:getPose())
        end
    end

    collision_object_message.type = ros.Message(objecttype_spec)
    collision_object_message.primitives = primitive_messages
    collision_object_message.primitive_poses = primitive_poses
    collision_object_message.planes = plane_messages
    collision_object_message.plane_poses = plane_poses
    collision_object_message.header = ros.Message(header_spec)
    collision_object_message.header.frame_id = frame_id

    collision_object_message.meshes = {}
    collision_object_message.mesh_poses = {}
    return collision_object_message
end

local function ToCollisionPrimitive(primitive, pose)
    if primitive.type == nil then
        assert(primitive.coef ~= nil)
        return CollisionPrimitive.CreatePlane(
            primitive.coef[1],
            primitive.coef[2],
            primitive.coef[3],
            primitive.coef[4],
            pose
        )
    end
    if primitive.type == CollisionPrimitiveKind.Box then
        return CollisionPrimitive.CreateBox(
            primitive.dimensions[1],
            primitive.dimensions[2],
            primitive.dimensions[3],
            pose
        )
    end

    if primitive.type == CollisionPrimitiveKind.Sphere then
        return CollisionPrimitive.CreateSphere(primitive.dimensions[0], pose)
    end

    if primitive.type == CollisionPrimitiveKind.Cylinder then
        return CollisionPrimitive.CreateCylinder(primitive.dimensions[0], primitive.dimensions[1], pose)
    end

    if primitive.type == CollisionPrimitiveKind.Cone then
        return CollisionPrimitive.CreateCone(primitive.dimensions[0], primitive.dimensions[1], pose)
    end

    error('Do not know collision primitive type')
end

local function ToCollisionObject(collision_object)
    local object_name = collision_object.id or ''
    local frame_id = collision_object.header.frame_id or ''
    local primitives = {}
    for i, plane in ipairs(collision_object.planes) do
        primitives[#primitives + 1] = ToCollisionPrimitive(plane, poseFromPoseMsg(collision_object.plane_poses[i]))
    end

    for i, prim in ipairs(collision_object.primitives) do
        primitives[#primitives + 1] =
            ToCollisionPrimitive(collision_object.primitives[i], poseFromPoseMsg(collision_object.primitive_poses[i]))
    end
    return CollisionObject(frame_id, primitives)
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
                    result[#result + 1] = {
                        name = responds.names[i],
                        element_path = responds.element_paths[i],
                        value = poseFromPoseStampedMsg(v)
                    }
                end
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:queryCartesianPath(prefix, folder_path, recursive)
    assert(torch.type(prefix) == 'string', 'element_path should be a string')
    folder_path = folder_path or ''
    recursive = recursive or false
    local request = self.query_cartesianpaths:createRequest()
    request.prefix = prefix
    request.folder_path = folder_path
    request.recursive = recursive

    if self.query_cartesianpaths:isValid() then
        local responds = self.query_cartesianpaths:call(request)
        if responds then
            if responds.success then
                local result = {}
                for i, v in ipairs(responds.paths) do
                    local path = {}
                    for ii, vv in ipairs(v.points) do
                        path[#path + 1] = poseFromPoseStampedMsg(vv)
                    end
                    result[#result + 1] = {
                        name = responds.names[i],
                        element_path = responds.element_paths[i],
                        value = path
                    }
                end
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:getCartesianPath(element_path)
    assert(torch.type(element_path) == 'string', 'element_path should be a string')
    assert(element_path ~= '', 'element_path should not be an empty string')
    local request = self.get_cartesianpath:createRequest()
    request.element_path = element_path
    if self.get_cartesianpath:isValid() then
        local responds = self.get_cartesianpath:call(request)
        if responds then
            if responds.success then
                local result = {}
                for i, v in ipairs(responds.path.points) do
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

function WorldViewClient:addCollisionObject(display_name, element_path, collision_object, transient)
    assert(
        torch.isTypeOf(collision_object, datatypes.CollisionObject),
        'element_path should be a table with datatypes.CollisionObject elements'
    )
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')

    element_path = element_path or ''
    transient = transient or false
    local request = self.set_collisionobject:createRequest()

    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    request.collision_object = ToCollisionObjectMessage(collision_object)
    if self.set_collisionobject:isValid() then
        print(request)
        local responds = self.set_collisionobject:call(request)
        if responds then
            if responds.success then
                return responds.success, responds.error
            else
                return responds.success, responds.error
            end
        end
        print('resp is nil')
    end
    return false, 'service not valid'
end

function WorldViewClient:getCollisionObject(element_path)
    assert(torch.type(element_path) == 'string', 'element_path should be a string')
    assert(element_path ~= '', 'element_path should not be an empty string')
    local request = self.get_collisionobject:createRequest()
    request.element_path = element_path
    if self.get_collisionobject:isValid() then
        local responds = self.get_collisionobject:call(request)
        if responds then
            if responds.success then
                local result = ToCollisionObject(responds.collision_object)
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:updateCollisionObject(display_name, element_path, collision_object, transient)
    assert(
        torch.isTypeOF(collision_object, datatypes.CollisionObject),
        'element_path should be datatypes.CollisionObject'
    )
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')

    element_path = element_path or ''
    transient = transient or false
    local request = self.update_collisionobject:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    --request.path = ros.Message(cartesian_path_spec)
    request.path.collision_object = ToCollisionObjectMessage(collision_object)
    if self.update_collisionobject:isValid() then
        local responds = self.update_collisionobject:call(request)
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

function WorldViewClient:queryCollisionObject(prefix, folder_path, recursive)
    assert(torch.type(prefix) == 'string', 'element_path should be a string')
    folder_path = folder_path or ''
    recursive = recursive or false
    local request = self.query_collisionobject:createRequest()
    request.prefix = prefix
    request.folder_path = folder_path
    request.recursive = recursive

    if self.query_collisionobject:isValid() then
        local responds = self.query_collisionobject:call(request)
        if responds then
            if responds.success then
                local result = {}
                for i, v in ipairs(responds.collision_objects) do
                    result[#result + 1] = {
                        name = responds.names[i],
                        element_path = responds.element_paths[i],
                        value = ToCollisionObject(v)
                    }
                end
                return responds.success, result, responds.error
            else
                return responds.success, nil, responds.error
            end
        end
    end
    return false, nil, 'service not valid'
end

function WorldViewClient:addCartesianPath(display_name, element_path, path, transient)
    assert(torch.type(path) == 'table', 'element_path should be a table with datatypes.Pose elements')
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')

    element_path = element_path or ''
    transient = transient or false
    local request = self.set_cartesianpath:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    local points = {}
    for i, point in ipairs(path) do
        assert(
            torch.isTypeOf(point, datatypes.Pose),
            'Should be xamlamoveit.datatypes.Pose but is:' .. torch.type(point)
        )
        points[#points + 1] = point:toStampedPoseMsg()
    end
    request.path = ros.Message(cartesian_path_spec)
    request.path.points = points

    if self.set_cartesianpath:isValid() then
        local responds = self.set_cartesianpath:call(request)
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

function WorldViewClient:updateCartesianPath(display_name, element_path, path, transient)
    assert(torch.type(path) == 'table', 'element_path should be a table with datatypes.Pose elements')
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')

    element_path = element_path or ''
    transient = transient or false
    local request = self.update_cartesianpath:createRequest()
    request.element_path = element_path
    request.display_name = display_name
    request.transient = transient
    local points = {}
    for i, point in ipairs(path) do
        assert(
            torch.isTypeOf(point, datatypes.Pose),
            'Should be xamlamoveit.datatypes.Pose but is:' .. torch.type(point)
        )
        points[#points + 1] = point:toStampedPoseMsg()
    end
    --request.path = ros.Message(cartesian_path_spec)
    request.path.points = points
    if self.update_cartesianpath:isValid() then
        local responds = self.update_cartesianpath:call(request)
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
                    result[#result + 1] = {
                        name = responds.names[i],
                        element_path = responds.element_paths[i],
                        value = datatypes.JointValues(joint_set, v.positions)
                    }
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

function WorldViewClient:removeCartesianPath(display_name, element_path)
    assert(torch.type(display_name) == 'string', 'element_path should be a string')
    assert(display_name ~= '', 'element_path should not be an empty string')
    local element_path = element_path or ''
    local path = paths.concat('/', element_path, display_name)
    return removeElement(self, path)
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
    disposeRos(self, 'query_joint_postures')

    disposeRos(self, 'get_pose')
    disposeRos(self, 'set_pose')
    disposeRos(self, 'query_poses')
    disposeRos(self, 'update_pose')

    disposeRos(self, 'remove_element')
    disposeRos(self, 'create_folder')

    disposeRos(self, 'set_cartesianpath')
    disposeRos(self, 'get_cartesianpath')
    disposeRos(self, 'query_cartesianpaths')
    disposeRos(self, 'update_cartesianpath')

    disposeRos(self, 'set_collisionobject')
    disposeRos(self, 'get_collisionobject')
    disposeRos(self, 'update_collisionobject')
    disposeRos(self, 'query_collisionobject')

    assert(self.update_pose == nil)
    assert(self.query_joint_postures == nil)
    assert(self.set_pose == nil)
end

return WorldViewClient
