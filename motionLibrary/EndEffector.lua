local ros = require 'ros'
local tf = ros.tf
local datatypes = require 'xamlamoveit.datatypes'
local xutils = require 'xamlamoveit.xutils'
local motionLibrary = require 'xamlamoveit.motionLibrary.env'

local EndEffector = torch.class('EndEffector', motionLibrary)

function EndEffector:__init(move_group, end_effector_name)
    self.move_group = move_group
    self.name = end_effector_name
    self.motion_service = move_group.motion_service
end

local function createTransformFromPoseMsg(msg)
    local T = tf.Transform()
    T:setOrigin({ msg.position.x, msg.position.y, msg.position.z })
    T:setRotation(tf.Quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
    return T
end

function EndEffector:computePose(joint_values)
    assert(joint_values ~= nil, 'Argument `joint_values` must not be nil.')
    assert(torch.isTypeOf(joint_values, datatypes.JointValues), 'Invalid argument `joint_values`: JointValues object expected.')
    local error_code, solution, error_msgs = self.motion_service:queryPose(self.move_group.name, joint_values, link_name)
    if error_code.val ~= 1 then
        error(error_msg)
    end

    local T = createTransformFromPoseMsg(solution.pose)
    local stamp = solution.header.stamp
    local frame_id = solution.header.frame_id
    return tf.StampedTransform(T, stamp, frame_id)
end

function EndEffector:getCurrentPose()
     return self:computePose(self.move_group:getCurrentJointValues())
end
