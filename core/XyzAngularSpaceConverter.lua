local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local core = require 'xamlamoveit.core.env'

local XyzAngularSpaceConverter = torch.class('xamlamoveit.core.XyzAngularSpaceConverter', core)

local function tensorToPose(start_pose, goal_pose, xyza)
    assert(xyza:size(1) == 4, 'Vector should be of size 4D (offset, anglevelocities)')
    local start_q = start_pose:getRotation()
    local goal_q = goal_pose:getRotation()
    local angle = start_q:angleShortestPath(goal_q)
    local d_angle = xyza[4]
    local end_pose = tf.StampedTransform()
    end_pose:setOrigin(xyza[{{1, 3}}])
    local fraction = d_angle / angle
    if fraction ~= fraction then
        fraction = 0
    end
    local end_pose_rotation = start_q:slerp(goal_q, fraction)
    end_pose:setRotation(end_pose_rotation)
    return end_pose
end

function XyzAngularSpaceConverter:__init(stamped_stransforms)
    local a = {}
    local s = torch.Tensor(#stamped_stransforms, 4)
    self.s = s
    self.input = stamped_stransforms
    self.base_angle = a

    local angle_sum = 0
    for i = 1, #stamped_stransforms do
        local p1 = stamped_stransforms[i]:toTransform()
        local angle = 0
        if i > 1 then
            local p0 = stamped_stransforms[i - 1]:toTransform()
            local start_q = p0:getRotation()
            local goal_q = p1:getRotation()
            angle = start_q:angleShortestPath(goal_q)
        end
        s[{i, {1, 3}}] = p1:getOrigin()
        angle_sum = angle_sum + angle
        a[#a + 1] = angle_sum
        s[{i, 4}] = angle_sum
    end
    --print('base_angle', a)
    --print('s', s)
end

function XyzAngularSpaceConverter:getXyzAngularPath()
    return self.s
end

function XyzAngularSpaceConverter:samplePose(xyza)
    local a = self.base_angle

    local angle_pos = xyza[4]
    local i = 2
    while i <= #a do
        if a[i] >= (angle_pos + 1e-6) then
            break
        end
        i = i + 1
    end

    -- make angle in xyza relative to 2-pose path segment
    local xyza = xyza:clone()
    xyza[4] = angle_pos - a[i - 1]
    --print('angle_pos', angle_pos, 'xyza', xyza[4])

    local p0 = self.input[i - 1]:toTransform()
    local p1 = self.input[math.min(i, #self.input)]:toTransform()

    local result_stamped_transform = tensorToPose(p0, p1, xyza)
    return result_stamped_transform
end
