local datatypes = require 'xamlamoveit.components.datatypes.env'
local tf = require 'ros'.tf
local Pose = torch.class('xamlamoveit.components.datatypes.Pose',datatypes)

function Pose:__init()
  self.stampedTransform = tf.StampedTransform()
end

function Pose:setTranslation(trans)
  self.stampedTransform:setOrigin(trans)
end

function Pose:setTranslation(trans)
  self.stampedTransform:setOrigin(trans)
end

function Pose:setRotation(quaternion)
  self.stampedTransform:setRotation(quaternion)
end

function Pose:setFrame(name)
  self.stampedTransform:set_frame_id(name)
end

function Pose:copy(other)
  self.stampedTransform = other.stampedTransform:clone()
end

function Pose:clone()
  local result = Pose.new()
  result:copy(self)
  return result
end

function Pose:toStampedPoseMsg()
  return self.stampedTransform:toStampedPoseMsg()
end

function Pose.__mul(a, b)
  local result = a:clone()
  result.stampedTransform:setData(a.stampedTransform:toTransform():mul(b.stampedTransform:toTransform()))
  return result
end

function Pose:__tostring()
  local res = 'Pose:'
  res = string.format('%s\n %s',res, tostring(self.stampedTransform:toTensor()))
  return res
end

return Pose
