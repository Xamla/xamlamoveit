local datatypes = require 'xamlamoveit.components.datatypes.env'
local tf = require 'ros'.tf
local Pose = torch.class('xamlamoveit.components.datatypes.Pose',datatypes)

function Pose:__init()
  self.transform = tf.StampedTransform()
end

function Pose:setTranslation(trans)
  self.transform:setOrigin(trans)
end

function Pose:setTranslation(trans)
  self.transform:setOrigin(trans)
end

function Pose:setRotation(quaternion)
  self.transform:setRotation(quaternion)
end

function Pose:setFrame(name)
  self.transform:set_frame_id(name)
end

function Pose:copy(other)
  self.transform:copy(other)
end

function Pose:clone()
  local result = Pose.new()
  result:copy(self)
  return result
end

function Pose:toStampedPoseMsg()
  return self.transform:toStampedPoseMsg()
end

function Pose.__mul(a, b)
  local result = a:clone()
  result.transform = a.transform * b.transform
  return result
end

function Pose:__tostring()
  local res = 'Pose:'
  res = string.format('%s\n %s', tostring(self.transform:toTensor()))
  return res
end

return Pose
