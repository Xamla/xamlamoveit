local datatypes = require 'xamlamoveit.components.datatypes.env'
local Pose = torch.class('xamlamoveit.components.datatypes.Pose',datatypes)
function Pose:__init()
  self.frame = "base_link"
  self.translation = torch.zeros(3)
  self.rotation = torch.zeros(4) -- x y z w
end

function Pose:setTranslation(trans)
  self.translation:copy(trans)
end

function Pose:setRotation(quaternion)
  self.rotation:copy(quaternion)
end

function Pose:setFrame(name)
    self.frame = name
end

function Pose:copy(other)
  self.frame = other.frame
  self.translation:copy(other.translation)
  self.rotation:copy(other.rotation)
end

return Pose
