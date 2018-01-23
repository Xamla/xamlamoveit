ros = require 'ros'
tf = ros.tf

function addDelta(pose, twist, dt)
  local result = tf.Transform()
  result:setOrigin(pose:getOrigin()+ twist[{{1, 3}}])
  local quaternion = result:getRotation()
  if (twist[{{4, 6}}]):norm() > 0 then
    quaternion:setRotation(twist[{{4, 6}}] * dt, (twist[{{4, 6}}] * dt):norm())
    result:setRotation(quaternion)
  end
  return result
end

local twist = torch.Tensor({1, 2, 3, math.pi, math.pi, math.pi})
local A = tf.Transform()

local result = addDelta(A, twist, 1)

print(result)
print(result:getRotation():getAxis(), -twist[{{4, 6}}]/twist[{{4, 6}}]:norm())
print(result:getOrigin(), twist[{{1, 3}}])

