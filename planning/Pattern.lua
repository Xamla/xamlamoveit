local ros = require 'ros'
local tf = ros.tf
local planning = require 'xamlamoveit.Planning.env'


function planning.gridFive(cp, radius)
    local radius = radius or 0.1
    assert(torch.isTypeOf(cp,'tf.Transform'), 'Should be a tf.Transform but is a ' .. torch.type(cp))
    assert(torch.type(radius) == 'number', 'Should be a number but is a ' .. torch.type(radius))
    local cp = cp:clone()
    local transform = tf.Transform()
    local pattern = {center = cp:clone(),
                     right = cp:clone():mul(transform:setOrigin(torch.Tensor({radius,0,0}))),
                     buttom = cp:clone():mul(transform:setOrigin(torch.Tensor({0,-radius,0}))),
                     left = cp:clone():mul(transform:setOrigin(torch.Tensor({-radius,0,0}))),
                     top = cp:clone():mul(transform:setOrigin(torch.Tensor({0,radius,0})))}
    return pattern
end


--- Constructs a plane from a collection of points
-- so that the summed squared distance to all points is minimzized
function planning.planeFromPoints(points) 
    local n = #points
    assert(n >= 3, "At least three points required")

    
    local sum = torch.zeros(3)
    for i,p in pairs(points) do
        sum = sum + p
    end
    local centroid = sum/n

    -- Calc full 3x3 covariance matrix, excluding symmetries:
    local xx = 0.0; local xy = 0.0; local xz = 0.0;
    local yy = 0.0; local yz = 0.0; local zz = 0.0;

    for i,p in pairs(points) do
      local r = p - centroid
      xx = xx + r[1] * r[1]
      xy = xy + r[1] * r[2]
      xz = xz + r[1] * r[3]
      yy = yy + r[2] * r[2]
      yz = yz + r[2] * r[3]
      zz = zz + r[3] * r[3]
    end

    local det_x = yy*zz - yz*yz
    local det_y = xx*zz - xz*xz
    local det_z = xx*yy - xy*xy

    local det_max = math.max(det_x, det_y, det_z)
    assert(det_max > 0.0, "The points don't span a plane")

    -- Pick path with best conditioning:
    local dir
    if det_max == det_x then
        local a = (xz*yz - xy*zz) / det_x
        local b = (xy*yz - xz*yy) / det_x
        dir = torch.Tensor({1.0, a, b})
    elseif det_max == det_y then
        local a = (yz*xz - xy*zz) / det_y
        local b = (xy*xz - yz*xx) / det_y
        dir = torch.Tensor({a, 1.0, b})
    else
        local a = (yz*xy - xz*yy) / det_z
        local b = (xz*xy - yz*xx) / det_z
        dir = torch.Tensor({ a, b,1.0})
    end

    return centroid, dir/dir:norm()
end


function planning.alignZOrientation(pose, apprzaxis)

    local zaxis = pose:toTensor()[{3,{1,3}}]
    local rot_angle = math.acos(torch.dot(zaxis, apprzaxis))
    print('Z-diff:' .. rot_angle)
    if rot_angle > 0.001 then

      local rot_axis = torch.cross(apprzaxis, zaxis)

      local q = tf.Quaternion(rot_axis, rot_angle)
      pose:setRotation(pose:getRotation() * q)
    else
      print('Already snapped in!')
    end
    return pose
end