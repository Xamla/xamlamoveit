local pcl = require 'pcl'
local cv = require 'cv'
require 'cv.imgcodecs'
require 'cv.imgproc'
require 'cv.calib3d'


local X3d = {}


function X3d.normalize (v)
  return v / v:norm()
end


function X3d.identity ()
  return torch.eye(4)
end


function X3d.scale (x, y, z)
  if type(x) == 'table' or torch.isTensor(x) then
    x,y,z = x[1],x[2],x[3]
  end
  y = y or x
  z = z or x
  local S = torch.eye(4)
  S[{1,1}] = x
  S[{2,2}] = y
  S[{3,3}] = z
  return S
end


function X3d.translate (x, y, z)
  if type(x) == 'table' or torch.isTensor(x) then
    x,y,z = x[1],x[2],x[3]
  end
  local T = torch.eye(4)
  T[{1,4}] = x
  T[{2,4}] = y
  T[{3,4}] = z
  return T
end


function X3d.rotateAxis (axis, theta, deg)
  if not torch.isTensor(axis) then
    axis = torch.Tensor(axis)
  end
  if deg then
    theta = math.rad(theta)
  end
  local n = axis:norm()
  if n == 0 then
    error('axis must not be the null vector')
  end
  local u = torch.div(axis, n)
  local ct, st = math.cos(theta), math.sin(theta)
  local d = 1-ct
  local R = torch.Tensor({
    {      ct+u[1]*u[1]*d, u[1]*u[2]*d-u[3]*st, u[1]*u[3]*d+u[2]*st, 0 },
    { u[2]*u[1]*d+u[3]*st,      ct+u[2]*u[2]*d, u[2]*u[3]*d-u[1]*st, 0 },
    { u[3]*u[1]*d-u[2]*st, u[3]*u[2]*d+u[1]*st,      ct+u[3]*u[3]*d, 0 },
    { 0, 0, 0, 1 },
  })
  return R
end


-- Generate a perpendicular vector to v. |v| must be > 0.
function X3d.perpendicular (v)
  if v[3] ~= 0 and -v[1] ~= v[2] then
    return torch.Tensor{v[3], v[3], -v[1]-v[2]}
  else
    return torch.Tensor{-v[2]-v[3], v[1], v[1]}
  end
end


-- returns the intersection of two lines in 2D. Line 1 is defined two points (p1 and p2) and
-- Line 2 by (p3 and p4)
function X3d.intersectLines2d (p1, p2, p3, p4)
  local x_nom = (p1[1]*p2[2] - p1[2]*p2[1])*(p3[1]-p4[1]) - (p1[1] - p2[1])*(p3[1]*p4[2]-p3[2]*p4[1])
  local y_nom = (p1[1]*p2[2] - p1[2]*p2[1])*(p3[2]-p4[2]) - (p1[2] - p2[2])*(p3[1]*p4[2]-p3[2]*p4[1])
  local denom = (p1[1] - p2[1])*(p3[2]-p4[2]) - (p1[2]-p2[2])*(p3[1]-p4[1])
  return torch.Tensor({ x_nom, y_nom }) / denom
end



--- Returns the distance of a point to a line defined by two points p1 and p2
-- see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
function X3d.pointLineDistance2d (p1, p2, x)
  local nom = (p2[2] - p1[2]) * x[1] - (p2[1]-p1[1]) * x[2] + p2[1]*p1[2]-p2[2]*p1[1]
  local denom = torch.norm(p2-p1)
  return math.abs(nom / denom)
end


-- find center and radius of circle through three 2D points.
local function circle3p2d (a, b, c)
  local A = 2 * torch.Tensor{
    {a[1], a[2], .5},
    {b[1], b[2], .5},
    {c[1], c[2], .5}
  }
  local B = torch.Tensor{
    torch.dot(a, a),
    torch.dot(b, b),
    torch.dot(c, c)
  }
  local X = torch.gesv(B:view(3, 1), A):view(3)
  local radius = math.sqrt(X[3] + X[1]^2 + X[2]^2)
  local center = X[{{1,2}}]
  return center, radius
end


function X3d.planeFromPoints (a, b, c)
  local normal = X3d.normalize(torch.cross(b-a, c-a))
  local d = -torch.dot(normal, c)         -- ensure dot(normal, p) + d = 0
  local plane_params = torch.Tensor(4)
  plane_params[{{1,3}}] = normal
  plane_params[4] = d
  return plane_params, normal, d
end


-- Find the points of closest proximity of two rays going through points a0,ab and b0,b1.
-- Returns the closest point on ray a0-a1, closet point on ray b0,b1, the point which is closest
-- simultaneously to both rays and the shortest distane betwen the two rays.
-- Note: Current implementation cannot handle parallel lines.
function X3d.minRayProximityPoint (a0, a1, b0, b1)
  local a,b,c = a1-a0,b1-b0,b0-a0   -- directions

  local alpha = (-torch.dot(a, b) * torch.dot(b, c) + torch.dot(a, c) * torch.dot(b, b)) /
    (torch.dot(a, a) * torch.dot(b, b) - torch.dot(a, b) * torch.dot(a, b))
  local d = a0 + a * alpha

  local beta = (torch.dot(a, b) * torch.dot(a, c) - torch.dot(b, c) * torch.dot(a, a)) /
    (torch.dot(a, a) * torch.dot(b, b) - torch.dot(a, b) * torch.dot(a, b))
  local e = b0 + b * beta

  return (e + d)*0.5, e, d, torch.norm(e-d)
end


--- Returns the distance of a point x to a line defined by two points p1 and p2 (all 3D points).
-- see https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
function X3d.pointLineDistance (p1, p2, x)
  return torch.norm(torch.cross(X3d.normalize(p2-p1), x-p1))
end


--- projects vector a onto b
-- see https://en.wikipedia.org/wiki/Vector_projection
function X3d.project(a, b)
  return torch.dot(a, b) / torch.dot(b, b) * b
end


--- Returns vector x projected on the line defined by two points p1 and p2.
function X3d.projectPointOnLine (p1, p2, x)
  local ap = x-p1
  local ab = p2-p1
  return p1 + X3d.project(ap, ab)
end


function X3d.projectOnPlane (p, plane_params)
  local plane_normal = plane_params[{{1,3}}]
  local at = p + plane_normal
  local _,p = X3d.intersectRayPlane(p, at, plane_params)
  return p
end


function X3d.intersectRayPlane (eye, at, plane_params)
  local n = plane_params[{{1,3}}]
  local p0 = n * -plane_params[4]
  local l = at - eye  -- the ray direction
  local d = n * l
  assert(math.abs(d) > 1e-6, "Plane and ray must not be parallel.")
  local t = (p0 - eye) * n / d
  return t, eye + l * t
end


function X3d.computeNormals (cloud, search_radius)
  local ne = pcl.NormalEstimation(cloud.pointType)
  ne:setRadiusSearch(search_radius)
  ne:setInputCloud(cloud)
  return ne:compute()
end


function X3d.estimatePlane (cloud, cloud_normals, distance_threshold, samplesMaxDist)
  local samplesMaxDist = samplesMaxDist or 0.1
  local kd = pcl.KdTree(cloud.pointType)
  kd:setInputCloud(cloud)

  local segmentation = pcl.SACSegmentationFromNormals(cloud.pointType)
  segmentation:setOptimizeCoefficients(true)
  segmentation:setModelType(pcl.SACMODEL.NORMAL_PLANE)
  segmentation:setNormalDistanceWeight(0.03)
  segmentation:setMethodType(pcl.SAC.RANSAC)
  segmentation:setDistanceThreshold(distance_threshold)
  segmentation:setSamplesMaxDist(samplesMaxDist, kd)
  segmentation:setMaxIterations(200)

  segmentation:setInputCloud(cloud)
  segmentation:setInputNormals(cloud_normals)

  local inliers = pcl.Indices()
  local coefficients = torch.FloatTensor()
  segmentation:segment(inliers, coefficients)

  local plane_cloud = pcl.filter.extractIndices(cloud, inliers, nil, false)

  if coefficients[4] < 0 then
    coefficients:mul(-1.0)
  end

  return plane_cloud, coefficients
end


function X3d.rotMatrixToAxisAngle (rotation_3x3)
  local R = rotation_3x3:clone()
  local U,S,V = torch.svd(R)
  R = U * V:t()
  local tr = (torch.trace(R)-1)/2
  local theta = math.acos(tr)
  local out
  if math.sin(theta) >= 1e-12 then
    local vth = 1/(2*math.sin(theta))
    local om1 = torch.DoubleTensor({R[3][2]-R[2][3], R[1][3]-R[3][1], R[2][1]-R[1][2]}):view(3,1):t():clone()
    local om = om1 * vth
    out = om * theta;
  else
    if tr > 0 then       -- case norm(om) == 0
      -- print("Case1")
      -- print(R)
      out = torch.DoubleTensor(3):zero()
    else
      -- print("Case2")
      local sign = torch.DoubleTensor(3, 1)
      sign[1][1] = 1
      sign[{{2,3}, 1}] = (((R[{1,{2,3}}]:ge(0))*2):type('torch.DoubleTensor')) - 1
      out = ((torch.sqrt((torch.diag(R)+1)/2)):cmul(sign)) * theta
    end
  end

  return out
end


--- transform a rotation vector as e.g. provided by solvePnP to a 3x3 rotation matrix using the Rodrigues' rotation formula
-- see e.g. http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues%28InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian%29
function X3d.rotVectorToMat3x3(vec)
  local theta = torch.norm(vec)
  local R
  if theta < 1e-14 then
    R = torch.eye(3)
  else
    local r = vec/theta
    r = torch.squeeze(r)
    local mat = torch.Tensor({{0, -1*r[3], r[2]}, {r[3], 0, -1*r[1]}, {-1*r[2], r[1], 0}})
    r = r:resize(3,1)
    R = torch.eye(3) * math.cos(theta) + (r*r:t()) * (1-math.cos(theta)) + mat*math.sin(theta)
  end
  return R
end


function X3d.generatePose (rot, pos, scale)
  scale = scale or 0.001 -- mm to m
  local pose = torch.eye(4)
  pose[{{1,3},{1,3}}] = X3d.rotVectorToMat3x3(rot)
  pose[{{1,3}, {4}}] = pos * scale
  return pose
end


--- Generate ground truth circle center points of the calibration pattern.
-- Z is set to 0 for all points.
function X3d.generatePatternPoints (pointsX, pointsY, pointDist)
  -- calculates the groundtruth x, y, z positions of the points of the asymmetric circle pattern
  local corners = torch.FloatTensor(pointsX * pointsY, 1, 3):zero()
  local i=1
  for y=1,pointsY do
    for x=1,pointsX do
      corners[i][1][1] = (2*(x-1) + (y-1)%2) * pointDist
      corners[i][1][2] = (y-1) * pointDist
      corners[i][1][3] = 0
      i = i+1
    end
  end
  return corners
end


function X3d.findCalibrationPatternPose (image, pattern, cameraMatrix3x3, distortionCoefficients)
  -- generate ground truth pattern
  local circlePositions = X3d.generatePatternPoints(pattern.width, pattern.height, pattern.pointSize)

  -- find circle pattern
  local ok, centers = cv.findCirclesGrid { image = image, patternSize = { height = pattern.height, width = pattern.width }, flags = pattern.type }
  assert(ok, "Calibration pattern not found!")

  -- estimate pattern pose
  local ok, targetRot, targetPos = cv.solvePnP { objectPoints = circlePositions, imagePoints = centers, cameraMatrix = cameraMatrix3x3, distCoeffs = distortionCoefficients }
  assert(ok, "Could not estimate pose: solvePnp failed!")

  local markerPose = X3d.generatePose(targetRot, targetPos)

  return markerPose, targetRot, targetPos
end


-- triangulate the 3d coordinates of a single point from 2d measurements at taken at different positions.
-- P is a table of 3x4 projection matrices
-- measurement is a table with 1x2 image measurements (point coordinates)
function X3d.triangulatePoint (P, measurements)
  assert(#P == #measurements)

  local m = #measurements

  local A = torch.zeros(2*#measurements, 3)
  local B = torch.zeros(2*#measurements, 1)

  for i = 1,#P do
    local p = P[i]
    local x = measurements[i][1][1]
    local y = measurements[i][1][2]

    A[{(i-1)*2+1, 1}] = p[{3,1}] * x - p[{1,1}]
    A[{(i-1)*2+1, 2}] = p[{3,2}] * x - p[{1,2}]
    A[{(i-1)*2+1, 3}] = p[{3,3}] * x - p[{1,3}]

    A[{(i-1)*2+2, 1}] = p[{3,1}] * y - p[{2,1}]
    A[{(i-1)*2+2, 2}] = p[{3,2}] * y - p[{2,2}]
    A[{(i-1)*2+2, 3}] = p[{3,3}] * y - p[{2,3}]

    B[{(i-1)*2+1, 1}] = p[{1,4}] - x * p[{3,4}]
    B[{(i-1)*2+2, 1}] = p[{2,4}] - y * p[{3,4}]
  end

  return torch.gels(B, A)
end


return X3d
