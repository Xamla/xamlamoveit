--[[
Cuboid.lua

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
local xutils = require 'xamlamoveit.xutils.env'
local Cuboid = torch.class('xamlamoveit.xutils.Cuboid',xutils)


function Cuboid:__init(center, width, height, depth)
  self.corners = torch.zeros(8,3)
  self.center = center:clone()
  self.width = width
  self.height = height
  self.depth = depth
  self.corners[{1,{}}] = (self.center+torch.Tensor({-width,-height,-depth})*0.5)
  self.corners[{2,{}}] = (self.center+torch.Tensor({-width, height,-depth})*0.5)
  self.corners[{3,{}}] = (self.center+torch.Tensor({ width, height,-depth})*0.5)
  self.corners[{4,{}}] = (self.center+torch.Tensor({ width,-height,-depth})*0.5)
  self.corners[{5,{}}] = (self.center+torch.Tensor({-width,-height, depth})*0.5)
  self.corners[{6,{}}] = (self.center+torch.Tensor({-width, height, depth})*0.5)
  self.corners[{7,{}}] = (self.center+torch.Tensor({ width, height, depth})*0.5)
  self.corners[{8,{}}] = (self.center+torch.Tensor({ width,-height, depth})*0.5)
end


function Cuboid:isInside(point)
  local pointv = point - self.corners[1]
  local u,v,w
  u = (self.corners[2] -self.corners[1])
  v = (self.corners[4] -self.corners[1])
  w = (self.corners[5] -self.corners[1])

  if 0< torch.dot(u,pointv) and torch.dot(u,pointv) < torch.dot(u,u) and 0< torch.dot(v,pointv) and torch.dot(v,pointv)<v:dot(v) and 0<torch.dot(w,pointv) and torch.dot(w,pointv)<torch.dot(w,w) then
    return true
  end
  return false
end


function Cuboid.empty()
  return Cuboid.new(torch.zeros(3), 0, 0, 0)
end


function Cuboid:setCenter(center)
  local offset = center -self.center
  self.center = center:clone()
  for i = 1,8 do
    self.corners[i] = self.corners[i] + offset
  end
end


function Cuboid:setParameters(width, height, depth)
  return Cuboid.new(self.center,width,heigth,depth)
end


function Cuboid.fromMinMax(minPt, maxPt)
  local width, height, depth
  local center = (maxPt - minPt) * 0.5
  width = maxPt[1] - minPt[1]
  height = maxPt[2] - minPt[2]
  depth = maxPt[3] - minPt[3]
  return Cuboid.new(center,width,height,depth)
end


function Cuboid:getXMin()
  return torch.min(self.corners[{{},1}])
end


function Cuboid:getYMin()
  return torch.min(self.corners[{{},2}])
end


function Cuboid:getZMin()
  return torch.min(self.corners[{{},3}])
end


function Cuboid:getXMax()
  return torch.max(self.corners[{{},1}])
end


function Cuboid:getYMax()
  return torch.max(self.corners[{{},2}])
end


function Cuboid:getZMax()
  return torch.max(self.corners[{{},3}])
end


function Cuboid:size()
  return self:Width(), self:Height(), self:Depth()
end


function Cuboid:Width()
  return self.width
end


function Cuboid:Height()
  return self.height
end


function Cuboid:Depth()
  return self.depth
end


function Cuboid:volume()
  return self:Width() * self:Height() * self:Depth()
end


function Cuboid:surfaceArea()
  local a,b,c = self:Width(),self:Height(),self:Depth()
  return 2 * (a*b + a*c + b*c)
end


function Cuboid:diagonal()
  local a,b,c = self:Width(),self:Height(),self:Depth()
  return math.sqrt(a*a + b*b + c*c)
end


function Cuboid:center()
  return self.center
end


function Cuboid:isEmpty()
  return self:getXMin() == self:getXMax() and self:getYMin() == self:getYMax() and self:getZMin() == self:getZMax()
end


function Cuboid:clone()
  return Cuboid.new(self.center,self.width,self.height,self.depth)
end


function Cuboid:__tostring()
  return string.format("{ min: (%.2f, %.2f, %.2f), max: (%.2f, %.2f, %.2f), size: (W: %.2f H: %.2f D: %.2f) }",
    self:getXMin(), self:getYMin(), self:getZMin(), self:getXMax(), self:getYMax(), self:getZMax(),
    self:Width(), self:Height(), self:Depth())
end


return Cuboid
