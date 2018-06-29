--[[
rosvitaWorldView.lua

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
local torch = require 'torch'
local ros = require 'ros'
local tf = ros.tf
local rosvita = require 'xamlamoveit.rosvita'
local datatypes = require 'xamlamoveit.datatypes'
--[[
    This example works with rosvita ros start and 'GO'
]]
--Init ros node
ros.init('MoveL')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()
local ok, value, values, error
local client = rosvita.WorldViewClient.new(nh)

local valueFolderPath = 'MyValues'
--[[
ok, error = client:addFolder('myFolder', '/some/path/where/it/should/go') --Subfolders are also created if they are not there
assert(ok, error)
ok, error = client:addFolder(valueFolderPath)
assert(ok, error)

local pose = datatypes.Pose()
local names = {'first', 'second', 'third'}
for i, name in ipairs(names) do
    ok, error = client:addPose(name, valueFolderPath, pose)
    assert(ok, error)
end

for i, name in ipairs(names) do
    ok, error = client:updatePose(name, valueFolderPath, pose)
    assert(ok, error)
end

ok, values, error = client:removePose(names[2], valueFolderPath)
assert(ok, error)

local recursive = false
local prefix = 't'
ok, values, error = client:queryPoses(prefix, valueFolderPath, recursive)
assert(ok, error)
for i, v in pairs(values) do
    print(v)
end
--]]

local primitive = datatypes.CollisionPrimitive.UnitBox:clone()
local obj = datatypes.CollisionObject('world', {primitive})
ok,  error = client:addCollisionObject("Box_4", "/", obj)
assert(not ok, error)
ok, result, error = client:getCollisionObject("Box_2")
assert(ok, error)
ok, result, error = client:queryCollisionObject("Box", "", true)
assert(ok, error)
for i, v in ipairs(result) do
    print(torch.type(v), v:getFrame())
end
os.exit()

local end_effector_link_name = "world"
local A, B, C, D, E, F
A = datatypes.Pose()
A:setFrame(end_effector_link_name)
A:setTranslation(torch.Tensor {1.2, 0.0, 0.01})
B = datatypes.Pose()
B:setFrame(end_effector_link_name)
B:setTranslation(torch.Tensor {-1.02, 0.02, -0.01})
C = datatypes.Pose()
C:setFrame(end_effector_link_name)
C:setTranslation(torch.Tensor {1.02, 0.0, 0.01})
D = datatypes.Pose()
D:setFrame(end_effector_link_name)
D:setTranslation(torch.Tensor {-1.01, 0.01, -0.01})
E = datatypes.Pose()
E:setFrame(end_effector_link_name)
E:setTranslation(torch.Tensor {-1.01, -0.01, 0.01})
F = datatypes.Pose()
F:setFrame(end_effector_link_name)
F:setTranslation(torch.Tensor { 1.0, -0.02, -0.01})

local cartesianpath = {A, B, C, D, E, F}
ok, error =client:addCartesianPath(end_effector_link_name, '', cartesianpath)
assert(ok, error)

ok, error =client:updateCartesianPath(end_effector_link_name, '', cartesianpath)
assert(ok, error)

local result
ok, result, error = client:getCartesianPath(end_effector_link_name)

assert(ok, error)
for i, v in ipairs(result) do
    print(v)
end
-- shutdown ROS
sp:stop()

ros.shutdown()
