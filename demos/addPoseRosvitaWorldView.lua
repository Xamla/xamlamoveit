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

local valueFolderPath = ''

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
ok, error = client:addPose("A", valueFolderPath, A)
assert(ok, error)

-- shutdown ROS
sp:stop()

ros.shutdown()
