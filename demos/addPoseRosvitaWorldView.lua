--[[
rosvitaWorldView.lua
Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.
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
